"""Torque estático que la gravedad pide a cada articulación del brazo (sin ROS: se prueba sola).

Para qué sirve: los motores paso a paso del EB300 tienen un torque de retención, y el límite
`arm_effort` del URDF (20 N·m) es provisorio. Con las masas reales de cada pieza (motores,
reductores, piezas impresas) este módulo dice cuánto torque necesita cada articulación para
sostener el brazo quieto, y con eso se puede elegir la relación de los reductores.

Física (estática, brazo quieto): la gravedad empuja cada eslabón que cuelga de la articulación
i con una fuerza m·g en su centro de masa c. Ese empuje hace un torque sobre el eje de la
articulación:

    tau_i = eje_i · sum_k  (c_k − p_i) × (m_k · g)

con p_i un punto del eje, eje_i su dirección en el mundo, y la suma sobre todos los links que
cuelgan de esa articulación (incluidos los motores, que son links fijos con solo masa). El signo
depende del sentido del eje; lo que importa para dimensionar es el valor absoluto.

Se lee del URDF procesado (como kinematics.py), así que si cambia una masa en el xacro, cambia
el resultado. No usa inercias: en reposo solo cuentan masa y centro de masa.
"""

import xml.etree.ElementTree as ET

import numpy as np

from andesrobot_arm.kinematics import _floats, axis_angle_to_matrix, make_pose

G = np.array([0.0, 0.0, -9.81])   # gravedad [m/s²] en el mundo (+Z arriba)


class ModeloMasas:
    """Árbol del URDF con la masa y el centro de masa de cada link."""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.mass = {}        # link -> masa [kg]
        self.com = {}         # link -> pose 4x4 del centro de masa en el frame del link
        for link in root.findall('link'):
            inertial = link.find('inertial')
            if inertial is None or inertial.find('mass') is None:
                continue
            origin = inertial.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, [0, 0, 0])
            rpy = _floats(origin.get('rpy') if origin is not None else None, [0, 0, 0])
            self.mass[link.get('name')] = float(inertial.find('mass').get('value'))
            self.com[link.get('name')] = make_pose(xyz, rpy)
        self.joints = {}      # joint -> dict
        self.children = {}    # link -> [joints que cuelgan de él]
        for el in root.findall('joint'):
            origin = el.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, [0, 0, 0])
            rpy = _floats(origin.get('rpy') if origin is not None else None, [0, 0, 0])
            axis_el = el.find('axis')
            axis = _floats(axis_el.get('xyz') if axis_el is not None else None, [1, 0, 0])
            joint = dict(name=el.get('name'), type=el.get('type'),
                         parent=el.find('parent').get('link'), child=el.find('child').get('link'),
                         origin=make_pose(xyz, rpy), axis=axis / np.linalg.norm(axis))
            self.joints[joint['name']] = joint
            self.children.setdefault(joint['parent'], []).append(joint)
        children = {j['child'] for j in self.joints.values()}
        parents = {j['parent'] for j in self.joints.values()}
        roots = parents - children
        if len(roots) != 1:
            raise ValueError(f'el URDF debe tener un solo link raíz, tiene {sorted(roots)}')
        self.root = roots.pop()

    def poses(self, q=None):
        """Pose 4x4 de cada link en el mundo. q = {nombre de articulación: valor} (0 si falta)."""
        q = q or {}
        poses = {self.root: np.eye(4)}
        stack = [self.root]
        while stack:
            link = stack.pop()
            for joint in self.children.get(link, []):
                motion = np.eye(4)
                value = q.get(joint['name'], 0.0)
                if joint['type'] in ('revolute', 'continuous'):
                    motion[:3, :3] = axis_angle_to_matrix(joint['axis'], value)
                elif joint['type'] == 'prismatic':
                    motion[:3, 3] = joint['axis'] * value
                poses[joint['child']] = poses[link] @ joint['origin'] @ motion
                stack.append(joint['child'])
        return poses

    def subtree(self, link):
        """El link y todos los que cuelgan de él."""
        out, stack = [], [link]
        while stack:
            current = stack.pop()
            out.append(current)
            stack.extend(j['child'] for j in self.children.get(current, []))
        return out

    def torque(self, joint_name, q=None, carga=None, poses=None):
        """
        Torque estático [N·m] que la gravedad hace sobre el eje de una articulación.

        carga = (masa [kg], link, xyz en ese link): una masa extra, por ejemplo un objeto en la
        pinza. Para una articulación prismática devuelve la fuerza [N] a lo largo de su eje.
        poses = las poses de self.poses(q), si ya se calcularon (ahorra repetirlo por cada joint).
        """
        joint = self.joints[joint_name]
        poses = poses if poses is not None else self.poses(q)
        frame = poses[joint['parent']] @ joint['origin']   # el eje, antes de su movimiento
        axis = frame[:3, :3] @ joint['axis']
        point = frame[:3, 3]
        total = 0.0
        masas = [(self.mass[link], (poses[link] @ self.com[link])[:3, 3])
                 for link in self.subtree(joint['child']) if link in self.mass]
        if carga is not None:
            masa, link, xyz = carga
            if link in self.subtree(joint['child']):
                masas.append((masa, (poses[link] @ np.append(xyz, 1.0))[:3]))
        for masa, centro in masas:
            fuerza = masa * G
            if joint['type'] == 'prismatic':
                total += float(axis @ fuerza)
            else:
                total += float(axis @ np.cross(centro - point, fuerza))
        return total

    def masa_colgante(self, joint_name):
        """Masa total [kg] de lo que cuelga de una articulación."""
        joint = self.joints[joint_name]
        return sum(self.mass.get(link, 0.0) for link in self.subtree(joint['child']))


def torque_maximo(modelo, articulaciones, limites, n=20000, semilla=0, carga=None):
    """
    Torque máximo (valor absoluto) de cada articulación sobre n posturas al azar dentro de los
    límites. Devuelve {articulación: (torque, postura que lo da)}.
    """
    rng = np.random.default_rng(semilla)
    mejor = {name: (0.0, {}) for name in articulaciones}
    for _ in range(n):
        q = {name: rng.uniform(*limites[name]) for name in limites}
        poses = modelo.poses(q)
        for name in articulaciones:
            valor = abs(modelo.torque(name, q, carga, poses))
            if valor > mejor[name][0]:
                mejor[name] = (valor, dict(q))
    return mejor
