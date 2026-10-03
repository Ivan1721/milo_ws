"""Cinemática directa e inversa del brazo de Milo, leída del URDF (sin ROS: se puede probar solo).

Ecuaciones y métodos explicados en docs/BRAZO.md.
- Cinemática directa: producto de transformaciones homogéneas, una por articulación, tomadas
  del URDF procesado (no hay tabla Denavit-Hartenberg escrita a mano: si cambia el xacro,
  cambia la cinemática).
- Cinemática inversa: mínimos cuadrados amortiguados (DLS) ponderados, con reinicios
  aleatorios. La muñeca no es esférica (joint_4 y joint_6 son paralelos, a ~9 cm), así que
  no hay solución cerrada.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np

# Cadena del brazo solo (6 articulaciones): de la base del brazo al punto de agarre.
ARM_BASE = 'arm_base_link_1'
ARM_TIP = 'gripper_tcp'
# Cadena brazo + lift (7 articulaciones), desde el suelo. Supone la base quieta.
LIFT_BASE = 'base_footprint'
LIFT_JOINT = 'vertical_lift_joint'


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll), la convención del <origin rpy> del URDF."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(rot):
    pitch = np.arctan2(-rot[2, 0], np.hypot(rot[0, 0], rot[1, 0]))
    roll = np.arctan2(rot[2, 1], rot[2, 2])
    yaw = np.arctan2(rot[1, 0], rot[0, 0])
    return np.array([roll, pitch, yaw])


def axis_angle_to_matrix(axis, angle):
    """Fórmula de Rodrigues: rotación de 'angle' rad alrededor del eje unitario 'axis'."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * k @ k


def rotation_log(rot):
    """Vector de rotación (eje * ángulo) de una matriz de rotación."""
    cos_angle = np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # Cerca de pi la parte antisimétrica se anula: el eje sale de la parte simétrica.
        diag = np.clip((np.diag(rot) + 1.0) / 2.0, 0.0, None)
        axis = np.sqrt(diag)
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i:
                axis[j] = np.copysign(axis[j], rot[i, j] + rot[j, i])
        return angle * axis / np.linalg.norm(axis)
    vee = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return angle * vee / (2.0 * np.sin(angle))


def quaternion_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(rot):
    """(x, y, z, w) de una matriz de rotación."""
    w = np.sqrt(max(0.0, 1.0 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])) / 2.0,
                    rot[2, 1] - rot[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - rot[0, 0] + rot[1, 1] - rot[2, 2])) / 2.0,
                    rot[0, 2] - rot[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - rot[0, 0] - rot[1, 1] + rot[2, 2])) / 2.0,
                    rot[1, 0] - rot[0, 1])
    return np.array([x, y, z, w])


def make_pose(xyz, rpy=(0.0, 0.0, 0.0)):
    """Transformación homogénea 4x4 a partir de posición y rpy."""
    pose = np.eye(4)
    pose[:3, :3] = rpy_to_matrix(*rpy)
    pose[:3, 3] = xyz
    return pose


def _floats(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, float)


class Joint:
    """Una articulación del URDF: su origin fijo, su eje y sus límites (si tiene)."""

    def __init__(self, name, jtype, parent, child, origin, axis, lower=None, upper=None):
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.origin = origin
        self.axis = axis / np.linalg.norm(axis)
        self.lower = lower
        self.upper = upper

    def transform(self, q):
        """Origin fijo por el movimiento: giro en el eje (rotativa) o traslación (prismática)."""
        motion = np.eye(4)
        if self.type in ('revolute', 'continuous'):
            motion[:3, :3] = axis_angle_to_matrix(self.axis, q)
        elif self.type == 'prismatic':
            motion[:3, 3] = self.axis * q
        return self.origin @ motion


class ArmKinematics:
    """Cadena serial entre dos links del URDF, como producto de transformaciones homogéneas."""

    def __init__(self, urdf_xml, base=ARM_BASE, tip=ARM_TIP):
        joints_by_child = {}
        for el in ET.fromstring(urdf_xml).findall('joint'):
            origin_el = el.find('origin')
            xyz = _floats(origin_el.get('xyz') if origin_el is not None else None, [0, 0, 0])
            rpy = _floats(origin_el.get('rpy') if origin_el is not None else None, [0, 0, 0])
            axis_el = el.find('axis')
            axis = _floats(axis_el.get('xyz') if axis_el is not None else None, [1, 0, 0])
            limit_el = el.find('limit')
            lower = upper = None
            if el.get('type') in ('revolute', 'prismatic') and limit_el is not None:
                lower = float(limit_el.get('lower', 0.0))
                upper = float(limit_el.get('upper', 0.0))
            joint = Joint(el.get('name'), el.get('type'), el.find('parent').get('link'),
                          el.find('child').get('link'), make_pose(xyz, rpy), axis, lower, upper)
            joints_by_child[joint.child] = joint

        # Recorrer desde la punta hacia la base (cada link tiene un solo joint padre).
        chain = []
        link = tip
        while link != base:
            if link not in joints_by_child:
                raise ValueError(f"'{tip}' no cuelga de '{base}'")
            chain.append(joints_by_child[link])
            link = chain[-1].parent
        chain.reverse()

        self.base = base
        self.tip = tip
        self.chain = chain
        self.joints = [j for j in chain if j.type != 'fixed']
        self.tool = np.eye(4)

    @classmethod
    def from_xacro(cls, xacro_path=None, mappings=None, **kwargs):
        """
        Construir desde el xacro de Milo (por defecto el instalado en andesrobot_description).

        mappings = argumentos del xacro. Por defecto lock_arm:=false, porque con el brazo
        bloqueado todas sus articulaciones son 'fixed'.
        """
        import xacro
        if xacro_path is None:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('andesrobot_description')
            xacro_path = os.path.join(share, 'urdf', 'andesrobot.urdf.xacro')
        args = {'lock_arm': 'false'}
        args.update(mappings or {})
        return cls(xacro.process_file(xacro_path, mappings=args).toxml(), **kwargs)

    @property
    def joint_names(self):
        return [j.name for j in self.joints]

    @property
    def dof(self):
        return len(self.joints)

    def set_tool(self, xyz, rpy=(0.0, 0.0, 0.0)):
        """Desplazamiento fijo extra del TCP, en el frame de la punta."""
        self.tool = make_pose(xyz, rpy)

    def frames(self, q):
        """Frame de cada articulación (origin aplicado, sin su movimiento) y pose del TCP."""
        q = np.asarray(q, float)
        pose = np.eye(4)
        joint_frames = []
        i = 0
        for joint in self.chain:
            if joint.type == 'fixed':
                pose = pose @ joint.origin
                continue
            joint_frames.append(pose @ joint.origin)
            pose = pose @ joint.transform(q[i])
            i += 1
        return joint_frames, pose @ self.tool

    def fk(self, q):
        """Cinemática directa: pose 4x4 del TCP en el frame base."""
        return self.frames(q)[1]

    def jacobian(self, q):
        """Jacobiano geométrico 6xN [lineal; angular] del TCP, en el frame base."""
        joint_frames, tcp = self.frames(q)
        jac = np.zeros((6, self.dof))
        for i, (joint, frame) in enumerate(zip(self.joints, joint_frames)):
            axis = frame[:3, :3] @ joint.axis
            if joint.type == 'prismatic':
                jac[:3, i] = axis
            else:
                jac[:3, i] = np.cross(axis, tcp[:3, 3] - frame[:3, 3])
                jac[3:, i] = axis
        return jac

    def _clamp(self, q):
        q = q.copy()
        for i, joint in enumerate(self.joints):
            if (joint.type == 'revolute' and joint.lower is not None
                    and joint.upper - joint.lower >= 2.0 * np.pi - 1e-6):
                # Un rango de vuelta completa (los ±π provisorios) da la vuelta en vez de
                # recortar: así una solución cerca de -π puede seguir hasta +π.
                q[i] = (q[i] - joint.lower) % (2.0 * np.pi) + joint.lower
            elif joint.lower is not None:
                q[i] = np.clip(q[i], joint.lower, joint.upper)
            elif joint.type == 'continuous':
                q[i] = (q[i] + np.pi) % (2.0 * np.pi) - np.pi
        return q

    def _random_q(self, rng):
        """Configuración al azar dentro de los límites del URDF (±π si no tiene)."""
        return np.array([rng.uniform(j.lower, j.upper) if j.lower is not None
                         else rng.uniform(-np.pi, np.pi) for j in self.joints])

    def ik(self, target, q0=None, position_only=False, tol_pos=1e-4, tol_rot=1e-3,
           max_iter=200, damping=0.01, max_step=0.5, restarts=20, seed=None, weights=None):
        """
        Cinemática inversa por DLS, reintentando desde configuraciones al azar.

        target: pose 4x4 deseada del TCP en el frame base. q0: desde dónde partir (la posición
        actual, para que la solución quede cerca). weights: costo de cada articulación
        (por defecto 1); una con más peso se mueve menos, p. ej. el lift en la cadena de 7.

        Devuelve (q, éxito, (error_posición_m, error_rotación_rad)). Si ningún intento
        converge devuelve el mejor con éxito=False: revisar siempre el éxito.
        """
        rng = np.random.default_rng(seed)
        inv_weights = np.ones(self.dof) if weights is None else 1.0 / np.asarray(weights, float)
        q_start = np.zeros(self.dof) if q0 is None else np.asarray(q0, float)
        best = None
        for attempt in range(restarts + 1):
            if attempt > 0:
                q_start = self._random_q(rng)
            q, ok, err = self._solve(target, q_start, position_only, tol_pos, tol_rot,
                                     max_iter, damping, max_step, inv_weights)
            if ok:
                return q, True, err
            if best is None or err[0] + 0.1 * err[1] < best[2][0] + 0.1 * best[2][1]:
                best = (q, ok, err)
        return best

    def _solve(self, target, q, position_only, tol_pos, tol_rot, max_iter, damping, max_step,
               inv_weights):
        q = self._clamp(q)
        rows = slice(0, 3) if position_only else slice(0, 6)
        for _ in range(max_iter):
            pose = self.fk(q)
            e_pos = target[:3, 3] - pose[:3, 3]
            e_rot = rotation_log(target[:3, :3] @ pose[:3, :3].T)
            err = (np.linalg.norm(e_pos), 0.0 if position_only else np.linalg.norm(e_rot))
            if err[0] < tol_pos and err[1] < tol_rot:
                return q, True, err
            e = np.concatenate([e_pos, e_rot])[rows]
            jac = self.jacobian(q)[rows]
            lam2 = damping ** 2
            # DLS ponderado: dq = W^-1 J^T (J W^-1 J^T + lambda^2 I)^-1 e
            jw = jac * inv_weights
            reg = lam2 * np.eye(jac.shape[0])
            dq = inv_weights * (jac.T @ np.linalg.solve(jw @ jac.T + reg, e))
            norm = np.linalg.norm(dq)
            if norm > max_step:
                dq *= max_step / norm
            q = self._clamp(q + dq)
        return q, False, err
