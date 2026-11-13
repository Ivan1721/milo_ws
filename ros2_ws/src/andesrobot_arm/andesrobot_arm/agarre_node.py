"""Nodo agarre_node: agarra un objeto de la mesa por su nombre, lo levanta y retrae el brazo.

  ros2 run andesrobot_arm agarre_node tenedor

El nombre es un id o una clase del catálogo config/objetos.yaml (tenedor, fork, taza, cup...).
Hace una sola vez la secuencia completa y termina. Fases 4 y 5 del plan de agarre (P15 en
docs/PENDIENTES.md): el objeto se ubica con la cámara, sin usar la pose verdadera de Gazebo.

Quién hace qué:
  - objeto_node (VISIÓN) detecta el objeto con YOLO y la profundidad y publica dónde está y cuánto
    mide: /objeto/pose (centro y eje largo), /objeto/medidas (largo, ancho, alto) y
    /objeto/distancia (del TCP al objeto). Si no está corriendo, agarre_node lo arranca con
    `clase` = la del objeto pedido.
  - agarre_node (ESTE) mueve el brazo y la pinza con lo que publica la visión.

Pasos:
  1. MIRAR: abre la pinza y la lleva a la primera de las `vistas`, de frente a la mesa y
     apuntando un poco antes del objeto, para que la cámara (sobre la pinza) lo vea sin que la
     pinza lo tape. Desde ahí el modelo reconoce el tenedor con 0.2-0.6 de confianza (es un
     modelo de fotos reales; desde arriba lo confunde con "card", ver evaluar_objetos.py).
  2. ENCONTRAR: espera `n_mediciones` mediciones nuevas de la visión que coincidan (centro a
     menos de `tol_estable`); si no llegan en `tiempo_por_vista`, prueba la vista siguiente.
     Avisa que lo encontró y a qué distancia. Si mide más que lo que abre la pinza
     (`ancho_max`), avisa que no cabe y termina.
  3. PREVIA: con esas medidas arma la pose de destino (agarre.pose_desde_arriba): la pinza va
     `sobre` (10 cm) encima del objeto, apuntando hacia abajo, con los dedos cruzados al eje
     largo (MoveIt, camino libre de choques).
  4. BAJAR en línea recta hasta la altura de agarre (camino cartesiano de MoveIt, lento). No se
     vuelve a detectar: tan cerca la cámara ve poco y la pinza tapa el objeto.
  5. CERRAR cada dedo `apriete` (2 mm) más adentro que el ancho medido, y comprobar por la
     posición de los dedos que se frenaron contra algo (si cerraron en vacío, no hay objeto:
     abre, sube y termina con error).
  6. UNIR el objeto a la pinza en la escena de MoveIt (una caja con sus medidas pegada a
     gripper_tcp): MoveIt lo cuida de chocar al moverse y sabe que los dedos lo tocan.
  7. LEVANTAR `subir` (10 cm) en línea recta y RETRAER: la pinza va a la pose `retraer`, cerca
     del cuerpo de Milo, con el objeto todavía en la mano y SIN GIRAR (en línea recta; si la
     recta no sale, termina con error sin mover el brazo). `mantener_orientacion:=false` vuelve
     al camino libre de antes del 13-nov.
  8. Al final revisa que los dedos sigan frenados por el objeto (si se cayó, cierran del todo).

Necesita la simulación con el objeto en la mesa y SIN OctoMap (con OctoMap el objeto que ve la
cámara es un obstáculo y MoveIt no deja acercar la pinza):
  ros2 launch andesrobot_arm arm_sim.launch.py objetos:=tenedor octomap:=false     # terminal 1
  ros2 run andesrobot_arm agarre_node tenedor                                       # terminal 2
Parámetros (--ros-args -p nombre:=valor): vistas, confianza, sobre, subir, retraer, apriete,
velocidad...
El objeto queda en la mano al terminar; para soltarlo, abrir la pinza (ver BRAZO.md).
La pinza abre 49 mm: la taza o el cubo Rubik enteros no caben (se agarrarán por el asa o el borde
en la fase 6); de lo que el modelo reconoce en la simulación, el tenedor sí.
"""
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import AttachedCollisionObject, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from andesrobot_arm.agarre import (
    ABIERTA, altura_de_agarre, apertura, estable, hay_objeto, objetivo_dedos, pose_desde_arriba,
    promediar)
from andesrobot_arm.arm_moveit_node import MOTIVOS, caja, meta_de_pose
from andesrobot_arm.catalogo_objetos import elegir, leer_catalogo
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix
from andesrobot_arm.manija import orientacion_pinza

FRAME = 'base_footprint'
TCP = 'gripper_tcp'
# Links que pueden tocar el objeto agarrado sin que MoveIt lo cuente como choque.
TOCAN = ['right_finger_link_1', 'left_finger_link_1', 'link_6_1']


class Fallo(Exception):
    """Un paso de la secuencia no se pudo hacer: se avisa y el nodo termina."""


def objeto_del_catalogo(nombre):
    """El objeto del catálogo config/objetos.yaml con ese id o clase (ValueError si no está)."""
    ruta = os.path.join(get_package_share_directory('andesrobot_arm'), 'config', 'objetos.yaml')
    with open(ruta) as f:
        return elegir(leer_catalogo(f.read()), nombre)[0]


class AgarreNode(Node):
    def __init__(self, objeto):
        # Tiempo de la simulación: las marcas de tiempo de la visión son de Gazebo.
        super().__init__('agarre_node', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.objeto = objeto
        self.clase = objeto['clase']
        # Vistas para buscar el objeto, de a 6 números: dónde va el TCP (x, y, z) y a qué punto
        # de la mesa apunta la pinza (x, y, z), en base_footprint. La cámara va 10 cm sobre la
        # pinza y mira por encima de ella: apuntando la pinza JUSTO al objeto, la pinza lo tapa.
        # Por eso apuntan 6 cm antes del objeto (en (0.80, 0)) y el objeto sale arriba de la
        # pinza en la imagen. Elegidas el 7-nov probando 12 vistas: el modelo vio el tenedor con
        # 0.60, 0.56 y 0.51 (apuntando al objeto mismo, 0.21-0.47). Se prueba una tras otra
        # hasta encontrarlo, `tiempo_por_vista` segundos cada una.
        self.declare_parameter('vistas', [0.56, 0.0, 0.92, 0.74, 0.0, 0.72,
                                          0.56, 0.0, 1.00, 0.74, 0.0, 0.72,
                                          0.50, 0.0, 0.92, 0.74, 0.0, 0.72])
        self.declare_parameter('tiempo_por_vista', 20.0)
        # Confianza mínima con que se arranca objeto_node. El modelo es de fotos reales y en la
        # simulación ve el tenedor con 0.2-0.6 (P14); con la mesa despejada, la clase pedida, 3
        # detecciones estables y un objeto de medidas razonables, 0.2 no da falsos positivos.
        self.declare_parameter('confianza', 0.2)
        self.declare_parameter('retraer', [0.45, 0.0, 1.0])   # TCP al final [m], hacia abajo
        self.declare_parameter('sobre', 0.10)       # pose previa: cuánto sobre el agarre [m]
        self.declare_parameter('subir', 0.10)       # cuánto levantar el objeto [m]
        self.declare_parameter('apriete', 0.002)    # cada dedo, más adentro que el ancho [m]
        self.declare_parameter('ancho_max', 0.045)  # más ancho no cabe en la pinza (abre 49 mm)
        self.declare_parameter('n_mediciones', 3)   # mediciones seguidas que deben coincidir
        self.declare_parameter('tol_estable', 0.01)
        self.declare_parameter('velocidad', 0.3)        # escala de velocidad de MoveIt
        self.declare_parameter('velocidad_cerca', 0.1)  # al bajar y subir en línea recta
        # Con el objeto en la mano, retraer sin girar la pinza (en línea recta).
        self.declare_parameter('mantener_orientacion', True)
        self.p = {n: self.get_parameter(n).value for n in (
            'vistas', 'tiempo_por_vista', 'confianza', 'retraer', 'sobre', 'subir', 'apriete',
            'ancho_max', 'n_mediciones', 'tol_estable', 'velocidad', 'velocidad_cerca',
            'mantener_orientacion')}

        self.dedos = None
        self.pose_obj = self.medidas = None    # lo último que publicó la visión
        self.distancia = None
        self.mediciones = []                   # pose + medidas con la misma marca de tiempo
        self.desde = None                      # solo valen mediciones posteriores a esto
        self.vision = None                     # objeto_node, si lo arrancó este nodo
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(JointState, '/joint_states', self.al_joints, 10)
        self.create_subscription(PoseStamped, '/objeto/pose', self.al_pose, 10)
        self.create_subscription(Vector3Stamped, '/objeto/medidas', self.al_medidas, 10)
        self.create_subscription(Float32, '/objeto/distancia', self.al_distancia, 10)
        self.mover_cli = ActionClient(self, MoveGroup, '/move_action')
        self.ejecutar_cli = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesiano_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.pinza_cli = ActionClient(self, FollowJointTrajectory,
                                      '/gripper_controller/follow_joint_trajectory')
        self.escena = self.create_publisher(PlanningScene, '/planning_scene', 10)

    # ------------------------------------------------------------------ suscripciones
    def al_joints(self, m):
        # Hay dos publicadores de /joint_states (ruedas y brazo): solo sirve el de los dedos.
        if 'right_finger_joint' in m.name and 'left_finger_joint' in m.name:
            self.dedos = (m.position[m.name.index('right_finger_joint')],
                          m.position[m.name.index('left_finger_joint')])

    def al_pose(self, m):
        self.pose_obj = m
        self.juntar()

    def al_medidas(self, m):
        self.medidas = m
        self.juntar()

    def al_distancia(self, m):
        self.distancia = m.data

    def juntar(self):
        """Une /objeto/pose y /objeto/medidas de la misma imagen (misma marca de tiempo) en una
        medición como las de agarre.analizar_objeto."""
        if self.pose_obj is None or self.medidas is None or self.desde is None:
            return
        t = Time.from_msg(self.pose_obj.header.stamp)
        if t != Time.from_msg(self.medidas.header.stamp) or t < self.desde:
            return
        o, c, v = self.pose_obj.pose.orientation, self.pose_obj.pose.position, self.medidas.vector
        eje = quaternion_to_matrix(o.x, o.y, o.z, o.w)[:3, 0]
        self.mediciones.append({
            'centro': np.array([c.x, c.y, c.z]), 'eje': eje, 'largo': v.x, 'ancho': v.y,
            'alto': v.z, 'mesa_z': c.z - v.z / 2.0, 'n': 1})
        self.pose_obj = self.medidas = None

    # ------------------------------------------------------------------ esperas
    def esperar(self, segundos):
        fin = time.time() + segundos
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)

    def resultado(self, futuro):
        while not futuro.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return futuro.result()

    def accion(self, cliente, meta):
        """Manda la meta de una acción y espera su resultado (None si la rechazaron)."""
        if not cliente.wait_for_server(timeout_sec=10.0):
            raise Fallo(f'no está el servidor de la acción {cliente._action_name}')
        manejo = self.resultado(cliente.send_goal_async(meta))
        if not manejo.accepted:
            return None
        return self.resultado(manejo.get_result_async()).result

    def transformacion(self, destino, origen):
        try:
            t = self.tf.lookup_transform(destino, origen, Time(),
                                         timeout=RclDuration(seconds=0.5))
        except TransformException as error:
            raise Fallo(f'sin TF {origen} -> {destino}: {error}')
        q, p = t.transform.rotation, t.transform.translation
        return quaternion_to_matrix(q.x, q.y, q.z, q.w)[:3, :3], np.array([p.x, p.y, p.z])

    # ------------------------------------------------------------------ movimientos
    def pose(self, posicion, giro):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = (float(v) for v in posicion)
        q = matrix_to_quaternion(giro)
        msg.orientation.x, msg.orientation.y = float(q[0]), float(q[1])
        msg.orientation.z, msg.orientation.w = float(q[2]), float(q[3])
        return msg

    def mover(self, pose, que):
        """Lleva el TCP a `pose` con MoveIt (camino libre de choques)."""
        meta = meta_de_pose(FRAME, pose)
        req = meta.request
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = float(
            self.p['velocidad'])
        self.get_logger().info(f'{que}: pidiendo camino a MoveIt...')
        r = self.accion(self.mover_cli, meta)
        codigo = r.error_code.val if r else MoveItErrorCodes.FAILURE
        if codigo != MoveItErrorCodes.SUCCESS:
            raise Fallo(f'{que}: MoveIt no movió el brazo (código {codigo}): '
                        f'{MOTIVOS.get(codigo, "error desconocido")}')

    def linea_recta(self, pose, que, evitar_choques=True):
        """Lleva el TCP en línea recta hasta `pose` (camino cartesiano de MoveIt) y lo ejecuta.
        Para bajar y subir sobre el objeto: un camino libre podría dar una vuelta y barrer la
        mesa o el objeto."""
        if not self.cartesiano_cli.wait_for_service(timeout_sec=10.0):
            raise Fallo('no está el servicio /compute_cartesian_path de move_group')
        pedido = GetCartesianPath.Request()
        pedido.header.frame_id = FRAME
        pedido.start_state.is_diff = True
        pedido.group_name = 'brazo'
        pedido.link_name = TCP
        pedido.waypoints = [pose]
        pedido.max_step = 0.005          # un punto cada 5 mm de recorrido
        # Saltos: entre dos puntos seguidos (5 mm) ninguna articulación puede girar más de
        # 0.3 rad ni el lift moverse más de 2 cm. Sin esto (medido el 13-nov) una recta de
        # 222 puntos dio media vuelta a la muñeca entre dos puntos: la pinza giró 180° aunque
        # en cada punto tenía la orientación pedida. Si salta, la recta se corta ahí.
        pedido.jump_threshold = 0.0      # el umbral relativo, apagado
        pedido.revolute_jump_threshold = 0.3
        pedido.prismatic_jump_threshold = 0.02
        pedido.avoid_collisions = evitar_choques
        pedido.max_velocity_scaling_factor = float(self.p['velocidad_cerca'])
        pedido.max_acceleration_scaling_factor = float(self.p['velocidad_cerca'])
        r = self.resultado(self.cartesiano_cli.call_async(pedido))
        if r.fraction < 0.99:
            raise Fallo(f'{que}: MoveIt solo encontró el {r.fraction * 100:.0f} % del camino en '
                        'línea recta (¿choca con la mesa o queda fuera de alcance?)')
        meta = ExecuteTrajectory.Goal(trajectory=r.solution)
        self.get_logger().info(f'{que}: en línea recta ({len(r.solution.joint_trajectory.points)}'
                               ' puntos)')
        res = self.accion(self.ejecutar_cli, meta)
        codigo = res.error_code.val if res else MoveItErrorCodes.FAILURE
        if codigo != MoveItErrorCodes.SUCCESS:
            raise Fallo(f'{que}: no se ejecutó (código {codigo}): '
                        f'{MOTIVOS.get(codigo, "error desconocido")}')

    def pinza(self, q):
        """Dedos en espejo (derecho q, izquierdo −q) en 1 s; espera a que se asienten."""
        meta = FollowJointTrajectory.Goal()
        meta.trajectory.joint_names = ['right_finger_joint', 'left_finger_joint']
        meta.trajectory.points = [JointTrajectoryPoint(positions=[q, -q],
                                                       time_from_start=Duration(sec=1))]
        r = self.accion(self.pinza_cli, meta)
        if r is None:
            raise Fallo('gripper_controller rechazó la orden de la pinza')
        self.esperar(1.0)
        return apertura(*self.dedos)

    # ------------------------------------------------------------------ visión
    def asegurar_vision(self):
        """Si nadie publica /objeto/pose, arranca objeto_node buscando la clase del objeto."""
        self.esperar(1.0)
        if self.count_publishers('/objeto/pose') > 0:
            self.get_logger().info('objeto_node ya está corriendo: uso su detección (ojo: debe '
                                   f'buscar la clase {self.clase} y con acercar:=false)')
            return
        self.get_logger().info(f'Arranco objeto_node buscando "{self.clase}"')
        self.vision = subprocess.Popen(
            ['ros2', 'run', 'andesrobot_arm', 'objeto_node', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', f'clase:={self.clase}',
             '-p', f'confianza:={self.p["confianza"]}'])

    def encontrar(self):
        """Recorre las vistas hasta que la visión encuentra el objeto (ver `encontrar_aqui`)."""
        v = np.array(self.p['vistas'], float).reshape(-1, 6)
        for i, vista in enumerate(v):
            self.mover(self.pose(vista[:3], orientacion_pinza(vista[3:] - vista[:3])),
                       f'Mirar (vista {i + 1} de {len(v)})')
            self.esperar(1.0)   # que la imagen sea de la pose nueva
            m = self.encontrar_aqui()
            if m is not None:
                return m
            self.get_logger().info(f'   no lo encontré desde la vista {i + 1}')
        raise Fallo(f'la visión no encontró {self.objeto["id"]} ({self.clase}) de forma estable '
                    f'desde ninguna de las {len(v)} vistas (¿está en la mesa?, ¿la cámara lo '
                    've desde ahí?)')

    def encontrar_aqui(self):
        """Espera `n_mediciones` nuevas de la visión con el centro estable; devuelve su mediana,
        o None si no las logra en `tiempo_por_vista`. Solo cuentan las de imágenes tomadas
        después de llegar a la vista."""
        n = int(self.p['n_mediciones'])
        self.mediciones = []
        self.desde = self.get_clock().now()
        fin = time.time() + self.p['tiempo_por_vista']
        while time.time() < fin:
            self.esperar(0.2)
            if len(self.mediciones) >= n:
                ultimas = self.mediciones[-n:]
                if estable(ultimas, self.p['tol_estable']):
                    return promediar(ultimas)
            if not self.mediciones:
                self.get_logger().info(f'espero que la visión encuentre {self.objeto["id"]} '
                                       f'({self.clase})...', throttle_duration_sec=5.0)
        return None

    # ------------------------------------------------------------------ escena de MoveIt
    def unir_objeto(self, m):
        """Caja con las medidas del objeto pegada a gripper_tcp. En el frame del TCP: X local
        = arriba (alto), Y local = entre los dedos (ancho), Z local = eje largo (largo)."""
        adjunto = AttachedCollisionObject()
        adjunto.link_name = TCP
        adjunto.object = caja('objeto_agarrado', (max(m['alto'], 0.005), max(m['ancho'], 0.005),
                                                  max(m['largo'], 0.01)), (0.0, 0.0, 0.0), TCP)
        adjunto.touch_links = TOCAN
        escena = PlanningScene(is_diff=True)
        escena.robot_state.is_diff = True
        escena.robot_state.attached_collision_objects.append(adjunto)
        self.escena.publish(escena)
        self.esperar(0.5)

    # ------------------------------------------------------------------ secuencia
    def alejarse_de_la_mesa(self):
        """Si la pinza quedó baja (por ejemplo, de una corrida que falló a medio camino), sube en
        línea recta y va a `retraer` antes de mirar. MoveIt no sabe dónde están los objetos de la
        mesa (sin OctoMap): un camino libre desde ahí puede barrerlos (medido el 7-nov: el brazo
        lanzó el tenedor al aire yendo a la vista)."""
        r, t = self.transformacion(FRAME, TCP)
        if t[2] >= self.p['retraer'][2] - 0.05:
            return
        self.get_logger().info(f'0. La pinza está baja (z = {t[2]:.2f} m): subo y la recojo '
                               'antes de mirar')
        arriba = t + np.array([0.0, 0.0, self.p['subir']])
        self.linea_recta(self.pose(arriba, r), 'Subir', evitar_choques=False)
        retraer = np.array(self.p['retraer'], float)
        _, abajo = pose_desde_arriba(retraer, [1.0, 0.0, 0.0], retraer[2])
        self.mover(self.pose(retraer, abajo), 'Recoger')

    def correr(self):
        log = self.get_logger()
        self.esperar(2.0)   # TF, cámara y /joint_states
        if self.dedos is None:
            raise Fallo('no llega /joint_states con los dedos: ¿está corriendo arm_sim?')

        self.asegurar_vision()
        self.alejarse_de_la_mesa()
        log.info(f'1-2. Mirar y encontrar: abro la pinza, llevo la cámara frente a la mesa y la '
                 f'visión busca {self.objeto["id"]} (clase {self.clase})')
        self.pinza(ABIERTA)
        m = self.encontrar()
        c = m['centro']
        d = f'a {self.distancia:.2f} m de la pinza, ' if self.distancia is not None else ''
        log.info(f'   ¡Encontrado! {d}en ({c[0]:.3f}, {c[1]:.3f}) de base_footprint, mesa a '
                 f'{m["mesa_z"]:.3f} m; mide {m["largo"] * 1000:.0f} x '
                 f'{m["ancho"] * 1000:.0f} mm y {m["alto"] * 1000:.0f} mm de alto; eje largo '
                 f'{np.degrees(np.arctan2(m["eje"][1], m["eje"][0])):.0f}°')
        if m['ancho'] > self.p['ancho_max']:
            raise Fallo(f'mide {m["ancho"] * 1000:.0f} mm de ancho: no cabe en la pinza '
                        f'(máximo {self.p["ancho_max"] * 1000:.0f} mm)')

        z = altura_de_agarre(m['mesa_z'], m['alto'])
        agarre, giro = pose_desde_arriba(c, m['eje'], z)
        previa = agarre + np.array([0.0, 0.0, self.p['sobre']])
        log.info(f'3. Previa: la pinza va {self.p["sobre"] * 100:.0f} cm sobre el objeto, '
                 'apuntando hacia abajo')
        self.mover(self.pose(previa, giro), 'Previa')

        log.info(f'4. Bajar en línea recta hasta z = {z:.3f} m '
                 f'({(z - m["mesa_z"]) * 1000:.0f} mm sobre la mesa)')
        self.linea_recta(self.pose(agarre, giro), 'Bajar')

        q = objetivo_dedos(m['ancho'], self.p['apriete'])
        log.info(f'5. Cerrar: cada dedo a {q * 1000:+.1f} mm (ancho medido '
                 f'{m["ancho"] * 1000:.0f} mm, apriete {self.p["apriete"] * 1000:.0f} mm)')
        abierta_en = self.pinza(q)
        if not hay_objeto(abierta_en, q):
            self.pinza(ABIERTA)
            self.linea_recta(self.pose(previa, giro), 'Subir sin objeto')
            raise Fallo(f'los dedos cerraron hasta {abierta_en * 1000:.1f} mm sin frenarse: '
                        'no agarré nada (¿el objeto se movió o la detección quedó corrida?)')
        log.info(f'   los dedos se frenaron a {abierta_en * 1000:.1f} mm: hay objeto')

        log.info('6. Unir el objeto a la pinza en la escena de MoveIt')
        self.unir_objeto(m)

        log.info(f'7. Levantar {self.p["subir"] * 100:.0f} cm y retraer el brazo')
        arriba = agarre + np.array([0.0, 0.0, self.p['subir']])
        # Sin revisar choques: al arrancar, el objeto unido está apoyado en la mesa.
        self.linea_recta(self.pose(arriba, giro), 'Levantar', evitar_choques=False)
        r = self.p['retraer']
        retraida = self.pose(np.array(r, float), giro)
        if self.p['mantener_orientacion']:
            # Sin girar la pinza: en línea recta la orientación no cambia nada. Un camino libre
            # de MoveIt llegó a girar la pinza 180° en traslaciones de 30-70 cm (13-nov).
            self.linea_recta(retraida, 'Retraer')
        else:
            self.mover(retraida, 'Retraer')

        self.esperar(1.0)
        final = apertura(*self.dedos)
        if not hay_objeto(final, q):
            raise Fallo(f'al terminar los dedos quedaron a {final * 1000:.1f} mm: el objeto se '
                        'cayó')
        log.info(f'Listo: {self.objeto["id"]} levantado y brazo retraído (dedos a '
                 f'{final * 1000:.1f} mm, el objeto sigue en la pinza)')


def main():
    argumentos = remove_ros_args(sys.argv)[1:]
    if len(argumentos) != 1:
        sys.exit('Uso: ros2 run andesrobot_arm agarre_node <objeto>   (id o clase de '
                 'config/objetos.yaml, por ejemplo tenedor o fork)')
    try:
        objeto = objeto_del_catalogo(argumentos[0])
    except ValueError as error:
        sys.exit(str(error))
    # Un kill o un "timeout" mandan SIGTERM: se trata como Ctrl+C, para que el finally de abajo
    # alcance a apagar la visión (si no, objeto_node queda huérfano y la próxima corrida lo usa).
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init()
    nodo = AgarreNode(objeto)
    try:
        nodo.correr()
    except Fallo as error:
        nodo.get_logger().error(f'No se pudo: {error}')
    except KeyboardInterrupt:
        pass
    finally:
        if nodo.vision is not None:      # la visión la arrancó este nodo: se apaga con él
            nodo.vision.send_signal(signal.SIGINT)
            try:
                nodo.vision.wait(timeout=10)
            except subprocess.TimeoutExpired:
                nodo.vision.kill()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
