#!/usr/bin/env python3
"""
Prueba manija_node de punta a punta SIN Gazebo ni YOLO: ¿la pinza llega a la manija?

Arma un "mundo falso" con ROS dentro del mismo proceso:
  - un brazo ideal: el TF base_footprint -> gripper_tcp va a cada objetivo de /arm_target_pose
    en 2 s, en línea recta (no hay MoveIt: se prueba la lógica del nodo, no la planificación);
  - una cámara 8 cm sobre el TCP, mirando hacia donde apunta la pinza (su −X local), con color y
    profundidad (una puerta plana en x = 0.80, la hoja de puerta_prueba.sdf);
  - en vez del modelo YOLO, la proyección exacta de los tres keypoints de la manija, con 1 píxel
    de ruido. La profundidad NO tiene la manija: el rayo de cada keypoint pasa al lado y mide la
    hoja de detrás, el peor caso con una palanca delgada (manija.corregir_saliente lo arregla).

Corre manija_node con acercar:=true y al final compara dónde quedó el TCP con la manija
verdadera. Sirve para revisar cambios en manija_node/manija.py antes de probar en Gazebo.

Uso (dentro del contenedor, después de ./sim.sh build; no necesita la simulación):
  python3 /ros2_ws/src/andesrobot_arm/scripts/probar_manija_sin_gazebo.py
Resultado medido el 5-nov-2026: la pinza queda a 0.050 m del punto de agarre (lo pedido) y la
manija estimada a 0.3 mm de la verdadera.
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from andesrobot_arm import manija_node
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix
from andesrobot_arm.manija import matriz_k, orientacion_pinza

PUERTA_X = 0.80                                   # la hoja de la puerta: el plano x = 0.80
EJE = np.array([0.75, -0.30, 1.0])                # la manija sobresale 5 cm de la hoja
AGARRE, PUNTA = EJE + [0.0, -0.07, 0.0], EJE + [0.0, -0.12, 0.0]
K_COLOR = matriz_k(640, 480, math.radians(71.0))  # las dos cámaras de la Gemini Plus simulada
K_PROF = matriz_k(640, 400, math.radians(67.9))
# Frame óptico (Z adelante, X derecha, Y abajo) en el frame del TCP (la pinza apunta con −X):
# Z óptico = −X del TCP, X óptico = +Y del TCP, Y óptico = −Z del TCP.
R_TCP_OPT = np.column_stack([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])
T_TCP_OPT = np.array([0.0, 0.0, 0.08])
DURACION = 2.0                                    # s que tarda el brazo ideal en cada movimiento
rng = np.random.default_rng(1)


class MundoFalso(Node):
    def __init__(self):
        super().__init__('mundo_falso')
        self.tf = TransformBroadcaster(self)
        # Pinza al comienzo: a unos 60 cm de la manija, mirando a +X, corrida a un lado.
        self.p = self.p0 = self.p1 = np.array([0.15, -0.25, 1.05])
        self.r = orientacion_pinza([1.0, 0.0, 0.0])
        self.t0 = None
        self.camara = None
        self.create_subscription(PoseStamped, '/arm_target_pose', self.on_objetivo, 1)
        self.pub_color = self.create_publisher(Image, '/gripper_camera/color/image_raw', 1)
        self.pub_prof = self.create_publisher(Image, '/gripper_camera/depth/image_raw', 1)
        self.pub_k_color = self.create_publisher(CameraInfo, '/gripper_camera/color/camera_info',
                                                 1)
        self.pub_k_prof = self.create_publisher(CameraInfo, '/gripper_camera/depth/camera_info', 1)
        self.create_timer(0.05, self.paso)

    def on_objetivo(self, msg):
        q = msg.pose.orientation
        self.p0 = self.p
        self.p1 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.r = quaternion_to_matrix(q.x, q.y, q.z, q.w)[:3, :3]
        self.t0 = time.time()
        print(f'brazo falso: objetivo {np.round(self.p1, 3)}', flush=True)

    def publicar_tf(self, padre, hijo, posicion, giro):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = padre, hijo
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
            float(v) for v in posicion)
        q = matrix_to_quaternion(giro)
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = (float(v) for v in q)
        self.tf.sendTransform(t)

    def paso(self):
        avance = 1.0 if self.t0 is None else min(1.0, (time.time() - self.t0) / DURACION)
        self.p = self.p0 + avance * (self.p1 - self.p0)
        self.publicar_tf('base_footprint', 'gripper_tcp', self.p, self.r)
        self.publicar_tf('gripper_tcp', 'camara_optica', T_TCP_OPT, R_TCP_OPT)
        giro_cam, pos_cam = self.r @ R_TCP_OPT, self.p + self.r @ T_TCP_OPT
        self.camara = (pos_cam, giro_cam)
        # Profundidad: cada píxel es un rayo (z óptico = 1); se cruza con el plano x = PUERTA_X.
        us, vs = np.meshgrid(np.arange(640), np.arange(400))
        rayos = np.stack([(us - K_PROF[0, 2]) / K_PROF[0, 0], (vs - K_PROF[1, 2]) / K_PROF[1, 1],
                          np.ones(us.shape)], axis=-1) @ giro_cam.T
        with np.errstate(divide='ignore', invalid='ignore'):
            z = (PUERTA_X - pos_cam[0]) / rayos[..., 0]
        z = np.where(np.isfinite(z) & (z >= 0.25) & (z <= 2.5), z, 0.0)   # rango de la Gemini
        ahora = self.get_clock().now().to_msg()
        prof = Image(height=400, width=640, encoding='32FC1', step=640 * 4,
                     data=z.astype(np.float32).tobytes())
        color = Image(height=480, width=640, encoding='rgb8', step=640 * 3,
                      data=bytes(640 * 480 * 3))
        for img in (prof, color):
            img.header.frame_id, img.header.stamp = 'camara_optica', ahora
        self.pub_prof.publish(prof)
        self.pub_color.publish(color)
        for pub, k in ((self.pub_k_color, K_COLOR), (self.pub_k_prof, K_PROF)):
            info = CameraInfo()
            info.k = [float(v) for v in k.ravel()]
            pub.publish(info)


def detector_falso(mundo):
    """Reemplaza ManijaNode.detectar: proyecta la manija verdadera sobre la imagen a color."""
    def detectar(_nodo):
        pos_cam, giro_cam = mundo.camara
        keypoints = []
        for punto in (EJE, AGARRE, PUNTA):
            q = giro_cam.T @ (punto - pos_cam)
            if q[2] <= 0.05:
                return None
            u = K_COLOR[0, 0] * q[0] / q[2] + K_COLOR[0, 2] + rng.normal(0.0, 1.0)
            v = K_COLOR[1, 1] * q[1] / q[2] + K_COLOR[1, 2] + rng.normal(0.0, 1.0)
            if not (0 <= u < 640 and 0 <= v < 480):
                return None
            keypoints.append((u, v))
        return np.array(keypoints)
    return detectar


def main():
    rclpy.init()
    mundo = MundoFalso()
    manija_node.ManijaNode.detectar = detector_falso(mundo)
    nodo = manija_node.ManijaNode()
    nodo.set_parameters([Parameter('acercar', value=True)])
    ejecutor = SingleThreadedExecutor()
    ejecutor.add_node(mundo)
    ejecutor.add_node(nodo)
    limite = time.time() + 40.0
    while time.time() < limite and nodo.acercamiento.fase != 'listo':
        ejecutor.spin_once(timeout_sec=0.05)
    distancia = float(np.linalg.norm(mundo.p - AGARRE))
    pedida = nodo.get_parameter('distancia_final').value
    error = (float(np.linalg.norm(nodo.acercamiento.agarre - AGARRE))
             if nodo.acercamiento.agarre is not None else float('nan'))
    print(f'fase final: {nodo.acercamiento.fase}; la pinza quedó a {distancia:.3f} m del punto de '
          f'agarre (pedido {pedida:.3f} m); error de la manija estimada: {error * 1000:.1f} mm')
    bien = nodo.acercamiento.fase == 'listo' and abs(distancia - pedida) < 0.01
    print('BIEN' if bien else 'MAL')
    rclpy.try_shutdown()
    sys.exit(0 if bien else 1)


if __name__ == '__main__':
    main()
