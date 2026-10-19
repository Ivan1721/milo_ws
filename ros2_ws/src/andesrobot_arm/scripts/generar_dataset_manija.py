"""Genera imágenes etiquetadas de la manija de la puerta de prueba, con la cámara de la pinza.

Para qué: los detectores preentrenados (YOLOE, YOLOv8 de Open Images) encuentran la región de la
manija en la imagen simulada, pero no la reconocen (19-10-2026: clase equivocada, confianza
< 0.06). Hay que afinar un modelo, y para eso hacen falta imágenes etiquetadas. En la simulación
no hace falta anotar a mano: se sabe exactamente dónde está la manija (la geometría del SDF + el
ángulo de la bisagra y de la manija en /puerta/joint_states) y dónde está la cámara (TF),
así que se proyecta y se escribe la etiqueta sola.

Cómo: mueve la pinza (con MoveIt, por /arm_target_pose) a vistas al azar alrededor de la manija,
espera a que llegue, saca una foto a color y escribe tres etiquetas en formato YOLO:
  labels_det/   caja                                    (para YOLOv8/11 detección)
  labels_seg/   polígono de la manija                   (para YOLOv8/11-seg, máscaras)
  labels_pose/  caja + 3 keypoints con su visibilidad   (para YOLOv8/11-pose)
Keypoints (en el frame de la manija, ver worlds/puerta_prueba.sdf):
  0 eje     centro de la roseta, cara delantera (donde gira la palanca)
  1 agarre  a 7 cm del eje sobre la palanca (donde cerrar la pinza)
  2 punta   extremo de la palanca (12 cm)
Con eje -> punta sale la dirección de la palanca; con eje y agarre, dónde y hacia dónde bajarla.

Uso (con la simulación de la puerta corriendo, en otra terminal):
  ./sim.sh brazo   ->  en realidad: ros2 launch andesrobot_arm arm_sim.launch.py puerta:=true
  ./sim.sh shell
  python3 /ros2_ws/src/andesrobot_arm/scripts/generar_dataset_manija.py --n 200
Deja todo en ros2_ws/datasets/manija_sim/ (no va a git) con un data_*.yaml por tarea.

Supuestos: la puerta está en el origen del mundo (como la pone arm_sim) y la base de Milo no se
mueve, así que el frame "odom" coincide con el mundo de Gazebo. Una vista que MoveIt rechaza o
que no llega en 25 s se salta.
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix

# ---------------- Geometría de la puerta (copiada de worlds/puerta_prueba.sdf) ----------------
BISAGRA = np.array([0.80, 0.45, 0.0])          # origen del link "hoja" (en la bisagra), mundo
MANIJA_EN_HOJA = np.array([-0.05, -0.83, 1.0])  # origen del link "manija" respecto de la hoja
# Keypoints en el frame de la manija.
KEYPOINTS = np.array([[0.0, 0.0, 0.0],      # 0 eje
                      [-0.005, 0.07, 0.0],  # 1 agarre
                      [-0.005, 0.12, 0.0]])  # 2 punta
NOMBRES_KP = ['eje', 'agarre', 'punta']


def esquinas_manija():
    """Puntos del contorno 3D de la manija (palanca + roseta), en el frame de la manija."""
    pts = []
    # Palanca: caja de 0.02 x 0.12 x 0.02 centrada en (-0.005, 0.06, 0).
    for dx in (-0.01, 0.01):
        for dy in (-0.06, 0.06):
            for dz in (-0.01, 0.01):
                pts.append([-0.005 + dx, 0.06 + dy, dz])
    # Roseta: cilindro de radio 0.025 y largo 0.03 a lo largo de X, centrado en x = 0.015.
    for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        for x in (0.0, 0.03):
            pts.append([x, 0.025 * np.cos(a), 0.025 * np.sin(a)])
    return np.array(pts)


def rot(eje, ang):
    c, s = math.cos(ang), math.sin(ang)
    if eje == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose_manija(ang_bisagra, ang_manija):
    """Transformación 4x4 mundo <- manija para los ángulos actuales de la bisagra y la manija."""
    t_hoja = np.eye(4)
    t_hoja[:3, :3] = rot('z', ang_bisagra)
    t_hoja[:3, 3] = BISAGRA
    t_man = np.eye(4)
    t_man[:3, :3] = rot('x', ang_manija)
    t_man[:3, 3] = MANIJA_EN_HOJA
    return t_hoja @ t_man


def tf_a_matriz(tf):
    m = np.eye(4)
    q = tf.transform.rotation
    m[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    t = tf.transform.translation
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def mirar_hacia(ojo, objetivo, giro):
    """Pose 4x4 de un frame óptico (Z adelante, X derecha, Y abajo) en 'ojo' mirando 'objetivo'.

    giro = rotación [rad] alrededor del eje óptico, para variar la orientación de la imagen.
    """
    z = objetivo - ojo
    z /= np.linalg.norm(z)
    arriba = np.array([0.0, 0.0, 1.0])
    x = np.cross(-arriba, z)          # derecha de la imagen (Y óptico apunta hacia abajo)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, -1.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r = np.column_stack([x, y, z]) @ rot('z', giro)
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = ojo
    return m


class Generador(Node):

    def __init__(self, args):
        super().__init__('generar_dataset_manija')
        self.args = args
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self, spin_thread=True)
        self.img = self.info = None
        self.ang = {}
        self.create_subscription(Image, '/gripper_camera/color/image_raw',
                                 lambda m: setattr(self, 'img', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/puerta/joint_states',
                                 lambda m: self.ang.update(zip(m.name, m.position)), 10)
        self.pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)
        self.rng = np.random.default_rng(args.semilla)

    def esperar(self, seg, cond=lambda: False):
        t0 = time.time()
        while time.time() - t0 < seg and not cond():
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def lookup(self, padre, hijo):
        try:
            return tf_a_matriz(self.tf.lookup_transform(padre, hijo, Time()))
        except TransformException:
            return None

    def vista_al_azar(self, t_manija):
        """Pose del TCP para que la cámara mire la manija desde un punto al azar."""
        centro = t_manija[:3, :3] @ KEYPOINTS[1] + t_manija[:3, 3]   # el punto de agarre
        d = self.rng.uniform(self.args.dmin, self.args.dmax)
        # Dirección desde la manija hacia la cámara: del lado de Milo (-X), ±40° a los lados,
        # -15° a +45° en altura.
        az = math.radians(180 + self.rng.uniform(-40, 40))
        el = math.radians(self.rng.uniform(-15, 45))
        ojo = centro + d * np.array([math.cos(el) * math.cos(az),
                                     math.cos(el) * math.sin(az), math.sin(el)])
        apunte = centro + self.rng.normal(0, 0.03, 3)   # no siempre al centro de la imagen
        return mirar_hacia(ojo, apunte, self.rng.uniform(-0.3, 0.3))

    def proyectar(self, pts_mundo, t_mundo_cam):
        """Puntos del mundo -> píxeles (u, v) y profundidad, con la matriz K de camera_info."""
        k = np.array(self.info.k).reshape(3, 3)
        cam = np.linalg.inv(t_mundo_cam)
        p = (cam[:3, :3] @ pts_mundo.T).T + cam[:3, 3]
        uv = (k @ p.T).T
        return uv[:, :2] / uv[:, 2:3], p[:, 2]

    def correr(self):
        a = self.args
        carpetas = {t: os.path.join(a.salida, t) for t in
                    ('images', 'labels_det', 'labels_seg', 'labels_pose')}
        for c in carpetas.values():
            os.makedirs(c, exist_ok=True)
        log = self.get_logger()
        if not self.esperar(20, lambda: self.img is not None and self.info is not None
                            and 'bisagra' in self.ang):
            log.error('No llegan la cámara o /puerta/joint_states: ¿arm_sim con puerta:=true?')
            return
        # Del TCP a la cámara (fijo): pasar de "dónde quiero la cámara" a "dónde va el TCP".
        t_tcp_cam = None
        while t_tcp_cam is None:
            t_tcp_cam = self.lookup('gripper_tcp', 'gripper_camera_color_optical_frame')
            self.esperar(0.2)
        hechas = intentos = 0
        while hechas < a.n and intentos < a.n * 3:
            intentos += 1
            t_man = pose_manija(self.ang['bisagra'], self.ang['eje_manija'])
            t_cam_deseada = self.vista_al_azar(t_man)
            t_tcp = t_cam_deseada @ np.linalg.inv(t_tcp_cam)
            # La meta va en base_footprint: la escena de MoveIt no conoce "odom".
            t_bf_odom = self.lookup('base_footprint', 'odom')
            if t_bf_odom is None:
                continue
            t_meta = t_bf_odom @ t_tcp
            msg = PoseStamped()
            msg.header.frame_id = 'base_footprint'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t_meta[:3, 3]
            q = matrix_to_quaternion(t_meta[:3, :3])
            msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
            msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
            self.pub.publish(msg)

            def llego():
                actual = self.lookup('odom', 'gripper_tcp')
                return actual is not None and np.linalg.norm(actual[:3, 3] - t_tcp[:3, 3]) < 0.01
            if not self.esperar(25, llego):
                log.info(f'vista {intentos}: no se alcanzó (MoveIt la rechazó o no llegó)')
                continue
            self.esperar(1.0)   # que se asiente y llegue una imagen nueva
            self.img = None
            self.esperar(2, lambda: self.img is not None)
            t_cam = self.lookup('odom', 'gripper_camera_color_optical_frame')
            if self.img is None or t_cam is None:
                continue
            t_man = pose_manija(self.ang['bisagra'], self.ang['eje_manija'])
            w, h = self.info.width, self.info.height
            uv_cont, z_cont = self.proyectar(
                (t_man[:3, :3] @ esquinas_manija().T).T + t_man[:3, 3], t_cam)
            uv_kp, z_kp = self.proyectar((t_man[:3, :3] @ KEYPOINTS.T).T + t_man[:3, 3], t_cam)
            if (z_cont <= 0.05).any():
                continue
            dentro = (uv_cont[:, 0] >= 0) & (uv_cont[:, 0] < w) & (uv_cont[:, 1] >= 0) & \
                (uv_cont[:, 1] < h)
            if dentro.mean() < 0.5:   # la manija casi fuera de la imagen: no sirve
                continue
            # Caja y polígono (envolvente convexa del contorno proyectado), dentro de la imagen.
            uv_c = np.clip(uv_cont, [0, 0], [w - 1, h - 1])
            x0, y0 = uv_c.min(0)
            x1, y1 = uv_c.max(0)
            casco = envolvente(uv_c)
            nombre = f'manija_{hechas:05d}'
            PILImage.fromarray(image_to_array(self.img)).save(
                os.path.join(carpetas['images'], nombre + '.png'))
            caja = f'{(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} {(x1 - x0) / w:.6f} ' \
                f'{(y1 - y0) / h:.6f}'
            with open(os.path.join(carpetas['labels_det'], nombre + '.txt'), 'w') as f:
                f.write(f'0 {caja}\n')
            with open(os.path.join(carpetas['labels_seg'], nombre + '.txt'), 'w') as f:
                f.write('0 ' + ' '.join(f'{u / w:.6f} {v / h:.6f}' for u, v in casco) + '\n')
            kps = []
            for (u, v), z in zip(uv_kp, z_kp):
                vis = 2 if (0 <= u < w and 0 <= v < h and z > 0) else 0
                kps.append(f'{u / w:.6f} {v / h:.6f} {vis}' if vis else '0 0 0')
            with open(os.path.join(carpetas['labels_pose'], nombre + '.txt'), 'w') as f:
                f.write(f'0 {caja} ' + ' '.join(kps) + '\n')
            hechas += 1
            log.info(f'{nombre}: distancia {np.linalg.norm(t_cam[:3, 3] - t_man[:3, 3]):.2f} m')
        escribir_yaml(a.salida)
        log.info(f'Listo: {hechas} imágenes en {a.salida} ({intentos} vistas intentadas).')


def envolvente(p):
    """Envolvente convexa 2D (monotone chain), en orden: el polígono de la máscara."""
    p = sorted(map(tuple, p))
    if len(p) < 3:
        return p

    def cruz(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    inf, sup = [], []
    for q in p:
        while len(inf) >= 2 and cruz(inf[-2], inf[-1], q) <= 0:
            inf.pop()
        inf.append(q)
    for q in reversed(p):
        while len(sup) >= 2 and cruz(sup[-2], sup[-1], q) <= 0:
            sup.pop()
        sup.append(q)
    return inf[:-1] + sup[:-1]


def escribir_yaml(salida):
    """Un data_*.yaml por tarea, como los espera Ultralytics (las etiquetas en labels/)."""
    for tarea in ('det', 'seg', 'pose'):
        extra = ''
        if tarea == 'pose':
            # 3 keypoints con visibilidad; no hay simetría izquierda-derecha que invertir.
            extra = 'kpt_shape: [3, 3]\nflip_idx: [0, 1, 2]\n'
        with open(os.path.join(salida, f'data_{tarea}.yaml'), 'w') as f:
            f.write(f'# Generado por generar_dataset_manija.py. Antes de entrenar, copiar o\n'
                    f'# enlazar labels_{tarea}/ como labels/ (Ultralytics las busca ahí).\n'
                    f'path: {salida}\ntrain: images\nval: images\n{extra}names:\n  0: manija\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--n', type=int, default=50, help='imágenes a generar')
    ap.add_argument('--salida', default='/ros2_ws/datasets/manija_sim')
    ap.add_argument('--dmin', type=float, default=0.35, help='distancia mínima a la manija [m]')
    ap.add_argument('--dmax', type=float, default=0.80, help='distancia máxima a la manija [m]')
    ap.add_argument('--semilla', type=int, default=0)
    args = ap.parse_args()
    rclpy.init()
    nodo = Generador(args)
    try:
        nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
