"""Guarda una imagen de cada canal de la cámara de la pinza (Orbbec Gemini Plus).

Uso (con ./sim.sh brazo corriendo, en otra terminal: ./sim.sh shell):
  ros2 run andesrobot_arm capturar_camara
  ros2 run andesrobot_arm capturar_camara --ros-args -p carpeta:=/ros2_ws/capturas/prueba1

Espera un mensaje de cada canal y escribe en la carpeta (por defecto /ros2_ws/capturas/<fecha>,
que en el PC es ~/milo_ws/ros2_ws/capturas/<fecha>):
  color.png             imagen a color
  ir.png                infrarrojo (escala de grises)
  profundidad_mm.png    profundidad en milímetros, PNG de 16 bits (0 = sin dato), igual que la
                        cámara real (16UC1); se abre con cualquier programa que lea PNG de 16 bits
  profundidad_vista.png la misma profundidad en colores para mirarla (cerca = claro)
  nube.ply              nube de puntos con color (se abre con MeshLab o CloudCompare)
  resumen.txt           tamaño, frame y rango de cada canal

Los nombres de los topics son parámetros (color, profundidad, ir, nube): con la cámara real y el
driver OrbbecSDK_ROS2 lanzado con camera_name:=gripper_camera son los mismos.
"""
import os
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

# Tipo numpy y canales de cada 'encoding' de sensor_msgs/Image que puede llegar.
ENCODINGS = {
    'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), 'mono8': (np.uint8, 1),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1), '32FC1': (np.float32, 1),
}


def image_to_array(msg):
    """sensor_msgs/Image -> arreglo numpy (alto x ancho [x canales])."""
    dtype, ch = ENCODINGS[msg.encoding]
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    # step = bytes por fila (puede traer relleno al final de cada fila).
    a = a.reshape(msg.height, msg.step // np.dtype(dtype).itemsize)
    a = a[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == 'bgr8':
        a = a[:, :, ::-1]
    return a[:, :, 0] if ch == 1 else a


def depth_to_mm(msg):
    """Profundidad en mm (uint16, 0 = sin dato). Gazebo la da en metros (32FC1), la real en mm."""
    d = image_to_array(msg)
    if msg.encoding == '32FC1':
        valid = np.isfinite(d) & (d > 0)
        out = np.zeros(d.shape, np.uint16)
        out[valid] = np.clip(np.round(d[valid] * 1000.0), 1, 65535).astype(np.uint16)
        return out
    return d.astype(np.uint16)


def depth_preview(mm):
    """Profundidad a colores: cerca = amarillo, lejos = morado, sin dato = negro."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import cm
    valid = mm > 0
    rgb = np.zeros(mm.shape + (3,), np.uint8)
    if valid.any():
        lo, hi = np.percentile(mm[valid], [1, 99])
        t = np.clip((mm.astype(float) - lo) / max(hi - lo, 1.0), 0, 1)
        rgb = (cm.viridis(1.0 - t)[:, :, :3] * 255).astype(np.uint8)
        rgb[~valid] = 0
    return rgb


def cloud_points(msg):
    """PointCloud2 -> (N x 3 puntos en metros, N x 3 colores 0-255 o None), sin los NaN."""
    fields = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, fields[k]:fields[k] + 4].copy().view(np.float32)[:, 0]
                    for k in ('x', 'y', 'z')], axis=1)
    rgb = None
    if 'rgb' in fields:
        # El color viene empaquetado en 4 bytes: B, G, R, (relleno).
        o = fields['rgb']
        rgb = raw[:, [o + 2, o + 1, o]]
    keep = np.isfinite(xyz).all(axis=1)
    return xyz[keep], (rgb[keep] if rgb is not None else None)


def write_ply(path, xyz, rgb):
    """Nube en formato PLY de texto (x y z [r g b] por línea)."""
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\n' % len(xyz))
        f.write('property float x\nproperty float y\nproperty float z\n')
        if rgb is not None:
            f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write('end_header\n')
        for i, p in enumerate(xyz):
            if rgb is not None:
                f.write('%.4f %.4f %.4f %d %d %d\n' % (p[0], p[1], p[2], *rgb[i]))
            else:
                f.write('%.4f %.4f %.4f\n' % tuple(p))


class CapturarCamara(Node):

    def __init__(self):
        super().__init__('capturar_camara')
        ns = '/gripper_camera'
        topics = {
            'color': self.declare_parameter('color', ns + '/color/image_raw').value,
            'profundidad': self.declare_parameter('profundidad', ns + '/depth/image_raw').value,
            'ir': self.declare_parameter('ir', ns + '/ir/image_raw').value,
            'nube': self.declare_parameter('nube', ns + '/depth/points').value,
        }
        carpeta = self.declare_parameter('carpeta', '').value
        self.carpeta = carpeta or os.path.join('/ros2_ws/capturas', time.strftime('%Y%m%d_%H%M%S'))
        self.timeout = self.declare_parameter('timeout', 15.0).value
        self.msgs = {}
        # qos_profile_sensor_data: la misma calidad de servicio que usan las cámaras.
        for canal, topic in topics.items():
            tipo = PointCloud2 if canal == 'nube' else Image
            self.create_subscription(tipo, topic, lambda m, c=canal: self.msgs.setdefault(c, m),
                                     qos_profile_sensor_data)
        self.topics = topics

    def esperar(self):
        """Gira hasta tener un mensaje de cada canal (o el timeout). Devuelve los que faltan."""
        t0 = time.time()
        while len(self.msgs) < len(self.topics) and time.time() - t0 < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return [c for c in self.topics if c not in self.msgs]

    def guardar(self):
        os.makedirs(self.carpeta, exist_ok=True)
        lineas = []
        if 'color' in self.msgs:
            m = self.msgs['color']
            PILImage.fromarray(image_to_array(m)).save(os.path.join(self.carpeta, 'color.png'))
            lineas.append('color        %dx%d %s  frame %s' % (
                m.width, m.height, m.encoding, m.header.frame_id))
        if 'ir' in self.msgs:
            m = self.msgs['ir']
            a = image_to_array(m)
            if a.dtype != np.uint8:
                # IR de 16 bits (cámara real): se escala a 8 bits para el PNG de vista.
                a = (255.0 * a / max(a.max(), 1)).astype(np.uint8)
            PILImage.fromarray(a).save(os.path.join(self.carpeta, 'ir.png'))
            lineas.append('ir           %dx%d %s  frame %s' % (
                m.width, m.height, m.encoding, m.header.frame_id))
        if 'profundidad' in self.msgs:
            m = self.msgs['profundidad']
            mm = depth_to_mm(m)
            PILImage.fromarray(mm).save(os.path.join(self.carpeta, 'profundidad_mm.png'))
            PILImage.fromarray(depth_preview(mm)).save(
                os.path.join(self.carpeta, 'profundidad_vista.png'))
            valid = mm > 0
            rango = 'sin datos'
            if valid.any():
                rango = '%d - %d mm' % (mm[valid].min(), mm[valid].max())
            lineas.append('profundidad  %dx%d %s  frame %s  válidos %.0f %%  rango %s' % (
                m.width, m.height, m.encoding, m.header.frame_id, 100.0 * valid.mean(), rango))
        if 'nube' in self.msgs:
            m = self.msgs['nube']
            xyz, rgb = cloud_points(m)
            write_ply(os.path.join(self.carpeta, 'nube.ply'), xyz, rgb)
            lineas.append('nube         %d puntos válidos  frame %s  %s' % (
                len(xyz), m.header.frame_id, 'con color' if rgb is not None else 'sin color'))
        with open(os.path.join(self.carpeta, 'resumen.txt'), 'w') as f:
            f.write('\n'.join(lineas) + '\n')
        return lineas


def main(args=None):
    rclpy.init(args=args)
    node = CapturarCamara()
    log = node.get_logger()
    try:
        faltan = node.esperar()
        for c in faltan:
            log.warn('No llegó nada por %s (¿está corriendo la simulación con la cámara?)'
                     % node.topics[c])
        if len(faltan) < len(node.topics):
            for linea in node.guardar():
                log.info(linea)
            log.info('Guardado en %s' % node.carpeta)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
