"""Nodo manija_node: detecta la manija de la puerta en vivo y publica su pose 3D.

SIN PROBAR en la simulación (la geometría que usa, andesrobot_arm/manija.py, sí tiene pruebas).

Cada 1/hz segundos toma la última imagen a color, le pasa el modelo YOLO de la manija
(scripts/entrenar_manija.py: caja + 3 keypoints eje / agarre / punta) y convierte los tres
keypoints a puntos 3D con la cámara de profundidad (rayo del píxel a color, ver manija.py). Si la
profundidad falla sobre la manija brillante, usa el plano de la puerta ajustado con los píxeles
de alrededor. Descarta las detecciones sin sentido (palanca de largo raro, fuera del plano de la
puerta) y publica, en el frame `frame_destino` (base_footprint):

  /manija/agarre       PointStamped   el punto donde se empuja la palanca
  /manija/pose_previa  PoseStamped    dónde poner la pinza antes de acercarse (mirando a la puerta)
  /manija/marcadores   MarkerArray    eje, agarre, punta y la normal de la puerta, para RViz

No mueve el brazo salvo con enviar_previa:=true, que manda la pose previa a /arm_target_pose
(como arm_marker_node).

Uso (con la simulación de la puerta corriendo, dentro del contenedor):
  ros2 run andesrobot_arm manija_node --ros-args -p use_sim_time:=true [-p enviar_previa:=true]
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix
from andesrobot_arm.manija import (
    normal_de_la_puerta, pose_previa, punto_en_plano, punto_en_rayo, puntos_anillo, rayo,
    validar_manija)

NOMBRES = ('eje', 'agarre', 'punta')      # el orden de los keypoints del modelo
COLORES = ((0.2, 0.4, 1.0), (0.1, 0.8, 0.3), (1.0, 0.2, 0.2))


def profundidad_en_metros(msg):
    """Imagen de profundidad en metros (0 = sin dato): la simulada es 32FC1 en metros y la real
    16UC1 en milímetros."""
    d = image_to_array(msg).astype(np.float64)
    if msg.encoding != '32FC1':
        d = d / 1000.0
    d[~np.isfinite(d)] = 0.0
    return d


class ManijaNode(Node):
    def __init__(self):
        super().__init__('manija_node')
        self.declare_parameter(
            'modelo', '/ros2_ws/modelos/entrenamientos/manija_pose/weights/best.pt')
        self.declare_parameter('hz', 2.0)
        self.declare_parameter('confianza', 0.5)
        self.declare_parameter('frame_destino', 'base_footprint')
        self.declare_parameter('distancia_previa', 0.30)
        self.declare_parameter('saliente', 0.05)
        self.declare_parameter('enviar_previa', False)
        self.color = self.profundidad = self.k_color = self.k_prof = None
        self.modelo = None
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/gripper_camera/color/image_raw', self.on_color, s)
        self.create_subscription(Image, '/gripper_camera/depth/image_raw', self.on_prof, s)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info', self.on_kc, s)
        self.create_subscription(CameraInfo, '/gripper_camera/depth/camera_info', self.on_kp, s)
        self.pub_agarre = self.create_publisher(PointStamped, '/manija/agarre', 1)
        self.pub_previa = self.create_publisher(PoseStamped, '/manija/pose_previa', 1)
        self.pub_marcas = self.create_publisher(MarkerArray, '/manija/marcadores', 1)
        self.pub_objetivo = self.create_publisher(PoseStamped, '/arm_target_pose', 1)
        self.create_timer(1.0 / self.get_parameter('hz').value, self.ciclo)
        self.get_logger().info('manija_node listo: esperando imágenes y el modelo (sin probar)')

    def on_color(self, msg):
        self.color = msg

    def on_prof(self, msg):
        self.profundidad = msg

    def on_kc(self, msg):
        self.k_color = np.array(msg.k).reshape(3, 3)

    def on_kp(self, msg):
        self.k_prof = np.array(msg.k).reshape(3, 3)

    def cargar_modelo(self):
        """El modelo se carga en el primer ciclo (tarda y usa la GPU)."""
        try:
            from ultralytics import YOLO
            self.modelo = YOLO(self.get_parameter('modelo').value)
        except Exception as error:   # sin ultralytics, o sin los pesos entrenados
            self.get_logger().error(f'No pude cargar el modelo: {error}')
            self.modelo = False

    def transformacion(self, destino, origen):
        """(R, t) que pasan puntos del frame `origen` al `destino`, o None si TF no la tiene."""
        try:
            tf = self.tf_buffer.lookup_transform(destino, origen, Time(),
                                                 timeout=Duration(seconds=0.2))
        except TransformException as error:
            self.get_logger().warn(f'sin TF {origen} -> {destino}: {error}',
                                   throttle_duration_sec=5.0)
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        return quaternion_to_matrix(q.x, q.y, q.z, q.w)[:3, :3], np.array([p.x, p.y, p.z])

    def detectar(self):
        """Keypoints (3x2, en píxeles de la imagen a color) de la mejor detección, o None."""
        if self.modelo is None:
            self.cargar_modelo()
        if not self.modelo:
            return None
        imagen = image_to_array(self.color)
        resultado = self.modelo.predict(imagen, conf=self.get_parameter('confianza').value,
                                        verbose=False)[0]
        if resultado.boxes is None or len(resultado.boxes) == 0 or resultado.keypoints is None:
            return None
        mejor = int(resultado.boxes.conf.argmax())
        puntos = resultado.keypoints.xy[mejor].cpu().numpy()
        return puntos if puntos.shape == (3, 2) else None

    def ciclo(self):
        if any(x is None for x in (self.color, self.profundidad, self.k_color, self.k_prof)):
            return
        keypoints = self.detectar()
        if keypoints is None:
            self.get_logger().info('no veo la manija', throttle_duration_sec=5.0)
            return
        frame_color, frame_prof = self.color.header.frame_id, self.profundidad.header.frame_id
        destino = self.get_parameter('frame_destino').value
        c2p = self.transformacion(frame_prof, frame_color)
        p2d = self.transformacion(destino, frame_prof)
        if c2p is None or p2d is None:
            return
        r, t = c2p
        profundidad = profundidad_en_metros(self.profundidad)
        rayos = [rayo(u, v, self.k_color) for u, v in keypoints]
        # Plano de la puerta: píxeles de profundidad alrededor del eje de la manija. Se ubica el
        # píxel del eje en la imagen de profundidad sin contar la distancia entre cámaras (es de
        # pocos cm: no importa para ajustar un plano con un anillo ancho).
        d_eje = r @ rayos[0]
        centro = (self.k_prof[0, 0] * d_eje[0] / d_eje[2] + self.k_prof[0, 2],
                  self.k_prof[1, 1] * d_eje[1] / d_eje[2] + self.k_prof[1, 2])
        plano = normal_de_la_puerta(puntos_anillo(profundidad, self.k_prof, centro, 90, 30))
        puntos = []
        for d_c in rayos:
            p = punto_en_rayo(profundidad, self.k_prof, d_c, r, t)
            if p is None and plano is not None:
                p = punto_en_plano(t, r @ d_c, plano[0], plano[1],
                                   self.get_parameter('saliente').value)
            if p is None:
                self.get_logger().warn('no pude ubicar un keypoint en 3D',
                                       throttle_duration_sec=5.0)
                return
            puntos.append(p)
        if plano is None:
            self.get_logger().warn('sin plano de la puerta (muy pocos píxeles con profundidad)',
                                   throttle_duration_sec=5.0)
            return
        rd, td = p2d
        eje, agarre, punta = (rd @ p + td for p in puntos)
        normal = rd @ plano[0]
        problemas = validar_manija(eje, agarre, punta, normal)
        if problemas:
            self.get_logger().warn('detección descartada: ' + '; '.join(problemas),
                                   throttle_duration_sec=5.0)
            return
        self.publicar(destino, eje, agarre, punta, normal)

    def publicar(self, frame, eje, agarre, punta, normal):
        ahora = self.get_clock().now().to_msg()
        punto = PointStamped()
        punto.header.frame_id, punto.header.stamp = frame, ahora
        punto.point.x, punto.point.y, punto.point.z = (float(v) for v in agarre)
        self.pub_agarre.publish(punto)
        posicion, giro = pose_previa(agarre, normal, self.get_parameter('distancia_previa').value)
        previa = PoseStamped()
        previa.header.frame_id, previa.header.stamp = frame, ahora
        previa.pose.position.x, previa.pose.position.y, previa.pose.position.z = (
            float(v) for v in posicion)
        q = matrix_to_quaternion(giro)
        previa.pose.orientation.x, previa.pose.orientation.y = float(q[0]), float(q[1])
        previa.pose.orientation.z, previa.pose.orientation.w = float(q[2]), float(q[3])
        self.pub_previa.publish(previa)
        if self.get_parameter('enviar_previa').value:
            self.pub_objetivo.publish(previa)
        marcas = MarkerArray()
        for i, (p, color) in enumerate(zip((eje, agarre, punta), COLORES)):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, ahora
            m.ns, m.id, m.type = 'manija', i, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
            m.lifetime = Duration(seconds=2.0).to_msg()
            marcas.markers.append(m)
        flecha = Marker()
        flecha.header.frame_id, flecha.header.stamp = frame, ahora
        flecha.ns, flecha.id, flecha.type = 'manija', 3, Marker.ARROW
        for fin in (agarre, agarre + 0.15 * normal):
            flecha.points.append(Point(x=float(fin[0]), y=float(fin[1]), z=float(fin[2])))
        flecha.scale.x, flecha.scale.y, flecha.scale.z = 0.01, 0.02, 0.03
        flecha.color.r, flecha.color.g, flecha.color.b, flecha.color.a = 1.0, 1.0, 0.0, 1.0
        flecha.lifetime = Duration(seconds=2.0).to_msg()
        marcas.markers.append(flecha)
        self.pub_marcas.publish(marcas)


def main():
    rclpy.init()
    nodo = ManijaNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
