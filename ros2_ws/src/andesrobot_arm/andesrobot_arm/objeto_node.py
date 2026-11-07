"""Nodo objeto_node: detecta un objeto (cup, bowl, bottle...) en vivo y lleva la pinza frente a él.

Es la versión para objetos de manija_node. Usa el modelo YOLO de detección de objetos del
reglamento (por defecto /ros2_ws/modelos/objetos/best.pt, 27 clases: bag, book, bottle, bowl,
box, cup...; la lista sale en el log al cargarlo). El modelo solo da una caja por objeto, así que
el punto 3D se saca de la profundidad (ver objetos.py):

  1. Cada 1/hz s corre el modelo sobre la imagen a color y elige la caja de la clase pedida
     (`clase`) con más confianza.
  2. Los píxeles de la parte central de la caja son rayos que se siguen hasta la superficie de la
     imagen de profundidad; el punto del objeto es la mediana (un punto de la cara que mira a la
     cámara).
  3. Publica, en `frame_destino` (base_footprint):
       /objeto/punto        PointStamped  el punto del objeto
       /objeto/pose_previa  PoseStamped   dónde poner la pinza antes de acercarse (mirándolo)
       /objeto/marcadores   MarkerArray   el punto y la dirección de llegada, para RViz
       /objeto/distancia    Float32       distancia del efector final (gripper_tcp) al objeto [m]
       /objeto/punto_tcp    PointStamped  el objeto visto desde gripper_tcp (x negativo = lo que
                                          falta avanzar)
     y anota en el log la clase, la confianza y cuánto le falta a la pinza.

Mover el brazo (por /arm_target_pose; lo planifica MoveIt):
  acercar:=true   lleva la pinza frente al objeto en dos tramos, igual que con la manija
                  (manija.Acercamiento): con 3 detecciones estables va a `distancia_previa`
                  (30 cm); desde ahí vuelve a detectar y va a `distancia_final` (10 cm). La
                  pinza llega desde el lado de Milo, inclinada `inclinacion` grados hacia abajo.
Conviene lanzar la simulación con octomap:=false: con el OctoMap, el objeto que ve la cámara es
un obstáculo y MoveIt rechaza una pose a pocos centímetros de él.

Uso (dentro del contenedor):
  python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py      # una vez (internet)
  ros2 launch andesrobot_arm arm_sim.launch.py objetos:=true octomap:=false     # terminal 1
  ros2 run andesrobot_arm objeto_node --ros-args -p use_sim_time:=true -p clase:=cup \
      -p acercar:=true
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix
from andesrobot_arm.manija import (
    Acercamiento, distancia_desde_tcp, orientacion_pinza, punto_en_rayo, rayo)
from andesrobot_arm.manija_node import profundidad_en_metros
from andesrobot_arm.objetos import (
    direccion_de_llegada, elegir_deteccion, pixeles_centrales, punto_del_objeto, validar_objeto)

TCP = 'gripper_tcp'      # el efector final: entre las yemas de los dedos


class ObjetoNode(Node):
    def __init__(self):
        super().__init__('objeto_node')
        self.declare_parameter('modelo', '/ros2_ws/modelos/objetos/best.pt')
        self.declare_parameter('clase', 'cup')        # vacío = la detección más segura
        self.declare_parameter('hz', 2.0)
        self.declare_parameter('confianza', 0.4)
        self.declare_parameter('frame_destino', 'base_footprint')
        self.declare_parameter('distancia_previa', 0.30)
        self.declare_parameter('distancia_final', 0.10)
        self.declare_parameter('inclinacion', 20.0)   # grados hacia abajo al llegar
        self.declare_parameter('acercar', False)
        self.declare_parameter('tiempo_max', 30.0)
        self.declare_parameter('tol_estable', 0.02)
        self.color = self.profundidad = self.k_color = self.k_prof = None
        self.modelo = None
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/gripper_camera/color/image_raw', self.on_color, s)
        self.create_subscription(Image, '/gripper_camera/depth/image_raw', self.on_prof, s)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info', self.on_kc, s)
        self.create_subscription(CameraInfo, '/gripper_camera/depth/camera_info', self.on_kp, s)
        self.pub_punto = self.create_publisher(PointStamped, '/objeto/punto', 1)
        self.pub_previa = self.create_publisher(PoseStamped, '/objeto/pose_previa', 1)
        self.pub_marcas = self.create_publisher(MarkerArray, '/objeto/marcadores', 1)
        self.pub_objetivo = self.create_publisher(PoseStamped, '/arm_target_pose', 1)
        self.pub_distancia = self.create_publisher(Float32, '/objeto/distancia', 1)
        self.pub_punto_tcp = self.create_publisher(PointStamped, '/objeto/punto_tcp', 1)
        distancias = (self.get_parameter('distancia_previa').value,
                      self.get_parameter('distancia_final').value)
        self.acercamiento = Acercamiento(distancias, tol_llegada=0.01,
                                         tol_estable=self.get_parameter('tol_estable').value)
        self.enviado = None
        self.create_timer(1.0 / self.get_parameter('hz').value, self.ciclo)
        clase = self.get_parameter('clase').value or 'cualquier clase'
        if self.get_parameter('acercar').value:
            self.get_logger().info(f'objeto_node listo, busco {clase}; acercar:=true: cuando lo '
                                   'vea estable llevo la pinza frente a él')
        else:
            self.get_logger().info(f'objeto_node listo, busco {clase}; solo mido '
                                   '(acercar:=false): para mover el brazo, -p acercar:=true')

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
        except Exception as error:   # sin ultralytics, o sin el archivo de pesos
            self.get_logger().error(f'No pude cargar el modelo: {error}')
            self.modelo = False
            return
        nombres = self.modelo.names
        self.get_logger().info(f'Modelo cargado: {len(nombres)} clases: '
                               + ', '.join(nombres[i] for i in sorted(nombres)))
        clase = self.get_parameter('clase').value
        if clase and clase not in nombres.values():
            self.get_logger().error(f'El modelo no tiene la clase "{clase}"')

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

    def pose_tcp(self, frame):
        """(posición, rotación) de gripper_tcp en `frame`, o None."""
        tf = self.transformacion(frame, TCP)
        return None if tf is None else (tf[1], tf[0])

    def vigilar_acercamiento(self, frame):
        """Mientras el brazo va a un objetivo: ¿llegó?, ¿se pasó el tiempo? Devuelve True si
        hay que saltarse la detección en este ciclo (el brazo se está moviendo)."""
        a = self.acercamiento
        if a.fase == 'listo':
            return True
        if a.fase != 'yendo':
            return False
        tcp = self.pose_tcp(frame)
        if tcp is not None and a.tcp(tcp[0]):
            d = distancia_desde_tcp(tcp[0], tcp[1], a.agarre)
            if a.fase == 'listo':
                self.get_logger().info(
                    f'Listo: la pinza quedó a {d["total"]:.3f} m del objeto '
                    f'({d["adelante"]:.3f} m al frente; medido con la última detección)')
            else:
                self.get_logger().info(f'Llegué a la pose previa ({d["total"]:.3f} m del '
                                       'objeto): lo vuelvo a detectar desde aquí')
            return True
        espera = (self.get_clock().now() - self.enviado).nanoseconds * 1e-9
        if espera > self.get_parameter('tiempo_max').value:
            self.get_logger().warn(f'El brazo no llegó en {espera:.0f} s (¿MoveIt no encontró '
                                   'camino? mirar su log): vuelvo a detectar el objeto')
            a.reintentar()
        return True

    def detectar(self):
        """(caja, confianza, clase) de la mejor detección de la clase pedida, o None."""
        if self.modelo is None:
            self.cargar_modelo()
        if not self.modelo:
            return None
        # RGB -> BGR: Ultralytics supone que un arreglo viene en BGR (ver manija_node).
        imagen = image_to_array(self.color)[:, :, ::-1]
        r = self.modelo.predict(imagen, conf=self.get_parameter('confianza').value,
                                verbose=False)[0]
        if r.boxes is None or len(r.boxes) == 0:
            return None
        vistas = sorted({self.modelo.names[int(c)] for c in r.boxes.cls})
        self.get_logger().info('veo: ' + ', '.join(vistas), throttle_duration_sec=5.0)
        return elegir_deteccion(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(),
                                r.boxes.cls.cpu().numpy(), self.modelo.names,
                                self.get_parameter('clase').value)

    def ciclo(self):
        if any(x is None for x in (self.color, self.profundidad, self.k_color, self.k_prof)):
            return
        destino = self.get_parameter('frame_destino').value
        if self.get_parameter('acercar').value and self.vigilar_acercamiento(destino):
            return
        deteccion = self.detectar()
        if deteccion is None:
            clase = self.get_parameter('clase').value or 'ningún objeto'
            self.get_logger().info(f'no veo {clase}', throttle_duration_sec=5.0)
            return
        caja, confianza, nombre = deteccion
        frame_color, frame_prof = self.color.header.frame_id, self.profundidad.header.frame_id
        c2p = self.transformacion(frame_prof, frame_color)
        p2d = self.transformacion(destino, frame_prof)
        if c2p is None or p2d is None:
            return
        r, t = c2p
        profundidad = profundidad_en_metros(self.profundidad)
        puntos = [punto_en_rayo(profundidad, self.k_prof, rayo(u, v, self.k_color), r, t)
                  for u, v in pixeles_centrales(caja)]
        punto = punto_del_objeto(puntos)
        if punto is None:
            self.get_logger().warn(f'{nombre}: casi sin profundidad dentro de la caja (¿a menos '
                                   'de 0.25 m o muy lejos?)', throttle_duration_sec=5.0)
            return
        rd, td = p2d
        punto = rd @ punto + td
        problemas = validar_objeto(punto)
        if problemas:
            self.get_logger().warn(f'{nombre} descartado: ' + '; '.join(problemas),
                                   throttle_duration_sec=5.0)
            return
        camara = td    # la cámara de profundidad en frame_destino: de ahí se mira al objeto
        llegada = direccion_de_llegada(camara, punto,
                                       np.radians(self.get_parameter('inclinacion').value))
        self.publicar(destino, punto, llegada)
        self.medir_desde_tcp(destino, punto, nombre, confianza)
        if self.get_parameter('acercar').value:
            self.acercar(destino, punto, llegada)

    def medir_desde_tcp(self, frame, punto, nombre, confianza):
        """Distancia del efector final (gripper_tcp) al objeto: se publica y se anota."""
        tcp = self.pose_tcp(frame)
        if tcp is None:
            return
        d = distancia_desde_tcp(tcp[0], tcp[1], punto)
        self.pub_distancia.publish(Float32(data=d['total']))
        p = PointStamped()
        p.header.frame_id, p.header.stamp = TCP, self.get_clock().now().to_msg()
        p.point.x, p.point.y, p.point.z = -d['adelante'], d['lateral'], d['vertical']
        self.pub_punto_tcp.publish(p)
        self.get_logger().info(
            f'{nombre} ({confianza:.2f}) en ({punto[0]:.3f}, {punto[1]:.3f}, {punto[2]:.3f}), a '
            f'{d["total"]:.3f} m del TCP: avanzar {d["adelante"]:.3f} m, lateral '
            f'{d["lateral"]:+.3f} m, vertical {d["vertical"]:+.3f} m', throttle_duration_sec=1.0)

    def acercar(self, frame, punto, llegada):
        """Le pasa la detección a la secuencia de acercamiento y, si toca, manda el objetivo."""
        a = self.acercamiento
        tramo = a.tramo
        objetivo = a.deteccion(punto, llegada)
        if objetivo is None:
            if a.fase == 'buscar' and a.dispersion is not None and a.dispersion > a.tol_estable:
                self.get_logger().info(
                    f'Espero detecciones estables: las últimas {a.n_estables} saltan '
                    f'{a.dispersion * 100:.1f} cm (tolerancia {a.tol_estable * 100:.0f} cm, '
                    'parámetro tol_estable)', throttle_duration_sec=2.0)
            return
        posicion, giro = objetivo
        self.pub_objetivo.publish(self.pose_stamped(frame, posicion, giro))
        self.enviado = self.get_clock().now()
        nombre = 'la pose previa' if tramo == 0 else 'frente al objeto (último tramo)'
        self.get_logger().info(f'Objeto estable: voy a {nombre}, a {a.distancias[tramo]:.2f} m '
                               'de él')

    def pose_stamped(self, frame, posicion, giro):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = frame, self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in posicion)
        q = matrix_to_quaternion(giro)
        msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
        msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
        return msg

    def publicar(self, frame, punto, llegada):
        ahora = self.get_clock().now().to_msg()
        p = PointStamped()
        p.header.frame_id, p.header.stamp = frame, ahora
        p.point.x, p.point.y, p.point.z = (float(v) for v in punto)
        self.pub_punto.publish(p)
        previa = punto - self.get_parameter('distancia_previa').value * llegada
        self.pub_previa.publish(self.pose_stamped(frame, previa, orientacion_pinza(llegada)))
        marcas = MarkerArray()
        esfera = Marker()
        esfera.header.frame_id, esfera.header.stamp = frame, ahora
        esfera.ns, esfera.id, esfera.type = 'objeto', 0, Marker.SPHERE
        esfera.pose.position.x, esfera.pose.position.y, esfera.pose.position.z = (
            float(v) for v in punto)
        esfera.pose.orientation.w = 1.0
        esfera.scale.x = esfera.scale.y = esfera.scale.z = 0.03
        esfera.color.r, esfera.color.g, esfera.color.b, esfera.color.a = 0.1, 0.8, 0.3, 1.0
        esfera.lifetime = Duration(seconds=2.0).to_msg()
        flecha = Marker()
        flecha.header.frame_id, flecha.header.stamp = frame, ahora
        flecha.ns, flecha.id, flecha.type = 'objeto', 1, Marker.ARROW
        for fin in (punto - 0.15 * llegada, punto):
            flecha.points.append(Point(x=float(fin[0]), y=float(fin[1]), z=float(fin[2])))
        flecha.scale.x, flecha.scale.y, flecha.scale.z = 0.01, 0.02, 0.03
        flecha.color.r, flecha.color.g, flecha.color.b, flecha.color.a = 1.0, 1.0, 0.0, 1.0
        flecha.lifetime = Duration(seconds=2.0).to_msg()
        marcas.markers = [esfera, flecha]
        self.pub_marcas.publish(marcas)


def main():
    rclpy.init()
    nodo = ObjetoNode()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C o "timeout"/kill
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
