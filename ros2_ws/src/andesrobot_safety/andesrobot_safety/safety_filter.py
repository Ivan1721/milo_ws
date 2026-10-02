"""Filtro de seguridad: /cmd_vel_teleop + /scan -> /cmd_vel.

Igual para simulación y para Milo físico. Frena si hay obstáculos a menos de
`stop_distance` en la dirección de avance; deja retroceder o girar para salir.

Este archivo es la parte "ROS" (topics, parámetros, timers). La matemática está en logic.py.

Entradas:  /scan (sensor_msgs/LaserScan)    lidar
           /cmd_vel_teleop (geometry_msgs/Twist)  lo que pides con el teclado
Salidas:   /cmd_vel (Twist)                 lo que realmente se le manda al robot
           /safety/state (String)           texto con lo que está haciendo el filtro
           /safety/stop_zone (PolygonStamped)  rectángulo para RViz
"""
import math

# numpy para los puntos del lidar; rclpy = la librería de ROS 2 para Python.
import numpy as np
import rclpy
# Tipos de mensajes que usamos:
#   Twist = velocidad (linear.x, angular.z); PolygonStamped/Point32 = polígono para RViz
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
# Calidad de servicio "sensor_data": la misma que usan los lidars (best effort, sin reintentos).
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
# TF: para saber dónde está el lidar respecto al centro del robot.
from tf2_ros import Buffer, TransformListener

# Nuestra lógica (logic.py, mismo paquete).
from andesrobot_safety.logic import SafetyParams, filter_cmd, remove_self


# Un nodo de ROS 2 en Python es una clase que hereda de Node.
class SafetyFilter(Node):

    def __init__(self):
        # Crear el nodo con el nombre "safety_filter" (debe coincidir con safety.yaml).
        super().__init__('safety_filter')
        # Parámetros: declarar uno por cada campo de SafetyParams, con su valor por defecto.
        # vars(d) = diccionario {nombre_campo: valor}. ROS reemplaza los valores con safety.yaml.
        d = SafetyParams()
        for name, default in vars(d).items():
            self.declare_parameter(name, default)
        # Parámetros extra que no son de la lógica:
        self.declare_parameter('base_frame', 'base_footprint')   # frame del robot
        self.declare_parameter('rate', 20.0)                      # Hz a los que publica /cmd_vel
        self.declare_parameter('scan_timeout', 0.5)               # s sin lidar -> parar
        self.declare_parameter('cmd_timeout', 0.0)  # 0 = mantener último comando (teleop teclado)

        # Leer los valores finales y armar el SafetyParams que usa la lógica.
        # {k: valor for k in ...} = diccionario por comprensión; ** = pasarlo como argumentos.
        self.p = SafetyParams(**{k: self.get_parameter(k).value for k in vars(d)})
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # TF: el Buffer guarda las transformadas que llegan; el Listener las escucha de /tf.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_tf = None  # (tx, ty, yaw) laser -> base, fijo

        # Estado interno del nodo:
        self.points = np.zeros((0, 2))   # últimos puntos del lidar (en base_footprint)
        self.last_scan = None            # cuándo llegó el último scan
        self.cmd = Twist()               # último comando del teclado (empieza en 0)
        self.last_cmd = None             # cuándo llegó
        self.last_state = ''             # último estado publicado (para no repetirlo)

        # Suscripciones (lo que escucha): (tipo, topic, función a llamar, cola/QoS).
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_teleop', self.on_cmd, 10)
        # Publicadores (lo que publica). "cmd_vel" se remapea en safety.launch.py.
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_state = self.create_publisher(String, 'safety/state', 10)
        self.pub_zone = self.create_publisher(PolygonStamped, 'safety/stop_zone', 1)
        # Timers: llamar a step() 20 veces por segundo y a publish_zone() 1 vez por segundo.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.create_timer(1.0, self.publish_zone)
        # Mensaje en la terminal al arrancar.
        self.get_logger().info(
            'Filtro de seguridad: stop %.2f m, slow %.2f m, v_max %.2f m/s'
            % (self.p.stop_distance, self.p.slow_distance, self.p.max_linear))

    # Se llama cada vez que llega un comando del teclado: solo lo guardamos.
    # (El que decide qué mandar al robot es step(), con el timer.)
    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd = self.get_clock().now()

    # Se llama cada vez que llega un scan del lidar (~10 veces por segundo).
    def on_scan(self, msg):
        # La primera vez: averiguar dónde está el lidar respecto a base_footprint (no cambia,
        # así que se calcula una sola vez y se guarda).
        if self.laser_tf is None:
            try:
                # Transformada desde el frame del lidar ("laser") hasta base_footprint.
                # Time() = "la más reciente que tengas"; timeout 0 = no esperar.
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time(), timeout=Duration(seconds=0.0))
            except Exception:
                # Todavía no llegan los TF (pasa al arrancar): ignorar este scan.
                return
            # La rotación viene como cuaternión (x, y, z, w); esta fórmula saca el ángulo yaw
            # (giro alrededor del eje vertical), que es lo único que importa en 2D.
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y, yaw)
        # Un LaserScan trae una lista de distancias (ranges), una por ángulo, empezando en
        # angle_min y avanzando de a angle_increment.
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        # Quedarse solo con lecturas válidas: números finitos (no inf/nan) dentro del rango.
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= msg.range_max)
        tx, ty, yaw = self.laser_tf
        # Pasar de polares (distancia, ángulo) en el frame del lidar a x, y en base_footprint:
        # girar por el yaw del lidar y sumar su posición.
        ang = a[ok] + yaw
        pts = np.column_stack((tx + r[ok] * np.cos(ang), ty + r[ok] * np.sin(ang)))
        # Sacar los puntos que caen sobre el propio robot y guardar.
        self.points = remove_self(pts, self.p)
        self.last_scan = self.get_clock().now()

    # Corre 20 veces por segundo: decide qué velocidad mandar al robot.
    def step(self):
        now = self.get_clock().now()
        # Twist() vacío = velocidad 0 (lo que se manda si algo está mal).
        out = Twist()
        # Sin lidar (nunca llegó, o hace más de scan_timeout): no moverse.
        # (now - last_scan) es una duración en nanosegundos; * 1e-9 la pasa a segundos.
        if self.last_scan is None or (now - self.last_scan).nanoseconds * 1e-9 > self.scan_timeout:
            state = 'PARADO: sin lidar'
        # Si se configuró cmd_timeout y el teclado dejó de mandar: no moverse.
        elif (self.cmd_timeout > 0.0 and self.last_cmd is not None
              and (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout):
            state = 'sin comando'
        # Todo bien: pasar el último comando por la lógica de seguridad.
        else:
            v, w, state = filter_cmd(self.cmd.linear.x, self.cmd.angular.z, self.points, self.p)
            out.linear.x, out.angular.z = v, w
        # Publicar SIEMPRE (aunque sea 0): si este nodo se cae, el robot deja de recibir y
        # se detiene solo (cmd_vel_timeout del diff_drive_controller).
        self.pub_cmd.publish(out)
        # Publicar el estado solo cuando cambia (para no llenar el topic de lo mismo).
        if state != self.last_state:
            self.pub_state.publish(String(data=state))
            # Si paró o bloqueó el giro, avisarlo también en la terminal como advertencia.
            if state.startswith('PARADO') or 'bloqueado' in state:
                self.get_logger().warn(state)
            self.last_state = state

    # Corre 1 vez por segundo: publica el rectángulo de la zona de parada para verlo en RViz.
    def publish_zone(self):
        p = self.p
        # Medio ancho del corredor y los extremos adelante (xf) y atrás (xr) de la zona.
        hw = p.footprint_half_width + p.side_margin
        xf, xr = p.footprint_x_max + p.stop_distance, p.footprint_x_min - p.stop_distance
        msg = PolygonStamped()
        # Coordenadas en base_footprint: el rectángulo se mueve con el robot.
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        # Las 4 esquinas del rectángulo (2 cm sobre el suelo para que se vea sobre el mapa).
        msg.polygon.points = [Point32(x=x, y=y, z=0.02) for x, y in
                              [(xr, -hw), (xf, -hw), (xf, hw), (xr, hw)]]
        self.pub_zone.publish(msg)


# Punto de entrada: lo que corre "ros2 run andesrobot_safety safety_filter" (ver setup.py).
def main():
    # Iniciar ROS 2 en este proceso y crear el nodo.
    rclpy.init()
    node = SafetyFilter()
    try:
        # spin = quedarse esperando mensajes y timers, llamando a las funciones, hasta Ctrl+C.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar: mandar velocidad 0 por si acaso, destruir el nodo y apagar ROS.
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


# Si este archivo se ejecuta directo (python3 safety_filter.py), llamar a main().
if __name__ == '__main__':
    main()
