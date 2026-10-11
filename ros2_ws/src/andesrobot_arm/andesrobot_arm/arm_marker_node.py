"""Nodo arm_marker_node: marcador interactivo en RViz sobre la pinza de Milo.

Aparece una esfera naranja en gripper_tcp con flechas (mover) y anillos (girar) en X, Y, Z.
Al soltar el mouse publica la pose en /arm_target_pose y arm_ik_node mueve el brazo.
Clic derecho en la esfera: "Enviar objetivo" y "Volver a la pinza" (útil si la IK rechazó
el objetivo, porque el marcador se queda donde lo soltaste).
En RViz hay que elegir la herramienta "Interact" (tecla i).
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker)

MARKER_NAME = 'arm_target'


def axis_controls():
    """Controles para mover y girar en/alrededor de X, Y y Z."""
    controls = []
    h = 0.5 ** 0.5
    # Orientación de cada control (w, x, y, z): un control actúa sobre su propio eje X.
    for name, (w, x, y, z) in (('x', (h, h, 0.0, 0.0)),
                               ('z', (h, 0.0, h, 0.0)),
                               ('y', (h, 0.0, 0.0, h))):
        for mode, prefix in ((InteractiveMarkerControl.MOVE_AXIS, 'mover_'),
                             (InteractiveMarkerControl.ROTATE_AXIS, 'girar_')):
            control = InteractiveMarkerControl()
            control.name = prefix + name
            control.orientation.w, control.orientation.x = w, x
            control.orientation.y, control.orientation.z = y, z
            control.interaction_mode = mode
            controls.append(control)
    return controls


def handle_control():
    """Esfera chica en el TCP; con clic derecho abre el menú."""
    sphere = Marker()
    sphere.type = Marker.SPHERE
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
    sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.6, 0.0, 0.8
    control = InteractiveMarkerControl()
    control.name = 'menu'
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = True
    control.markers.append(sphere)
    return control


class ArmMarkerNode(Node):

    def __init__(self):
        super().__init__('arm_marker_node')
        # frame fijo al robot (no sube con el lift) en el que se publica el objetivo.
        self.frame = self.declare_parameter('frame', 'base_footprint').value
        self.tcp_frame = self.declare_parameter('tcp_frame', 'gripper_tcp').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.target_pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)
        self.server = InteractiveMarkerServer(self, 'arm_target_marker')

        self.menu = MenuHandler()
        self.menu.insert('Enviar objetivo', callback=self.on_send)
        self.menu.insert('Volver a la pinza', callback=self.on_reset)

        # Esperar a que exista la TF para crear el marcador justo sobre la pinza.
        self.startup_timer = self.create_timer(0.5, self.try_create_marker)

    def current_tcp_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, self.tcp_frame, Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose

    def try_create_marker(self):
        pose = self.current_tcp_pose()
        if pose is None:
            return
        self.startup_timer.cancel()

        marker = InteractiveMarker()
        marker.header.frame_id = self.frame
        marker.name = MARKER_NAME
        marker.description = 'Objetivo de la pinza (arrastra y suelta)'
        marker.scale = 0.15
        marker.pose = pose
        marker.controls.append(handle_control())
        marker.controls.extend(axis_controls())

        self.server.insert(marker, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, MARKER_NAME)
        self.server.applyChanges()
        self.get_logger().info(
            f"Marcador listo sobre '{self.tcp_frame}' (frame '{self.frame}'). En RViz usa la "
            'herramienta Interact; al soltar el mouse se envía el objetivo.')

    def publish_target(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.pose = pose
        self.target_pub.publish(msg)

    def on_feedback(self, feedback):
        # Solo al soltar el mouse (no mientras se arrastra, para no llenar de objetivos).
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.publish_target(feedback.pose)

    def on_send(self, feedback):
        self.publish_target(feedback.pose)

    def on_reset(self, feedback):
        pose = self.current_tcp_pose()
        if pose is not None:
            self.server.setPose(MARKER_NAME, pose)
            self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
