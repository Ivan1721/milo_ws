"""Nodo arm_ik_node: lleva la pinza de Milo a una pose (cinemática inversa + trayectoria).

Escucha:  /arm_target_pose (geometry_msgs/PoseStamped, en cualquier frame de TF)
          /joint_states    (para partir desde donde está el brazo)
Manda:    una trayectoria de un punto a /arm_controller/follow_joint_trajectory
          (lift + joint_1..6 a la vez).

Si la pose no es alcanzable avisa en el log y NO mueve el brazo.
No revisa choques: el brazo va directo (en el espacio de articulaciones) a la solución.
"""

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener

from andesrobot_arm.kinematics import (
    ArmKinematics, LIFT_BASE, LIFT_JOINT, quaternion_to_matrix)


def to_matrix(translation, rotation):
    """Transformación 4x4 desde un Point/Vector3 y un Quaternion de geometry_msgs."""
    pose = np.eye(4)
    pose[:3, :3] = quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    pose[:3, 3] = [translation.x, translation.y, translation.z]
    return pose


def trajectory_duration(q_from, q_to, max_velocity, min_duration):
    """
    Tiempo que tarda la articulación más lenta en llegar, nunca menos que min_duration.

    max_velocity: un valor para todas o uno por articulación (rad/s o m/s).
    """
    moves = np.abs(np.asarray(q_to) - np.asarray(q_from)) / np.asarray(max_velocity, float)
    return max(min_duration, float(np.max(moves)))


class ArmIkNode(Node):

    def __init__(self):
        super().__init__('arm_ik_node')
        # Parámetros (se cambian con --ros-args -p nombre:=valor).
        self.position_only = self.declare_parameter('position_only', False).value
        self.max_joint_velocity = self.declare_parameter('max_joint_velocity', 0.5).value
        use_lift = self.declare_parameter('use_lift', True).value
        max_lift_velocity = self.declare_parameter('max_lift_velocity', 0.1).value
        lift_weight = self.declare_parameter('lift_weight', 10.0).value
        self.min_duration = self.declare_parameter('min_duration', 1.0).value
        self.action_name = self.declare_parameter(
            'action_name', '/arm_controller/follow_joint_trajectory').value

        # Con el lift la IK va de base_footprint a gripper_tcp (7 articulaciones). lift_weight > 1
        # hace que prefiera mover el brazo y use el lift solo cuando el brazo no alcanza.
        self.arm = ArmKinematics.from_xacro(**({'base': LIFT_BASE} if use_lift else {}))
        is_lift = np.array([n == LIFT_JOINT for n in self.arm.joint_names])
        self.weights = np.where(is_lift, lift_weight, 1.0)
        self.max_velocity = np.where(is_lift, max_lift_velocity, self.max_joint_velocity)
        self.current = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(PoseStamped, '/arm_target_pose', self.on_target, 10)

        self.get_logger().info(
            f'Escuchando /arm_target_pose (TCP = {self.arm.tip}, frame de la IK = '
            f"{self.arm.base}, {'solo posición' if self.position_only else 'pose completa'})")

    def on_joint_states(self, msg):
        # /joint_states puede venir de varias fuentes (ruedas, brazo): se juntan por nombre.
        self.current.update(zip(msg.name, msg.position))

    def on_target(self, msg):
        log = self.get_logger()
        names = self.arm.joint_names
        if not all(n in self.current for n in names):
            log.warn('Todavía no llegan /joint_states del brazo; ignoro el objetivo.')
            return
        q0 = np.array([self.current[n] for n in names])

        target = to_matrix(msg.pose.position, msg.pose.orientation)
        frame = msg.header.frame_id or self.arm.base
        if frame != self.arm.base:
            try:
                # La TF más reciente: el objetivo es una meta, no una medición con hora.
                tf = self.tf_buffer.lookup_transform(self.arm.base, frame, Time())
            except TransformException as exc:
                log.error(f"No hay TF '{frame}' -> '{self.arm.base}': {exc}")
                return
            target = to_matrix(tf.transform.translation, tf.transform.rotation) @ target

        q, ok, (err_pos, err_rot) = self.arm.ik(
            target, q0=q0, position_only=self.position_only, weights=self.weights)
        if not ok:
            log.warn(f'IK sin solución (mejor error {err_pos * 1000:.1f} mm, '
                     f'{np.degrees(err_rot):.1f}°): probablemente fuera de alcance. No me muevo.')
            return

        if not self.action_client.server_is_ready():
            log.error(f'El controlador {self.action_name} no está disponible.')
            return

        duration = trajectory_duration(q0, q, self.max_velocity, self.min_duration)
        point = JointTrajectoryPoint()
        point.positions = q.tolist()
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [point]

        moves = ', '.join(f'{n}={v:.3f}' for n, v in zip(names, q))
        log.info(f'IK ok -> {moves} (rad / m), moviendo en {duration:.1f} s')
        self.action_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('arm_controller rechazó la trayectoria.')
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            # Con las tolerancias de arm_controllers.yaml esto significa que el brazo llegó de
            # verdad (cada articulación a menos de 0.02 rad de su objetivo, el lift a 5 mm).
            self.get_logger().info('Objetivo alcanzado.')
            return
        # Códigos de FollowJointTrajectory.Result, explicados.
        motivos = {
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                'el brazo no llegó: quedó lejos del objetivo al terminar (¿chocó con algo?)',
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
                'el brazo se desvió demasiado durante el movimiento (¿chocó con algo?)',
            FollowJointTrajectory.Result.INVALID_GOAL: 'trayectoria inválida',
            FollowJointTrajectory.Result.INVALID_JOINTS: 'articulaciones que no son suyas',
            FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: 'la trayectoria llegó tarde',
        }
        self.get_logger().warn('La trayectoria terminó con error %d: %s. %s' % (
            result.error_code, motivos.get(result.error_code, 'error desconocido'),
            result.error_string))


def main(args=None):
    rclpy.init(args=args)
    node = ArmIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
