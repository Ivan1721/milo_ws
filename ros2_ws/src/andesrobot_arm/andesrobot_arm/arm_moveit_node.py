"""Nodo arm_moveit_node: lleva la pinza de Milo a una pose usando MoveIt 2 (planifica sin choques).

Hace lo mismo que arm_ik_node (escucha /arm_target_pose y mueve el brazo), pero en vez de
resolver la cinemática inversa y mandar un solo punto, le pide a move_group que:
  1. resuelva la IK (KDL) para la pose,
  2. busque un camino completo desde la postura actual que no choque con el robot, la mesa ni lo
     que ve la cámara (OctoMap), y
  3. le ponga tiempo con rampas de aceleración y lo ejecute en arm_controller, vigilando que
     llegue.
Es el "cliente de MoveIt" de la figura C del informe.

Escucha:  /arm_target_pose (geometry_msgs/PoseStamped, en cualquier frame que conozca TF)
Manda:    la acción /move_action (moveit_msgs/MoveGroup) con la pose como meta del grupo "brazo"
          y, al arrancar, la mesa de prueba como obstáculo conocido en /planning_scene.

Si MoveIt no encuentra camino (meta fuera de alcance, en choque, o sin IK) avisa en el log con
el motivo en español y NO mueve el brazo.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, CollisionObject, Constraints, MoveItErrorCodes, OrientationConstraint,
    PlanningScene, PositionConstraint)
from shape_msgs.msg import SolidPrimitive

# Códigos de resultado de MoveIt (moveit_msgs/MoveItErrorCodes), explicados.
MOTIVOS = {
    # FAILURE es lo que devuelve con una meta en pose cuando ninguna solución de IK está libre de
    # choques (OMPL: "Unable to sample any valid states for goal tree").
    MoveItErrorCodes.FAILURE:
        'no encontró ninguna postura válida para la meta: todas las soluciones de IK chocan con '
        'algo (la mesa, el propio robot, lo que ve la cámara) o la pose no tiene solución',
    MoveItErrorCodes.PLANNING_FAILED:
        'no encontró un camino: la meta está fuera de alcance, choca con algo (la mesa, la '
        'columna, lo que ve la cámara) o no tiene solución de IK',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'el plan quedó inválido',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        'la escena cambió mientras se movía (apareció un obstáculo) y el plan ya no sirve',
    MoveItErrorCodes.CONTROL_FAILED: 'el controlador no ejecutó la trayectoria (¿chocó?)',
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'no llegan datos de la cámara',
    MoveItErrorCodes.TIMED_OUT: 'se acabó el tiempo de planificación',
    MoveItErrorCodes.PREEMPTED: 'llegó otro objetivo y este se canceló',
    MoveItErrorCodes.START_STATE_IN_COLLISION:
        'la postura actual ya está en choque: mover a mano (RViz o arm_ik_node) a una libre',
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        'la postura actual viola restricciones',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'la meta está en choque con algo',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'la meta viola restricciones',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'terminó lejos de la meta',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'grupo desconocido (ver el SRDF)',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'meta mal formada',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'estado del robot inválido',
    MoveItErrorCodes.INVALID_LINK_NAME: 'link desconocido (¿tcp_frame?)',
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'no pudo pasar la pose al frame del robot (TF)',
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'no hay revisión de choques',
    MoveItErrorCodes.ROBOT_STATE_STALE: 'no llegan /joint_states',
    MoveItErrorCodes.SENSOR_INFO_STALE: 'la cámara dejó de publicar',
    MoveItErrorCodes.NO_IK_SOLUTION: 'sin solución de cinemática inversa para esa pose',
}


def caja(nombre, tam, xyz, frame):
    """CollisionObject con una caja de lado tam = (x, y, z) centrada en xyz, en el frame dado."""
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = nombre
    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = [float(v) for v in tam]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    pose.orientation.w = 1.0
    obj.primitives.append(caja)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class ArmMoveItNode(Node):

    def __init__(self):
        super().__init__('arm_moveit_node')
        # Parámetros (se cambian con --ros-args -p nombre:=valor).
        self.group = self.declare_parameter('group', 'brazo').value
        self.tcp_frame = self.declare_parameter('tcp_frame', 'gripper_tcp').value
        self.planning_time = self.declare_parameter('planning_time', 5.0).value
        self.attempts = self.declare_parameter('planning_attempts', 5).value
        # Fracción de los límites de joint_limits.yaml que se usa (0.5 = la mitad).
        self.velocity_scaling = self.declare_parameter('velocity_scaling', 0.5).value
        self.acceleration_scaling = self.declare_parameter('acceleration_scaling', 0.5).value
        # Tolerancias de la meta: radio en metros y ángulo en radianes por eje.
        self.tol_pos = self.declare_parameter('tolerancia_pos', 0.002).value
        self.tol_rot = self.declare_parameter('tolerancia_rot', 0.02).value
        self.position_only = self.declare_parameter('position_only', False).value
        # Mesa de prueba (worlds/mesa_prueba.sdf) como obstáculo conocido; mesa:=false si se
        # lanzó arm_sim con mesa:=false. Va en base_footprint (el frame del modelo): la escena de
        # MoveIt solo conoce los frames del robot, no los demás de TF ("Unknown frame: odom").
        # Vale mientras la base no se mueva, que es el caso en la simulación del brazo.
        self.mesa = self.declare_parameter('mesa', True).value
        self.mesa_frame = self.declare_parameter('mesa_frame', 'base_footprint').value

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.create_subscription(PoseStamped, '/arm_target_pose', self.on_target, 10)
        self.en_curso = None
        # Esperar a move_group y, cuando esté, publicar la mesa en la escena.
        self.mesa_publicada = 0
        self.startup_timer = self.create_timer(1.0, self.al_arrancar)
        self.get_logger().info(
            f"Escuchando /arm_target_pose; MoveIt planifica el grupo '{self.group}' hasta "
            f"'{self.tcp_frame}' ({'solo posición' if self.position_only else 'pose completa'})")

    def al_arrancar(self):
        if not self.client.server_is_ready():
            return
        if self.mesa and self.mesa_publicada < 3:
            # Tres veces, por si move_group todavía no escuchaba /planning_scene.
            self.publicar_mesa()
            self.mesa_publicada += 1
            if self.mesa_publicada == 1:
                self.get_logger().info('MoveIt listo. Mesa de prueba agregada a la escena como '
                                       f"obstáculo (frame '{self.mesa_frame}').")
            return
        if not self.mesa and self.mesa_publicada == 0:
            self.get_logger().info('MoveIt listo.')
            self.mesa_publicada = 1
        self.startup_timer.cancel()

    def publicar_mesa(self):
        """La mesa de mesa_prueba.sdf: tablero y pata, con las mismas medidas y posiciones."""
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            caja('mesa_tablero', (0.6, 0.8, 0.04), (1.0, 0.0, 0.70), self.mesa_frame))
        scene.world.collision_objects.append(
            caja('mesa_pata', (0.5, 0.7, 0.68), (1.0, 0.0, 0.34), self.mesa_frame))
        self.scene_pub.publish(scene)

    def on_target(self, msg):
        log = self.get_logger()
        if not self.client.server_is_ready():
            log.error('move_group no está disponible (¿se lanzó con moveit:=true?).')
            return
        frame = msg.header.frame_id or 'base_footprint'

        # Meta = restricciones sobre el TCP: posición dentro de una esfera chica y orientación
        # dentro de una tolerancia por eje. Es la forma en que MoveIt recibe "ir a esta pose".
        pos = PositionConstraint()
        pos.header.frame_id = frame
        pos.link_name = self.tcp_frame
        pos.target_point_offset = Vector3()
        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [self.tol_pos]
        centro = Pose()
        centro.position = msg.pose.position
        centro.orientation.w = 1.0
        pos.constraint_region = BoundingVolume(primitives=[esfera], primitive_poses=[centro])
        pos.weight = 1.0
        meta = Constraints(name='pose_de_la_pinza', position_constraints=[pos])
        if not self.position_only:
            ori = OrientationConstraint()
            ori.header.frame_id = frame
            ori.link_name = self.tcp_frame
            ori.orientation = msg.pose.orientation
            ori.absolute_x_axis_tolerance = self.tol_rot
            ori.absolute_y_axis_tolerance = self.tol_rot
            ori.absolute_z_axis_tolerance = self.tol_rot
            ori.weight = 1.0
            meta.orientation_constraints.append(ori)

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.goal_constraints.append(meta)
        req.num_planning_attempts = self.attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling
        # Partir de la postura actual (is_diff = no traigo un estado, usa el que ves).
        req.start_state.is_diff = True
        # Espacio de trabajo: una caja de 4 x 4 x 3 m alrededor de Milo.
        req.workspace_parameters.header.frame_id = 'base_footprint'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -0.1
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 3.0
        # Planificar Y ejecutar, sobre la escena actual de move_group.
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        p = msg.pose.position
        log.info(f"Objetivo ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) en '{frame}': pidiendo a MoveIt "
                 f'un camino sin choques (hasta {self.planning_time:.0f} s)...')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('move_group rechazó el objetivo.')
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            n = len(result.planned_trajectory.joint_trajectory.points)
            puntos = result.planned_trajectory.joint_trajectory.points
            t = puntos[-1].time_from_start if n else None
            dur = f'{t.sec + t.nanosec * 1e-9:.1f} s' if t else '?'
            self.get_logger().info(f'Objetivo alcanzado: camino de {n} puntos, {dur}.')
            return
        self.get_logger().warn('MoveIt no movió el brazo (código %d): %s.' % (
            code, MOTIVOS.get(code, 'error desconocido')))


def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveItNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
