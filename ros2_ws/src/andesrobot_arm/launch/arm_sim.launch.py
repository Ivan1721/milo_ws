"""Brazo de Milo en Gazebo: robot con el brazo controlado + IK + marcador interactivo + RViz.

Orden de lo que pasa:
 1. Se procesa el xacro de Milo con lock_arm:=false y arm_control:=true (brazo, lift y pinza
    con ros2_control) y gripper_camera:=true (cámara Orbbec sobre la pinza), y se publica en
    /robot_description.
 2. Gazebo abre el mundo y spawn_entity.py pone a Milo y la mesa de prueba frente a él.
 3. Si Milo apareció bien, el spawner arranca los controladores del brazo.
 4. arm_ik_node espera poses en /arm_target_pose; arm_marker_node las publica desde RViz.

Uso:  ros2 launch andesrobot_arm arm_sim.launch.py   [gui:=false] [rviz:=false]
                                                    [camara:=false] [mesa:=false]
                                                    [moveit:=false] [octomap:=false]
                                                    [puerta:=true]
Con puerta:=true pone la puerta de prueba (worlds/puerta_prueba.sdf: se abre empujando, manija de
palanca a 1 m) en vez de la mesa.
Con moveit:=true (default) arranca move_group (MoveIt 2) con la configuración de
andesrobot_moveit_config y arm_moveit_node reemplaza a arm_ik_node: las poses de
/arm_target_pose se planifican sin choques (robot, mesa y, con octomap:=true, lo que ve la
cámara). Con moveit:=false queda la cinemática inversa propia (arm_ik_node), sin revisión
de choques.
Cámara: topics /gripper_camera/{color,depth,ir}/image_raw y /gripper_camera/depth/points
(ver andesrobot_description/urdf/andesrobot.gripper_camera.xacro). Guardar una imagen de cada
canal:  ros2 run andesrobot_arm capturar_camara
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _strip_comments(node):
    """Borrar los comentarios XML del URDF (ver robot_description_urdf)."""
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def robot_description_urdf(simple_collision, camara):
    xacro_file = os.path.join(get_package_share_directory('andesrobot_description'),
                              'urdf', 'andesrobot.urdf.xacro')
    controllers = os.path.join(get_package_share_directory('andesrobot_arm'),
                               'config', 'arm_controllers.yaml')
    doc = xacro.process_file(xacro_file, mappings={
        'lock_arm': 'false',
        'arm_control': 'true',
        'arm_controllers_file': controllers,
        'simple_collision': simple_collision,
        'gripper_camera': camara,
    })
    # gazebo_ros2_control (Humble) le vuelve a pasar robot_description al controller_manager
    # como argumento de línea de comandos, y si el URDF tiene comentarios no lo puede leer
    # (el controller_manager nunca arranca). El xacro de Milo tiene muchos: se borran aquí.
    _strip_comments(doc)
    return doc.toxml()


def moveit_config(simple_collision, camara, octomap):
    """Parámetros de MoveIt 2 (URDF + SRDF + cinemática + límites + controladores + OctoMap).

    MoveItConfigsBuilder lee los archivos de andesrobot_moveit_config/config y procesa el MISMO
    xacro que robot_state_publisher (con los mismos argumentos), así move_group y la simulación
    ven el mismo robot. Devuelve un objeto cuyo .to_dict() son los parámetros de move_group.
    """
    xacro_file = os.path.join(get_package_share_directory('andesrobot_description'),
                              'urdf', 'andesrobot.urdf.xacro')
    controllers = os.path.join(get_package_share_directory('andesrobot_arm'),
                               'config', 'arm_controllers.yaml')
    # (Al arrancar avisa "Cannot infer URDF/SRDF ... using config/andesrobot.urdf": es porque el
    # paquete no trae su propio xacro; el URDF se le pasa explícito abajo. No es un error.)
    builder = (
        MoveItConfigsBuilder('andesrobot', package_name='andesrobot_moveit_config')
        .robot_description(file_path=xacro_file, mappings={
            'lock_arm': 'false', 'arm_control': 'true', 'arm_controllers_file': controllers,
            'simple_collision': simple_collision, 'gripper_camera': camara})
        .robot_description_semantic(file_path='config/andesrobot.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        # Publicar la escena (obstáculos, OctoMap, estado) para RViz.
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True, publish_geometry_updates=True,
                                publish_state_updates=True, publish_transforms_updates=True)
    )
    if octomap == 'true':
        # OctoMap desde la nube de la cámara de la pinza (config/sensors_3d.yaml).
        builder = builder.sensors_3d(file_path='config/sensors_3d.yaml')
    return builder.to_moveit_configs()


def launch_setup(context):
    urdf = robot_description_urdf(LaunchConfiguration('simple_collision').perform(context),
                                  LaunchConfiguration('camara').perform(context))
    moveit = LaunchConfiguration('moveit').perform(context) == 'true'
    # Escenario: la puerta reemplaza a la mesa (las dos ocuparían el mismo lugar frente a Milo).
    puerta = LaunchConfiguration('puerta').perform(context) == 'true'
    con_mesa = LaunchConfiguration('mesa').perform(context) == 'true' and not puerta

    # robot_state_publisher: publica /robot_description y las TF, con el reloj de Gazebo.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
    )

    # Poner a Milo en Gazebo, 2 cm sobre el suelo.
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'andesrobot', '-z', '0.02'],
        output='screen',
    )

    # Arrancar los controladores (definidos en config/arm_controllers.yaml).
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Solo si spawn_entity terminó bien; si no, el spawner se quedaría esperando para siempre.
    def start_controllers_if_spawned(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='spawn_entity falló: no arranco los controladores. Si el '
                                "error es \"No module named 'rclpy._rclpy_pybind11'\", hay un "
                                'python3 que no es el de ROS primero en el PATH (Anaconda): '
                                'conda deactivate.')]
        return [spawner]

    controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=start_controllers_if_spawned))

    sim_time = [{'use_sim_time': True}]
    marker = Node(package='andesrobot_arm', executable='arm_marker_node',
                  parameters=sim_time, output='screen')
    rviz_params = list(sim_time)
    if moveit:
        config = moveit_config(LaunchConfiguration('simple_collision').perform(context),
                               LaunchConfiguration('camara').perform(context),
                               LaunchConfiguration('octomap').perform(context))
        # move_group: el planificador. Arranca cuando el spawner terminó (controladores
        # activos): así encuentra las acciones FollowJointTrajectory al conectarse.
        move_group = Node(package='moveit_ros_move_group', executable='move_group',
                          output='screen', parameters=[config.to_dict(), {'use_sim_time': True}])
        # Cliente: /arm_target_pose -> acción MoveGroup (y la mesa como obstáculo).
        mover = Node(package='andesrobot_arm', executable='arm_moveit_node', output='screen',
                     parameters=[{'use_sim_time': True, 'mesa': con_mesa}])
        after_controllers = RegisterEventHandler(
            OnProcessExit(target_action=spawner, on_exit=[move_group, mover]))
        # RViz necesita el URDF/SRDF como parámetros para dibujar la escena y la trayectoria.
        rviz_params += [config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics]
        rviz_file = 'arm_moveit.rviz'
    else:
        # Sin MoveIt: la cinemática inversa propia, sin revisión de choques.
        after_controllers = Node(package='andesrobot_arm', executable='arm_ik_node',
                                 parameters=sim_time, output='screen')
        rviz_file = 'arm.rviz'
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('andesrobot_arm'),
                                      'rviz', rviz_file)],
        parameters=rviz_params,
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    # Mesa con tres objetos frente a Milo (worlds/mesa_prueba.sdf): algo que la cámara vea a
    # 0.6-1 m (en la arena no hay nada a menos de 2.5 m, el alcance de la profundidad) y que el
    # brazo alcance.
    escenario = 'puerta_prueba' if puerta else 'mesa_prueba'
    objetos = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', os.path.join(get_package_share_directory('andesrobot_arm'),
                                         'worlds', escenario + '.sdf'),
                   '-entity', escenario],
        output='screen',
    )
    extra = [objetos] if (puerta or con_mesa) else []
    return [rsp, spawn, controllers, after_controllers, marker, rviz] + extra


def generate_launch_description():
    # Gazebo (gzserver + gzclient) con el launch oficial de gazebo_ros.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'verbose': 'false'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('andesrobot_gazebo'), 'worlds', 'andesrobot_arena.world']),
            description='Ruta al .world'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la ventana de Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz con el marcador'),
        # Con cajas simples el brazo NO tiene colisión propia (y queda una caja fija donde
        # estaría el brazo bloqueado): por eso aquí las mallas CAD son el default.
        DeclareLaunchArgument('simple_collision', default_value='false',
                              description='true = colisiones con cajas en vez de mallas CAD'),
        DeclareLaunchArgument('camara', default_value='true',
                              description='Cámara Orbbec Gemini Plus sobre la pinza'),
        DeclareLaunchArgument('mesa', default_value='true',
                              description='Mesa con objetos de prueba frente a Milo'),
        DeclareLaunchArgument('puerta', default_value='false',
                              description='Puerta de prueba (abre empujando) en vez de la mesa'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='true = MoveIt 2 planifica sin choques '
                                          '(arm_moveit_node); false = IK propia (arm_ik_node)'),
        DeclareLaunchArgument('octomap', default_value='true',
                              description='Con moveit: lo que ve la cámara entra a la escena '
                                          'como obstáculo (OctoMap)'),
        gazebo,
        OpaqueFunction(function=launch_setup),
    ])
