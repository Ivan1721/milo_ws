"""Milo físico: hoverboard (ros2_control) + RPLIDAR C1 + filtro de seguridad.

Topics iguales que en simulación: /scan, /cmd_vel_teleop (entrada), TF odom->base_footprint.

ROS2_CONTROL en 3 piezas:
  - hardware interface (nuestro driver hoverboard_hardware_interface): habla por el puerto serie
    con el hoverboard. Expone "velocidad de cada rueda" (comando) y "posición/velocidad" (estado).
  - controller_manager (ros2_control_node): carga el driver y corre los controladores a 50 Hz.
  - controladores:
      joint_state_broadcaster -> publica /joint_states (ángulo de cada rueda) para los TF
      diff_drive_controller   -> recibe velocidad (v, w) del robot, la convierte en velocidad
                                 de cada rueda, y calcula la odometría (/odom + TF odom->base_footprint)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpetas instaladas de los paquetes.
    pkg_desc = FindPackageShare('andesrobot_description')
    pkg_bringup = FindPackageShare('andesrobot_bringup')
    pkg_safety = FindPackageShare('andesrobot_safety')

    # Puertos serie (robot.sh los pasa; por defecto los nombres fijos de udev).
    hoverboard_port = LaunchConfiguration('hoverboard_port')
    lidar_port = LaunchConfiguration('lidar_port')

    # URDF de Milo físico:
    #   lock_arm=true     -> brazo fijo (todavía no se controla)
    #   use_hardware=true -> incluye el bloque <ros2_control> con el driver del hoverboard
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([pkg_desc, 'urdf', 'andesrobot.urdf.xacro']),
        ' lock_arm:=true use_hardware:=true hoverboard_port:=', hoverboard_port,
    ]), value_type=str)

    # TF de todas las piezas (las ruedas giran según /joint_states del joint_state_broadcaster).
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # controller_manager: lee el bloque <ros2_control> del URDF, carga el driver del hoverboard
    # y los controladores definidos en milo_controllers.yaml.
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[{'robot_description': robot_description},
                    PathJoinSubstitution([pkg_bringup, 'config', 'milo_controllers.yaml'])],
    )

    # "spawner" = programa que le pide al controller_manager cargar y activar un controlador,
    # y después termina. "-c" = a qué controller_manager hablarle.
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )
    diff_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller', '-c', '/controller_manager'],
    )
    # diff drive después del joint_state_broadcaster
    # RegisterEventHandler + OnProcessExit = "cuando termine jsb_spawner, arranca diff_spawner".
    diff_after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner,
                                                        on_exit=[diff_spawner]))

    # Driver del RPLIDAR C1 (sllidar_ros2 de Slamtec). Publica /scan.
    lidar = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', output='screen',
        parameters=[{'channel_type': 'serial',
                     'serial_port': lidar_port,
                     'serial_baudrate': 460800,      # RPLIDAR C1
                     'frame_id': 'laser',            # mismo frame que en el URDF / simulación
                     # inverted=False: el lidar va montado hacia arriba (no de cabeza).
                     'inverted': False,
                     # angle_compensate: reordena las medidas en ángulos fijos (scan más limpio).
                     'angle_compensate': True,
                     'scan_mode': 'Standard'}],
    )

    # Filtro de seguridad: su salida va directo al diff_drive_controller
    # (el diff_drive_controller escucha en /diff_drive_controller/cmd_vel_unstamped).
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_safety, 'launch', 'safety.launch.py'])),
        launch_arguments={'use_sim_time': 'false',
                          'cmd_vel_out': '/diff_drive_controller/cmd_vel_unstamped'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('hoverboard_port', default_value='/dev/hoverboard',
                              description='Puerto serie del hoverboard (ver docker/udev)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar',
                              description='Puerto serie del RPLIDAR C1 (ver docker/udev)'),
        rsp,
        controller_manager,
        jsb_spawner,
        diff_after_jsb,
        lidar,
        safety,
    ])
