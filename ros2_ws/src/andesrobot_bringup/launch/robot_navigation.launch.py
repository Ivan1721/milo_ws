"""Navegación AUTÓNOMA con Milo FÍSICO: robot.launch.py (modo nav) + filtro lidar + AMCL + Nav2.

Lo corre:  ./robot.sh nav <mapa>        (mapa hecho antes con ./robot.sh + ./robot.sh mapa)
Mandar:    ./robot.sh ir <lugar>         (o RViz desde el PC: ./robot.sh rviz-nav)

Pose inicial: por defecto (0, 0, 0) = donde partió Milo cuando se hizo el mapa. Si lo dejas en
otro lado: x:=1.0 y:=0.5 yaw:=1.57 (con punto decimal) o "2D Pose Estimate" en RViz.
Ubicaciones: hay que medirlas en el mapa REAL (con tf2_echo, igual que en la sim) y anotarlas en
  un archivo como maps/casa_lugares.yaml; se pasa con ubicaciones:=<ruta>.

SIN PROBAR en el robot real todavía: la primera vez con Milo en un espacio abierto y alguien
con la mano en el apagado.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('andesrobot_bringup')
    pkg_slam = FindPackageShare('andesrobot_slam')
    pkg_nav = FindPackageShare('andesrobot_navigation')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'robot.launch.py'])
        ),
        launch_arguments={
            'hoverboard_port': LaunchConfiguration('hoverboard_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'safety_config': 'safety_nav.yaml',
            'cmd_vel_in': '/safety/cmd_vel_in',
        }.items(),
    )
    scan_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'mapping.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'false', 'slam': 'false'}.items(),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'ubicaciones': LaunchConfiguration('ubicaciones'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_nav, 'rviz', 'navigation.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('map', description='Ruta al .yaml del mapa'),
            DeclareLaunchArgument('hoverboard_port', default_value='/dev/hoverboard'),
            DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
            DeclareLaunchArgument(
                'ubicaciones',
                default_value='/ros2_ws/src/andesrobot_slam/maps/casa_lugares.yaml',
            ),
            DeclareLaunchArgument('x', default_value='0.0'),
            DeclareLaunchArgument('y', default_value='0.0'),
            DeclareLaunchArgument('yaw', default_value='0.0'),
            DeclareLaunchArgument('rviz', default_value='false'),
            robot,
            scan_filter,
            nav,
            rviz,
        ]
    )
