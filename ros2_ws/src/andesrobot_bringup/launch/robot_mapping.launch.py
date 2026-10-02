"""Milo físico mapeando: robot.launch.py + slam_toolbox (+ RViz opcional).

Teleop:        ./robot.sh teleop
Guardar mapa:  ./robot.sh mapa <nombre>

Es el launch que corre ./robot.sh. Equivale a sim_mapping.launch.py pero con el robot real
en vez de Gazebo (el SLAM es exactamente el mismo).
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

    # 1) Milo físico: ruedas + lidar + filtro de seguridad (robot.launch.py), con los puertos.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_bringup, 'launch', 'robot.launch.py'])),
        launch_arguments={'hoverboard_port': LaunchConfiguration('hoverboard_port'),
                          'lidar_port': LaunchConfiguration('lidar_port')}.items(),
    )
    # 2) SLAM con el reloj real del computador (use_sim_time:=false).
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_slam, 'launch', 'mapping.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )
    # 3) RViz en el mismo computador de Milo. robot.sh pasa rviz:=true si el portátil tiene pantalla.
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_slam, 'rviz', 'mapping.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('hoverboard_port', default_value='/dev/hoverboard'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true si Milo tiene pantalla; si no, abrir RViz en el PC'),
        robot,
        mapping,
        rviz,
    ])
