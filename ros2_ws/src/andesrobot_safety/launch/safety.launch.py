"""Filtro de seguridad: /cmd_vel_teleop + /scan -> cmd_vel_out.

Simulación: cmd_vel_out=/cmd_vel (plugin de Gazebo).
Milo físico: cmd_vel_out=/diff_drive_controller/cmd_vel_unstamped.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # A dónde manda el filtro los comandos ya revisados (cambia entre sim y robot).
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel'),
        Node(
            package='andesrobot_safety',
            # "safety_filter" es el ejecutable que define setup.py (entry_points).
            executable='safety_filter',
            name='safety_filter',
            output='screen',
            # Parámetros: los de config/safety.yaml (distancias, velocidades) + el reloj.
            parameters=[PathJoinSubstitution([FindPackageShare('andesrobot_safety'),
                                              'config', 'safety.yaml']),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # El nodo publica en "cmd_vel"; aquí lo renombramos al destino real.
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_out'))],
        ),
    ])
