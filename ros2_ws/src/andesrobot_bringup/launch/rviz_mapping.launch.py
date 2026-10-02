"""RViz de mapeo para abrir en el PC mientras Milo mapea (misma red y ROS_DOMAIN_ID).

ROS 2 descubre los nodos de otros computadores por la red automáticamente (DDS), así que
este RViz ve /map, /scan y los TF que publica Milo sin configurar nada más.
Lo corre ./robot.sh rviz.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Solo RViz, con la misma configuración de vista que en simulación.
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('andesrobot_slam'),
                                                    'rviz', 'mapping.rviz'])]),
    ])
