"""Navegación AUTÓNOMA en simulación: Gazebo + Milo + filtro lidar + AMCL + Nav2 + RViz.

Lo corre:   ./sim.sh nav <mapa> [casa|arena]
Mandar a Milo:  ./sim.sh ir kitchen      (o "2D Goal Pose" en RViz)
Teleop encima de Nav2 (le quita el control mientras tocas teclas):  ./sim.sh teleop

Flujo de datos:
  Gazebo -> /scan -> filtro -> /scan_filtered -> AMCL (dónde estoy en el mapa) + costmaps
  Nav2 -> /nav/cmd_vel --\
  teclado -> /cmd_vel_teleop --> twist_mux -> /safety/cmd_vel_in -> safety_filter
                                                  -> /cmd_vel -> Gazebo

OJO: el mapa debe haberse hecho en el MISMO mundo y partiendo del MISMO punto (el spawn por
defecto), así el frame "map" coincide con las ubicaciones del .yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')
    pkg_slam = FindPackageShare('andesrobot_slam')
    pkg_safety = FindPackageShare('andesrobot_safety')
    pkg_nav = FindPackageShare('andesrobot_navigation')

    # 1) Gazebo + mundo + Milo
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )
    # 2) Solo el filtro del lidar (slam:=false: aquí localiza AMCL sobre el mapa guardado)
    scan_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'mapping.launch.py'])
        ),
        launch_arguments={'use_sim_time': 'true', 'slam': 'false'}.items(),
    )
    # 3) Filtro de seguridad en modo navegación (lee lo que sale de twist_mux)
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_safety, 'launch', 'safety.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'config': 'safety_nav.yaml',
            'cmd_vel_in': '/safety/cmd_vel_in',
            'cmd_vel_out': '/cmd_vel',
        }.items(),
    )
    # 4) AMCL + Nav2 + twist_mux + marcadores de ubicaciones
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'ubicaciones': LaunchConfiguration('ubicaciones'),
        }.items(),
    )
    # 5) RViz de navegación
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_nav, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('map', description='Ruta al .yaml del mapa'),
            DeclareLaunchArgument('mundo', default_value='robocup_casa'),
            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution(
                    [pkg_gazebo, 'worlds', [LaunchConfiguration('mundo'), '.world']]
                ),
            ),
            DeclareLaunchArgument(
                'ubicaciones',
                default_value=PathJoinSubstitution(
                    [pkg_nav, 'config', 'ubicaciones_robocup_casa.yaml']
                ),
            ),
            DeclareLaunchArgument('gazebo_gui', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            sim,
            scan_filter,
            safety,
            nav,
            rviz,
        ]
    )
