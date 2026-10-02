"""Todo junto: Gazebo + Milo + filtro de seguridad + slam_toolbox + RViz.

Teleop (en otra terminal):  ./sim.sh teleop
  (= ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop)
Guardar mapa:               ./sim.sh mapa <nombre>

Es el launch que corre ./sim.sh. No arranca nada propio: junta otros 3 launch + RViz.
  teclado -> /cmd_vel_teleop -> safety_filter -> /cmd_vel -> Milo en Gazebo
  Gazebo -> /scan -> filtro -> /scan_filtered -> slam_toolbox -> /map -> RViz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpetas instaladas de los paquetes que se combinan aquí.
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')
    pkg_slam = FindPackageShare('andesrobot_slam')
    pkg_safety = FindPackageShare('andesrobot_safety')

    # 1) Simulación: Gazebo + mundo + Milo (andesrobot_gazebo/launch/sim.launch.py).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gazebo_gui')}.items(),
    )
    # 2) SLAM: filtro del lidar + slam_toolbox, con el reloj de Gazebo.
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_slam, 'launch', 'mapping.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    # 3) Filtro de seguridad. En simulación su salida es /cmd_vel (lo que escucha el plugin
    #    diff drive de Gazebo), que es el valor por defecto de safety.launch.py.
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_safety, 'launch', 'safety.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    # 4) RViz con la vista de mapeo (mapa, scan, robot, zona de parada). Solo si rviz:=true.
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_slam, 'rviz', 'mapping.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_gazebo, 'worlds', 'andesrobot_arena.world'])),
        # ./sim.sh sin-gazebo pasa gazebo_gui:=false.
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sim,
        safety,
        mapping,
        rviz,
    ])
