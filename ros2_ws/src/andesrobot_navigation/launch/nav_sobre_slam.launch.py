"""Nav2 SOBRE EL SLAM QUE YA ESTÁ CORRIENDO (sin AMCL ni mapa guardado).

Para practicar: con ./sim.sh casa abierto (Gazebo + slam_toolbox + RViz), esto agrega el
planificador y el controlador de Nav2. La localización la sigue haciendo slam_toolbox (publica
el mapa /map y la TF map->odom), así que las poses que anotaste con tf2_echo sirven directo.

Lanzar (dentro del contenedor, ./sim.sh shell):
  ros2 launch /ros2_ws/src/andesrobot_navigation/launch/nav_sobre_slam.launch.py

Velocidades: Nav2 publica en /cmd_vel_teleop, la MISMA entrada que el teclado, y de ahí pasa por
el filtro de seguridad (modo teleop) hasta Gazebo:
  controller -> /nav/cmd_vel_raw -> velocity_smoother -> /cmd_vel_teleop -> safety_filter -> /cmd_vel
Por eso: mientras Nav2 maneja, NO toques el teleop (los dos escriben en el mismo tópico).
Es solo para practicar en simulación; la versión "de verdad" (AMCL + twist_mux) es
navigation.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('andesrobot_navigation')
    # Mismos parámetros de Nav2 que la navegación completa (los de AMCL no se usan aquí).
    common = [LaunchConfiguration('params_file'), {'use_sim_time': True}]
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    salida = '/cmd_vel_teleop'  # entrada del filtro de seguridad

    nodos = [
        # Sigue el camino: calcula la velocidad 20 veces por segundo.
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common,
             remappings=tf + [('cmd_vel', '/nav/cmd_vel_raw')]),
        # Suaviza el camino planeado.
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=common, remappings=tf),
        # Planifica el camino en el mapa (/map de slam_toolbox).
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common, remappings=tf),
        # Recuperaciones: girar, retroceder, esperar (si se atasca).
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common, remappings=tf + [('cmd_vel', salida)]),
        # El "cerebro": recibe la meta (acción /navigate_to_pose) y coordina a los demás.
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=common, remappings=tf),
        # Limita aceleraciones a lo que Milo puede hacer.
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common,
             remappings=tf + [('cmd_vel', '/nav/cmd_vel_raw'), ('cmd_vel_smoothed', salida)]),
        # Prende todos los nodos de arriba en orden (son "lifecycle nodes").
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server', 'smoother_server',
                                         'planner_server', 'behavior_server',
                                         'bt_navigator', 'velocity_smoother']}]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])),
    ] + nodos)
