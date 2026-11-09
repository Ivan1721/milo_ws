"""Visualiza AndesRobot en RViz con sliders para las articulaciones.

Uso:  ros2 launch andesrobot_description display.launch.py
Sirve para revisar el modelo (URDF) sin Gazebo: mueves cada articulación con un slider y
ves en RViz cómo se mueve el robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpeta instalada de este paquete (para encontrar launch/ y rviz/).
    pkg = FindPackageShare('andesrobot_description')
    # Argumento "gui": true = ventana con sliders.
    gui = LaunchConfiguration('gui')

    # Incluir rsp.launch.py (robot_state_publisher). lock_arm=false: el brazo se puede mover.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'false', 'lock_arm': 'false'}.items(),
    )

    # Pose inicial de los sliders = POSE PLEGADA (la misma que queda fija con lock_arm:=true):
    # brazo recogido dentro de la huella de la base y carro abajo del todo. Sin esto todo parte
    # en 0 (brazo estirado).
    pose_plegada = {'zeros': {
        'vertical_lift_joint': -0.40,
        'joint_1': 0.2775, 'joint_2': 0.5965, 'joint_3': 2.4000,
        'joint_4': -1.0642, 'joint_5': 1.8334, 'joint_6': 0.8381,
    }}

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Sliders de joint_state_publisher_gui'),
        rsp,
        # Con gui=true: ventana con un slider por articulación; publica /joint_states.
        # IfCondition = este nodo solo arranca si la condición es verdadera.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[pose_plegada],
             condition=IfCondition(gui)),
        # Con gui=false: publica las articulaciones en la pose plegada, sin ventana.
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[pose_plegada],
             condition=UnlessCondition(gui)),
        # RViz con la configuración guardada en rviz/display.rviz ("-d" = cargar ese archivo).
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
