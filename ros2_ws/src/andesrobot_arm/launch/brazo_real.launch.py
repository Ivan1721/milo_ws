"""Brazo REAL de Milo: ROS 2 + MoveIt moviendo el brazo de verdad por el Arduino Mega.

Es lo mismo que arm_sim.launch.py pero sin Gazebo:
  1. robot_state_publisher con el MISMO URDF que la simulación (las funciones de
     arm_sim.launch.py), para que las TF y MoveIt vean el mismo robot.
  2. brazo_real_node: le habla al Arduino (firmware/brazo_mega) por el USB y ofrece las mismas
     acciones que los controladores de Gazebo (/arm_controller/follow_joint_trajectory y
     /gripper_controller/...) y /joint_states.
  3. move_group (MoveIt 2) con la misma configuración, y arm_moveit_node con el grupo
     "brazo_sin_lift" (6 articulaciones): el Arduino no mueve el lift, así que MoveIt no lo usa.
  4. arm_marker_node (el marcador de RViz) y RViz.
  5. Con sliders:=true, sliders_node: una ventana con un slider por articulación para moverlas
     una por una (primeras pruebas con los motores; mejor con moveit:=false, para que no haya
     dos cosas mandando al brazo).

Antes: cargar firmware/brazo_mega/brazo_mega.ino en el Arduino y encenderlo con el brazo en la
pose "cero" de config/brazo_real.yaml (el Arduino empieza a contar desde ahí). Probar primero cada
articulación sola con firmware/probar_arduino.py.

Uso:
  ros2 launch andesrobot_arm brazo_real.launch.py                       # /dev/arduino_brazo
  ros2 launch andesrobot_arm brazo_real.launch.py puerto:=/dev/ttyUSB0 rviz:=false
  ros2 launch andesrobot_arm brazo_real.launch.py moveit:=false sliders:=true   # calibrar
  ros2 launch andesrobot_arm brazo_real.launch.py seta_linea:=144   # en la Jetson, con la seta
  (sin hardware: ros2 run andesrobot_arm arduino_falso, y puerto:=/tmp/arduino_falso)
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def arm_sim():
    """Carga arm_sim.launch.py como módulo para usar sus funciones robot_description_urdf y
    moveit_config (así el robot real y la simulación no pueden quedar distintos)."""
    ruta = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'arm_sim.launch.py')
    spec = importlib.util.spec_from_file_location('arm_sim_launch', ruta)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    return modulo


def launch_setup(context):
    sim = arm_sim()
    camara = LaunchConfiguration('camara').perform(context)
    urdf = sim.robot_description_urdf('false', camara)
    real = [{'use_sim_time': False}]
    nodos = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': urdf, 'use_sim_time': False}]),
        Node(package='andesrobot_arm', executable='brazo_real_node', output='screen',
             parameters=[{'puerto': LaunchConfiguration('puerto').perform(context),
                          'seta_linea': int(LaunchConfiguration('seta_linea').perform(context))}]),
        Node(package='andesrobot_arm', executable='arm_marker_node', output='screen',
             parameters=real),
    ]
    rviz_params = list(real)
    if LaunchConfiguration('moveit').perform(context) == 'true':
        config = sim.moveit_config('false', camara, 'false')
        nodos += [
            Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=[config.to_dict(), {'use_sim_time': False}]),
            # Sin mesa de prueba en la escena (en el robot real no hay una mesa fija conocida).
            # Velocidad y aceleración al 15 % de joint_limits.yaml (1 rad/s -> 0.15 rad/s): el
            # firmware da a lo más 2000 pasos/s, ~0.2 rad/s con 1/8 y 38.4:1; más rápido, el
            # brazo se atrasa respecto del plan de MoveIt.
            Node(package='andesrobot_arm', executable='arm_moveit_node', output='screen',
                 parameters=[{'use_sim_time': False, 'mesa': False,
                              'group': 'brazo_sin_lift', 'velocity_scaling': 0.15,
                              'acceleration_scaling': 0.15}]),
        ]
        rviz_params += [config.robot_description, config.robot_description_semantic,
                        config.robot_description_kinematics]
    if LaunchConfiguration('sliders').perform(context) == 'true':
        nodos.append(Node(package='andesrobot_arm', executable='sliders_node', output='screen',
                          parameters=real))
    nodos.append(Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('andesrobot_arm'), 'rviz',
                                      'arm_moveit.rviz')],
        parameters=rviz_params, condition=IfCondition(LaunchConfiguration('rviz'))))
    return nodos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('puerto', default_value='/dev/arduino_brazo',
                              description='puerto serie del Arduino del brazo'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='MoveIt (move_group + arm_moveit_node)'),
        DeclareLaunchArgument('camara', default_value='true',
                              description='la cámara de la pinza en el URDF (sus frames)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # La seta de emergencia por GPIO de la Jetson (pin 7 del header = línea 144, la misma de
        # la base y del driver de Xbox). -1 = sin seta (PC de desarrollo, Docker).
        DeclareLaunchArgument('seta_linea', default_value='-1',
                              description='línea GPIO de la seta (144 en la Jetson; -1 = sin)'),
        DeclareLaunchArgument('sliders', default_value='false',
                              description='ventana con un slider por articulación'),
        OpaqueFunction(function=launch_setup),
    ])
