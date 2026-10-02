"""robot_state_publisher de AndesRobot (lo reutilizan display y la simulación).

CONCEPTOS BÁSICOS DE UN LAUNCH (aplican a todos los .launch.py del workspace):
- Un archivo launch es Python que DESCRIBE qué programas (nodos) arrancar y con qué parámetros.
  ROS llama a generate_launch_description() y ejecuta lo que devuelve.
- Node(...)                       = un programa ROS a ejecutar (paquete + ejecutable).
- DeclareLaunchArgument(...)      = un argumento que se puede pasar desde afuera:
                                    ros2 launch paquete archivo.launch.py nombre:=valor
- LaunchConfiguration('nombre')   = "el valor que tenga ese argumento". No es un string todavía:
                                    se resuelve recién cuando el launch se ejecuta.
- PathJoinSubstitution / FindPackageShare = construir rutas a archivos instalados de un paquete
                                    (install/<paquete>/share/<paquete>/...).
- Command([...])                  = ejecutar un comando de terminal y usar lo que imprime.
- IncludeLaunchDescription        = incluir otro launch (como llamar a una función).

ROBOT_STATE_PUBLISHER: lee el URDF (la descripción del robot: piezas, medidas, articulaciones)
y publica:
  /robot_description  -> el URDF como texto (lo usan RViz y Gazebo)
  /tf, /tf_static     -> dónde está cada pieza respecto a las otras (las transformadas, "TF")
Para las articulaciones móviles (ruedas, brazo) necesita sus ángulos, que llegan por /joint_states.
"""
# Importar las piezas de la librería de launch de ROS 2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # "Variables" que toman el valor de cada argumento declarado más abajo.
    # use_sim_time: true = usar el reloj de Gazebo (/clock) en vez del reloj del computador.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Opciones del URDF (ver andesrobot.urdf.xacro):
    lock_arm = LaunchConfiguration('lock_arm')                  # brazo fijo
    simple_collision = LaunchConfiguration('simple_collision')  # colisiones con cajas
    sim_lidar = LaunchConfiguration('sim_lidar')                # lidar simulado sí/no
    sim_imu = LaunchConfiguration('sim_imu')                    # IMU simulada sí/no
    use_hardware = LaunchConfiguration('use_hardware')          # ros2_control del hoverboard
    hoverboard_port = LaunchConfiguration('hoverboard_port')    # puerto serie del hoverboard

    # Ruta al URDF principal: install/andesrobot_description/share/andesrobot_description/urdf/...
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('andesrobot_description'), 'urdf', 'andesrobot.urdf.xacro'])
    # Ejecutar "xacro andesrobot.urdf.xacro lock_arm:=... ..." : xacro expande las macros y
    # devuelve el URDF final como texto. ParameterValue(..., value_type=str) le dice a ROS que
    # el resultado es un string (si no, intentaría interpretarlo como YAML y fallaría).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' lock_arm:=', lock_arm,
                 ' simple_collision:=', simple_collision,
                 ' sim_lidar:=', sim_lidar,
                 ' sim_imu:=', sim_imu,
                 ' use_hardware:=', use_hardware,
                 ' hoverboard_port:=', hoverboard_port]), value_type=str)

    # Lo que devuelve la función: la lista de cosas que el launch va a hacer, en orden.
    return LaunchDescription([
        # Declarar cada argumento con su valor por defecto (si nadie pasa otro).
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lock_arm', default_value='false',
                              description='true = brazo/lift/dedos fijos (simulación de base)'),
        DeclareLaunchArgument('simple_collision', default_value='false',
                              description='true = colisiones primitivas en vez de mallas CAD'),
        DeclareLaunchArgument('sim_lidar', default_value='true'),
        DeclareLaunchArgument('sim_imu', default_value='true'),
        DeclareLaunchArgument('use_hardware', default_value='false',
                              description='true = agrega ros2_control del hoverboard (Milo físico)'),
        DeclareLaunchArgument('hoverboard_port', default_value='/dev/hoverboard'),
        # El nodo robot_state_publisher con el URDF generado como parámetro.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # output='screen' = mostrar sus mensajes en la terminal.
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),
    ])
