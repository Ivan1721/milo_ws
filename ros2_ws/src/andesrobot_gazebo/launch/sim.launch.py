"""AndesRobot en Gazebo Classic 11: mundo + robot_state_publisher + spawn.

Expone: /scan, /cmd_vel, /odom, /imu, /joint_states, TF odom->base_footprint.

Orden de lo que pasa:
 1. Gazebo abre el mundo (la arena).
 2. robot_state_publisher publica el URDF en /robot_description (versión simulación).
 3. spawn_entity.py lee ese URDF y "pone" a Milo dentro de Gazebo.
 4. Los plugins del URDF (andesrobot.sim.xacro) empiezan a publicar lidar, odometría, IMU.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Carpetas instaladas de los paquetes que usamos.
    pkg_gazebo = FindPackageShare('andesrobot_gazebo')
    pkg_desc = FindPackageShare('andesrobot_description')

    # Valores de los argumentos (declarados al final).
    world = LaunchConfiguration('world')        # qué mundo abrir
    gui = LaunchConfiguration('gui')            # abrir o no la ventana de Gazebo
    verbose = LaunchConfiguration('verbose')    # mostrar mensajes de Gazebo
    # Posición inicial de Milo en el mundo (metros y radianes).
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    # Gazebo: incluir el launch oficial de gazebo_ros, que arranca gzserver (la física) y
    # gzclient (la ventana) y carga los plugins que conectan Gazebo con ROS.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world, 'gui': gui, 'verbose': verbose}.items(),
    )

    # robot_state_publisher en modo simulación:
    #   use_sim_time=true -> usa el reloj de Gazebo
    #   lock_arm=true     -> brazo/lift/dedos fijos (sin motores en la sim se caerían)
    #   simple_collision  -> colisiones con cajas (más estable que las mallas CAD)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_desc, 'launch', 'rsp.launch.py'])),
        launch_arguments={'use_sim_time': 'true',
                          'lock_arm': 'true',
                          'simple_collision': LaunchConfiguration('simple_collision'),
                          'sim_lidar': LaunchConfiguration('lidar'),
                          'sim_imu': LaunchConfiguration('imu')}.items(),
    )

    # spawn_entity.py: toma el URDF del topic robot_description y crea el robot en Gazebo
    # con el nombre "andesrobot", en (x, y), 2 cm sobre el suelo (para que caiga suave) y
    # girado "yaw" radianes.
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'andesrobot',
                   '-x', x, '-y', y, '-z', '0.02', '-Y', yaw],
        output='screen',
    )

    return LaunchDescription([
        # Por defecto: la arena de mapeo de este paquete.
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_gazebo, 'worlds', 'andesrobot_arena.world']),
            description='Ruta al .world'),
        DeclareLaunchArgument('gui', default_value='true', description='Abrir gzclient'),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Logs de Gazebo ([Err]/[Wrn]) en la terminal'),
        DeclareLaunchArgument('simple_collision', default_value='true',
                              description='false = usar mallas CAD como colisión (más pesado)'),
        # lidar:=false / imu:=false sirven para depurar si Gazebo se cae.
        DeclareLaunchArgument('lidar', default_value='true', description='Plugin RPLIDAR C1'),
        DeclareLaunchArgument('imu', default_value='true', description='Plugin IMU'),
        # Pose inicial: el centro de la arena (0, 0) mirando hacia +X.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        rsp,
        spawn,
    ])
