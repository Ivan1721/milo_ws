"""Cámara real de la pinza (Orbbec Gemini Plus) con los mismos nombres que en la simulación.

Lanza el driver orbbec_camera (instalado en la imagen de Docker, ver docker/camara.repos) con:
  camera_name:=gripper_camera  topics /gripper_camera/{color,depth,ir}/image_raw y
                               /gripper_camera/depth/points, y frames
                               gripper_camera_<canal>_optical_frame, iguales a los del xacro.
  publish_tf:=false            esos frames los publica robot_state_publisher desde el URDF; si
                               también los publicara el driver, un frame tendría dos padres.

Sin robot_state_publisher no hay árbol de TF: en RViz, usar como Fixed Frame
gripper_camera_depth_optical_frame (o gripper_camera_color_optical_frame con alineada:=true).

Uso (en el PC, con la cámara conectada y docker/udev/99-gemini.rules instalada):
  ./robot.sh camara                   profundidad de 640x400 con su propia lente
  ./robot.sh camara alineada:=true    profundidad y nube pasadas al punto de vista de la lente
                                      de color (640x480): un píxel de color y uno de profundidad
                                      son el mismo punto. Sirve para medir la distancia de lo que
                                      detecta YOLO en la imagen a color.
                                      En este modo no usar ir/camera_info (ver docker/parches/).
  ./robot.sh camara rviz:=true        además abre RViz con los tres canales en Best Effort
                                      (rviz/camara_real.rviz).
  ./robot.sh camara alineada:=true deteccion:=true
                                      además detección de objetos EN VIVO (deteccion_node, YOLOE)
                                      y una ventana (rqt_image_view) con la imagen con las cajas y
                                      la distancia de cada objeto (/arm/deteccion/imagen).
                                      clases:=can,cup cambia lo que busca (sin espacios: robot.sh
                                      corta en los espacios). modelo:=objetos (el del equipo,
                                      modelos/objetos/best.pt, con sus propias clases) o la ruta
                                      de otro .pt en vez de YOLOE.
Guardar una imagen de cada canal (en otra terminal, ./robot.sh shell):
  ros2 run andesrobot_arm capturar_camara
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from andesrobot_arm.deteccion_node import CLASES


def generate_launch_description():
    # gemini.launch.xml viene con el driver y trae los modos de esta cámara: color 640x480 MJPG,
    # profundidad 640x400 Y11 e IR 640x400 Y10, todos a 30 Hz.
    driver = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch', 'gemini.launch.xml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'alineada', default_value='false',
            description='true: profundidad y nube en el punto de vista de la lente de color'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(driver),
            launch_arguments={
                'camera_name': 'gripper_camera',
                'publish_tf': 'false',
                'depth_registration': LaunchConfiguration('alineada'),
            }.items()),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true: abre RViz con los tres canales (rviz/camara_real.rviz)'),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('andesrobot_arm'), 'rviz', 'camara_real.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))),
        DeclareLaunchArgument(
            'deteccion', default_value='false',
            description='true: deteccion_node (YOLOE en vivo) + ventana con las cajas'),
        DeclareLaunchArgument(
            'modelo', default_value='/ros2_ws/modelos/yoloe-11s-seg.pt',
            description='YOLOE (por defecto), "objetos" = el modelo del equipo, o una ruta'),
        DeclareLaunchArgument(
            'clases', default_value=CLASES,
            description='lo que busca YOLOE, en inglés, separado por comas'),
        Node(
            package='andesrobot_arm', executable='deteccion_node', output='screen',
            parameters=[{'clases': LaunchConfiguration('clases'),
                         'modelo': LaunchConfiguration('modelo')}],
            condition=IfCondition(LaunchConfiguration('deteccion'))),
        Node(
            package='rqt_image_view', executable='rqt_image_view', output='screen',
            arguments=['/arm/deteccion/imagen'],
            condition=IfCondition(LaunchConfiguration('deteccion'))),
    ])
