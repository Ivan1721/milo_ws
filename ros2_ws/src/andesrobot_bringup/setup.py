# setup.py: instrucciones para instalar este paquete Python de ROS 2 (lo usa "colcon build").
# os/glob = utilidades para armar rutas y buscar archivos.
import os
from glob import glob

# setuptools = la herramienta estándar de Python para instalar paquetes.
from setuptools import setup

# Nombre del paquete (igual que la carpeta y que <name> en package.xml).
package_name = 'andesrobot_bringup'

setup(
    # Datos básicos del paquete.
    name=package_name,
    version='0.1.0',
    # Carpeta con el código Python del paquete (la que tiene __init__.py).
    packages=[package_name],
    # Archivos que NO son código Python y se copian a install/<paquete>/share/<paquete>/:
    # (carpeta_destino, [lista de archivos]). glob('launch/*.py') = todos los .py de launch/.
    data_files=[
        # Marcador para que ROS encuentre el paquete (ros2 pkg list, FindPackageShare).
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # El package.xml (dependencias y metadatos).
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # Dependencias de Python para instalar (solo setuptools).
    install_requires=['setuptools'],
    # Se puede instalar comprimido (valor estándar en ROS 2).
    zip_safe=True,
    maintainer='Aleja',
    maintainer_email='m.pinzonfarfan@uandresbello.edu',
    description='Launch de alto nivel de AndesRobot (simulación + mapeo).',
    license='Apache-2.0',
    # Ejecutables: 'nombre = modulo:funcion' crea el comando que se usa con
    # ros2 run <paquete> <nombre>. Vacío = este paquete no tiene programas propios.
    entry_points={'console_scripts': []},
)
