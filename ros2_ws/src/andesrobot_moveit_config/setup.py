# setup.py: instala los archivos de configuración de MoveIt en install/<paquete>/share/<paquete>/config,
# que es donde los busca MoveItConfigsBuilder (ver andesrobot_arm/launch/arm_sim.launch.py).
import os
from glob import glob

from setuptools import setup

package_name = 'andesrobot_moveit_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan1721',
    maintainer_email='garciaivan172@gmail.com',
    description='Configuración de MoveIt 2 para el brazo de Milo.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
