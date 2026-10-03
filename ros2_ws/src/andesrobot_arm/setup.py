# setup.py: instrucciones para instalar este paquete Python de ROS 2 (lo usa "colcon build").
import os
from glob import glob

from setuptools import setup

package_name = 'andesrobot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # Archivos que no son código Python y se copian a install/<paquete>/share/<paquete>/.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan1721',
    maintainer_email='garciaivan172@gmail.com',
    description='Brazo de Milo: cinemática, control en Gazebo y marcador interactivo.',
    license='Apache-2.0',
    # ros2 run andesrobot_arm <nombre>
    entry_points={'console_scripts': [
        'arm_ik_node = andesrobot_arm.arm_ik_node:main',
        'arm_marker_node = andesrobot_arm.arm_marker_node:main',
    ]},
)
