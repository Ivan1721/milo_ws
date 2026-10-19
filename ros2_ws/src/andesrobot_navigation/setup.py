# setup.py del paquete de navegación (mismo formato que andesrobot_slam).
import os
from glob import glob

from setuptools import setup

package_name = 'andesrobot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aleja',
    maintainer_email='m.pinzonfarfan@uandresbello.edu',
    description='Navegación autónoma (Nav2) para AndesRobot / Milo.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        # ros2 run andesrobot_navigation ir_a kitchen
        'ir_a = andesrobot_navigation.ir_a:main',
        # publica las ubicaciones como marcadores en RViz
        'marcadores = andesrobot_navigation.marcadores:main',
    ]},
)
