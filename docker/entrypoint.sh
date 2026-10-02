#!/bin/bash
# Se ejecuta cada vez que arranca el contenedor (ENTRYPOINT del Dockerfile).
# Carga los entornos de ROS y después ejecuta el comando principal (CMD = "sleep infinity",
# que deja el contenedor vivo esperando a que entres con "docker compose exec").
#
# Si algo falla, detenerse.
set -e
# ROS 2 Humble (comandos ros2, librerías, paquetes instalados con apt).
source /opt/ros/humble/setup.bash
# Gazebo Classic (rutas de modelos y plugins).
source /usr/share/gazebo/setup.sh
# Driver del RPLIDAR C1 (sllidar_ros2), compilado dentro de la imagen.
source /opt/milo_drivers/install/setup.bash
# Nuestro workspace, solo si ya se compiló alguna vez.
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
# Ejecutar el comando que venía (los argumentos "$@"), reemplazando a este script.
exec "$@"
