# Milo (AndesRobot) — ROS 2 Humble + Gazebo Classic 11 sobre Ubuntu 22.04 (jammy)
#
# Un Dockerfile es la "receta" de la imagen: cada instrucción (FROM, RUN, COPY...) crea una capa.
# Docker guarda las capas en caché: si una línea no cambia, la reutiliza y no la vuelve a hacer.
# Por eso lo pesado (instalar paquetes) va primero y lo que cambia seguido va al final.
# (Los comentarios no cuentan para la caché: agregarlos no obliga a reconstruir.)

# Imagen base oficial de Open Robotics: Ubuntu 22.04 + ROS 2 Humble + RViz y herramientas de escritorio.
FROM osrf/ros:humble-desktop

# ARG = variables que solo existen mientras se construye la imagen.
# compose.yaml les pasa tu UID/GID reales (de lib.sh) para crear un usuario igual al tuyo.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# ENV = variable que queda en la imagen. "noninteractive" evita que apt haga preguntas
# (por ejemplo la zona horaria) y se quede esperando una respuesta que nadie va a dar.
ENV DEBIAN_FRONTEND=noninteractive

# Instalar todos los paquetes de ROS y herramientas. Las líneas que terminan en "\" son una sola
# instrucción partida en varias líneas. "&&" = hacer lo siguiente solo si lo anterior funcionó.
#   gazebo-ros-pkgs        Gazebo Classic 11 + plugins ROS (diff drive, lidar, IMU simulados)
#   xacro                  procesa los .xacro del URDF (macros, variables, includes)
#   robot-state-publisher  publica los TF del robot a partir del URDF + /joint_states
#   joint-state-publisher(-gui)  posiciones de articulaciones (con sliders) para display.launch.py
#   rviz2                  visualizador 3D
#   slam-toolbox           SLAM 2D: arma el mapa con el lidar + odometría
#   nav2-map-server        map_saver_cli: guarda el mapa en .pgm + .yaml
#   laser-filters          filtra el scan del C1 (saca la columna del robot)
#   teleop-twist-keyboard  manejar con el teclado
#   ros2-control / ros2-controllers  controller_manager + diff_drive_controller (Milo físico)
#   libboost-system-dev    librería que usa el driver del hoverboard para el puerto serie
#   git                    para clonar el driver del lidar
#   colcon                 la herramienta que compila el workspace
#   mesa-utils             glxinfo (diagnóstico de GPU: ./sim.sh gpu)
#   sudo, bash-completion, nano: comodidades dentro del contenedor
# "--no-install-recommends" = solo lo necesario (imagen más chica).
# "rm -rf /var/lib/apt/lists/*" borra los índices de apt descargados (no se necesitan después).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-rviz2 \
      ros-humble-slam-toolbox \
      ros-humble-nav2-map-server \
      ros-humble-laser-filters \
      ros-humble-teleop-twist-keyboard \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      libboost-system-dev \
      git \
      python3-colcon-common-extensions \
      mesa-utils \
      sudo bash-completion nano \
    && rm -rf /var/lib/apt/lists/*

# Driver del RPLIDAR C1 (Slamtec), fijado a una versión revisada.
# El driver del hoverboard NO se clona: va en ros2_ws/src/drivers con la calibración de Milo.
# Pasos: crear /opt/milo_drivers/src, clonar sllidar_ros2, fijarlo al commit 3430009 (así todos
# los PCs compilan exactamente la misma versión), cargar ROS ("." = source en sh), compilar solo
# ese paquete en modo Release (optimizado) y borrar los temporales de compilación.
# Queda instalado en /opt/milo_drivers/install (entrypoint.sh y .bashrc lo cargan).
RUN mkdir -p /opt/milo_drivers/src && cd /opt/milo_drivers/src \
 && git clone https://github.com/Slamtec/sllidar_ros2.git && git -C sllidar_ros2 checkout 3430009 \
 && cd /opt/milo_drivers && . /opt/ros/humble/setup.sh \
 && colcon build --packages-select sllidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf build log

# Usuario con tu mismo UID/GID (los archivos de build no quedan de root en tu PC)
# 1) Si ya existe un grupo con tu GID, se renombra a "ros"; si no, se crea.
# 2) Si ya existe un usuario con tu UID (la imagen base a veces trae uno), se renombra a "ros" y
#    se le cambia el home a /home/ros; si no, se crea con bash como terminal.
# 3) Se agrega a los grupos: video (GPU), sudo (administrador) y dialout (puertos serie /dev/tty*).
# 4) sudo sin contraseña dentro del contenedor (solo dentro del contenedor, no en tu PC).
RUN if getent group ${USER_GID} >/dev/null; then \
        groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1); \
    else groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && if getent passwd ${USER_UID} >/dev/null; then \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $(getent passwd ${USER_UID} | cut -d: -f1); \
    else useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; fi \
 && usermod -aG video,sudo,dialout ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# Copiar el script de arranque a la imagen y darle permiso de ejecución.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Desde aquí todo corre como el usuario "ros" (no root), parado en /ros2_ws
# (que compose.yaml conecta con milo_ws/ros2_ws de tu PC).
USER ${USERNAME}
WORKDIR /ros2_ws

# Configurar ~/.bashrc del usuario: cada terminal nueva dentro del contenedor carga ROS, Gazebo,
# el driver del lidar y nuestro workspace (si ya está compilado), y tiene dos atajos:
#   cb      = compilar el workspace y recargarlo
#   teleop  = teclado que publica en /cmd_vel_teleop (pasa por el filtro de seguridad)
# ~/.gazebo se crea vacía para que el volumen gazebo_cache quede con dueño "ros".
RUN mkdir -p ~/.gazebo \
 && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
 && echo 'source /opt/milo_drivers/install/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc \
 && echo "alias cb='cd /ros2_ws && colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc \
 && echo "alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'" >> ~/.bashrc

# Al arrancar el contenedor: correr entrypoint.sh (carga ROS) y después CMD.
# "sleep infinity" = no hacer nada y quedarse vivo; los scripts entran con "docker compose exec".
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
