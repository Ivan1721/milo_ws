# Imagen mínima para medir la deriva de la base: Gazebo 11 de los repositorios de Ubuntu 22.04
# (la misma versión que usa milo:humble) sobre la imagen base de Humble. No trae
# gazebo_ros_pkgs ni ros2_control: deriva.cc hace su trabajo sin ROS. Así se puede correr aunque
# packages.ros.org no esté disponible (por ejemplo, en un entorno con la red restringida).
FROM ros:humble-ros-base
RUN rm -f /etc/apt/sources.list.d/ros*.list && apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends gazebo libgazebo-dev \
    cmake g++ python3-numpy && rm -rf /var/lib/apt/lists/*
