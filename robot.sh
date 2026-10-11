#!/bin/bash
# Milo FÍSICO — en el computador de Milo (hoverboard + RPLIDAR C1 por USB).
#
#   ./robot.sh               levanta Milo: ruedas, lidar, filtro de seguridad y slam_toolbox
#   ./robot.sh teleop        (otra terminal) manejar a Milo con el teclado
#   ./robot.sh mapa [nombre] (otra terminal) guardar el mapa  (default: milo_AAAAMMDD_HHMM)
#   ./robot.sh rviz          abrir RViz aparte (en el portátil de Milo o en otro PC de la red)
#   ./robot.sh puertos       lista los puertos USB conectados (para docker/udev)
#   ./robot.sh shell         terminal dentro del contenedor
#   ./robot.sh build         recompilar el workspace
#   ./robot.sh stop          apagar el contenedor
#
# Puertos (por defecto /dev/rplidar y /dev/hoverboard, ver docker/udev/README.md). Sin udev:
#   LIDAR=/dev/ttyUSB0 HOVER=/dev/ttyUSB1 ./robot.sh
set -e
# (Las líneas 2-14 de arriba son la ayuda que imprime "sed -n '2,14p'" si escribes una opción
#  que no existe. Los comentarios explicativos van desde aquí.)
#
# "set -e": si un comando falla, el script se detiene.
# Ir a la carpeta de este script (milo_ws) y cargar las funciones comunes.
cd "$(dirname "$0")"
source docker/lib.sh

# Puertos serie. ${LIDAR:-/dev/rplidar} = usa la variable LIDAR si la escribiste antes del comando
# (LIDAR=/dev/ttyUSB0 ./robot.sh); si no, /dev/rplidar (nombre fijo que crean las reglas udev).
LIDAR="${LIDAR:-/dev/rplidar}"
HOVER="${HOVER:-/dev/hoverboard}"
# RViz junto con Milo: sí si hay pantalla (portátil de Milo), no si se corre sin pantalla (SSH).
# Se puede forzar: RVIZ=false ./robot.sh
if [ -n "$DISPLAY" ]; then RVIZ="${RVIZ:-true}"; else RVIZ="${RVIZ:-false}"; fi

# Primer "case": opciones que NO necesitan el contenedor en modo robot.
case "$1" in
  puertos)
    # Recorre todos los puertos serie USB conectados (ttyUSB = adaptadores tipo CP2102/CH340,
    # ttyACM = placas tipo Arduino) y muestra sus datos de fábrica para las reglas udev.
    for d in /dev/ttyUSB* /dev/ttyACM*; do
      # Si no hay ninguno, el patrón queda literal ("/dev/ttyUSB*"): "-e" lo descarta.
      [ -e "$d" ] || continue
      echo "== $d"
      # udevadm muestra los atributos del dispositivo; grep -m3 se queda con los 3 primeros
      # idVendor / idProduct / serial (los que se copian en docker/udev/99-milo.rules).
      udevadm info -a -n "$d" | grep -m3 -E 'ATTRS\{(idVendor|idProduct|serial)\}'
    done
    # Si las reglas udev ya están instaladas, se ven los enlaces fijos.
    ls -l /dev/rplidar /dev/hoverboard 2>/dev/null || true
    exit 0 ;;
  rviz)
    # Si el contenedor ya está corriendo (Milo andando en este mismo portátil): abrir RViz
    # DENTRO de ese contenedor, sin reiniciarlo.
    milo_init robot
    if milo_running; then
      milo_display
      exec $DC exec ros bash -ic 'ros2 launch andesrobot_bringup rviz_mapping.launch.py'
    fi
    # Si no (otro PC de la red): contenedor normal (sin USB), solo RViz.
    # Se conecta por la red a los topics que publica Milo (misma red y mismo ROS_DOMAIN_ID).
    milo_init sim
    milo_up
    exec $DC exec ros bash -ic 'ros2 launch andesrobot_bringup rviz_mapping.launch.py' ;;
esac

# Desde aquí: modo robot (docker compose suma compose.robot.yaml = acceso a los USB).
milo_init robot
case "$1" in
  # Mismas opciones que en sim.sh (ver comentarios allá).
  teleop) milo_need_running; exec $DC exec ros bash -ic "$TELEOP_CMD" ;;
  mapa)   milo_save_map "${2:-milo_$(date +%Y%m%d_%H%M)}"; exit 0 ;;
  shell)  milo_need_running; exec $DC exec ros bash ;;
  build)  milo_need_running; exec $DC exec ros bash -ic 'cd /ros2_ws && colcon build --symlink-install' ;;
  stop)   $DC down; exit 0 ;;
  # Sin palabra: sigue abajo a levantar a Milo.
  "") ;;
  # Opción desconocida: mostrar ayuda.
  *) sed -n '2,14p' "$0"; exit 1 ;;
esac

# Antes de arrancar, revisar que el lidar y el hoverboard estén conectados.
# Si falta alguno, es mejor no arrancar a medias (sin lidar el filtro no deja moverse;
# sin hoverboard ros2_control falla).
for p in "$LIDAR" "$HOVER"; do
  [ -e "$p" ] || { echo "[milo] No existe $p. Revisa la conexión, ./robot.sh puertos, o usa LIDAR=... HOVER=... ./robot.sh"; exit 1; }
done

# Levantar el contenedor (con USB) y compilar si es la primera vez.
milo_up
echo
echo "[milo] Milo arrancando (lidar $LIDAR, hoverboard $HOVER). En otra terminal:  ./robot.sh teleop"
echo "[milo] Guardar el mapa:  ./robot.sh mapa <nombre>      (RViz: $RVIZ)"
echo
# Lanzar a Milo (ver andesrobot_bringup/launch/robot_mapping.launch.py) pasándole los puertos
# y si abrir RViz.
exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup robot_mapping.launch.py lidar_port:=$LIDAR hoverboard_port:=$HOVER rviz:=$RVIZ"
