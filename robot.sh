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
#   ./robot.sh nav <mapa>    navegación autónoma (AMCL + Nav2) con un mapa ya guardado (SIN PROBAR)
#   ./robot.sh ir <lugar>    (otra terminal) mandar a Milo a una ubicación (ver ./robot.sh ir --lista)
#   ./robot.sh camara [alineada:=true]   solo la cámara de la pinza (no necesita lidar ni ruedas)
#
# Puertos (por defecto /dev/rplidar y /dev/hoverboard, ver docker/udev/README.md). Sin udev:
#   LIDAR=/dev/ttyUSB0 HOVER=/dev/ttyUSB1 ./robot.sh
set -e
# (Las líneas 2-17 de arriba son la ayuda que imprime "sed -n '2,17p'" si escribes una opción
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
  # Navegación (ver sim.sh): ir a ubicaciones con nombre / levantar Nav2 con un mapa guardado.
  ir)     milo_need_running; shift; exec $DC exec ros bash -ic "ros2 run andesrobot_navigation ir_a $*" ;;
  nav)    [ -f "ros2_ws/src/andesrobot_slam/maps/$2.yaml" ] || { echo "[milo] Uso: ./robot.sh nav <mapa>  (no existe ros2_ws/src/andesrobot_slam/maps/$2.yaml)"; exit 1; } ;;
  camara)
    # Antes de levantar nada, revisar la cámara. lsusb -d filtra por vendedor:producto y da
    # "Bus 003 Device 006: ...": sed lo convierte en /dev/bus/usb/003/006, el archivo que abre el driver.
    camara_usb="$(lsusb -d 2bc5:0614 | sed -E 's#^Bus ([0-9]+) Device ([0-9]+).*#/dev/bus/usb/\1/\2#')"
    [ -n "$camara_usb" ] || { echo "[milo] No encuentro la Gemini (2bc5:0614). Revisa el cable USB."; exit 1; }
    # -w = "puedo escribir": sin la regla udev solo puede root, y el driver fallaría con OB_USB_STATUS_ACCESS.
    [ -w "$camara_usb" ] || { echo "[milo] Sin permiso sobre $camara_usb. Instala la regla: ver docker/udev/README.md"; exit 1; }
    milo_up
    # shift quita la palabra "camara"; lo que queda (alineada:=true) va al launch.
    shift
    exec $DC exec ros bash -ic "ros2 launch andesrobot_arm camara_real.launch.py $*" ;;
  # Sin palabra: sigue abajo a levantar a Milo.
  "") ;;
  # Opción desconocida: mostrar ayuda.
  *) sed -n '2,17p' "$0"; exit 1 ;;
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
# Navegación autónoma: AMCL + Nav2 sobre el mapa guardado (robot_navigation.launch.py).
if [ "$1" = "nav" ]; then
  echo "[milo] Navegación con el mapa $2. Otra terminal: ./robot.sh ir --lista / ./robot.sh ir <lugar>"
  exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup robot_navigation.launch.py map:=/ros2_ws/src/andesrobot_slam/maps/$2.yaml lidar_port:=$LIDAR hoverboard_port:=$HOVER rviz:=$RVIZ"
fi
# Lanzar a Milo (ver andesrobot_bringup/launch/robot_mapping.launch.py) pasándole los puertos
# y si abrir RViz.
exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup robot_mapping.launch.py lidar_port:=$LIDAR hoverboard_port:=$HOVER rviz:=$RVIZ"
