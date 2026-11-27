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
#   ./robot.sh camara [alineada:=true] [deteccion:=true]   solo la cámara de la pinza (+ detección YOLOE en vivo)
#   ./robot.sh brazo [rviz:=false] [moveit:=false sliders:=true]   brazo real: Arduino Mega + MoveIt o sliders
#   ./robot.sh brazo-pc      TODO el brazo: driver en la Orin por IP + MoveIt, sliders, grabar/reproducir y cámara
#   ./robot.sh imagen        reconstruir la imagen de Docker (después de cambiar algo en docker/)
#
# Puertos (por defecto /dev/rplidar y /dev/hoverboard, ver docker/udev/README.md). Sin udev:
#   LIDAR=/dev/ttyUSB0 HOVER=/dev/ttyUSB1 ./robot.sh
set -e
# (Las líneas 2-20 de arriba son la ayuda que imprime "sed -n '2,20p'" si escribes una opción
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
# Arduino Mega del brazo (firmware/brazo_mega): /dev/arduino_brazo con la regla udev.
ARDUINO="${ARDUINO:-/dev/arduino_brazo}"
# Dominio de ROS 2 del robot real: 7, el mismo que usa la Jetson (Orin) en su ~/.bashrc. Así el
# contenedor de este PC y la Orin se ven por el cable. (Antes era 0 si no se escribía; y como
# docker compose recrea el contenedor cuando cambia, mezclar comandos con y sin la variable
# cerraba lo que estuviera corriendo.) Se puede cambiar: ROS_DOMAIN_ID=3 ./robot.sh ...
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
# La Orin por el cable de red (PC 10.43.0.1, Orin 10.43.0.146; ver firmware/README.md, paso 10).
ORIN="${ORIN:-10.43.0.146}"
# Puerto del Arduino del brazo EN LA ORIN: por id, no cambia con el orden de conexión.
ORIN_PUERTO="${ORIN_PUERTO:-/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0}"
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
    ls -l /dev/rplidar /dev/hoverboard /dev/arduino_brazo 2>/dev/null || true
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
  imagen) milo_imagen; exit 0 ;;
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
  brazo)
    # Brazo real: el Arduino del brazo + robot_state_publisher + MoveIt (brazo_real.launch.py).
    # No necesita lidar ni ruedas. Antes: firmware cargado y el brazo en su pose cero (ver
    # firmware/README.md). Lo que venga después de "brazo" (rviz:=false) va al launch.
    [ -e "$ARDUINO" ] || { echo "[milo] No existe $ARDUINO. Conecta el Arduino, ./robot.sh puertos, o ARDUINO=/dev/ttyUSB0 ./robot.sh brazo"; exit 1; }
    milo_up
    shift
    exec $DC exec ros bash -ic "ros2 launch andesrobot_arm brazo_real.launch.py puerto:=$ARDUINO rviz:=$RVIZ $*" ;;
  brazo-pc)
    # TODO el brazo real con un solo comando, en el PC (Docker), con la Orin por el cable:
    #   1. Si la Orin contesta en $ORIN, arranca ahí el driver del brazo (brazo_real_node con el
    #      Arduino y la seta) por SSH, en segundo plano, y lo apaga al salir. Si ya estaba
    #      corriendo lo deja; si el Arduino lo tiene otro programa (el control de Xbox), avisa y
    #      no lo toca. Sin la Orin sigue igual (RViz y la cámara sirven sin el brazo).
    #   2. Aquí: robot_state_publisher, MoveIt + RViz (marcador), los sliders, la ventana de
    #      grabar y reproducir, y, si la cámara de la pinza está enchufada a este PC, la cámara
    #      con la detección del modelo del equipo y una ventana para ver sus canales.
    # Antes: el brazo en la pose de arranque y el Arduino en 0 (Y en el control; paso 9 de
    # firmware/README.md). Lo que venga después de "brazo-pc" va al launch (moveit:=false...).
    milo_up
    shift
    orin_driver=0
    if ping -c 1 -W 1 "$ORIN" >/dev/null 2>&1; then
      # Una sola línea de bash en la Orin: si nuestro driver ya corre, nada; si el puerto lo tiene
      # otro programa, avisar; si no, arrancarlo desacoplado (setsid nohup) con log.
      salida=$(ssh -o BatchMode=yes -o ConnectTimeout=4 "milo@$ORIN" "
        if pgrep -f '[b]razo_real_node' >/dev/null; then echo YA;
        elif fuser '$ORIN_PUERTO' >/dev/null 2>&1; then echo OCUPADO;
        else source /opt/ros/humble/setup.bash && source ~/andes_ws/install/setup.bash &&
             export ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=0 &&
             setsid nohup ros2 launch andesrobot_arm brazo_real.launch.py parte:=driver \
               seta_linea:=144 puerto:=$ORIN_PUERTO > ~/andes_ws/driver.log 2>&1 < /dev/null &
             echo ARRANCADO; fi" 2>&1) || salida="SSH: $salida"
      case "$salida" in
        *ARRANCADO*) orin_driver=1; echo "[milo] Driver del brazo arrancado en la Orin ($ORIN), log en ~/andes_ws/driver.log" ;;
        *YA*)        echo "[milo] El driver del brazo ya estaba corriendo en la Orin." ;;
        *OCUPADO*)   echo "[milo] OJO: en la Orin el Arduino lo tiene otro programa (¿el control de Xbox?). Ciérralo y vuelve a correr esto." ;;
        *)           echo "[milo] No pude arrancar el driver en la Orin: $salida" ;;
      esac
    else
      echo "[milo] La Orin ($ORIN) no contesta por el cable: sigo solo con el PC (sin el brazo)."
    fi
    # Al salir (Ctrl+C), apagar el driver de la Orin si lo arrancamos nosotros.
    if [ "$orin_driver" = 1 ]; then
      trap 'ssh -o BatchMode=yes "milo@$ORIN" "pkill -INT -f \"[b]razo_real.launch.py\"" && echo "[milo] Driver de la Orin apagado."' EXIT
    fi
    # La cámara de la pinza enchufada a este PC (y con permiso, regla udev): con detección.
    camara_usb=false
    camara_dev="$(lsusb -d 2bc5:0614 | sed -E 's#^Bus ([0-9]+) Device ([0-9]+).*#/dev/bus/usb/\1/\2#')"
    if [ -n "$camara_dev" ] && [ -w "$camara_dev" ]; then camara_usb=true
    else echo "[milo] La cámara de la pinza no está enchufada a este PC: sigo sin cámara."; fi
    $DC exec ros bash -ic "ros2 launch andesrobot_arm brazo_real.launch.py parte:=pc rviz:=$RVIZ sliders:=true rutina:=true camara_usb:=$camara_usb $*" ;;
  # Sin palabra: sigue abajo a levantar a Milo.
  "") ;;
  # Opción desconocida: mostrar ayuda.
  *) sed -n '2,20p' "$0"; exit 1 ;;
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
