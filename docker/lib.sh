#!/bin/bash
# Funciones comunes de sim.sh y robot.sh (no se ejecuta solo).
# Uso: source docker/lib.sh; milo_init <sim|robot> [cpu]
#
# "source" significa que sim.sh / robot.sh cargan estas funciones dentro de su propio proceso,
# como si las hubieran escrito ellos. Así no repetimos el mismo código en los dos scripts.

# ---------------------------------------------------------------------------------------------
# milo_init: prepara las variables que necesita "docker compose".
#   $1 = modo: "sim" (tu PC) o "robot" (computador de Milo, con acceso a los USB)
#   $2 = opción extra: "cpu" (no usar NVIDIA) o "software" (dibujar con la CPU)
# Al final deja lista la variable DC = el comando docker compose con los archivos correctos.
# ---------------------------------------------------------------------------------------------
milo_init() {
  # "local" = variables que solo existen dentro de esta función.
  local mode="$1" force_cpu="$2"
  # UID/GID = tu número de usuario y de grupo en el PC. compose.yaml se los pasa al Dockerfile
  # para crear dentro del contenedor un usuario igual a ti: así lo que se compila en ros2_ws
  # queda a tu nombre y no de "root". (UID ya existe en bash; "export" lo hace visible a docker.)
  export UID GID="$(id -g)"
  # GIDs del host para que el contenedor pueda usar la GPU (/dev/dri)
  # getent group video -> "video:x:44:..." ; cut -d: -f3 se queda con el 3er campo = el número 44.
  export MILO_VIDEO_GID="$(getent group video | cut -d: -f3)"
  export MILO_RENDER_GID="$(getent group render | cut -d: -f3)"
  # ":" es un comando que no hace nada; se usa para el truco ${VAR:=valor}, que asigna un valor
  # por defecto si la variable quedó vacía (por ejemplo, un PC sin grupo "render").
  : "${MILO_VIDEO_GID:=44}"
  # Docker no acepta el mismo grupo dos veces en group_add: si no hay grupo "render" o tiene el
  # mismo número que "video", usamos el de "dialout" (puertos serie), que es inofensivo.
  if [ -z "$MILO_RENDER_GID" ] || [ "$MILO_RENDER_GID" = "$MILO_VIDEO_GID" ]; then
    MILO_RENDER_GID="$(getent group dialout | cut -d: -f3)"; MILO_RENDER_GID="${MILO_RENDER_GID:-20}"
  fi
  # Modo "software": compose.yaml pasa esta variable como LIBGL_ALWAYS_SOFTWARE=1 (OpenGL por CPU).
  [ "$force_cpu" = "software" ] && export MILO_SOFTWARE_GL=1
  # Archivo base de compose, siempre.
  MILO_FILES="-f docker/compose.yaml"
  # En el robot se suma compose.robot.yaml (acceso a /dev, o sea a los USB del lidar y hoverboard).
  [ "$mode" = "robot" ] && MILO_FILES="$MILO_FILES -f docker/compose.robot.yaml"
  # Si el PC tiene NVIDIA (existe nvidia-smi) y Docker tiene el runtime de NVIDIA instalado,
  # se suma compose.nvidia.yaml para que el contenedor use esa GPU. Con "cpu" o "software" no.
  if [ "$force_cpu" != "cpu" ] && [ "$force_cpu" != "software" ] && command -v nvidia-smi >/dev/null && docker info 2>/dev/null | grep -qi nvidia; then
    MILO_FILES="$MILO_FILES -f docker/compose.nvidia.yaml"
  fi
  # Comando final que usan todas las demás funciones, por ejemplo: $DC up -d
  DC="docker compose $MILO_FILES"
}

# Permite que el contenedor abra ventanas (RViz, Gazebo). Sin pantalla, no hace nada.
milo_display() {
  # -n = "no está vacío". Si DISPLAY está vacío (computador sin pantalla), salimos sin error.
  [ -n "$DISPLAY" ] || return 0
  if command -v xhost >/dev/null; then
    # xhost controla quién puede dibujar ventanas en tu pantalla. Damos permiso a tu usuario
    # y a los procesos locales (el contenedor corre en este mismo PC).
    xhost +si:localuser:"$(whoami)" >/dev/null
    # ">/dev/null 2>&1" esconde la salida; "|| true" evita que un error aquí detenga el script.
    xhost +local:docker >/dev/null 2>&1 || true
  else
    echo "[milo] Falta xhost: sudo apt install x11-xserver-utils"
  fi
}

# Muestra con qué está dibujando OpenGL dentro del contenedor.
# Si dice tu tarjeta (Intel/AMD/NVIDIA) hay aceleración; si dice "llvmpipe" está usando la CPU.
milo_gpu_info() {
  milo_run 'glxinfo -B 2>/dev/null | grep -E "OpenGL (vendor|renderer)|Accelerated" || echo "glxinfo falló: no hay contexto OpenGL"'
}

# Levanta el contenedor (construye la imagen si cambió) y compila la 1a vez.
milo_up() {
  milo_display
  # up = crear/arrancar el contenedor; -d = en segundo plano; --build = reconstruir la imagen
  # si el Dockerfile cambió (si no cambió, Docker reutiliza lo ya construido y es instantáneo).
  $DC up -d --build
  # Dentro del contenedor ("exec ros"): si todavía no existe install/setup.bash (nunca se compiló),
  # compila el workspace. "bash -ic" = bash interactivo, para que cargue ~/.bashrc con ROS.
  $DC exec ros bash -ic '[ -f /ros2_ws/install/setup.bash ] || (cd /ros2_ws && colcon build --symlink-install)'
}

# Corre un comando dentro del contenedor con ROS cargado.
# "$*" = todos los argumentos de la función juntos como un solo texto.
milo_run() {
  $DC exec ros bash -ic "$*"
}

# ¿Está corriendo el contenedor? "ps -q ros" devuelve su ID si existe; vacío si no.
milo_running() {
  [ -n "$($DC ps -q ros 2>/dev/null)" ]
}

# Igual que milo_running, pero si NO está corriendo avisa y termina el script.
# $0 = nombre del script que llamó (./sim.sh o ./robot.sh), para decirte qué correr primero.
milo_need_running() {
  milo_running || { echo "[milo] El contenedor no está levantado. Primero: $0"; exit 1; }
}

# Guarda el mapa que está armando slam_toolbox (topic /map) como .pgm (imagen) + .yaml (metadatos).
# Se guarda en ros2_ws/src/andesrobot_slam/maps/ que está montada desde tu PC: no se pierde.
milo_save_map() {
  local name="$1"
  milo_need_running
  milo_run "ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/andesrobot_slam/maps/$name"
  echo "[milo] Mapa guardado en ros2_ws/src/andesrobot_slam/maps/$name.pgm / .yaml"
}

# Teleop por teclado. "-r cmd_vel:=cmd_vel_teleop" renombra (remapea) su salida: en vez de mandar
# directo al robot (/cmd_vel), manda a /cmd_vel_teleop, que es la entrada del filtro de seguridad.
TELEOP_CMD='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop'
