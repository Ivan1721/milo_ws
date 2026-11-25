#!/usr/bin/env bash
# Abre el driver del brazo de Milo (ventana + control de Xbox), NATIVO (sin Docker), como en la
# Jetson. La primera vez compila el paquete brazo_driver solo, en ~/.cache/brazo_milo (FUERA de
# ros2_ws: en el PC de desarrollo, ros2_ws/install/ es del contenedor y una compilación nativa ahí
# lo rompe). Registro: ~/.ros/brazo_driver.log. Los argumentos van al launch.
AQUI="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PAQUETE="$(dirname "$AQUI")"                 # ros2_ws/src/drivers/brazo_driver
COMPILADO="$HOME/.cache/brazo_milo"
LOG="$HOME/.ros/brazo_driver.log"
mkdir -p "$HOME/.ros"
export DISPLAY="${DISPLAY:-:0}"

aviso() {  # en la terminal si hay una; si no (doble clic), en una ventana
  echo "$1" >&2
  [ -t 1 ] || { command -v zenity >/dev/null && zenity --error --title="Brazo MILO" --width=600 \
    --text="$(echo "$1" | sed 's/[<>&]/ /g')" 2>/dev/null; }
}

set +u
if [ ! -f /opt/ros/humble/setup.bash ]; then
  aviso "No está ROS 2 Humble (/opt/ros/humble). Instálalo primero."; exit 1
fi
source /opt/ros/humble/setup.bash

# Compilar si falta o si el código es más nuevo que lo compilado.
if [ ! -f "$COMPILADO/install/setup.bash" ] || \
   [ -n "$(find "$PAQUETE" -newer "$COMPILADO/install/setup.bash" -type f 2>/dev/null | head -1)" ]; then
  echo "Compilando el driver (solo la primera vez)..."
  if ! (mkdir -p "$COMPILADO" && cd "$COMPILADO" && \
        colcon build --symlink-install --base-paths "$PAQUETE" --build-base "$COMPILADO/build" \
          --install-base "$COMPILADO/install" >"$LOG" 2>&1); then
    aviso "Falló la compilación del driver:
$(tail -n 15 "$LOG")"; exit 1
  fi
fi
source "$COMPILADO/install/setup.bash"

# Uno solo a la vez (pelearían por el puerto del Arduino). El candado se suelta solo al cerrarse.
exec 9>"$HOME/.ros/brazo_driver.lock"
if ! flock -n 9; then
  command -v zenity >/dev/null && zenity --info --title="Brazo MILO" \
    --text="El driver del brazo ya está abierto." 2>/dev/null
  echo "El driver del brazo ya está abierto."; exit 0
fi

ros2 launch brazo_driver brazo.launch.py "$@" 2>&1 | tee "$LOG"
estado=${PIPESTATUS[0]}
[ "$estado" -ne 0 ] && aviso "El driver del brazo terminó con error:
$(tail -n 15 "$LOG")"
exit "$estado"
