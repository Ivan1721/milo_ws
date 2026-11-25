#!/usr/bin/env bash
# Instala el driver del brazo en la Jetson (o un PC con ROS 2 Humble NATIVO, sin Docker):
# dependencias, compilación (en ~/.cache/brazo_milo, fuera de ros2_ws), comando `brazo_milo` e
# ícono en el escritorio. Se corre una vez:
#   ros2_ws/src/drivers/brazo_driver/scripts/instalar.sh
set -e
AQUI="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
chmod +x "$AQUI"/*.sh

[ -f /opt/ros/humble/setup.bash ] || { echo "Falta ROS 2 Humble (/opt/ros/humble)."; exit 1; }

# Dependencias del sistema (solo las que falten).
faltan=()
python3 -c 'import serial' 2>/dev/null || faltan+=(python3-serial)
python3 -c 'import tkinter' 2>/dev/null || faltan+=(python3-tk)
command -v gpioget >/dev/null || faltan+=(gpiod)
command -v zenity  >/dev/null || faltan+=(zenity)
command -v colcon  >/dev/null || faltan+=(python3-colcon-common-extensions)
if [ ${#faltan[@]} -gt 0 ]; then
  echo "Instalando: ${faltan[*]}"
  sudo apt-get install -y "${faltan[@]}"
fi

# Permisos: puerto serie (dialout), control de Xbox (input) y GPIO de la seta (gpio).
for g in dialout input gpio; do
  if getent group "$g" >/dev/null && ! id -nG "$USER" | grep -qw "$g"; then
    echo "Agregando $USER al grupo $g (hay que cerrar sesión y volver a entrar)"
    sudo usermod -aG "$g" "$USER"
  fi
done

# Compilar (fuera de ros2_ws: ver brazo.sh).
PAQUETE="$(dirname "$AQUI")"
COMPILADO="$HOME/.cache/brazo_milo"
mkdir -p "$COMPILADO"
set +u; source /opt/ros/humble/setup.bash; set -u
(cd "$COMPILADO" && colcon build --symlink-install --base-paths "$PAQUETE" \
   --build-base "$COMPILADO/build" --install-base "$COMPILADO/install")

# Comando y lanzadores.
mkdir -p "$HOME/.local/bin" "$HOME/.local/share/applications"
ln -sf "$AQUI/brazo.sh" "$HOME/.local/bin/brazo_milo"
ESCRITORIO="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$ESCRITORIO"
for destino in "$ESCRITORIO/brazo_milo.desktop" "$HOME/.local/share/applications/brazo_milo.desktop"; do
  cat > "$destino" <<DESK
[Desktop Entry]
Type=Application
Name=Brazo MILO (control Xbox)
Comment=Driver del brazo: RT + joystick izq. mueve, joystick der. garra, A cambia articulación, Y home, B parar
Exec=$AQUI/brazo.sh
Icon=input-gaming
Terminal=false
Categories=Utility;
DESK
  chmod +x "$destino"
done
gio set "$ESCRITORIO/brazo_milo.desktop" metadata::trusted true 2>/dev/null || true

echo
echo "Listo. Abre el driver con el ícono \"Brazo MILO\" del escritorio o con:  brazo_milo"
echo "Si el Arduino del brazo todavía no tiene el programa v3:  firmware/cargar_firmware.sh"
