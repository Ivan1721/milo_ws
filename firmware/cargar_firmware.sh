#!/usr/bin/env bash
# Carga el programa del brazo (firmware/brazo_mega/brazo_mega.hex, ya compilado) al Arduino Mega.
#   firmware/cargar_firmware.sh            -> al Mega del brazo (CH340 "USB2.0-Serial")
#   firmware/cargar_firmware.sh /dev/ttyUSBx
# Sirve sin el IDE de Arduino ni arduino-cli (por ejemplo en la Jetson): solo avrdude. Cerrar
# antes el driver del brazo o brazo_real_node (usan el mismo puerto). Cargar reinicia el Arduino:
# vuelve a contar 0 pasos donde esté el brazo.
# Si cambias el .ino, vuelve a generar el .hex (en el PC de desarrollo, con arduino-cli):
#   arduino-cli compile --fqbn arduino:avr:mega --output-dir /tmp/brazo_mega firmware/brazo_mega
#   cp /tmp/brazo_mega/brazo_mega.ino.hex firmware/brazo_mega/brazo_mega.hex
# (Viene de la entrega del equipo de la prueba del brazo, nov-2026.)
set -e
AQUI="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
HEX="$AQUI/brazo_mega/brazo_mega.hex"
BRAZO=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
HOVER=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0     # Mega de la hoverboard: NO

PUERTO="${1:-$BRAZO}"
[ -e "$PUERTO" ] || { echo "No encuentro el puerto $PUERTO. ¿Está enchufado el Mega del brazo?"; \
  ls -l /dev/serial/by-id/ 2>/dev/null; exit 1; }
if [ -e "$HOVER" ] && [ "$(readlink -f "$PUERTO")" = "$(readlink -f "$HOVER")" ]; then
  echo "$PUERTO es el Arduino de la HOVERBOARD, no el del brazo. No cargo nada."; exit 1
fi

# avrdude: el del sistema o el que trae el IDE de Arduino.
AVRDUDE=$(command -v avrdude || true); CONF=()
if [ -z "$AVRDUDE" ]; then
  for d in "$HOME"/.local/share/arduino-*/hardware/tools/avr /usr/share/arduino/hardware/tools/avr \
           "$HOME"/arduino-*/hardware/tools/avr /opt/arduino*/hardware/tools/avr; do
    if [ -x "$d/bin/avrdude" ]; then AVRDUDE="$d/bin/avrdude"; CONF=(-C "$d/etc/avrdude.conf"); break; fi
  done
fi
if [ -z "$AVRDUDE" ]; then
  echo "Falta avrdude. Instalando..."; sudo apt-get install -y avrdude; AVRDUDE=$(command -v avrdude)
fi

if command -v fuser >/dev/null && fuser "$(readlink -f "$PUERTO")" >/dev/null 2>&1; then
  echo "El puerto está en uso (¿el driver del brazo abierto?). Ciérralo y vuelve a intentar."; exit 1
fi

echo "Cargando $HEX en $PUERTO ($(readlink -f "$PUERTO"))..."
"$AVRDUDE" "${CONF[@]}" -p atmega2560 -c wiring -P "$(readlink -f "$PUERTO")" -b 115200 -D \
  -U "flash:w:$HEX:i"

echo "Comprobando..."
python3 - "$PUERTO" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=1)
time.sleep(2.5); s.reset_input_buffer(); s.write(b'P\n')
r = s.readline().decode(errors='replace').strip()
print('Arduino:', r or '(sin respuesta)')
sys.exit(0 if r.startswith('PONG brazo_mega') else 1)
PY
echo "Firmware cargado."
