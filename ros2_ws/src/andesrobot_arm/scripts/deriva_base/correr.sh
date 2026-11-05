#!/bin/bash
# Mide cuánto se corre la base de Milo en Gazebo cuando el brazo se mueve en el aire.
# Uso (en el PC, desde cualquier carpeta; no necesita el contenedor milo):
#   ros2_ws/src/andesrobot_arm/scripts/deriva_base/correr.sh NOMBRE [VAR=valor ...]
# Ejemplos:
#   correr.sh base N=30                 # como está el repo: 50 iteraciones de ODE
#   correr.sh iters200 N=30 ITERS=200   # con el arreglo propuesto
# Variables: N (movimientos), ESCALA (multiplica la duración), ESPERA (s quieto tras cada
# movimiento), TASA (Hz de la orden), TORQUE (N·m por rueda) y las de preparar.py.
# Resultado: ros2_ws/resultados/deriva_base/NOMBRE.txt (una fila por movimiento) y un resumen.
# xacro y el plugin compilado van a ~/.cache/milo_deriva: fuera de ros2_ws/src, porque colcon
# tomaría el clon de xacro como un paquete más.
set -e
aqui=$(cd "$(dirname "$0")" && pwd)
src=$(cd "$aqui/../../.." && pwd)            # ros2_ws/src
cache=${XDG_CACHE_HOME:-$HOME/.cache}/milo_deriva
if [ "$1" != "--dentro" ]; then
  docker image inspect milo-deriva >/dev/null 2>&1 || docker build -t milo-deriva "$aqui"
  mkdir -p "$cache" "$src/../resultados/deriva_base"
  [ -d "$cache/xacro_src" ] || git clone -q --depth 1 -b ros2 https://github.com/ros/xacro.git \
    "$cache/xacro_src"
  exec docker run --rm -v "$src":/src:ro -v "$aqui":/deriva:ro -v "$cache":/cache \
    -v "$src/../resultados/deriva_base":/resultados milo-deriva bash /deriva/correr.sh --dentro "$@"
fi
# --- dentro del contenedor ---
shift
nombre=$1; shift
for kv in "$@"; do export "$kv"; done
if [ ! -f /cache/build/libderiva.so ] || [ /deriva/deriva.cc -nt /cache/build/libderiva.so ]; then
  mkdir -p /cache/build && cd /cache/build && cmake /deriva >/dev/null 2>&1 && make 2>&1 | tail -1
fi
source /opt/ros/humble/setup.bash
python3 /deriva/preparar.py
export SALIDA=/resultados/$nombre.txt
rm -f "$SALIDA"
timeout "${LIMITE:-1500}" gzserver /tmp/mundo.sdf > "/resultados/$nombre.log" 2>&1 &
until grep -q FIN "$SALIDA" 2>/dev/null; do sleep 2; pgrep gzserver >/dev/null || break; done
pkill gzserver || true
python3 - "$SALIDA" <<'PY'
import math, sys
f = [l.split() for l in open(sys.argv[1]) if l[0].isdigit()]
d = [math.hypot(float(a[2]), float(a[3])) for a in f]
x, y = sum(float(a[2]) for a in f), sum(float(a[3]) for a in f)
giro = sum(float(a[4]) for a in f)
rueda = [sum(float(a[k]) for a in f) * 0.08255 * 1000 for k in (5, 6)]   # radio 0.08255 m
print(f'{len(f)} movimientos: la base se corrió {math.hypot(x, y):.1f} mm (x {x:.1f}, '
      f'y {y:.1f}) y giró {giro:.2f}°; {sum(d) / len(d):.2f} mm por movimiento (máx '
      f'{max(d):.2f}). Las ruedas rodaron {rueda[0]:.1f} / {rueda[1]:.1f} mm.')
PY
