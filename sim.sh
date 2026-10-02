#!/bin/bash
# Milo en SIMULACIÓN (Gazebo + SLAM + RViz) — en tu PC.
#
#   ./sim.sh                 levanta todo: Gazebo, Milo, filtro de seguridad, slam_toolbox y RViz
#   ./sim.sh teleop          (otra terminal) manejar a Milo con el teclado
#   ./sim.sh mapa [nombre]   (otra terminal) guardar el mapa  (default: sim_AAAAMMDD_HHMM)
#   ./sim.sh shell           terminal dentro del contenedor
#   ./sim.sh build           recompilar el workspace
#   ./sim.sh stop            apagar el contenedor
#   ./sim.sh cpu             igual que ./sim.sh pero sin GPU NVIDIA
#   ./sim.sh sin-gazebo      sin la ventana de Gazebo (más liviano; RViz igual se abre)
#   ./sim.sh software        render por CPU (si Gazebo/RViz se ven NEGROS); más lento
#   ./sim.sh gpu             muestra con qué tarjeta está dibujando el contenedor
set -e
# (Las líneas 2-15 de arriba son la ayuda: el "sed -n '2,15p'" de más abajo las imprime
#  cuando escribes una opción que no existe. Por eso los comentarios explicativos van aquí.)
#
# Línea 1 "#!/bin/bash": le dice al sistema que este archivo se ejecuta con bash.
# "set -e": si cualquier comando falla, el script se detiene (en vez de seguir a ciegas).
#
# Ir a la carpeta donde está este script (milo_ws), aunque lo llames desde otra parte.
# $0 = ruta del script; dirname = su carpeta.
cd "$(dirname "$0")"
# Cargar las funciones comunes (milo_init, milo_up, milo_run...).
source docker/lib.sh
# Preparar docker compose en modo simulación. $1 = la primera palabra que escribiste
# (por ejemplo "cpu" o "software"); milo_init la usa para decidir GPU / CPU.
milo_init sim "$1"

# "case" = elegir qué hacer según la primera palabra ($1).
#   exec   = reemplazar este script por el comando (al salir del comando, termina todo)
#   ;;     = fin de esa opción
case "$1" in
  # Teleop: abre el teclado dentro del contenedor ya levantado.
  teleop) milo_need_running; exec $DC exec ros bash -ic "$TELEOP_CMD" ;;
  # Mapa: ${2:-...} = el nombre que diste, o si no diste ninguno, "sim_" + fecha y hora.
  mapa)   milo_save_map "${2:-sim_$(date +%Y%m%d_%H%M)}"; exit 0 ;;
  # Shell: una terminal bash dentro del contenedor.
  shell)  milo_need_running; exec $DC exec ros bash ;;
  # Build: recompilar ros2_ws (después de cambiar código en tu PC).
  build)  milo_need_running; exec $DC exec ros bash -ic 'cd /ros2_ws && colcon build --symlink-install' ;;
  # Stop: apaga y borra el contenedor (tu código y mapas quedan, están en tu PC).
  stop)   $DC down; exit 0 ;;
  # GPU: diagnóstico de con qué está dibujando OpenGL.
  gpu)    milo_need_running; milo_gpu_info; exit 0 ;;
  # Estas opciones no hacen nada aquí: siguen hacia abajo a levantar la simulación.
  ""|cpu|sin-gazebo|software) ;;
  # Cualquier otra cosa: mostrar la ayuda (líneas 2 a 15 de este archivo) y salir con error.
  *) sed -n '2,15p' "$0"; exit 1 ;;
esac

# Por defecto se abre la ventana de Gazebo; con "sin-gazebo" no (ahorra GPU, la física igual corre).
GAZEBO_GUI=true
[ "$1" = "sin-gazebo" ] && GAZEBO_GUI=false

# Levantar el contenedor y compilar si es la primera vez.
milo_up
echo
echo "[milo] Simulación arrancando. En otra terminal:  ./sim.sh teleop"
echo "[milo] Para guardar el mapa:                     ./sim.sh mapa <nombre>"
echo
# Lanzar todo (ver andesrobot_bringup/launch/sim_mapping.launch.py). Queda en primer plano:
# Ctrl+C lo detiene.
exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup sim_mapping.launch.py gazebo_gui:=$GAZEBO_GUI"
