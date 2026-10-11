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
#   ./sim.sh brazo           brazo de Milo: Gazebo + IK + marcador en RViz (ver andesrobot_arm)
#   ./sim.sh casa            igual que ./sim.sh pero en la casa RoboCup@Home (para mapearla)
#   ./sim.sh nav <mapa> [arena]   navegación autónoma (AMCL + Nav2) con un mapa ya guardado
#   ./sim.sh ir <lugar> [...]     (otra terminal) mandar a Milo: ./sim.sh ir kitchen  / --lista
#   ./sim.sh tour [veces]         (otra terminal) visitar todas las ubicaciones y guardar un CSV
set -e
# (Las líneas 2-18 de arriba son la ayuda: el "sed -n '2,18p'" de más abajo las imprime
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
  # Brazo: levanta el contenedor y lanza la simulación del brazo (en vez de la de mapeo).
  brazo)  milo_up
          # Si el contenedor ya estaba compilado de antes, puede faltar andesrobot_arm: compilar.
          $DC exec ros bash -ic '[ -d /ros2_ws/install/andesrobot_arm ] || (cd /ros2_ws && colcon build --symlink-install)'
          echo
          echo "[milo] Brazo arrancando. En RViz: herramienta Interact (tecla i), arrastra la esfera naranja."
          echo "[milo] Mandar una pose a mano: ./sim.sh shell  y ver ros2_ws/src/andesrobot_arm/docs/BRAZO.md"
          echo
          exec $DC exec ros bash -ic "ros2 launch andesrobot_arm arm_sim.launch.py" ;;
  # Ir a una o varias ubicaciones con nombre (ver ./sim.sh ir --lista). "shift" saca la palabra
  # "ir" y "$*" pasa el resto tal cual.
  ir)     milo_need_running; shift; exec $DC exec ros bash -ic "ros2 run andesrobot_navigation ir_a $*" ;;
  # Tour para el paper: todas las ubicaciones del rulebook en orden aleatorio, resultados en CSV
  # (quedan en ros2_ws/resultados/ en tu PC).
  tour)   milo_need_running
          CSV="/ros2_ws/resultados/tour_$(date +%Y%m%d_%H%M).csv"
          exec $DC exec ros bash -ic "ros2 run andesrobot_navigation ir_a --tour --repeticiones ${2:-1} --csv $CSV" ;;
  # Navegación: necesita el nombre de un mapa guardado con ./sim.sh mapa <nombre>.
  nav)    [ -n "$2" ] || { echo "[milo] Uso: ./sim.sh nav <mapa> [arena]   (mapas en ros2_ws/src/andesrobot_slam/maps/)"; ls ros2_ws/src/andesrobot_slam/maps/*.pgm 2>/dev/null | xargs -n1 basename 2>/dev/null | sed 's/.pgm$//; s/^/  /'; exit 1; }
          [ -f "ros2_ws/src/andesrobot_slam/maps/$2.yaml" ] || { echo "[milo] No existe el mapa ros2_ws/src/andesrobot_slam/maps/$2.yaml"; exit 1; } ;;
  # Estas opciones no hacen nada aquí: siguen hacia abajo a levantar la simulación.
  ""|cpu|sin-gazebo|software|casa) ;;
  # Cualquier otra cosa: mostrar la ayuda (líneas 2 a 18 de este archivo) y salir con error.
  *) sed -n '2,18p' "$0"; exit 1 ;;
esac

# Por defecto se abre la ventana de Gazebo; con "sin-gazebo" no (ahorra GPU, la física igual corre).
GAZEBO_GUI=true
[ "$1" = "sin-gazebo" ] && GAZEBO_GUI=false

# Levantar el contenedor y compilar si es la primera vez.
milo_up

# En la casa, Milo parte afuera de la puerta del living mirando hacia adentro. Estos números los
# imprime andesrobot_gazebo/tools/gen_casa_robocup.py ("Partida de Milo"): si cambias la casa,
# cópialos aquí. En la sim el mapa queda igual al mundo de Gazebo, así que AMCL parte en esta
# misma pose.
CASA_SPAWN="x:=-7.2 y:=-1.862 yaw:=0.0"

# Navegación autónoma: otro launch (AMCL sobre el mapa guardado en vez de SLAM).
if [ "$1" = "nav" ]; then
  MUNDO=robocup_casa
  SPAWN="$CASA_SPAWN"
  [ "$3" = "arena" ] && MUNDO=andesrobot_arena && SPAWN=""
  echo
  echo "[milo] Navegación en $MUNDO con el mapa $2. En otra terminal:"
  echo "[milo]   ./sim.sh ir --lista        ver ubicaciones"
  echo "[milo]   ./sim.sh ir kitchen        mandar a Milo (o '2D Goal Pose' en RViz)"
  echo "[milo]   ./sim.sh tour              visitar todas y guardar CSV"
  echo
  exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup sim_navigation.launch.py map:=/ros2_ws/src/andesrobot_slam/maps/$2.yaml mundo:=$MUNDO $SPAWN"
fi

# Mundo para mapear: la arena SAR (default) o la casa RoboCup@Home.
MUNDO=andesrobot_arena
SPAWN=""
[ "$1" = "casa" ] && MUNDO=robocup_casa && SPAWN="$CASA_SPAWN"
echo
echo "[milo] Simulación arrancando. En otra terminal:  ./sim.sh teleop"
echo "[milo] Para guardar el mapa:                     ./sim.sh mapa <nombre>"
echo
# Lanzar todo (ver andesrobot_bringup/launch/sim_mapping.launch.py). Queda en primer plano:
# Ctrl+C lo detiene.
exec $DC exec ros bash -ic "ros2 launch andesrobot_bringup sim_mapping.launch.py gazebo_gui:=$GAZEBO_GUI mundo:=$MUNDO $SPAWN"
