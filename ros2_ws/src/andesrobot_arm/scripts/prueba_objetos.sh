#!/bin/bash
# Prueba del detector de objetos con el brazo en Gazebo, SIN ventanas: para correrla junto con el
# equipo cuando llega un modelo nuevo (modelos/objetos/best.pt) y ver qué parte de la cadena
# cámara -> YOLO -> profundidad -> MoveIt -> brazo -> pinza funciona con él.
#
#   ./sim.sh shell
#   bash /ros2_ws/src/andesrobot_arm/scripts/prueba_objetos.sh                     # todo (~10 min)
#   bash /ros2_ws/src/andesrobot_arm/scripts/prueba_objetos.sh acercar cup         # una prueba
#   bash /ros2_ws/src/andesrobot_arm/scripts/prueba_objetos.sh agarrar cuchara
#   MODELO=/ros2_ws/modelos/objetos/anterior_2026-11-06/best.pt \
#     bash /ros2_ws/src/andesrobot_arm/scripts/prueba_objetos.sh agarrar tenedor  # otro modelo
#
# Cada prueba arranca una simulación limpia (arm_sim sin gzclient ni RViz, octomap:=false) y la
# apaga al final, así una no deja el brazo o la base mal puestos para la siguiente:
#   acercar <clase>  con la taza y el bowl en la mesa (objetos:=true), lleva la pinza a la pose de
#                    partida de docs/prueba_desde_cero.md (paso 10) y corre objeto_node con
#                    acercar:=true. OK si termina con "Listo: la pinza quedó a ... m del objeto".
#   agarrar <id>     con solo ese objeto del catálogo (config/objetos.yaml) en (0.80, 0), corre
#                    agarre_node <id>. OK si termina con "Listo: <id> levantado".
# Sin argumentos corre: acercar cup, agarrar tenedor, agarrar cuchara y agarrar martillo.
# Con MODELO=... se usa ese .pt en vez de best.pt: en "agarrar" se arranca objeto_node a mano con
# ese modelo antes de agarre_node (agarre_node usa el que ya publica /objeto/pose).
#
# Al final de cada prueba anota también dónde quedó la base (TF odom -> base_footprint): si se
# corrió más de 1 cm, la prueba no es confiable (deriva de la base, P13 en docs/PENDIENTES.md).
# Logs en /tmp/prueba_objetos_<prueba>.log (el nodo) y /tmp/prueba_objetos_sim.log (el launch).
# Antes de correrla no debe haber otra simulación andando (gzserver ocupa el puerto 11345).
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /ros2_ws/install/setup.bash

SIM_LOG=/tmp/prueba_objetos_sim.log
RESUMEN=()

# Matar todo lo de ROS y Gazebo (las pruebas anteriores dejan nodos huérfanos, ver CLAUDE.md).
matar() {
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|arm_moveit_node|arm_marker_node|move_group|spawner|ros2 launch|objeto_node|agarre_node" 2>/dev/null
  sleep 4
}

# Lanzar la simulación del brazo con los objetos pedidos y esperar a que MoveIt esté listo.
sim() {
  matar
  ros2 launch andesrobot_arm arm_sim.launch.py gui:=false rviz:=false objetos:=$1 \
    octomap:=false > $SIM_LOG 2>&1 &
  for _ in $(seq 1 90); do grep -q "You can start planning now" $SIM_LOG && break; sleep 2; done
  sleep 20     # que se terminen de poner los objetos sobre la mesa
}

# Dónde está la base respecto de donde partió (x, y en m).
base() {
  timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null \
    | grep -m1 Translation | sed 's/.*\[\(.*\), .*\]/\1/'
}

# Anotar el resultado de una prueba: OK si el log tiene la frase de éxito, si no FALLO con la
# última línea de error (o la última del nodo).
anotar() {
  local nombre=$1 log=$2 exito=$3 fin
  fin=$(grep -m1 "$exito" "$log" | sed 's/^.*\]: //')
  if [ -n "$fin" ]; then
    RESUMEN+=("OK     $nombre: $fin (base en $(base))")
  else
    fin=$(grep -E "No se pudo|ERROR" "$log" | tail -1 | sed 's/^.*\]: //')
    [ -z "$fin" ] && fin="sin terminar: $(grep -v 'no veo' "$log" | tail -1 | sed 's/^.*\]: //')"
    RESUMEN+=("FALLO  $nombre: $fin (base en $(base))")
  fi
}

modelo_args=()
[ -n "$MODELO" ] && modelo_args=(-p modelo:=$MODELO)

acercar() {
  local clase=$1 log=/tmp/prueba_objetos_acercar_$1.log
  echo ">> acercar $clase"
  sim true
  ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_footprint}, pose: {position: {x: 0.531, y: -0.009, z: 0.941}, orientation: {x: -0.1736, y: 0.0, z: -0.9848, w: 0.0}}}" > /dev/null
  sleep 15
  timeout 150 ros2 run andesrobot_arm objeto_node --ros-args -p use_sim_time:=true \
    -p clase:=$clase -p acercar:=true "${modelo_args[@]}" > $log 2>&1
  anotar "acercar $clase" $log "Listo: la pinza quedó a"
}

agarrar() {
  local id=$1 log=/tmp/prueba_objetos_agarrar_$1.log clase
  echo ">> agarrar $id"
  sim $id
  if [ -n "$MODELO" ]; then
    clase=$(python3 -c "
import sys; sys.path.insert(0, '/ros2_ws/src/andesrobot_arm')
from andesrobot_arm.catalogo_objetos import elegir, leer_catalogo
c = leer_catalogo(open('/ros2_ws/src/andesrobot_arm/config/objetos.yaml').read())
print(elegir(c, '$id')[0]['clase'])")
    ros2 run andesrobot_arm objeto_node --ros-args -p use_sim_time:=true -p clase:=$clase \
      -p confianza:=0.2 "${modelo_args[@]}" > /tmp/prueba_objetos_objeto_node.log 2>&1 &
    sleep 5
  fi
  # La salida va a un archivo, no a un pipe: el objeto_node que arranca agarre_node hereda la
  # salida y, si fuera un pipe, lo dejaría abierto y el script no seguiría.
  timeout 400 ros2 run andesrobot_arm agarre_node $id > $log 2>&1
  anotar "agarrar $id" $log "Listo: $id levantado"
}

if [ $# -eq 0 ]; then
  acercar cup
  agarrar tenedor
  agarrar cuchara
  agarrar martillo
else
  "$@"
fi
matar
echo
echo "Modelo: ${MODELO:-/ros2_ws/modelos/objetos/best.pt}"
printf '%s\n' "${RESUMEN[@]}"
