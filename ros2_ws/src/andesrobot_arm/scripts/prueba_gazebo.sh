#!/bin/bash
# Prueba de humo del brazo en Gazebo, SIN ventanas: arranca arm_sim.launch.py y comprueba que todo
# lo básico funciona. Sirve para revisar que un cambio no rompió nada, sin mirar RViz.
#
#   ./sim.sh shell
#   bash /ros2_ws/src/andesrobot_arm/scripts/prueba_gazebo.sh      # ~2 minutos
#
# Qué comprueba (cada línea sale como OK o FALLO):
#   1. los 3 controladores (joint_state_broadcaster, arm_controller, gripper_controller) activos
#   2. /joint_states trae el lift, joint_1..6 y los dedos
#   3. los 4 canales de la cámara de la pinza publican a ~15 Hz
#   4. una pose alcanzable: el TCP llega a menos de 5 mm (medido por TF)
#   5. con MoveIt (el default de arm_sim): la mesa está en la escena de planificación, una pose
#      2 cm dentro de la mesa se RECHAZA (MoveIt no encuentra camino y el brazo no se mueve), y
#      al mirar la mesa con la cámara el OctoMap de la escena se llena de celdas
#   6. la pinza abre a 0.016 m y cierra a -0.007 m, y el dedo izquierdo copia en espejo
#   7. al apagar no quedan procesos de ROS vivos
# Con MOVEIT=false corre arm_ik_node en vez de MoveIt y salta el paso 5 (en Gazebo con mando por
# posición un choque no frena el brazo, ver docs/BRAZO.md, "Limitaciones").
#   MOVEIT=false bash .../prueba_gazebo.sh
#
# Antes de correrla no debe haber otra simulación andando (gzserver ocupa el puerto 11345).
# El log del launch queda en /tmp/prueba_gazebo.log.
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /ros2_ws/install/setup.bash

LOG=/tmp/prueba_gazebo.log
FALLOS=0
ok()    { echo "  OK     $*"; }
fallo() { echo "  FALLO  $*"; FALLOS=$((FALLOS + 1)); }

# Apagar todo (también si el script se interrumpe con Ctrl+C) y revisar que no queden procesos.
apagar() {
  kill -INT "$LPID" 2>/dev/null; sleep 8
  pkill -INT -f gzserver 2>/dev/null; sleep 3
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|arm_ik_node|arm_moveit_node|arm_marker_node|move_group|spawner|ros2 launch" 2>/dev/null
  sleep 1
  # Los zombis (estado Z) no cuentan: ya terminaron, solo falta que el sistema los recoja.
  vivos=$(ps -eo stat,cmd | grep -E 'gzserver|robot_state_publisher|arm_ik_node|arm_moveit_node|move_group|ros2 launch' | grep -v grep | grep -vc '^Z')
  [ "$vivos" = 0 ] && ok "7. sin procesos de ROS vivos al apagar" || fallo "7. quedaron $vivos procesos vivos (./sim.sh stop los mata)"
  echo; [ "$FALLOS" = 0 ] && echo "TODO OK" || echo "$FALLOS FALLO(S); log en $LOG"
  exit $FALLOS
}
trap apagar INT TERM

if pgrep -f gzserver >/dev/null; then echo "Ya hay un gzserver corriendo: apágalo primero (./sim.sh stop)."; exit 1; fi
echo "Arrancando arm_sim.launch.py sin ventanas (log: $LOG)..."
MOVEIT="${MOVEIT:-true}"
ros2 launch andesrobot_arm arm_sim.launch.py gui:=false rviz:=false moveit:=$MOVEIT > "$LOG" 2>&1 &
LPID=$!

# 1. Controladores activos (hasta 120 s: la primera vez Gazebo carga las mallas y tarda más).
activos=0
for i in $(seq 1 60); do
  sleep 2
  [ "$(timeout 10 ros2 control list_controllers 2>/dev/null | grep -c active)" = 3 ] && { activos=1; break; }
done
if [ $activos = 1 ]; then ok "1. 3 controladores activos a los $((i * 2)) s"; else fallo "1. controladores no activos en 120 s"; apagar; fi

# 2. /joint_states con las articulaciones del brazo.
nombres=$(timeout 10 ros2 topic echo /joint_states --once --field name 2>/dev/null | tr -d '\n')
faltan=""
for j in vertical_lift_joint joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 right_finger_joint; do
  echo "$nombres" | grep -q "$j" || faltan="$faltan $j"
done
[ -z "$faltan" ] && ok "2. /joint_states trae el lift, joint_1..6 y la pinza" || fallo "2. faltan en /joint_states:$faltan"

# 3. Cámara: 4 canales a ~15 Hz (se acepta de 10 a 20).
for t in color/image_raw depth/image_raw depth/points ir/image_raw; do
  hz=$(timeout 8 ros2 topic hz /gripper_camera/$t 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  if [ -n "$hz" ] && awk "BEGIN{exit !($hz > 10 && $hz < 20)}"; then ok "3. cámara $t a $hz Hz"; else fallo "3. cámara $t: ${hz:-sin datos} Hz"; fi
done

# Con MoveIt: esperar a move_group (la acción /move_action) y a que la mesa esté en la escena.
if [ "$MOVEIT" = true ]; then
  for i in $(seq 1 45); do sleep 2; timeout 5 ros2 action list 2>/dev/null | grep -q "^/move_action" && break; done
  sleep 6
  # /monitored_planning_scene solo se publica cuando cambia: mejor preguntar por el servicio.
  # components 8 = nombres de los objetos del mundo (moveit_msgs/PlanningSceneComponents).
  # (la respuesta sale como texto de Python: id='mesa_tablero', id='mesa_pata')
  escena=$(timeout 15 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 8}}" 2>/dev/null | grep -o "id='mesa_[a-z]*'" | sort -u | wc -l)
  [ "$escena" -ge 2 ] && ok "5a. la mesa está en la escena de MoveIt ($escena objetos)" || fallo "5a. la mesa no está en la escena de MoveIt"
fi

# 4. Pose alcanzable (la de la validación del informe) y comprobación por TF.
ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.5048, y: -0.2163, z: 1.862}, orientation: {x: -0.1252, y: 0.5501, z: -0.7869, w: 0.2502}}}" > /dev/null
sleep $([ "$MOVEIT" = true ] && echo 25 || echo 14)
tf=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint gripper_tcp 2>/dev/null | grep -m1 Translation)
err=$(echo "$tf" | python3 -c "
import sys, re, math
v = [float(x) for x in re.findall(r'-?\d+\.\d+', sys.stdin.read())[:3]]
print('%.1f' % (1000 * math.dist(v, [0.5048, -0.2163, 1.862])) if len(v) == 3 else 'nan')")
if [ "$err" != nan ] && awk "BEGIN{exit !($err < 5)}"; then ok "4. pose alcanzable: TCP a $err mm del objetivo"; else fallo "4. pose alcanzable: TCP a $err mm ($tf)"; fi

# 5b. Con MoveIt: una pose 2 cm dentro del tablero de la mesa tiene que rechazarse (no hay camino
#     sin choque) y el brazo quedarse donde estaba.
if [ "$MOVEIT" = true ]; then
  antes=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint gripper_tcp 2>/dev/null | grep -m1 Translation)
  ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: base_footprint}, pose: {position: {x: 0.80, y: 0.0, z: 0.70}, orientation: {x: 0.0, y: -0.7071, z: 0.0, w: 0.7071}}}" > /dev/null
  sleep 20
  despues=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint gripper_tcp 2>/dev/null | grep -m1 Translation)
  rechazo=$(grep -c "MoveIt no movió el brazo" "$LOG")
  if [ "$rechazo" -ge 1 ] && [ "$antes" = "$despues" ]; then ok "5b. pose dentro de la mesa rechazada por MoveIt; el brazo no se movió"
  else fallo "5b. pose dentro de la mesa: rechazos=$rechazo, TCP antes $antes, después $despues"; fi
fi

# 5c. Con MoveIt y la cámara: mirar la mesa desde arriba (45° hacia abajo) y comprobar que el
#     OctoMap de la escena tiene celdas (la nube de la cámara entró como obstáculo).
if [ "$MOVEIT" = true ]; then
  ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: base_footprint}, pose: {position: {x: 0.5, y: 0.0, z: 1.4}, orientation: {x: 0.3827, y: 0.0, z: 0.9239, w: 0.0}}}" > /dev/null
  sleep 25
  # components 32 = el OctoMap; se cuentan los bytes del campo data.
  celdas=$(timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 32}}" 2>/dev/null | grep -o "data=\[[^]]*\]" | head -1 | grep -o -- "-\?[0-9]\+" | wc -l)
  [ "${celdas:-0}" -gt 100 ] && ok "5c. OctoMap con datos tras mirar la mesa ($celdas bytes)" || fallo "5c. OctoMap vacío (${celdas:-0} bytes): ¿octomap:=false o falta moveit_ros_perception?"
fi

# 6. Pinza: abrir y cerrar; el dedo izquierdo (mimic) tiene que copiar con signo opuesto.
# Lee la posición de los dos dedos de /joint_states (lo publican dos nodos, ruedas y brazo:
# se espera el mensaje que trae los dedos).
dedos() {
  timeout 10 python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init()
n = Node('lee_dedos')
out = []
def cb(m):
    if 'right_finger_joint' in m.name:
        out.append((m.position[m.name.index('right_finger_joint')], m.position[m.name.index('left_finger_joint_mimic')]))
n.create_subscription(JointState, '/joint_states', cb, 10)
while rclpy.ok() and not out:
    rclpy.spin_once(n, timeout_sec=0.5)
print('%.4f %.4f' % out[0]); rclpy.shutdown()
EOF
}
for p in 0.016 -0.007; do
  # Hasta 2 intentos: a veces "ros2 action send_goal" pierde la respuesta del servidor (fallo del
  # middleware: "Failed to send goal response ... client will not receive response") y la meta
  # no se ejecuta; en ese caso se manda de nuevo.
  for intento in 1 2; do
    timeout 15 ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{trajectory: {joint_names: [right_finger_joint], points: [{positions: [$p], time_from_start: {sec: 1}}]}}" > /dev/null 2>&1
    sleep 1
    set -- $(dedos)
    [ -n "$1" ] && awk "BEGIN{exit !(($1 - $p) < 0.002 && ($1 - $p) > -0.002)}" && break
  done
  if [ -n "$1" ] && awk "BEGIN{exit !(($1 - $p) < 0.002 && ($1 - $p) > -0.002 && ($2 + $1) < 0.002 && ($2 + $1) > -0.002)}"; then
    ok "6. pinza en $p: derecho $1, izquierdo $2$([ "$intento" = 2 ] && echo ' (al 2.º intento)')"
  else fallo "6. pinza en $p: derecho ${1:-?}, izquierdo ${2:-?}"; fi
done

apagar
