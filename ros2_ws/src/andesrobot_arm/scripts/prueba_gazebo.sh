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
#   5. la pinza abre a 0.016 m y cierra a -0.007 m, y el dedo izquierdo copia en espejo
#   6. al apagar no quedan procesos de ROS vivos
# NO prueba choques: en Gazebo con mando por posición un choque no frena el brazo (ver
# docs/BRAZO.md, "Limitaciones"), así que no hay nada que medir.
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
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|arm_ik_node|arm_marker_node|spawner|ros2 launch" 2>/dev/null
  sleep 1
  # Los zombis (estado Z) no cuentan: ya terminaron, solo falta que el sistema los recoja.
  vivos=$(ps -eo stat,cmd | grep -E 'gzserver|robot_state_publisher|arm_ik_node|ros2 launch' | grep -v grep | grep -vc '^Z')
  [ "$vivos" = 0 ] && ok "6. sin procesos de ROS vivos al apagar" || fallo "6. quedaron $vivos procesos vivos (./sim.sh stop los mata)"
  echo; [ "$FALLOS" = 0 ] && echo "TODO OK" || echo "$FALLOS FALLO(S); log en $LOG"
  exit $FALLOS
}
trap apagar INT TERM

if pgrep -f gzserver >/dev/null; then echo "Ya hay un gzserver corriendo: apágalo primero (./sim.sh stop)."; exit 1; fi
echo "Arrancando arm_sim.launch.py sin ventanas (log: $LOG)..."
ros2 launch andesrobot_arm arm_sim.launch.py gui:=false rviz:=false > "$LOG" 2>&1 &
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

# 4. Pose alcanzable (la de la validación del informe) y comprobación por TF.
ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_footprint}, pose: {position: {x: 0.5048, y: -0.2163, z: 1.862}, orientation: {x: -0.1252, y: 0.5501, z: -0.7869, w: 0.2502}}}" > /dev/null
sleep 14
tf=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint gripper_tcp 2>/dev/null | grep -m1 Translation)
err=$(echo "$tf" | python3 -c "
import sys, re, math
v = [float(x) for x in re.findall(r'-?\d+\.\d+', sys.stdin.read())[:3]]
print('%.1f' % (1000 * math.dist(v, [0.5048, -0.2163, 1.862])) if len(v) == 3 else 'nan')")
if [ "$err" != nan ] && awk "BEGIN{exit !($err < 5)}"; then ok "4. pose alcanzable: TCP a $err mm del objetivo"; else fallo "4. pose alcanzable: TCP a $err mm ($tf)"; fi

# 5. Pinza: abrir y cerrar; el dedo izquierdo (mimic) tiene que copiar con signo opuesto.
for p in 0.016 -0.007; do
  timeout 15 ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
    "{trajectory: {joint_names: [right_finger_joint], points: [{positions: [$p], time_from_start: {sec: 1}}]}}" > /dev/null 2>&1
  sleep 1
  # /joint_states lo publican dos nodos (ruedas y brazo): esperar el mensaje que trae los dedos.
  dedos=$(timeout 10 python3 - <<'EOF'
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
)
  set -- $dedos
  if [ -n "$1" ] && awk "BEGIN{exit !(($1 - $p) < 0.002 && ($1 - $p) > -0.002 && ($2 + $1) < 0.002 && ($2 + $1) > -0.002)}"; then
    ok "5. pinza en $p: derecho $1, izquierdo $2"
  else fallo "5. pinza en $p: derecho ${1:-?}, izquierdo ${2:-?}"; fi
done

apagar
