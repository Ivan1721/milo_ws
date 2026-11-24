"""Nodo del brazo REAL: hace de puente entre ROS 2 / MoveIt y el Arduino Mega del brazo.

En palabras simples: en la simulación, MoveIt le manda las trayectorias a dos controladores de
Gazebo (arm_controller y gripper_controller) y lee las posiciones en /joint_states. En el robot
real no hay Gazebo: este nodo ofrece las MISMAS dos acciones con los mismos nombres, y publica
/joint_states, así que MoveIt, arm_moveit_node, agarre_node, etc. funcionan sin cambiar nada. Por
dentro le habla al Arduino (firmware/brazo_mega/brazo_mega.ino) por el puerto serie USB.

  Ofrece:   /arm_controller/follow_joint_trajectory      (control_msgs/FollowJointTrajectory)
            /gripper_controller/follow_joint_trajectory  (ídem, los dos dedos)
  Publica:  /joint_states  joint_1..6 (del Arduino), vertical_lift_joint (fijo, ver abajo) y los
            dedos (calculados del último ángulo pedido al servo: el servo no avisa dónde está)

Cómo sigue una trayectoria: unas `frecuencia` veces por segundo calcula dónde tendría que estar
el brazo un instante más adelante (brazo_real.interpolar) y se lo manda al Arduino con G, que
mueve cada motor a la velocidad justa para llegar en ese tiempo. Al final espera que el Arduino
diga que llegó. Si se cancela la acción, manda X (parar). Si el nodo se cae, el Arduino se detiene
solo (perro guardián del firmware).

El lift (vertical_lift_joint): el Arduino del brazo no lo mueve. Se publica fijo en `lift_fijo` y
se rechaza una trayectoria que lo mueva más de `tolerancia_lift` (si no, el brazo llegaría a otra
parte que la planificada).

Parámetros:
  puerto          /dev/arduino_brazo (o /dev/ttyUSB0)
  config          ruta de brazo_real.yaml (por defecto, la instalada)
  frecuencia      [Hz] de G y de /joint_states (20)
  lift_fijo       [m] (0.0) y tolerancia_lift [m] (0.005)
  tolerancia      [rad] para dar por llegada una trayectoria (0.01)
  espera_final    [s] cuánto esperar después del último punto a que el Arduino llegue (5.0)
  tiempo_garra    [s] que se espera después de mover el servo (1.0)
  vel_max_pasos   [pasos/s] la VEL_MAX del firmware (2000): avisa si una trayectoria pide más

Uso: ros2 launch andesrobot_arm brazo_real.launch.py   (o ./robot.sh brazo)
     ros2 run andesrobot_arm brazo_real_node --ros-args -p puerto:=/dev/ttyUSB0
"""
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from andesrobot_arm.brazo_real import (
    ARTICULACIONES, DEDO_ABIERTA, DEDOS, LIFT, Enlace, cmd_ir, dedo_a_servo, interpolar,
    leer_config, leer_estado, servo_a_dedo)

Resultado = FollowJointTrajectory.Result


class BrazoReal(Node):
    def __init__(self):
        super().__init__('brazo_real_node')
        compartido = get_package_share_directory('andesrobot_arm')
        self.declare_parameter('puerto', '/dev/arduino_brazo')
        self.declare_parameter('config', os.path.join(compartido, 'config', 'brazo_real.yaml'))
        self.declare_parameter('frecuencia', 20.0)
        self.declare_parameter('lift_fijo', 0.0)
        self.declare_parameter('tolerancia_lift', 0.005)
        self.declare_parameter('tolerancia', 0.01)
        self.declare_parameter('espera_final', 5.0)
        self.declare_parameter('tiempo_garra', 1.0)
        self.declare_parameter('vel_max_pasos', 2000.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.periodo = 1.0 / p('frecuencia')
        self.lift = p('lift_fijo')
        self.tol_lift = p('tolerancia_lift')
        self.tol = p('tolerancia')
        self.espera_final = p('espera_final')
        self.tiempo_garra = p('tiempo_garra')
        self.vel_max_pasos = p('vel_max_pasos')
        with open(p('config')) as f:
            datos = yaml.safe_load(f)
        self.arts = leer_config(datos)
        garra = datos.get('garra', {})
        self.servo_abierta = garra.get('servo_abierta', 40)
        self.servo_cerrada = garra.get('servo_cerrada', 110)

        # El puerto serie: un solo hilo a la vez (el timer y las acciones lo comparten).
        self.candado = threading.Lock()
        puerto = p('puerto')
        try:
            self.enlace = Enlace(puerto)
        except OSError as error:
            raise SystemExit(f'No pude abrir {puerto}: {error}. ¿Está conectado el Arduino? '
                             '(ls -l /dev/ttyUSB*, firmware/README.md)')
        respuesta, reinicio = self.enlace.conectar()
        if not respuesta or not respuesta.startswith('PONG brazo_mega'):
            raise SystemExit(f'El Arduino de {puerto} no contesta como brazo_mega (P -> '
                             f'{respuesta!r}). ¿Está cargado firmware/brazo_mega/brazo_mega.ino?')
        self.get_logger().info(f'Arduino en {puerto}: {respuesta}')
        if reinicio:
            self.get_logger().warn(
                'El Arduino se acaba de reiniciar: cuenta 0 pasos en TODAS las articulaciones, '
                'o sea cree que el brazo está en la pose "cero" de brazo_real.yaml. Si no está '
                'ahí, parar y llevarlo (o mover y usar Z).')
        else:
            self.get_logger().info('El Arduino no se reinició: sigue con su cuenta de pasos.')

        self.q = [a.a_rad(0) for a in self.arts]   # última posición leída [rad]
        self.moviendo = False
        self.dedo = DEDO_ABIERTA                    # último pedido a la garra [m]
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(self.periodo, self.leer_y_publicar,
                          callback_group=MutuallyExclusiveCallbackGroup())
        grupo = ReentrantCallbackGroup()
        self.ocupado = threading.Lock()             # una trayectoria del brazo a la vez
        ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                     execute_callback=self.ejecutar_brazo, goal_callback=self.aceptar_brazo,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=grupo)
        ActionServer(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory',
                     execute_callback=self.ejecutar_garra,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=grupo)
        self.get_logger().info(
            'Listo: /arm_controller y /gripper_controller (follow_joint_trajectory) ofrecidos; '
            f'lift fijo en {self.lift} m. El Arduino cuenta 0 en la pose "cero" de '
            'brazo_real.yaml: encenderlo con el brazo ahí.')

    # --- puerto serie ---
    def preguntar(self, comando, espera=0.3):
        with self.candado:
            return self.enlace.preguntar(comando, espera)

    def leer_y_publicar(self):
        """Pide el estado al Arduino y publica /joint_states."""
        try:
            pasos, self.moviendo, servo = leer_estado(self.preguntar('E'))
            self.q = [a.a_rad(n) for a, n in zip(self.arts, pasos)]
        except ValueError:
            self.get_logger().warn('El Arduino no contestó el estado', throttle_duration_sec=2.0)
            return
        if servo is not None:
            self.dedo = servo_a_dedo(servo, self.servo_abierta, self.servo_cerrada)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [LIFT] + ARTICULACIONES + list(DEDOS)
        msg.position = [self.lift] + list(self.q) + [self.dedo, -self.dedo]
        self.pub.publish(msg)

    # --- brazo ---
    def aceptar_brazo(self, goal):
        nombres = list(goal.trajectory.joint_names)
        desconocidas = set(nombres) - set(ARTICULACIONES) - {LIFT}
        if desconocidas or not goal.trajectory.points:
            self.get_logger().error(f'Trayectoria rechazada: articulaciones {desconocidas} o '
                                    'sin puntos')
            return GoalResponse.REJECT
        if LIFT in nombres:
            k = nombres.index(LIFT)
            peor = max(abs(pt.positions[k] - self.lift) for pt in goal.trajectory.points)
            if peor > self.tol_lift:
                self.get_logger().error(
                    f'Trayectoria rechazada: mueve el lift {peor * 1000:.0f} mm y en el robot '
                    'real el lift no lo controla este nodo (lift_fijo).')
                return GoalResponse.REJECT
        if self.ocupado.locked():
            self.get_logger().error('Trayectoria rechazada: ya hay otra en curso')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def ejecutar_brazo(self, goal_handle):
        with self.ocupado:
            return self._seguir(goal_handle)

    def _seguir(self, goal_handle):
        tr = goal_handle.request.trajectory
        nombres = list(tr.joint_names)
        # Cada punto, para las 6 articulaciones: las que no vienen en la trayectoria se quedan
        # donde están.
        actual = list(self.q)
        tiempos, puntos = [0.0], [actual]
        for pt in tr.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            fila = [pt.positions[nombres.index(n)] if n in nombres else actual[i]
                    for i, n in enumerate(ARTICULACIONES)]
            if t <= tiempos[-1]:
                t = tiempos[-1] + 1e-3
            tiempos.append(t)
            puntos.append(fila)
        resultado = Resultado()
        ms = self.periodo * 1000.0
        # La velocidad que pide la trayectoria, en pasos/s del motor, tramo por tramo.
        pide = max((abs(a.a_pasos(q1) - a.a_pasos(q0)) / (t1 - t0)
                    for k in range(1, len(tiempos))
                    for a, q0, q1, t0, t1 in ((art, puntos[k - 1][i], puntos[k][i],
                                               tiempos[k - 1], tiempos[k])
                                              for i, art in enumerate(self.arts))),
                   default=0.0)
        if pide > self.vel_max_pasos:
            self.get_logger().warn(
                f'La trayectoria pide hasta {pide:.0f} pasos/s y el Arduino da '
                f'{self.vel_max_pasos:.0f}: el brazo va a ir atrasado respecto del plan. '
                'Bajar velocity_scaling de arm_moveit_node.')
        inicio = time.monotonic()
        self.get_logger().info(f'Trayectoria de {len(tr.points)} puntos, {tiempos[-1]:.1f} s')
        while True:
            if goal_handle.is_cancel_requested:
                self.preguntar('X')
                goal_handle.canceled()
                resultado.error_code = Resultado.SUCCESSFUL
                resultado.error_string = 'cancelada: brazo detenido'
                return resultado
            t = time.monotonic() - inicio
            # Dónde tiene que estar dentro de un período: llega justo cuando sale el próximo G.
            meta = interpolar(tiempos, puntos, t + self.periodo)
            r = self.preguntar(cmd_ir([a.a_pasos(q) for a, q in zip(self.arts, meta)], ms))
            if r != 'OK G':
                self.preguntar('X')
                goal_handle.abort()
                resultado.error_code = Resultado.INVALID_GOAL
                resultado.error_string = f'el Arduino respondió {r!r} a G'
                self.get_logger().error(resultado.error_string)
                return resultado
            if t >= tiempos[-1]:
                break
            time.sleep(max(0.0, self.periodo - ((time.monotonic() - inicio) - t)))
        # Esperar que llegue al último punto.
        final = puntos[-1]
        limite = time.monotonic() + self.espera_final
        while time.monotonic() < limite:
            error = max(abs(a - b) for a, b in zip(self.q, final))
            if not self.moviendo and error <= self.tol:
                goal_handle.succeed()
                resultado.error_code = Resultado.SUCCESSFUL
                self.get_logger().info(f'Objetivo alcanzado (error {error:.4f} rad)')
                return resultado
            time.sleep(self.periodo)
        goal_handle.abort()
        resultado.error_code = Resultado.GOAL_TOLERANCE_VIOLATED
        resultado.error_string = (f'no llegó en {self.espera_final} s (error '
                                  f'{max(abs(a - b) for a, b in zip(self.q, final)):.4f} rad)')
        self.get_logger().error(resultado.error_string)
        return resultado

    # --- garra ---
    def ejecutar_garra(self, goal_handle):
        tr = goal_handle.request.trajectory
        resultado = Resultado()
        nombres = list(tr.joint_names)
        if not tr.points or not any(n in nombres for n in DEDOS):
            goal_handle.abort()
            resultado.error_code = Resultado.INVALID_JOINTS
            return resultado
        ultimo = tr.points[-1].positions
        q = (ultimo[nombres.index(DEDOS[0])] if DEDOS[0] in nombres
             else -ultimo[nombres.index(DEDOS[1])])
        angulo = dedo_a_servo(q, self.servo_abierta, self.servo_cerrada)
        r = self.preguntar(f'S {angulo}')
        if not r or not r.startswith('OK S'):
            goal_handle.abort()
            resultado.error_code = Resultado.INVALID_GOAL
            resultado.error_string = f'el Arduino respondió {r!r} a S'
            return resultado
        self.get_logger().info(f'Garra: dedo {q * 1000:.1f} mm -> servo {angulo}°')
        time.sleep(self.tiempo_garra)
        goal_handle.succeed()
        resultado.error_code = Resultado.SUCCESSFUL
        return resultado

    def cerrar(self):
        try:
            self.preguntar('X')
        finally:
            self.enlace.cerrar()


def main():
    rclpy.init()
    nodo = BrazoReal()
    ejecutor = MultiThreadedExecutor(num_threads=4)
    ejecutor.add_node(nodo)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
