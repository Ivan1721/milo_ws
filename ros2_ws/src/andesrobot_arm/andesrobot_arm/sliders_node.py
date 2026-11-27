"""Ventana con un slider por articulación para mover el brazo a mano, sin MoveIt.

En palabras simples: sirve para las primeras pruebas con los motores (ver el sentido de giro de
cada eje, calibrar config/brazo_real.yaml, abrir y cerrar la garra). Cada slider es el ángulo de
una articulación en grados; al SOLTARLO, el nodo manda ese ángulo como una trayectoria de un solo
punto al mismo controlador que usa MoveIt. Así funciona igual con el brazo real (brazo_real_node,
que ofrece esas acciones) y con la simulación (los controladores de Gazebo).

  Manda a:  /arm_controller/follow_joint_trajectory      (joint_1..6; con lift:=true también
                                                          vertical_lift_joint, solo en simulación)
            /gripper_controller/follow_joint_trajectory  (los dos dedos en espejo: [q, -q])
  Lee:      /joint_states       el ángulo medido, que se muestra al lado de cada slider
            /robot_description  los topes del URDF, para que el slider no deje pedir más

OJO: no revisa choques (MoveIt sí). Un slider mueve UNA articulación y el brazo puede chocar con la
columna, la base o la mesa. Va despacio a propósito (`velocidad`) y el botón «Parar» detiene el
brazo (cancela la trayectoria; brazo_real_node le manda X al Arduino).

Si se mueve un slider mientras el brazo todavía va a la meta anterior, se cancela esa y se manda
la nueva cuando el controlador confirma que paró.

Parámetros:
  velocidad       [rad/s] la más rápida que se deja ir a una articulación (0.1 ≈ 6°/s)
  velocidad_lift  [m/s] lo mismo para el lift (0.02)
  lift            true para agregar el slider del lift (solo simulación: el Arduino no lo mueve)
  garra           false para no mostrar la garra
  tiempo_minimo   [s] ningún movimiento dura menos que esto (1.0)

Uso:
  Brazo real, sin MoveIt:   ./robot.sh brazo moveit:=false sliders:=true
  Simulación (./sim.sh brazo en otra terminal):
      ./sim.sh shell
      ros2 run andesrobot_arm sliders_node --ros-args -p use_sim_time:=true -p lift:=true
"""
import math
import signal
import threading
import time
import tkinter as tk

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from andesrobot_arm.sliders import (
    DEDO_ABIERTA, DEDO_CERRADA, a_segundos_nanos, acotar, dedos, duracion, limites_de_urdf)

ARTICULACIONES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
LIFT = 'vertical_lift_joint'
DEDOS = ('right_finger_joint', 'left_finger_joint')


class Sliders(Node):
    """La parte ROS: lee /joint_states y /robot_description y manda las metas. Los callbacks
    corren en el hilo de ROS y solo guardan datos; la ventana (otro hilo) los lee cada 100 ms."""

    def __init__(self):
        super().__init__('sliders_node')
        self.declare_parameter('velocidad', 0.1)
        self.declare_parameter('velocidad_lift', 0.02)
        self.declare_parameter('lift', False)
        self.declare_parameter('garra', True)
        self.declare_parameter('tiempo_minimo', 1.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.velocidad = p('velocidad')
        self.velocidad_lift = p('velocidad_lift')
        self.con_garra = p('garra')
        self.tiempo_minimo = p('tiempo_minimo')
        self.nombres = ([LIFT] if p('lift') else []) + ARTICULACIONES

        self.candado = threading.Lock()
        self.medido = {}             # {articulación: posición} del último /joint_states
        self.limites = None          # {articulación: (inf, sup)} del URDF, cuando llegue
        self.estado = 'Esperando /joint_states...'
        self.meta_activa = None      # goal handle de la trayectoria en curso
        self.pendiente = None        # meta nueva que espera a que la anterior termine de parar

        self.create_subscription(JointState, '/joint_states', self.al_leer, 10)
        # robot_state_publisher publica el URDF una vez, "transient local": el que se suscribe
        # tarde lo recibe igual.
        latcheado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.al_urdf, latcheado)
        self.brazo = ActionClient(self, FollowJointTrajectory,
                                  '/arm_controller/follow_joint_trajectory')
        self.garra = ActionClient(self, FollowJointTrajectory,
                                  '/gripper_controller/follow_joint_trajectory')
        # Solo el brazo real (brazo_real_node) ofrece /arm/fijar_cero; en Gazebo no existe.
        self.cliente_cero = self.create_client(Trigger, '/arm/fijar_cero')

    # --- lo que llega ---
    def al_leer(self, msg):
        with self.candado:
            for n, q in zip(msg.name, msg.position):
                self.medido[n] = q

    def al_urdf(self, msg):
        try:
            limites = limites_de_urdf(msg.data, self.nombres)
        except Exception as error:  # noqa: BLE001  (un URDF roto no debe tumbar la ventana)
            self.get_logger().warn(f'No pude leer los topes del URDF: {error}')
            return
        with self.candado:
            self.limites = limites
        self.get_logger().info('Topes leídos del URDF: ' + ', '.join(
            f'{n} [{math.degrees(a):.0f}°, {math.degrees(b):.0f}°]'
            for n, (a, b) in limites.items() if n in ARTICULACIONES))

    def poner_estado(self, texto, error=False):
        self.estado = texto
        # Dos llamadas separadas: rclpy no deja usar la MISMA línea con dos severidades distintas
        # ("Logger severity cannot be changed between calls"; tumbó el nodo el 26-nov al rechazar
        # el controlador una meta, después de varios mensajes info desde aquí).
        if error:
            self.get_logger().error(texto)
        else:
            self.get_logger().info(texto)

    # --- brazo ---
    def mover(self, metas):
        """metas = {articulación: posición}. Si hay una trayectoria en curso, la cancela y deja
        esta pendiente (se manda cuando llega el resultado de la anterior)."""
        if self.meta_activa is not None:
            self.pendiente = metas
            self.meta_activa.cancel_goal_async()
            self.poner_estado('Cancelando el movimiento anterior...')
            return
        self._mandar(metas)

    def _mandar(self, metas):
        if not self.brazo.server_is_ready():
            self.poner_estado('No hay /arm_controller/follow_joint_trajectory: ¿está corriendo '
                              'brazo_real_node o la simulación?', error=True)
            return
        with self.candado:
            actual = [self.medido.get(n, metas[n]) for n in metas]
        nombres = list(metas)
        vel = [self.velocidad_lift if n == LIFT else self.velocidad for n in nombres]
        t = duracion(actual, [metas[n] for n in nombres], vel, self.tiempo_minimo)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = nombres
        punto = JointTrajectoryPoint()
        punto.positions = [float(metas[n]) for n in nombres]
        punto.velocities = [0.0] * len(nombres)
        punto.time_from_start.sec, punto.time_from_start.nanosec = a_segundos_nanos(t)
        goal.trajectory.points = [punto]
        self.poner_estado(f'Moviendo ({t:.1f} s)...')
        self.brazo.send_goal_async(goal).add_done_callback(self._aceptada)

    def _aceptada(self, futuro):
        handle = futuro.result()
        if not handle.accepted:
            self.poner_estado('El controlador rechazó la meta (mirar su terminal)', error=True)
            return
        self.meta_activa = handle
        handle.get_result_async().add_done_callback(self._terminada)

    def _terminada(self, futuro):
        self.meta_activa = None
        r = futuro.result().result
        if self.pendiente is not None:
            metas, self.pendiente = self.pendiente, None
            self._mandar(metas)
            return
        if r.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.poner_estado('Listo' + (f' ({r.error_string})' if r.error_string else ''))
        else:
            self.poner_estado(f'No llegó: código {r.error_code} {r.error_string}', error=True)

    def parar(self):
        self.pendiente = None
        if self.meta_activa is not None:
            self.meta_activa.cancel_goal_async()
            self.poner_estado('Parando...')

    # --- cero ---
    def fijar_cero(self):
        """Pide a brazo_real_node que la pose actual sea la pose de arranque (Z)."""
        if not self.cliente_cero.service_is_ready():
            self.poner_estado('No hay /arm/fijar_cero (solo existe con el brazo real)', error=True)
            return
        futuro = self.cliente_cero.call_async(Trigger.Request())
        futuro.add_done_callback(lambda f: self.poner_estado(
            f.result().message, error=not f.result().success))

    # --- garra ---
    def mover_garra(self, apertura):
        if not self.garra.server_is_ready():
            self.poner_estado('No hay /gripper_controller/follow_joint_trajectory', error=True)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(DEDOS)
        punto = JointTrajectoryPoint()
        punto.positions = dedos(apertura)
        punto.time_from_start.sec = 1
        goal.trajectory.points = [punto]
        self.poner_estado(f'Garra a {apertura * 1000:.1f} mm por dedo')
        self.garra.send_goal_async(goal)


class Ventana:
    """La parte Tk: un slider por articulación (grados; mm para el lift y la garra)."""

    def __init__(self, nodo):
        self.nodo = nodo
        self.raiz = tk.Tk()
        self.raiz.title('Milo: brazo por articulación')
        self.filas = {}          # {articulación: (Scale, Label medido)}
        self.listo = False       # los sliders se acomodan a la posición medida una vez
        marco = tk.Frame(self.raiz, padx=8, pady=8)
        marco.pack(fill='both', expand=True)
        tk.Label(marco, text='Suelta el slider para mover. Sin chequeo de choques: ir despacio.',
                 fg='#a33').grid(row=0, column=0, columnspan=3, sticky='w')
        fila = 1
        for n in nodo.nombres:
            self._fila(marco, fila, n)
            fila += 1
        if nodo.con_garra:
            self._fila(marco, fila, DEDOS[0], texto='garra (mm por dedo)')
            fila += 1
        botones = tk.Frame(marco)
        botones.grid(row=fila, column=0, columnspan=3, pady=6, sticky='w')
        tk.Button(botones, text='PARAR', bg='#d33', fg='white', width=10,
                  command=nodo.parar).pack(side='left', padx=2)
        tk.Button(botones, text='Leer posición', command=self.sincronizar).pack(side='left',
                                                                                padx=2)
        tk.Button(botones, text='Todo a cero', command=self.a_cero).pack(side='left', padx=2)
        tk.Button(botones, text='Fijar cero aquí',
                  command=self.fijar_cero).pack(side='left', padx=2)
        if nodo.con_garra:
            tk.Button(botones, text='Abrir garra',
                      command=lambda: self.garra_a(DEDO_ABIERTA)).pack(side='left', padx=2)
            tk.Button(botones, text='Cerrar garra',
                      command=lambda: self.garra_a(DEDO_CERRADA)).pack(side='left', padx=2)
        self.estado = tk.Label(marco, text='', anchor='w', justify='left', wraplength=520)
        self.estado.grid(row=fila + 1, column=0, columnspan=3, sticky='we')
        self.raiz.after(100, self.refrescar)

    @staticmethod
    def unidad(nombre):
        """Cuánto vale 1 en el slider: 1° para las articulaciones, 1 mm para lift y dedos."""
        return 0.001 if nombre in (LIFT,) + DEDOS else math.radians(1)

    def _fila(self, marco, fila, nombre, texto=None):
        u = self.unidad(nombre)
        if nombre in DEDOS:
            inf, sup = DEDO_CERRADA, DEDO_ABIERTA
        else:
            inf, sup = (-0.4, 0.0) if nombre == LIFT else (-math.pi, math.pi)
        tk.Label(marco, text=texto or nombre, width=20, anchor='w').grid(row=fila, column=0)
        escala = tk.Scale(marco, from_=inf / u, to=sup / u, orient='horizontal', length=380,
                          resolution=0.5 if nombre not in DEDOS else 0.1, state='disabled')
        escala.grid(row=fila, column=1)
        # Mover al soltar el mouse, o al soltar una flecha del teclado (paso fino).
        escala.bind('<ButtonRelease-1>', lambda _e, n=nombre: self.soltar(n))
        escala.bind('<KeyRelease>', lambda _e, n=nombre: self.soltar(n))
        medido = tk.Label(marco, text='—', width=16, anchor='w')
        medido.grid(row=fila, column=2)
        self.filas[nombre] = (escala, medido)

    def poner_topes(self, limites):
        for n, (escala, _) in self.filas.items():
            if n in limites and n not in DEDOS:
                u = self.unidad(n)
                escala.configure(from_=limites[n][0] / u, to=limites[n][1] / u)

    def sincronizar(self, salvo=()):
        """Pone cada slider (menos los de `salvo`) en la posición medida, para no mandar un salto
        al tocar otro."""
        with self.nodo.candado:
            medido = dict(self.nodo.medido)
        for n, (escala, _) in self.filas.items():
            if n in medido and n not in salvo:
                escala.configure(state='normal')
                escala.set(medido[n] / self.unidad(n))

    def metas_brazo(self):
        return {n: self.filas[n][0].get() * self.unidad(n) for n in self.nodo.nombres}

    def soltar(self, nombre):
        if not self.listo:
            return
        if nombre in DEDOS:
            self.nodo.mover_garra(self.filas[nombre][0].get() * 0.001)
            return
        # Si el brazo está quieto, las demás articulaciones se quedan donde ESTÁN (lo medido), no
        # donde quedaron sus sliders: si MoveIt u otro nodo movió el brazo entretanto, mandar los
        # sliders viejos lo devolvería entero. Si está yendo a una meta nuestra, se respetan las
        # metas de los otros sliders (se tocaron varios seguidos).
        if self.nodo.meta_activa is None and self.nodo.pendiente is None:
            self.sincronizar(salvo=(nombre,) + DEDOS)
        metas = self.metas_brazo()
        with self.nodo.candado:
            limites = dict(self.nodo.limites or {})
        for n in metas:
            if n in limites:
                metas[n] = acotar(metas[n], *limites[n])
        self.nodo.mover(metas)

    def a_cero(self):
        """joint_1..6 a 0° (la pose cero del URDF); el lift se queda donde está."""
        if not self.listo:
            return
        self.sincronizar(salvo=DEDOS)
        for n in ARTICULACIONES:
            self.filas[n][0].set(0)
        self.nodo.mover(self.metas_brazo())

    def fijar_cero(self):
        """Con el brazo real QUIETO en la pose de arranque (paso 9 de firmware/README.md): desde
        ahora el Arduino cuenta 0 ahí. Pide confirmación porque si el brazo no está en esa pose,
        RViz y MoveIt quedan corridos."""
        from tkinter import messagebox
        if messagebox.askyesno('Fijar cero', '¿El brazo real está QUIETO en la pose de arranque '
                               '(estirado al frente, muñeca arriba)?\n\nDesde ahora esa pose '
                               'será el cero del Arduino.'):
            self.nodo.fijar_cero()
            self.raiz.after(500, self.sincronizar)

    def garra_a(self, q):
        self.filas[DEDOS[0]][0].set(q * 1000)
        self.nodo.mover_garra(q)

    def refrescar(self):
        with self.nodo.candado:
            medido = dict(self.nodo.medido)
            limites = self.nodo.limites
        if limites is not None and not getattr(self, '_topes', False):
            self.poner_topes(limites)
            self._topes = True
        if not self.listo and all(n in medido for n in self.nodo.nombres):
            self.sincronizar()
            self.listo = True
            self.nodo.estado = 'Listo: suelta un slider para mover esa articulación'
        for n, (_, etiqueta) in self.filas.items():
            if n in medido:
                if n in (LIFT,) + DEDOS:
                    etiqueta.configure(text=f'medido {medido[n] * 1000:6.1f} mm')
                else:
                    etiqueta.configure(text=f'medido {math.degrees(medido[n]):7.1f}°')
        self.estado.configure(text=self.nodo.estado)
        self.raiz.after(100, self.refrescar)


def main():
    rclpy.init()
    nodo = Sliders()
    ejecutor = MultiThreadedExecutor(num_threads=2)
    ejecutor.add_node(nodo)
    hilo = threading.Thread(target=ejecutor.spin, daemon=True)
    hilo.start()
    ventana = Ventana(nodo)
    # Ctrl+C (o el launch al cerrarse): Tk no deja pasar el KeyboardInterrupt, así que se cierra
    # la ventana a mano; el `after` de refrescar le da a Python la ocasión de atender la señal.
    signal.signal(signal.SIGINT, lambda *_: ventana.raiz.quit())
    signal.signal(signal.SIGTERM, lambda *_: ventana.raiz.quit())
    try:
        ventana.raiz.mainloop()
    finally:
        nodo.parar()          # si el brazo se estaba moviendo, que pare
        time.sleep(0.3)       # que alcance a salir la cancelación
        ejecutor.shutdown()
        hilo.join(timeout=2.0)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
