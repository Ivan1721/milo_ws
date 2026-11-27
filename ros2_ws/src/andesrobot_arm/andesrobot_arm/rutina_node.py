"""Ventana "Grabar y reproducir": guarda poses del brazo a mano y después las repite sola.

En palabras simples: se lleva el brazo (con los sliders o con el marcador de MoveIt en RViz) a cada
pose que hace falta para agarrar un objeto y en cada una se aprieta "Guardar punto". Por ejemplo:
  1. arriba del objeto, garra abierta          -> Guardar punto
  2. bajar hasta el objeto                     -> Guardar punto
  3. cerrar la garra (botón de los sliders)    -> Guardar punto
  4. subir con el objeto                       -> Guardar punto
  5. ir a dejarlo y abrir la garra             -> Guardar punto (y otro con la garra abierta)
"Reproducir" vuelve al primer punto, despacio, y hace todo de nuevo: el brazo pasa por los puntos
seguidos sin parar mientras la garra no cambia; donde cambia, primero llega al punto y después abre
o cierra (rutina.py). "Guardar archivo" deja la rutina en ros2_ws/rutinas/<nombre>.yaml y "Abrir"
la vuelve a cargar.

Manda a:  /arm_controller/follow_joint_trajectory      (joint_1..6; el lift no: está fijo)
          /gripper_controller/follow_joint_trajectory  (los dos dedos en espejo)
Lee:      /joint_states

No revisa choques (igual que los sliders): reproduce lo que se grabó. Si el robot o el objeto
cambiaron de lugar, la rutina no lo sabe. PARAR (o la seta) corta en cualquier momento.

Parámetros: velocidad [rad/s] (0.15), tiempo_minimo [s] por punto (1.0),
            carpeta (/ros2_ws/rutinas), espera_garra [s] después de mover la garra (1.5)

Uso: ros2 run andesrobot_arm rutina_node        (o brazo_real.launch.py rutina:=true; ./robot.sh
     brazo-pc la abre sola)
"""
import math
import os
import signal
import threading
import time
import tkinter as tk
from tkinter import filedialog

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from andesrobot_arm.rutina import (
    ARTICULACIONES, cargar, guardar, nuevo_punto, tiempos, tramos)
from andesrobot_arm.sliders import a_segundos_nanos, dedos

DEDOS = ('right_finger_joint', 'left_finger_joint')


class Rutina(Node):
    """La parte ROS. Reproducir corre en un hilo aparte que espera cada resultado."""

    def __init__(self):
        super().__init__('rutina_node')
        self.declare_parameter('velocidad', 0.15)
        self.declare_parameter('tiempo_minimo', 1.0)
        self.declare_parameter('carpeta', '/ros2_ws/rutinas')
        self.declare_parameter('espera_garra', 1.5)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.velocidad = p('velocidad')
        self.tiempo_minimo = p('tiempo_minimo')
        self.carpeta = p('carpeta')
        self.espera_garra = p('espera_garra')
        self.candado = threading.Lock()
        self.medido = {}
        self.puntos = []
        self.estado = 'Esperando /joint_states...'
        self.reproduciendo = False
        self.parar_pedido = False
        self.meta_activa = None
        self.create_subscription(JointState, '/joint_states', self.al_leer, 10)
        self.brazo = ActionClient(self, FollowJointTrajectory,
                                  '/arm_controller/follow_joint_trajectory')
        self.garra = ActionClient(self, FollowJointTrajectory,
                                  '/gripper_controller/follow_joint_trajectory')

    def al_leer(self, msg):
        with self.candado:
            for n, q in zip(msg.name, msg.position):
                self.medido[n] = q

    def decir(self, texto, error=False):
        self.estado = texto
        if error:
            self.get_logger().error(texto)
        else:
            self.get_logger().info(texto)

    def actual(self):
        """(6 ángulos, apertura de la garra) medidos, o None si falta algo."""
        with self.candado:
            m = dict(self.medido)
        if not all(n in m for n in ARTICULACIONES):
            return None
        return [m[n] for n in ARTICULACIONES], m.get(DEDOS[0], 0.016)

    # --- grabar ---
    def guardar_punto(self, nombre=''):
        a = self.actual()
        if a is None:
            self.decir('Todavía no llega /joint_states: no hay nada que guardar', error=True)
            return
        nombre = nombre or f'p{len(self.puntos) + 1}'
        self.puntos.append(nuevo_punto(a[0], a[1], nombre))
        self.decir(f'Guardado {nombre}: ' + ', '.join(f'{math.degrees(q):.0f}°' for q in a[0])
                   + f'; garra {a[1] * 1000:.1f} mm')

    # --- reproducir (hilo aparte) ---
    def reproducir(self, puntos=None):
        """Repite `puntos` (por defecto, toda la rutina) en un hilo aparte."""
        puntos = list(self.puntos if puntos is None else puntos)
        if self.reproduciendo or not puntos:
            return
        self.reproduciendo, self.parar_pedido = True, False
        threading.Thread(target=self._reproducir, args=(puntos,), daemon=True).start()

    def _reproducir(self, puntos):
        try:
            a = self.actual()
            if a is None:
                self.decir('Sin /joint_states: no se puede reproducir', error=True)
                return
            pasos = tramos(puntos, a[1])
            inicio = a[0]
            for i, (tipo, dato) in enumerate(pasos, 1):
                if self.parar_pedido:
                    self.decir('Reproducción detenida')
                    return
                if tipo == 'brazo':
                    self.decir(f'Paso {i}/{len(pasos)}: brazo por {len(dato)} punto(s)...')
                    if not self._esperar(self.brazo, self._meta_brazo(inicio, dato)):
                        return
                    inicio = dato[-1]
                else:
                    self.decir(f'Paso {i}/{len(pasos)}: garra a {dato * 1000:.1f} mm por dedo...')
                    if not self._esperar(self.garra, self._meta_garra(dato)):
                        return
                    time.sleep(self.espera_garra)
            self.decir(f'Rutina terminada ({len(puntos)} puntos)')
        finally:
            self.reproduciendo = False

    def _meta_brazo(self, inicio, poses):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARTICULACIONES)
        for q, t in zip(poses, tiempos(inicio, poses, self.velocidad, self.tiempo_minimo)):
            punto = JointTrajectoryPoint()
            punto.positions = [float(v) for v in q]
            punto.time_from_start.sec, punto.time_from_start.nanosec = a_segundos_nanos(t)
            goal.trajectory.points.append(punto)
        return goal

    @staticmethod
    def _meta_garra(apertura):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(DEDOS)
        punto = JointTrajectoryPoint()
        punto.positions = dedos(apertura)
        punto.time_from_start.sec = 1
        goal.trajectory.points = [punto]
        return goal

    def _esperar(self, cliente, goal):
        """Manda la meta y espera su resultado (desde el hilo de reproducir); True si salió
        bien."""
        if not cliente.wait_for_server(timeout_sec=2.0):
            self.decir(f'No hay {cliente._action_name}: ¿está corriendo brazo_real_node?',
                       error=True)
            return False
        futuro = cliente.send_goal_async(goal)
        while not futuro.done():
            time.sleep(0.05)
        handle = futuro.result()
        if not handle.accepted:
            self.decir('El controlador rechazó la meta (¿seta presionada?)', error=True)
            return False
        self.meta_activa = handle
        resultado = handle.get_result_async()
        while not resultado.done():
            if self.parar_pedido:
                handle.cancel_goal_async()
            time.sleep(0.05)
        self.meta_activa = None
        r = resultado.result().result
        if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.decir(f'No llegó: código {r.error_code} {r.error_string}', error=True)
            return False
        return not self.parar_pedido

    def parar(self):
        self.parar_pedido = True
        if self.meta_activa is not None:
            self.meta_activa.cancel_goal_async()
        self.decir('Parando...')


class Ventana:
    def __init__(self, nodo):
        self.nodo = nodo
        self.raiz = tk.Tk()
        self.raiz.title('Milo: grabar y reproducir')
        marco = tk.Frame(self.raiz, padx=8, pady=8)
        marco.pack(fill='both', expand=True)
        tk.Label(marco, text='Lleva el brazo a cada pose (sliders o MoveIt) y aprieta Guardar '
                 'punto.\nReproducir repite todo, sin revisar choques.', justify='left',
                 fg='#a33').grid(row=0, column=0, columnspan=4, sticky='w')
        tk.Label(marco, text='Nombre del punto:').grid(row=1, column=0, sticky='w')
        self.nombre = tk.Entry(marco, width=24)
        self.nombre.grid(row=1, column=1, columnspan=2, sticky='w')
        tk.Button(marco, text='Guardar punto', bg='#2a9d3f', fg='white',
                  command=self.guardar_punto).grid(row=1, column=3, sticky='we', padx=2)
        self.lista = tk.Listbox(marco, width=78, height=12, font=('DejaVu Sans Mono', 9))
        self.lista.grid(row=2, column=0, columnspan=4, pady=6)
        fila = tk.Frame(marco)
        fila.grid(row=3, column=0, columnspan=4, sticky='w')
        for texto, orden in (('Borrar último', self.borrar_ultimo), ('Borrar todo', self.borrar),
                             ('Ir al seleccionado', self.ir_a),
                             ('Guardar archivo', self.guardar_archivo),
                             ('Abrir archivo', self.abrir_archivo)):
            tk.Button(fila, text=texto, command=orden).pack(side='left', padx=2)
        fila2 = tk.Frame(marco)
        fila2.grid(row=4, column=0, columnspan=4, sticky='w', pady=4)
        tk.Button(fila2, text='REPRODUCIR', bg='#2d6cdf', fg='white', width=14,
                  command=nodo.reproducir).pack(side='left', padx=2)
        tk.Button(fila2, text='PARAR', bg='#d33', fg='white', width=10,
                  command=nodo.parar).pack(side='left', padx=2)
        tk.Label(fila2, text='  velocidad [°/s]:').pack(side='left')
        self.vel = tk.Scale(fila2, from_=2, to=15, orient='horizontal', length=140,
                            command=self.cambiar_velocidad)
        self.vel.set(round(math.degrees(nodo.velocidad)))
        self.vel.pack(side='left')
        self.estado = tk.Label(marco, text='', anchor='w', justify='left', wraplength=560)
        self.estado.grid(row=5, column=0, columnspan=4, sticky='we')
        self.raiz.after(150, self.refrescar)

    def cambiar_velocidad(self, valor):
        self.nodo.velocidad = math.radians(float(valor))

    def guardar_punto(self):
        self.nodo.guardar_punto(self.nombre.get().strip())
        self.nombre.delete(0, 'end')
        self.redibujar()

    def borrar_ultimo(self):
        if self.nodo.puntos and not self.nodo.reproduciendo:
            self.nodo.puntos.pop()
            self.redibujar()

    def borrar(self):
        if not self.nodo.reproduciendo:
            self.nodo.puntos.clear()
            self.redibujar()

    def ir_a(self):
        """Lleva el brazo (y la garra) solo al punto seleccionado, para revisarlo."""
        sel = self.lista.curselection()
        if sel:
            self.nodo.reproducir([self.nodo.puntos[sel[0]]])

    def guardar_archivo(self):
        os.makedirs(self.nodo.carpeta, exist_ok=True)
        ruta = filedialog.asksaveasfilename(initialdir=self.nodo.carpeta, defaultextension='.yaml',
                                            filetypes=[('rutina', '*.yaml')])
        if ruta:
            guardar(ruta, self.nodo.puntos, nota=time.strftime('%Y-%m-%d %H:%M'))
            self.nodo.decir(f'Rutina guardada en {ruta}')

    def abrir_archivo(self):
        ruta = filedialog.askopenfilename(initialdir=self.nodo.carpeta,
                                          filetypes=[('rutina', '*.yaml')])
        if ruta and not self.nodo.reproduciendo:
            try:
                self.nodo.puntos = cargar(ruta)
                self.nodo.decir(f'{len(self.nodo.puntos)} puntos de {ruta}')
            except (OSError, ValueError, KeyError) as error:
                self.nodo.decir(f'No pude abrir {ruta}: {error}', error=True)
            self.redibujar()

    def redibujar(self):
        self.lista.delete(0, 'end')
        for i, p in enumerate(self.nodo.puntos, 1):
            angulos = ' '.join(f'{math.degrees(q):6.0f}' for q in p['angulos'])
            self.lista.insert('end', f'{i:2d} {p["nombre"][:12]:12s} {angulos}   garra '
                                     f'{p["garra"] * 1000:5.1f} mm')

    def refrescar(self):
        self.estado.configure(text=self.nodo.estado)
        if self.nodo.estado.startswith('Esperando') and self.nodo.actual() is not None:
            self.nodo.estado = 'Listo: lleva el brazo a la primera pose y aprieta Guardar punto'
        self.raiz.after(150, self.refrescar)


def main():
    rclpy.init()
    nodo = Rutina()
    ejecutor = MultiThreadedExecutor(num_threads=2)
    ejecutor.add_node(nodo)
    hilo = threading.Thread(target=ejecutor.spin, daemon=True)
    hilo.start()
    ventana = Ventana(nodo)
    signal.signal(signal.SIGINT, lambda *_: ventana.raiz.quit())
    signal.signal(signal.SIGTERM, lambda *_: ventana.raiz.quit())
    try:
        ventana.raiz.mainloop()
    finally:
        nodo.parar()
        time.sleep(0.3)
        ejecutor.shutdown()
        hilo.join(timeout=2.0)
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
