#!/usr/bin/env python3
"""Prueba de la pinza mandada por fuerza: ¿se frena contra un objeto sin atravesarlo?

Fase 1 del plan de agarre (docs/PENDIENTES.md, P15). Los dedos se mandan por ESFUERZO con un
tope de fuerza (gripper_force en el URDF), como el servo real: al cerrar sobre algo rígido el
dedo se frena y sigue empujando. Esta prueba lo comprueba con cajas de ancho conocido:

  1. abre la pinza;
  2. pone una caja FIJA (estática: no la mueve nada) entre los dedos, en la pose del TCP leída
     por TF (en la simulación odom = mundo de Gazebo);
  3. manda cerrar del todo (-0.007) y mide durante 3 s dónde quedan los dedos y cuánto tiemblan;
  4. abre y borra la caja.
Al final cierra en vacío, que tiene que llegar a -0.007.

Los dos dedos se mandan en espejo (izquierdo = −derecho). La apertura de la pinza es
0.017 + q_der − q_izq (las caras de los dedos están a ±8.5 mm del TCP con q = 0).
Pasa si la apertura queda a menos de 1 mm del ancho de la caja y tiembla menos de 1 mm (rango
en 3 s). Los dedos pueden quedar corridos los dos hacia un lado (se informa, no cuenta): al
cerrar del todo cada uno empuja con el tope y nada los centra.

La caja mide 15 mm de alto y de fondo: más chica que la cara del dedo (21 mm de alto). Con 20 mm
tocaba justo el borde de la cara y los dedos temblaban hasta 13 mm (medido el 7-nov): para
agarrar bien, el objeto tiene que apoyar en la cara plana, no en una arista.

Uso (con ./sim.sh brazo corriendo, o sin ventanas:
  ros2 launch andesrobot_arm arm_sim.launch.py gui:=false rviz:=false mesa:=false camara:=false):
  python3 /ros2_ws/src/andesrobot_arm/scripts/prueba_pinza.py        # 10, 20, 30, 40 y 45 mm
  python3 /ros2_ws/src/andesrobot_arm/scripts/prueba_pinza.py 0.03   # una sola caja
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ABIERTA, CERRADA = 0.016, -0.007   # límites de right_finger_joint [m]
TOLERANCIA = 0.001                 # error y temblor máximos aceptados [m]


def sdf_caja(ancho):
    """Caja estática: ancho [m] a lo largo del eje de los dedos (Y de link_6_1)."""
    tam = f'0.015 {ancho} 0.015'
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="caja_pinza">'
            '<static>true</static><link name="link">'
            f'<collision name="c"><geometry><box><size>{tam}</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{tam}</size></box></geometry></visual>'
            '</link></model></sdf>')


class PruebaPinza(Node):
    def __init__(self):
        super().__init__('prueba_pinza')
        self.dedos = None   # (derecho, izquierdo) [m]
        self.create_subscription(JointState, '/joint_states', self.al_leer, 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.accion = ActionClient(self, FollowJointTrajectory,
                                   '/gripper_controller/follow_joint_trajectory')
        self.poner = self.create_client(SpawnEntity, '/spawn_entity')
        self.sacar = self.create_client(DeleteEntity, '/delete_entity')

    def al_leer(self, m):
        # Hay dos publicadores de /joint_states (ruedas y brazo): solo sirve el que trae los dedos.
        if 'right_finger_joint' in m.name and 'left_finger_joint' in m.name:
            self.dedos = (m.position[m.name.index('right_finger_joint')],
                          m.position[m.name.index('left_finger_joint')])

    def esperar(self, segundos):
        fin = time.time() + segundos
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)

    def resultado(self, futuro):
        while not futuro.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return futuro.result()

    def pinza(self, posicion):
        """Manda los dedos en espejo (derecho = posicion, izquierdo = −posicion) en 1 s y espera a
        que el controlador termine."""
        self.accion.wait_for_server()
        meta = FollowJointTrajectory.Goal()
        meta.trajectory.joint_names = ['right_finger_joint', 'left_finger_joint']
        meta.trajectory.points = [JointTrajectoryPoint(positions=[posicion, -posicion],
                                                       time_from_start=Duration(sec=1))]
        manejo = self.resultado(self.accion.send_goal_async(meta))
        self.resultado(manejo.get_result_async())
        self.esperar(1.0)

    def medir(self, segundos):
        """Promedio y rango (máximo − mínimo) durante unos segundos de: dedo derecho, dedo
        izquierdo y apertura de la pinza (0.017 + derecho − izquierdo)."""
        muestras = []
        fin = time.time() + segundos
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)
            der, izq = self.dedos
            muestras.append((der, izq, 0.017 + der - izq))
        a = np.array(muestras)
        return a.mean(0), a.max(0) - a.min(0)

    def poner_en_tcp(self, nombre, xml, dz=0.0):
        """Pone un modelo en la pose del TCP, girado como la pinza y corrido dz [m] en Z del
        mundo (lo usa también prueba_levantar.py)."""
        t = self.tf.lookup_transform('odom', 'gripper_tcp', rclpy.time.Time())
        pedido = SpawnEntity.Request(name=nombre, xml=xml, reference_frame='world')
        pedido.initial_pose.position.x = t.transform.translation.x
        pedido.initial_pose.position.y = t.transform.translation.y
        pedido.initial_pose.position.z = t.transform.translation.z + dz
        pedido.initial_pose.orientation = t.transform.rotation
        self.poner.wait_for_service()
        return self.resultado(self.poner.call_async(pedido)).success

    def caja(self, ancho):
        """Pone la caja centrada en el TCP, girada como la pinza."""
        return self.poner_en_tcp('caja_pinza', sdf_caja(ancho))

    def probar(self, ancho):
        self.pinza(ABIERTA)
        if not self.caja(ancho):
            print(f'FALLO caja de {ancho * 1000:.0f} mm: no se pudo poner')
            return False
        self.esperar(0.5)
        self.pinza(CERRADA)
        (der, izq, apertura), rango = self.medir(3.0)
        error = abs(apertura - ancho)
        bien = error < TOLERANCIA and rango[2] < TOLERANCIA
        # Corrimiento lateral: cuánto se movieron los dos dedos juntos hacia un lado. Al cerrar
        # del todo cada dedo empuja con el tope de fuerza, así que nada los centra (equilibrio
        # neutro) y el par se mece de lado sin cambiar la apertura. Se informa pero no cuenta:
        # para agarrar se manda un poco menos que el ancho (ver prueba_levantar.py).
        lado = (der + izq) / 2
        print(f'{"OK   " if bien else "FALLO"} caja de {ancho * 1000:4.1f} mm: apertura '
              f'{apertura * 1000:5.1f} mm, error {error * 1000:.2f} mm, temblor '
              f'{rango[2] * 1000:.2f} mm (dedos {der * 1000:+.1f} / {izq * 1000:+.1f} mm, '
              f'corridos {lado * 1000:+.1f} mm hacia un lado)')
        self.pinza(ABIERTA)
        self.resultado(self.sacar.call_async(DeleteEntity.Request(name='caja_pinza')))
        return bien


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('anchos', nargs='*', type=float,
                    default=[0.01, 0.02, 0.03, 0.04, 0.045],
                    help='anchos de caja [m], entre 0.003 y 0.049 (lo que abre la pinza)')
    a = ap.parse_args()
    rclpy.init()
    nodo = PruebaPinza()
    nodo.esperar(2.0)   # TF y /joint_states
    if nodo.dedos is None:
        sys.exit('No llega /joint_states con los dedos: ¿está corriendo arm_sim?')
    bien = [nodo.probar(w) for w in a.anchos]
    nodo.pinza(CERRADA)
    (der, izq, _), _ = nodo.medir(1.0)
    vacio = abs(der - CERRADA) < TOLERANCIA and abs(izq + CERRADA) < TOLERANCIA
    print(f'{"OK   " if vacio else "FALLO"} cerrar en vacío: dedos {der * 1000:+.1f} / '
          f'{izq * 1000:+.1f} mm (esperado {CERRADA * 1000:+.1f} / {-CERRADA * 1000:+.1f})')
    rclpy.shutdown()
    sys.exit(0 if all(bien) and vacio else 1)


if __name__ == '__main__':
    main()
