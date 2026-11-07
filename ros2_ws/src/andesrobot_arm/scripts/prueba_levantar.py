#!/usr/bin/env python3
"""Prueba de la pinza con un objeto SUELTO: ¿lo levanta y lo sostiene sin que resbale?

Fase 2 del plan de agarre (docs/PENDIENTES.md, P15). La fase 1 (scripts/prueba_pinza.py) probó
que los dedos, mandados por fuerza, se frenan contra una caja FIJA. Ahora la caja está suelta:
solo la sostiene el roce de los dedos (μ = 1 en las caras, 20 N por dedo), así que si la fuerza o
el roce no alcanzan, se cae o resbala. Pasos:

  1. abre la pinza;
  2. pone un pedestal fijo bajo el TCP y, encima, la caja suelta (con masa) entre los dedos.
     El pedestal es más angosto que la caja para que no lo toquen los dedos (las caras de los
     dedos miden 21 mm de alto y la caja 15 mm: bajan 3 mm más que ella);
  3. cierra (-0.007) y mide dónde quedaron los dedos, como en la fase 1;
  4. sube el lift `subir` m (5 cm): el TCP sube en línea recta y el pedestal queda abajo;
  5. sostiene `sostener` s (10 s) y mide cuánto se corre la caja respecto del TCP;
  6. baja el lift, abre y borra la caja y el pedestal.

La posición de la caja la publica ella misma: su SDF trae el plugin gazebo_ros_p3d, que da su
pose en el mundo en /caja_suelta/odom. La del TCP sale de TF (odom = mundo de Gazebo).

Pasa si:
  - la caja subió lo mismo que el TCP (± 2 mm): no se quedó en el pedestal ni se cayó;
  - durante el sostén se corrió menos de 1 mm respecto del TCP: no resbala.

Uso (con la simulación del brazo corriendo, igual que prueba_pinza.py; necesita la pinza en una
pose donde apunte horizontal, como la inicial con todo en 0):
  python3 /ros2_ws/src/andesrobot_arm/scripts/prueba_levantar.py               # 30 mm, 0.1 kg
  python3 /ros2_ws/src/andesrobot_arm/scripts/prueba_levantar.py --masa 1.0    # más pesada
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import DeleteEntity
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# Este script está en la misma carpeta que prueba_pinza.py: reusa su nodo (abrir/cerrar en
# espejo, medir los dedos, poner modelos en el TCP).
from prueba_pinza import ABIERTA, CERRADA, TOLERANCIA, PruebaPinza

ALTO = 0.015   # alto y fondo de la caja [m], como en la fase 1 (cabe en la cara del dedo)


def sdf_caja_suelta(ancho, masa):
    """Caja con masa (no estática), roce 1.0 y el plugin que publica su pose."""
    tam = f'{ALTO} {ancho} {ALTO}'
    # Inercia de una caja maciza: I = m·(a² + b²) / 12 sobre cada eje.
    ixx = masa * (ancho ** 2 + ALTO ** 2) / 12
    iyy = masa * (2 * ALTO ** 2) / 12
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="caja_suelta">'
            '<link name="link">'
            f'<inertial><mass>{masa}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy>'
            f'<izz>{ixx}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            f'<collision name="c"><geometry><box><size>{tam}</size></box></geometry>'
            '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>'
            '</collision>'
            f'<visual name="v"><geometry><box><size>{tam}</size></box></geometry></visual>'
            '</link>'
            '<plugin name="pose" filename="libgazebo_ros_p3d.so">'
            '<ros><namespace>/caja_suelta</namespace></ros>'
            '<body_name>link</body_name><frame_name>world</frame_name>'
            '<update_rate>50</update_rate></plugin>'
            '</model></sdf>')


def sdf_pedestal(ancho):
    """Columna fija de 5 cm de alto, de ancho/3 a lo largo de los dedos (no la tocan)."""
    tam = f'0.010 {ancho / 3} 0.05'
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="pedestal_pinza">'
            '<static>true</static><link name="link">'
            f'<collision name="c"><geometry><box><size>{tam}</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{tam}</size></box></geometry></visual>'
            '</link></model></sdf>')


class PruebaLevantar(PruebaPinza):
    def __init__(self):
        super().__init__()
        self.caja_xyz = None   # posición de la caja en el mundo [m]
        self.lift = None       # posición del lift [m]
        self.create_subscription(Odometry, '/caja_suelta/odom', self.al_ver_caja, 10)
        self.create_subscription(JointState, '/joint_states', self.al_leer_lift, 10)
        self.brazo = ActionClient(self, FollowJointTrajectory,
                                  '/arm_controller/follow_joint_trajectory')

    def al_ver_caja(self, m):
        p = m.pose.pose.position
        self.caja_xyz = np.array([p.x, p.y, p.z])

    def al_leer_lift(self, m):
        if 'vertical_lift_joint' in m.name:
            self.lift = m.position[m.name.index('vertical_lift_joint')]

    def tcp(self):
        t = self.tf.lookup_transform('odom', 'gripper_tcp', rclpy.time.Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z])

    def mover_lift(self, posicion, segundos=2):
        """Solo el lift (arm_controller acepta metas con una parte de las articulaciones)."""
        self.brazo.wait_for_server()
        meta = FollowJointTrajectory.Goal()
        meta.trajectory.joint_names = ['vertical_lift_joint']
        meta.trajectory.points = [JointTrajectoryPoint(positions=[posicion],
                                                       time_from_start=Duration(sec=segundos))]
        manejo = self.resultado(self.brazo.send_goal_async(meta))
        self.resultado(manejo.get_result_async())
        self.esperar(0.5)

    def relativa(self, segundos):
        """Posición de la caja respecto del TCP [m], muestreada durante unos segundos."""
        muestras = []
        fin = time.time() + segundos
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)
            muestras.append(self.caja_xyz - self.tcp())
        return np.array(muestras)

    def borrar(self):
        for nombre in ('caja_suelta', 'pedestal_pinza'):
            self.resultado(self.sacar.call_async(DeleteEntity.Request(name=nombre)))

    def probar(self, ancho, masa, subir, sostener, apriete):
        self.pinza(ABIERTA)
        lift0 = self.lift
        # Pedestal: su cara de arriba queda 1 mm bajo la caja (que cae ese milímetro y se apoya).
        dz_pedestal = -ALTO / 2 - 0.001 - 0.025
        if not (self.poner_en_tcp('pedestal_pinza', sdf_pedestal(ancho), dz_pedestal)
                and self.poner_en_tcp('caja_suelta', sdf_caja_suelta(ancho, masa))):
            print('FALLO: no se pudo poner la caja o el pedestal')
            self.borrar()
            return False
        self.esperar(1.0)
        if self.caja_xyz is None:
            print('FALLO: no llega /caja_suelta/odom (¿falta libgazebo_ros_p3d?)')
            self.borrar()
            return False

        self.pinza(max(CERRADA, (ancho - 0.017) / 2 - apriete) if apriete else CERRADA)
        (_, _, apertura), _ = self.medir(1.0)
        antes = self.relativa(1.0).mean(0)       # caja respecto del TCP, apoyada y apretada
        caja_z0, tcp_z0 = self.caja_xyz[2], self.tcp()[2]

        self.mover_lift(lift0 + subir)
        subio_tcp = self.tcp()[2] - tcp_z0
        subio_caja = self.caja_xyz[2] - caja_z0
        durante = self.relativa(sostener)
        corrida = np.linalg.norm(durante - durante[0], axis=1).max()
        hacia = (durante[-1] - durante[0]) * 1000
        total = np.linalg.norm(durante[-1] - antes)

        bien = abs(subio_caja - subio_tcp) < 0.002 and corrida < TOLERANCIA
        print(f'{"OK   " if bien else "FALLO"} caja de {ancho * 1000:.0f} mm y {masa:.2f} kg: '
              f'apertura {apertura * 1000:.1f} mm; el TCP subió '
              f'{subio_tcp * 1000:.1f} mm y la caja {subio_caja * 1000:.1f} mm; en '
              f'{sostener:.0f} s se corrió {corrida * 1000:.2f} mm respecto del TCP '
              f'({total * 1000:.2f} mm desde que cerró; x y z del mundo: '
              f'{hacia[0]:+.2f} {hacia[1]:+.2f} {hacia[2]:+.2f} mm)')

        self.mover_lift(lift0)
        self.pinza(ABIERTA)
        self.borrar()
        return bien


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--ancho', type=float, default=0.03, help='ancho de la caja [m]')
    ap.add_argument('--masa', type=float, nargs='+', default=[0.1],
                    help='masa de la caja [kg]; con varias, una prueba por masa')
    ap.add_argument('--subir', type=float, default=0.05, help='cuánto sube el lift [m]')
    ap.add_argument('--sostener', type=float, default=10.0, help='tiempo sostenida [s]')
    ap.add_argument('--apriete', type=float, default=0.002,
                    help='cuánto más adentro del objeto se manda cada dedo [m]; 0 = cerrar '
                         'del todo')
    a = ap.parse_args()
    rclpy.init()
    nodo = PruebaLevantar()
    nodo.esperar(2.0)   # TF y /joint_states
    if nodo.dedos is None or nodo.lift is None:
        sys.exit('No llega /joint_states con los dedos y el lift: ¿está corriendo arm_sim?')
    bien = [nodo.probar(a.ancho, m, a.subir, a.sostener, a.apriete) for m in a.masa]
    rclpy.shutdown()
    sys.exit(0 if all(bien) else 1)


if __name__ == '__main__':
    main()
