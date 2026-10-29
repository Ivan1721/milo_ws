#!/usr/bin/env python3
"""Torque estático de cada articulación del brazo con las masas del URDF (sin ROS ni Gazebo).

Dos usos, los dos para cerrar el pendiente de masas (docs/PENDIENTES.md, P05):

  1. Torques: cuánto torque necesita cada articulación para sostener el brazo quieto en la
     peor postura (barrido al azar dentro de los límites), y qué relación de reductor hace falta
     para que el motor lo sostenga con margen:

         python3 calcular_torques.py                    # masas del URDF actual
         python3 calcular_torques.py --carga 0.5        # con 0.5 kg en la pinza
         python3 calcular_torques.py --margen 3         # margen de seguridad 3 (por defecto 2)

  2. Lista para pesar: una tabla con cada pieza del URDF y su masa de hoy, con una columna
     vacía para anotar el peso real:

         python3 calcular_torques.py --pesaje > pesaje.md

Cuando se pesan las piezas, se corrigen las masas en andesrobot.urdf.xacro y se vuelve a correr.

Necesita el xacro procesado: dentro del contenedor (./sim.sh shell) funciona tal cual. Fuera,
pasa un URDF ya procesado con --urdf archivo.urdf. Solo usa numpy.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from andesrobot_arm.torques import ModeloMasas, torque_maximo  # noqa: E402

# Torque de retención de cada motor [N·m], de la guía eléctrica y el ensamblaje del fabricante
# (informe, sección 6). Con el reductor, el torque en la articulación es este por la relación.
RETENCION = {'joint_1': 2.4, 'joint_2': 2.4, 'joint_3': 1.85,
             'joint_4': 0.65, 'joint_5': 0.65, 'joint_6': 0.65}
ARTICULACIONES = list(RETENCION) + ['vertical_lift_joint']
PUNTA = 'gripper_tcp'   # frame donde va la carga extra (link de referencia, masa 0)


def leer_urdf(args):
    if args.urdf:
        return open(args.urdf, encoding='utf-8').read()
    import xacro
    ruta = args.xacro or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                      'andesrobot_description', 'urdf', 'andesrobot.urdf.xacro')
    return xacro.process_file(ruta, mappings={'lock_arm': 'false'}).toxml()


def limites(urdf_xml):
    """Rango de cada articulación del brazo según el URDF (±π si no tiene)."""
    out = {}
    for el in ET.fromstring(urdf_xml).findall('joint'):
        if el.get('name') in ARTICULACIONES:
            lim = el.find('limit')
            out[el.get('name')] = (float(lim.get('lower')), float(lim.get('upper'))) \
                if lim is not None else (-np.pi, np.pi)
    return out


def tabla_pesaje(modelo):
    print('| Pieza (link del URDF) | Masa en el URDF [kg] | Peso real [g] | Notas |')
    print('| --- | ---: | ---: | --- |')
    for link in sorted(modelo.mass, key=lambda n: (n.split('_')[0], n)):
        if link.startswith(('motor_joint', 'link_', 'arm_base', 'gripper', 'finger',
                            'left_finger', 'right_finger')) or 'finger' in link:
            print(f'| `{link}` | {modelo.mass[link]:.4f} |  |  |')
    print('| reductor EBA-23 (×3) | 0 (no está en el URDF) |  | ver docs/PENDIENTES.md P05 |')
    print('| reductor EBA-17 (×3) | 0 (no está en el URDF) |  |  |')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--urdf', help='URDF ya procesado (si no, procesa el xacro del repo)')
    ap.add_argument('--xacro', help='ruta del xacro (por defecto el del repo)')
    ap.add_argument('--carga', type=float, default=0.0, help='masa extra en la pinza [kg]')
    ap.add_argument('--margen', type=float, default=2.0, help='factor de seguridad (def. 2)')
    ap.add_argument('--muestras', type=int, default=20000, help='posturas al azar')
    ap.add_argument('--pesaje', action='store_true', help='imprimir la tabla para pesar')
    args = ap.parse_args()

    urdf_xml = leer_urdf(args)
    modelo = ModeloMasas(urdf_xml)
    if args.pesaje:
        tabla_pesaje(modelo)
        return
    carga = (args.carga, PUNTA, np.zeros(3)) if args.carga > 0 else None
    mejor = torque_maximo(modelo, ARTICULACIONES, limites(urdf_xml), args.muestras, carga=carga)

    print(f'Masas del URDF; carga en la pinza: {args.carga} kg; margen: {args.margen}; '
          f'{args.muestras} posturas.\n')
    print('| Articulación | Cuelga de ella [kg] | Torque máx. [N·m] | Retención del motor [N·m] '
          '| Relación mínima del reductor |')
    print('| --- | ---: | ---: | ---: | ---: |')
    for name in ARTICULACIONES:
        valor, _ = mejor[name]
        masa = modelo.masa_colgante(name)
        if name == 'vertical_lift_joint':
            print(f'| `{name}` | {masa:.2f} | {valor:.1f} N (fuerza) | — | — |')
            continue
        retencion = RETENCION[name]
        print(f'| `{name}` | {masa:.2f} | {valor:.2f} | {retencion} '
              f'| {args.margen * valor / retencion:.1f} : 1 |')
    print('\nRelación mínima = margen · torque máx. / retención del motor. Sin reductor la '
          'relación es 1 : 1; si el número sale mayor, el motor solo no sostiene el brazo.')


if __name__ == '__main__':
    main()
