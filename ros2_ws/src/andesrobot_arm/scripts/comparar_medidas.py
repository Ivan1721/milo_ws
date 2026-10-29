#!/usr/bin/env python3
"""Compara las medidas del brazo real con el URDF y el DH oficial; escribe los topes para el xacro.

Usos (procedimiento completo en docs/medicion_brazo.md):

  # distancias entre ejes, en mm, medidas con el brazo en la pose cero:
  python3 comparar_medidas.py --d4 121 --d5 98 --d6 56

  # topes medidos, en grados desde la pose cero (mínimo,máximo), uno por articulación:
  python3 comparar_medidas.py --tope joint_2=-10,170 --tope joint_3=-120,120

Sin argumentos imprime solo lo que dice el URDF y el DH oficial. Necesita el xacro procesado:
dentro del contenedor (./sim.sh shell) funciona tal cual; fuera, pasa --urdf archivo.urdf.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from andesrobot_arm.kinematics import ArmKinematics  # noqa: E402
from andesrobot_arm.medidas import (  # noqa: E402
    PARAMETROS, distancias_urdf, lineas_topes, mas_cercano)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--urdf', help='URDF ya procesado (si no, procesa el xacro instalado)')
    for nombre in PARAMETROS:
        ap.add_argument(f'--{nombre}', type=float, help=f'{nombre} medido [mm]')
    ap.add_argument('--tope', action='append', default=[], metavar='JOINT=MIN,MAX',
                    help='tope medido en grados desde la pose cero (se puede repetir)')
    args = ap.parse_args()

    if args.urdf:
        kin = ArmKinematics(open(args.urdf, encoding='utf-8').read())
    else:
        kin = ArmKinematics.from_xacro()
    urdf = distancias_urdf(kin)

    print('| Parámetro | Qué mide | URDF [mm] | EB300 [mm] | Medido [mm] | Medido − URDF '
          '| Se parece a |')
    print('| --- | --- | ---: | ---: | ---: | ---: | --- |')
    for nombre, (que, oficial) in PARAMETROS.items():
        medido = getattr(args, nombre)
        if medido is None:
            print(f'| {nombre} | {que} | {urdf[nombre]:.1f} | {oficial:.1f} |  |  |  |')
        else:
            cercano = mas_cercano(medido, urdf[nombre], oficial)
            print(f'| {nombre} | {que} | {urdf[nombre]:.1f} | {oficial:.1f} | {medido:.1f} '
                  f'| {medido - urdf[nombre]:+.1f} | {cercano} |')

    if args.tope:
        topes = {}
        for texto in args.tope:
            joint, rango = texto.split('=')
            bajo, alto = (float(v) for v in rango.split(','))
            topes[joint.strip()] = (bajo, alto)
        print('\nLíneas para andesrobot.urdf.xacro (sección ARTICULACIONES):\n')
        print('\n'.join(lineas_topes(topes)))


if __name__ == '__main__':
    main()
