#!/usr/bin/env python3
"""Dibuja la POSE DE ARRANQUE del brazo real (docs/figuras/pose_arranque.png).

En palabras simples: el brazo real no tiene encoders, así que el Arduino empieza a contar pasos
desde donde esté el brazo al encenderse. Por eso se enciende siempre en la misma pose, la "pose de
arranque", y config/brazo_real.yaml dice qué ángulo tiene cada articulación en ella (`cero`). Este
script lee esos ángulos (y la altura fija del lift) y dibuja el robot así, con las mallas del URDF,
para mostrarle al equipo cómo dejar el brazo antes de encender. Si se cambia algún `cero` en el
yaml, volver a correrlo.

Necesita el contenedor (importa figuras.py, que lee el URDF instalado):
    python3 scripts/fig_pose_arranque.py            [--lift -0.047]
"""
import argparse
import math
import os
import sys

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import figuras as f  # noqa: E402

YAML = os.path.join(AQUI, '..', 'config', 'brazo_real.yaml')
SALIDA = os.path.join(AQUI, '..', 'docs', 'figuras', 'pose_arranque.png')


def angulos_de_arranque(ruta=YAML):
    """{joint_N: ángulo [rad]} de la pose de arranque: el `cero` de cada articulación."""
    with open(ruta) as archivo:
        datos = yaml.safe_load(archivo)
    comun = datos.get('comun', {}).get('cero', 0.0)
    return {nombre: (extra or {}).get('cero', comun)
            for nombre, extra in datos['articulaciones'].items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--lift', type=float, default=-0.047,
                        help='posición fija del lift [m] (la de brazo_real.launch.py)')
    args = parser.parse_args()
    q = angulos_de_arranque()
    T = f.world(dict(q, vertical_lift_joint=args.lift))
    tcp = T['gripper_tcp'][:3, 3]
    texto = ', '.join(f'J{n[-1]} {math.degrees(v):+.0f}°' for n, v in sorted(q.items()))
    fig = plt.figure(figsize=(14, 6.5))
    luz = np.array([0.6, -0.3, 0.8]) / np.linalg.norm([0.6, -0.3, 0.8])
    for n, (vista, elev, azim) in enumerate([('de lado (desde la derecha de Milo)', 5, -90),
                                             ('en perspectiva', 20, -55)]):
        ax = fig.add_subplot(1, 2, n + 1, projection='3d', proj_type='ortho')
        polys, cols = [], []
        for link, t in f.placed(T):
            polys.append(t)
            cols.append(f.shade(f.color_of(link), f.normals_of(t), luz))
        ax.add_collection3d(Poly3DCollection(np.vstack(polys), facecolors=np.vstack(cols),
                                             linewidths=0))
        ax.scatter(*tcp, color='k', s=20, depthshade=False)
        ax.text(tcp[0], tcp[1], tcp[2] + 0.07,
                f'pinza\nx={tcp[0]:.2f} m, z={tcp[2]:.2f} m', fontsize=9)
        ax.set_xlim(-0.4, 1.1)
        ax.set_ylim(-0.75, 0.75)
        ax.set_zlim(0, 1.5)
        ax.set_box_aspect((1.5, 1.5, 1.5))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel('X adelante (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Pose de arranque: {texto}\n{vista}', fontsize=11)
    fig.tight_layout()
    fig.savefig(SALIDA, dpi=85)
    print(f'{SALIDA}: pinza en x={tcp[0]:.3f} y={tcp[1]:.3f} z={tcp[2]:.3f} m')


if __name__ == '__main__':
    main()
