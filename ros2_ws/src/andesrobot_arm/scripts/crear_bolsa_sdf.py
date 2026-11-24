#!/usr/bin/env python3
"""Escribe worlds/bolsa_prueba.sdf: una bolsa con dos asas parada en el suelo frente a Milo, CON
colisión, para probar bolsa_node en Gazebo (P17).

Las medidas son las de las bolsas que hizo el equipo de visión para entrenar su modelo de asas
(crear_asas.py y asas.json de su entrega del 20-nov-2026, `entrega_VC.zip`): cuerpo = una caja,
cada asa = un arco (bases separadas 2·r, alto h) armado con 24 tramos rectos de cinta. Sus modelos
traen solo la parte visual (no chocan: la pinza los atravesaría), y cada asa es un modelo aparte.
Aquí todo es UN cuerpo rígido (el reglamento habla de "asas rígidas verticales") de `masa` kg, con
la misma forma para lo que se ve y para lo que choca (cajas, sin mallas), así que el modelo de
puntos del equipo debería reconocerla igual que en su mundo.

Las asas van en las dos paredes largas (x = ±fondo/2 del frame de la bolsa); los arcos quedan en
planos paralelos a Y. La bolsa queda en (x, y) con su fondo a lo largo de X: de frente a Milo se
ve el asa cercana entera y la lejana detrás.

Uso:  python3 scripts/crear_bolsa_sdf.py [--tipo bolsa_papel] [--x 0.685] [--masa 0.3]
      (después ./sim.sh build si worlds/bolsa_prueba.sdf es nuevo; con --symlink-install basta
      relanzar si ya existía)
"""
import argparse
import math
import os

# Del asas.json del equipo [m]: fondo, ancho, alto del cuerpo; r = media separación de las bases,
# h = alto del arco sobre la boca; la cinta mide asa_ancho (en el plano del arco) x asa_grueso.
BOLSAS = {
    'bolsa_tela_cruda': dict(fondo=0.10, ancho=0.36, alto=0.38, r=0.10, h=0.24,
                             asa_ancho=0.025, asa_grueso=0.004,
                             color=(0.92, 0.89, 0.82), color_asa=(0.72, 0.68, 0.60)),
    'bolsa_tela_negra': dict(fondo=0.10, ancho=0.38, alto=0.40, r=0.11, h=0.26,
                             asa_ancho=0.030, asa_grueso=0.004,
                             color=(0.12, 0.12, 0.12), color_asa=(0.24, 0.24, 0.26)),
    'bolsa_papel': dict(fondo=0.17, ancho=0.32, alto=0.33, r=0.06, h=0.09,
                        asa_ancho=0.008, asa_grueso=0.008,
                        color=(0.66, 0.48, 0.30), color_asa=(0.58, 0.42, 0.26)),
}
TRAMOS = 24
AQUI = os.path.dirname(os.path.abspath(__file__))
SALIDA = os.path.join(AQUI, '..', 'worlds', 'bolsa_prueba.sdf')


def material(rgb):
    c = ' '.join(f'{v:.2f}' for v in rgb) + ' 1'
    return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'


def pieza(nombre, pose, tamano, rgb, friccion=1.0):
    """Una caja que se ve y choca. pose = x y z roll pitch yaw en el frame de la bolsa."""
    p = ' '.join(f'{v:.5f}' for v in pose)
    t = ' '.join(f'{v:.5f}' for v in tamano)
    return (f'      <collision name="{nombre}_c"><pose>{p}</pose>'
            f'<geometry><box><size>{t}</size></box></geometry>\n'
            f'        <surface><friction><ode><mu>{friccion}</mu><mu2>{friccion}</mu2></ode>'
            f'</friction></surface></collision>\n'
            f'      <visual name="{nombre}_v"><pose>{p}</pose>'
            f'<geometry><box><size>{t}</size></box></geometry>{material(rgb)}</visual>\n')


def asa(nombre, x_pared, b):
    """Los tramos del arco, como crear_asas.py del equipo: de la base en −Y a la de +Y."""
    texto = ''
    angulos = [math.pi * (1 - i / TRAMOS) for i in range(TRAMOS + 1)]
    pts = [(b['r'] * math.cos(t), b['alto'] + b['h'] * math.sin(t)) for t in angulos]
    for i, ((y0, z0), (y1, z1)) in enumerate(zip(pts[:-1], pts[1:])):
        largo = math.hypot(y1 - y0, z1 - z0) + 0.003        # se traslapan 3 mm: sin rendijas
        giro = math.atan2(z1 - z0, y1 - y0)                  # alrededor de X
        texto += pieza(f'{nombre}_{i:02d}', (x_pared, (y0 + y1) / 2, (z0 + z1) / 2, giro, 0, 0),
                       (b['asa_grueso'], largo, b['asa_ancho']), b['color_asa'])
    return texto


def sdf(tipo, x, y, masa):
    b = BOLSAS[tipo]
    f, a, h = b['fondo'], b['ancho'], b['alto']
    # Inercia de una caja llena (aproximación: las asas pesan poco).
    ixx = masa * (a ** 2 + h ** 2) / 12
    iyy = masa * (f ** 2 + h ** 2) / 12
    izz = masa * (f ** 2 + a ** 2) / 12
    cuerpo = pieza('cuerpo', (0, 0, h / 2, 0, 0, 0), (f, a, h), b['color'])
    return f'''<?xml version="1.0"?>
<!-- GENERADO por scripts/crear_bolsa_sdf.py (no editar a mano): {tipo} del equipo de visión
     (asas.json, entrega_VC.zip del 20-nov-2026), con colisión. La pone arm_sim.launch.py con
     bolsa:=true. Milo aparece en (0, 0) mirando a +X; su frente está en x = 0.436. La bolsa queda
     en ({x}, {y}) con el fondo ({f} m) a lo largo de X: el asa cercana en x = {x - f / 2:.3f}, su
     punta a {h + b["h"]:.2f} m del suelo. Un solo cuerpo rígido de {masa} kg. -->
<sdf version="1.6">
  <model name="bolsa_prueba">
    <pose>{x} {y} 0 0 0 0</pose>
    <link name="bolsa">
      <inertial><pose>0 0 {h / 2:.4f} 0 0 0</pose><mass>{masa}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{cuerpo}{asa('asa_cerca', -f / 2, b)}{asa('asa_lejos', f / 2, b)}    </link>
  </model>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--tipo', default='bolsa_papel', choices=sorted(BOLSAS))
    ap.add_argument('--x', type=float, default=0.685, help='centro [m], base_footprint')
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--masa', type=float, default=0.3, help='[kg]')
    ap.add_argument('--salida', default=SALIDA)
    a = ap.parse_args()
    with open(a.salida, 'w') as archivo:
        archivo.write(sdf(a.tipo, a.x, a.y, a.masa))
    print(f'Escrito {os.path.normpath(a.salida)} ({a.tipo} en x = {a.x}, y = {a.y})')


if __name__ == '__main__':
    main()
