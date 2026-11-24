#!/usr/bin/env python3
"""Diagramas de conexión del Arduino Mega del brazo de Milo (firmware/brazo_mega).

Dibuja dos figuras para firmware/README.md:
  - img/conexiones_mega.png: el Arduino Mega 2560 con los pines que usa el firmware, los 6
    drivers TB6600 (J1..J6) con sus motores, el servo de la garra y las dos fuentes (24 V para
    los motores, 5-6 V para el servo) con la tierra (GND) común.
  - img/conexion_tb6600.png: el detalle de UN driver (qué borne va a qué pin del Arduino, al
    motor y a la fuente; los interruptores de micropasos y corriente) y del servo.

Es un ESQUEMA, no un plano a escala: la posición de los pines en el dibujo imita la placa (el
cabezal doble de los pines 22-53 en el borde derecho, con los GND al final), pero hay que guiarse
por lo impreso en la placa (la serigrafía) al conectar. Los números de pin son los de
brazo_mega.ino (PIN_STEP, PIN_DIR, PIN_SERVO): si se cambian ahí, cambiarlos también aquí (PINES).

El cableado de las señales del TB6600 es el de "cátodo común" (PUL- y DIR- a GND del Arduino,
PUL+ y DIR+ a los pines): con él un HIGH del pin es un pulso, como supone el firmware. Si los
drivers ya están cableados al revés (ánodo común: PUL+ y DIR+ a 5 V, PUL- y DIR- a los pines)
también funciona, pero cada articulación gira al revés (se corrige con invertir: true en
ros2_ws/src/andesrobot_arm/config/brazo_real.yaml).

Solo necesita matplotlib (no ROS):
  python3 firmware/fig_conexiones.py            # escribe en firmware/img/
"""
import os

import matplotlib
matplotlib.use('Agg')  # dibujar a archivo, sin ventana
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
SALIDA = os.path.join(AQUI, 'img')

# Pines de brazo_mega.ino: articulación -> (STEP, DIR), y el motor de cada una (CLAUDE.md:
# J1/J3 NEMA23 0.70 kg, J2 NEMA23 1.60 kg, J4-J6 NEMA17).
PINES = {1: (45, 44), 2: (47, 46), 3: (49, 48), 4: (51, 50), 5: (53, 52), 6: (43, 42)}
MOTOR = {1: 'NEMA 23', 2: 'NEMA 23', 3: 'NEMA 23', 4: 'NEMA 17', 5: 'NEMA 17', 6: 'NEMA 17'}
PIN_SERVO = 9

# Colores: uno por articulación (distinguibles y oscuros para leerse sobre blanco), y los de
# siempre en electrónica: rojo = positivo de una fuente, negro = GND.
SURF, INK, MUTED = '#fcfcfb', '#2b2b29', '#6b6a63'
COLOR_J = {1: '#1c5fb0', 2: '#b8430f', 3: '#2e7d32', 4: '#7b3fa0', 5: '#00838f', 6: '#a0004f'}
ROJO, NEGRO, NARANJA = '#c62828', '#212121', '#e08a00'
PCB, PCB_BORDE = '#1f6f8b', '#134a5e'          # la placa (azul verdoso, como el Mega)
DRIVER, DRIVER_BORDE = '#eef3fa', '#2a78d6'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'text.color': INK,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF,
                     'savefig.facecolor': SURF})


def caja(ax, x0, y0, w, h, fc, ec, lw=1.5, r=0.15, z=2):
    ax.add_patch(FancyBboxPatch((x0, y0), w, h, boxstyle=f'round,pad=0.02,rounding_size={r}',
                                fc=fc, ec=ec, lw=lw, zorder=z))


def cable(ax, pts, color, lw=1.8, ls='-', z=3):
    """Un cable como polilínea (lista de puntos), con un punto en cada extremo."""
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=z, solid_capstyle='round')
    ax.plot([xs[0], xs[-1]], [ys[0], ys[-1]], 'o', color=color, ms=3.5, zorder=z + 1)


def pin(ax, x, y, texto, color, lado='izq', resaltado=True):
    """Un pin del cabezal: cuadradito (de color si se usa) con su número al lado."""
    ax.add_patch(Rectangle((x - 0.13, y - 0.13), 0.26, 0.26, zorder=5,
                           fc=color if resaltado else '#d9d9d6', ec='#111', lw=0.6))
    if texto:
        ax.text(x - 0.25 if lado == 'izq' else x + 0.25, y, texto, zorder=6, fontsize=7.4,
                ha='right' if lado == 'izq' else 'left', va='center',
                color='white' if lado == 'izq' else INK, fontweight='bold')


def figura_general():
    fig, ax = plt.subplots(figsize=(15, 9.2))
    ax.set_xlim(-0.5, 28.5)
    ax.set_ylim(-1.6, 17.2)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.text(0, 16.9, 'Conexiones del Arduino Mega del brazo (firmware/brazo_mega v2)',
            fontsize=14, fontweight='bold', va='top')
    ax.text(0, 16.15, 'Esquema, no a escala: al conectar, guiarse por el número impreso junto '
            'a cada pin de la placa.', fontsize=9.5, color=MUTED, va='top')

    # --- La placa ---
    bx0, by0, bw, bh = 2.0, 2.2, 10.6, 7.0
    caja(ax, bx0, by0, bw, bh, PCB, PCB_BORDE, lw=2, r=0.3)
    ax.text(bx0 + 4.6, by0 + 4.0, 'Arduino Mega 2560', color='white', fontsize=13,
            fontweight='bold', ha='center', zorder=6)
    ax.text(bx0 + 4.6, by0 + 3.35, 'clon con chip USB CH340', color='#d8eef5', fontsize=9.5,
            ha='center', zorder=6)
    ax.text(bx0 + 4.6, by0 + 2.75, 'ATmega2560 · 5 V · firmware brazo_mega v2',
            color='#d8eef5', fontsize=8.5, ha='center', zorder=6)
    ax.add_patch(Rectangle((bx0 + 3.6, by0 + 0.7), 2.0, 1.4, fc='#222', ec='#000', zorder=5))
    ax.text(bx0 + 4.6, by0 + 1.4, 'ATmega\n2560', color='#ccc', fontsize=7, ha='center',
            va='center', zorder=6)
    # USB-B y conector de alimentación (borde izquierdo)
    ax.add_patch(Rectangle((bx0 - 0.7, by0 + 4.6), 1.5, 1.3, fc='#bdbdbd', ec='#555', zorder=5))
    ax.text(bx0 + 0.05, by0 + 5.25, 'USB', fontsize=7.5, ha='center', va='center', zorder=6)
    ax.add_patch(Rectangle((bx0 - 0.5, by0 + 0.8), 1.3, 1.1, fc='#333', ec='#000', zorder=5))
    ax.text(bx0 + 0.15, by0 + 1.35, 'jack', fontsize=7, color='#ddd', ha='center',
            va='center', zorder=6)
    # PC por USB
    caja(ax, -0.4, by0 + 6.4, 2.6, 1.7, '#f3f3f1', '#8a8982')
    ax.text(0.9, by0 + 7.6, 'PC (ROS 2)', fontsize=9, fontweight='bold', ha='center')
    ax.text(0.9, by0 + 7.0, '/dev/arduino_brazo\n115200 baudios', fontsize=7.3, ha='center',
            va='center', color=MUTED)
    cable(ax, [(0.9, by0 + 6.4), (0.9, by0 + 5.25), (bx0 - 0.7, by0 + 5.25)], '#555', lw=3)
    ax.text(0.95, by0 + 4.45, 'cable USB:\ndatos y 5 V\npara el Arduino', fontsize=7.2,
            ha='center', va='top', color=MUTED)
    ax.text(bx0 + 0.25, by0 + 0.35, 'jack sin usar (lo alimenta el USB)', fontsize=7,
            color='#d8eef5', zorder=6)

    # --- Cabezal superior (pines 0-21): solo el 9 se usa ---
    yt = by0 + bh - 0.35
    xs_sup = [bx0 + 1.6 + 0.36 * i for i in range(18)]
    nombres_sup = ['GND', '13', '12', '11', '10', '9', '8', '', '7', '6', '5', '4', '3', '2',
                   '1', '0', '', '']
    for x, n in zip(xs_sup, nombres_sup):
        if n == '':
            continue
        usado = n in (str(PIN_SERVO), 'GND')
        color = NARANJA if n == str(PIN_SERVO) else NEGRO
        pin(ax, x, yt, None, color, resaltado=usado)
        ax.text(x, yt - 0.42, n, fontsize=6.3, ha='center', va='top', color='white', zorder=6,
                rotation=90)
    x_servo = xs_sup[nombres_sup.index(str(PIN_SERVO))]
    x_gnd_sup = xs_sup[0]

    # --- Cabezal doble (pines 22-53), borde derecho, de arriba (5V) a abajo (GND) ---
    xa, xb = bx0 + bw - 1.05, bx0 + bw - 0.55      # columna de los pares y de los impares
    filas = [('5V', '5V')] + [(str(p), str(p + 1)) for p in range(22, 54, 2)] + [('GND', 'GND')]
    dy = (bh - 0.7) / (len(filas) - 1)
    ypin = {}
    for i, (a, b) in enumerate(filas):
        y = by0 + bh - 0.35 - i * dy
        for x, n in ((xa, a), (xb, b)):
            ypin[n] = y
            num = int(n) if n.isdigit() else None
            j = next((k for k, (s, d) in PINES.items() if num in (s, d)), None)
            if j is not None:
                pin(ax, x, y, None, COLOR_J[j])
            elif n == 'GND':
                pin(ax, x, y, None, NEGRO)
            else:
                pin(ax, x, y, None, None, resaltado=False)
        if a in ('5V', 'GND') or int(a) >= 42:
            t = a if not a.isdigit() else f'{a} · {b}'   # par (columna izq.) · impar (der.)
            ax.text(xa - 0.25, y, t, fontsize=7, ha='right', va='center', color='white',
                    fontweight='bold', zorder=6)
    ax.text(xa - 0.3, ypin['32'], 'pines\n22 … 41\nsin usar', fontsize=7, ha='right',
            va='center', color='#d8eef5', zorder=6)

    # --- Drivers TB6600 y motores ---
    dx0, dw, dh = 17.3, 4.3, 1.75
    x_gnd_bus = 15.9                                 # bus de GND (PUL- y DIR-)
    x_24 = 22.6                                      # bus de +24 V
    orden = [1, 2, 3, 4, 5, 6]
    ytop = 13.2
    for k, j in enumerate(orden):
        y0 = ytop - k * 2.25
        caja(ax, dx0, y0, dw, dh, DRIVER, DRIVER_BORDE)
        ax.text(dx0 + 0.15, y0 + dh - 0.2, f'TB6600 · J{j}', fontsize=9.5, fontweight='bold',
                color=COLOR_J[j], va='top')
        step, dirp = PINES[j]
        yp, yd, yg = y0 + 1.0, y0 + 0.62, y0 + 0.24
        for yy, t in ((yp, 'PUL+'), (yd, 'DIR+'), (yg, 'PUL−  DIR−')):
            ax.text(dx0 + 0.15, yy, t, fontsize=7.4, va='center', color=INK)
        ax.text(dx0 + dw - 0.12, y0 + 0.24, 'ENA: sin conectar', fontsize=6.8, ha='right',
                va='center', color=MUTED)
        ax.text(dx0 + dw - 0.12, y0 + 1.0, 'VCC  GND', fontsize=7.2, ha='right',
                va='center', color=INK)
        ax.text(dx0 + dw - 0.12, y0 + 0.62, 'A+ A− B+ B−', fontsize=7.2, ha='right',
                va='center', color=INK)
        # STEP (continua) y DIR (punteada) desde el cabezal: cada una baja por su carril.
        carril = 13.0 + 0.36 * k
        for p, yy, ls in ((step, yp, '-'), (dirp, yd, (0, (4, 2)))):
            ysal = ypin[str(p)]
            if p % 2:                                # impar: columna derecha, sale directo
                inicio = [(xb, ysal)]
                xc = carril
            else:                                    # par: baja media fila y pasa entre los
                inicio = [(xa, ysal), (xa, ysal - dy / 2)]   # pines de la columna derecha
                ysal -= dy / 2
                xc = carril + 0.15
            cable(ax, inicio + [(xc, ysal), (xc, yy), (dx0, yy)], COLOR_J[j], lw=1.6, ls=ls)
        ax.text(dx0 - 0.1, yp + 0.17, f'pin {step}', fontsize=7, ha='right',
                color=COLOR_J[j], fontweight='bold')
        ax.text(dx0 - 0.1, yd - 0.33, f'pin {dirp}', fontsize=7, ha='right',
                color=COLOR_J[j], fontweight='bold')
        # PUL- y DIR- al bus de GND
        cable(ax, [(x_gnd_bus, yg), (dx0, yg)], NEGRO, lw=1.2)
        # 24 V
        cable(ax, [(dx0 + dw, y0 + 1.0), (x_24, y0 + 1.0)], ROJO, lw=1.4)
        cable(ax, [(dx0 + dw, y0 + 0.85), (x_24 + 0.35, y0 + 0.85)], NEGRO, lw=1.4)
        # Motor
        cm = (25.6, y0 + dh / 2)
        cable(ax, [(dx0 + dw, y0 + 0.62), (cm[0] - 0.8, y0 + 0.62)], '#6d4c41', lw=2.4)
        ax.add_patch(Circle(cm, 0.78, fc='#cfd8dc', ec='#37474f', lw=1.4, zorder=5))
        ax.add_patch(Circle(cm, 0.22, fc='#90a4ae', ec='#37474f', lw=1, zorder=6))
        ax.text(cm[0] + 0.95, cm[1] + 0.2, f'motor J{j}', fontsize=8.2, fontweight='bold',
                va='center', color=COLOR_J[j])
        ax.text(cm[0] + 0.95, cm[1] - 0.25, MOTOR[j], fontsize=7.4, va='center', color=MUTED)
    ybajo = ytop - 5 * 2.25
    # Bus de GND de las señales: baja hasta el GND del cabezal doble
    cable(ax, [(x_gnd_bus, ytop + 0.24), (x_gnd_bus, ybajo - 0.9), (xb, ybajo - 0.9),
               (xb, ypin['GND'])], NEGRO, lw=1.6)
    ax.text(x_gnd_bus + 0.12, ybajo - 0.75, 'GND común (PUL− y DIR− de los 6)', fontsize=7.4,
            va='bottom', color=NEGRO)
    # Fuente de 24 V
    caja(ax, 21.9, 15.0, 2.8, 1.0, '#fdecea', ROJO)
    ax.text(23.3, 15.75, 'Fuente 24 V', fontsize=9, fontweight='bold', ha='center',
            color=ROJO)
    ax.text(23.3, 15.25, '+ rojo · − negro', fontsize=7.4, ha='center', color=MUTED)
    cable(ax, [(x_24, 15.0), (x_24, ybajo + 1.0)], ROJO, lw=2)
    cable(ax, [(x_24 + 0.35, 15.0), (x_24 + 0.35, ybajo + 0.85)], NEGRO, lw=2)

    # --- Servo de la garra y su fuente ---
    sx0, sy0 = 6.0, 11.4
    caja(ax, sx0, sy0, 3.4, 1.5, '#fff4e5', NARANJA)
    ax.text(sx0 + 1.7, sy0 + 1.15, 'Servo de la garra', fontsize=9, fontweight='bold',
            ha='center', color='#b8430f')
    ax.text(sx0 + 1.7, sy0 + 0.65, 'señal · + · −', fontsize=7.6, ha='center', color=INK)
    ax.text(sx0 + 1.7, sy0 + 0.25, '(naranja · rojo · marrón)', fontsize=6.8, ha='center',
            color=MUTED)
    cable(ax, [(x_servo, yt), (x_servo, sy0 + 0.0 - 0.0), (sx0 + 0.5, sy0)], NARANJA, lw=2)
    ax.text(x_servo + 0.15, (yt + sy0) / 2 + 0.2, f'pin {PIN_SERVO} (señal PWM)', fontsize=7.4,
            ha='left', va='center', color='#b8430f', fontweight='bold')
    fx0, fy0 = 0.0, 12.9
    caja(ax, fx0, fy0, 3.6, 1.2, '#fdecea', ROJO)
    ax.text(fx0 + 1.8, fy0 + 0.8, 'Fuente 5-6 V', fontsize=9, fontweight='bold', ha='center',
            color=ROJO)
    ax.text(fx0 + 1.8, fy0 + 0.3, 'solo el servo', fontsize=7.4, ha='center', color=MUTED)
    cable(ax, [(fx0 + 3.6, fy0 + 0.8), (sx0 + 1.7, fy0 + 0.8), (sx0 + 1.7, sy0 + 1.5)], ROJO,
          lw=2)
    cable(ax, [(fx0 + 3.6, fy0 + 0.35), (sx0 + 2.9, fy0 + 0.35), (sx0 + 2.9, sy0 + 1.5)],
          NEGRO, lw=2)
    cable(ax, [(fx0 + 3.0, fy0), (fx0 + 3.0, yt + 0.9), (x_gnd_sup, yt + 0.9), (x_gnd_sup, yt)],
          NEGRO, lw=1.6)
    ax.text(fx0 + 0.1, fy0 - 0.25, 'su − también a\nGND del Arduino', fontsize=7,
            color=NEGRO, va='top')

    # --- Notas ---
    notas = ('Reglas: (1) todas las GND unidas (Arduino, fuente del servo y señales PUL−/DIR−); '
             '(2) los motores NO se alimentan del Arduino;\n(3) ENA sin conectar: los drivers '
             'quedan siempre habilitados y sostienen J2/J3; (4) conectar y desconectar todo con '
             'la fuente de 24 V APAGADA.\nLínea continua = STEP (PUL+), punteada = DIR (DIR+). '
             'Pines sin verificar contra el cableado real (sobre todo J6: 43/42).')
    ax.text(0, -0.2, notas, fontsize=8.4, va='top', color=INK, linespacing=1.5)
    return fig


def figura_detalle():
    fig, ax = plt.subplots(figsize=(15, 8.6))
    ax.set_xlim(-0.5, 28.5)
    ax.set_ylim(-0.2, 15.6)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.text(0, 15.3, 'Detalle: un driver TB6600 (J1 de ejemplo) y el servo de la garra',
            fontsize=14, fontweight='bold', va='top')
    ax.text(0, 14.6, 'Los otros 5 drivers se conectan igual, cada uno a sus pines (ver la '
            'tabla). Todo con la fuente de 24 V apagada.', fontsize=9.5, color=MUTED, va='top')
    j = 1
    step, dirp = PINES[j]
    c = COLOR_J[j]

    # TB6600: bornera de señales a la izquierda y de potencia a la derecha, de abajo hacia
    # arriba en el orden del driver.
    tx0, ty0, tw, th = 10.0, 5.0, 11.0, 8.0
    caja(ax, tx0, ty0, tw, th, DRIVER, DRIVER_BORDE, lw=2)
    ax.text(tx0 + tw / 2, ty0 + th - 0.3, 'TB6600 (driver de J1)', fontsize=11,
            fontweight='bold', ha='center', va='top', color='#1c5fb0')
    senales = ['ENA−', 'ENA+', 'DIR−', 'DIR+', 'PUL−', 'PUL+']
    potencia = ['B−', 'B+', 'A−', 'A+', 'GND', 'VCC']
    ys = [ty0 + 0.9 + 0.95 * i for i in range(6)]
    yb = {}
    xl, xr = tx0 - 0.35, tx0 + tw + 0.35            # bornes izquierdo y derecho
    for y, n in zip(ys, senales):
        usado = not n.startswith('ENA')
        ax.add_patch(Rectangle((tx0 - 0.35, y - 0.25), 0.7, 0.5, zorder=5,
                               fc='#4caf50' if usado else '#c8c8c4', ec='#1b5e20'))
        ax.text(tx0 + 0.5, y, n, fontsize=8.5, va='center', fontweight='bold',
                color=INK if usado else MUTED)
        yb[n] = y
    for y, n in zip(ys, potencia):
        ax.add_patch(Rectangle((tx0 + tw - 0.35, y - 0.25), 0.7, 0.5, fc='#4caf50',
                               ec='#1b5e20', zorder=5))
        ax.text(tx0 + tw - 0.5, y, n, fontsize=8.5, va='center', ha='right', fontweight='bold')
        yb[n] = y
    # Interruptores SW1..SW6
    xm = tx0 + tw / 2
    ax.text(xm, ty0 + 5.6, 'interruptores SW1 … SW6', fontsize=8.4, ha='center',
            fontweight='bold')
    for i in range(6):
        x = xm - 1.65 + 0.6 * i
        ax.add_patch(Rectangle((x, ty0 + 4.4), 0.42, 0.85, fc='#c62828', ec='#7f0000',
                               zorder=5))
        ax.add_patch(Rectangle((x + 0.06, ty0 + 4.85), 0.3, 0.3, fc='white', ec='#555',
                               zorder=6))
        ax.text(x + 0.21, ty0 + 4.2, str(i + 1), fontsize=7, ha='center', va='top')
    ax.text(xm, ty0 + 3.4, 'SW1-3 micropasos: 1/8 (= 1600 pulsos por vuelta)\n'
            'SW4-6 corriente: la de la placa del motor\n'
            '(ver la tabla impresa en el driver)', fontsize=7.8, ha='center', va='top',
            linespacing=1.45, zorder=6, bbox=dict(fc='white', ec='#bbb',
                                                  boxstyle='round,pad=0.3'))

    # Arduino (recorte): cada pin a la altura de su borne, para que los cables vayan rectos.
    caja(ax, 0.0, 5.2, 4.2, 7.2, PCB, PCB_BORDE, lw=2, r=0.3)
    ax.text(2.1, 12.15, 'Arduino Mega', color='white', fontsize=10.5, fontweight='bold',
            ha='center', va='top')
    xa = 3.9
    pines_a = [(yb['PUL+'], f'pin {step}  (STEP J1)', c), (yb['PUL−'], 'GND', NEGRO),
               (yb['DIR+'], f'pin {dirp}  (DIR J1)', c), (yb['DIR−'], 'GND', NEGRO),
               (6.6, 'GND', NEGRO), (5.8, f'pin {PIN_SERVO}  (servo)', NARANJA)]
    for y, t, col in pines_a:
        pin(ax, xa, y, None, col)
        ax.text(xa - 0.3, y, t, color='white', fontsize=8, ha='right', va='center',
                fontweight='bold', zorder=6)
    cable(ax, [(xa, yb['PUL+']), (xl, yb['PUL+'])], c)
    cable(ax, [(xa, yb['PUL−']), (xl, yb['PUL−'])], NEGRO)
    cable(ax, [(xa, yb['DIR+']), (xl, yb['DIR+'])], c, ls=(0, (4, 2)))
    cable(ax, [(xa, yb['DIR−']), (xl, yb['DIR−'])], NEGRO)
    ax.text(tx0 + 0.2, ty0 + 0.3, 'ENA+ y ENA−: sin conectar', fontsize=7.6, va='center',
            color=MUTED)
    ax.text(0, 13.95, 'Cátodo común: PUL− y DIR− a GND, PUL+ y DIR+ a los pines '
            '(un HIGH del pin = un pulso).\nEl Mega tiene varios pines GND, todos unidos '
            'por dentro: sirve cualquiera.', fontsize=8.2, color=INK, va='top',
            linespacing=1.45)

    # Potencia: fuente de 24 V y motor
    caja(ax, 23.0, 11.8, 3.4, 1.4, '#fdecea', ROJO)
    ax.text(24.7, 12.75, 'Fuente 24 V', fontsize=9.5, fontweight='bold', ha='center',
            color=ROJO)
    ax.text(24.7, 12.2, '+ (V+)      − (V−)', fontsize=8, ha='center')
    cable(ax, [(xr, yb['VCC']), (23.9, yb['VCC']), (23.9, 11.8)], ROJO, lw=2)
    cable(ax, [(xr, yb['GND']), (25.5, yb['GND']), (25.5, 11.8)], NEGRO, lw=2)
    cm, rm = (25.8, 6.6), 1.3
    colores_bob = {'A+': '#c62828', 'A−': '#ef9a9a', 'B+': '#1565c0', 'B−': '#90caf9'}
    for k, n in enumerate(['A+', 'A−', 'B+', 'B−']):
        xv = 23.1 - 0.3 * k                       # A+ más a la derecha: sin cruces
        ye = 7.1 - 0.25 * k
        cable(ax, [(xr, yb[n]), (xv, yb[n]), (xv, ye), (cm[0] - rm, ye)], colores_bob[n],
              lw=2)
    ax.add_patch(Circle(cm, rm, fc='#cfd8dc', ec='#37474f', lw=1.6, zorder=5))
    ax.add_patch(Circle(cm, 0.4, fc='#90a4ae', ec='#37474f', lw=1, zorder=6))
    ax.text(cm[0], cm[1] - rm - 0.2, 'motor J1 (NEMA 23)', fontsize=8.6, ha='center',
            va='top', fontweight='bold', color=c)
    ax.text(21.6, 4.4, 'A+ A− = una bobina, B+ B− = la otra: los dos\ncables de una bobina '
            'tienen continuidad\n(se prueba con el multímetro). Si gira al revés:\n'
            'invertir: true en brazo_real.yaml.', fontsize=7.6, va='top', linespacing=1.4)

    # Servo y su fuente
    caja(ax, 0.0, 0.3, 5.8, 3.6, '#fff4e5', NARANJA)
    ax.text(2.9, 3.6, 'Servo de la garra', fontsize=10, fontweight='bold', ha='center',
            va='top', color='#b8430f')
    for k, (t, col) in enumerate([('señal (naranja / amarillo)', NARANJA),
                                  ('+ (rojo)', ROJO), ('− (marrón / negro)', NEGRO)]):
        y = 2.6 - 0.75 * k
        ax.add_patch(Rectangle((5.5, y - 0.18), 0.6, 0.36, fc=col, ec='#111', zorder=5))
        ax.text(5.3, y, t, fontsize=8, ha='right', va='center')
    cable(ax, [(xa, 5.8), (6.6, 5.8), (6.6, 2.6), (6.1, 2.6)], NARANJA, lw=2)
    caja(ax, 9.0, 0.5, 3.6, 1.9, '#fdecea', ROJO)
    ax.text(10.8, 1.75, 'Fuente 5-6 V', fontsize=9.5, fontweight='bold', ha='center',
            color=ROJO)
    ax.text(10.8, 1.15, 'aparte del Arduino', fontsize=7.6, ha='center', color=MUTED)
    cable(ax, [(6.1, 1.85), (9.0, 1.85)], ROJO, lw=2)
    cable(ax, [(6.1, 1.1), (9.0, 1.1)], NEGRO, lw=2)
    # La GND de la fuente del servo también va a una GND del Arduino.
    cable(ax, [(12.6, 1.1), (13.4, 1.1), (13.4, 4.2), (8.4, 4.2), (8.4, 6.6), (xa, 6.6)],
          NEGRO, lw=1.4)
    ax.text(14.2, 1.95, 'Del Arduino al servo va solo la señal. La GND de su fuente se une a '
            'una GND del Arduino (si no, la señal\nno tiene referencia y el servo tiembla). Un '
            'servo que se traba puede pedir 1-2 A: desde el pin 5V del\nArduino lo '
            'reiniciaría.', fontsize=8, va='top', linespacing=1.45)
    return fig


def main():
    os.makedirs(SALIDA, exist_ok=True)
    for nombre, hacer in (('conexiones_mega.png', figura_general),
                          ('conexion_tb6600.png', figura_detalle)):
        fig = hacer()
        ruta = os.path.join(SALIDA, nombre)
        fig.savefig(ruta, dpi=130, bbox_inches='tight')
        plt.close(fig)
        print('escrita', ruta)


if __name__ == '__main__':
    main()
