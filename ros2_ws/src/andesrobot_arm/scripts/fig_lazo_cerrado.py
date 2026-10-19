"""Figura del lazo cerrado del brazo de Milo: diagrama de bloques del control, tal como corre hoy.

Dibuja el camino de una orden («lleva la pinza aquí») hasta el movimiento, y las señales que
vuelven (la realimentación) para cerrar cada lazo:
  - lazo de la tarea: la cinemática inversa parte de la posición medida de cada motor,
  - lazo de posición: el controlador compara la posición pedida con la medida (100 Hz),
  - lazo de percepción: la cámara de la pinza mide el objeto y corrige el objetivo (pendiente).
Las flechas continuas existen hoy; las punteadas naranjas faltan. El recuadro de abajo explica
por qué en Gazebo (mando por posición) el lazo de posición se cierra sobre la propia orden.

Es un esquema: no lee el URDF ni necesita ROS, solo matplotlib.
  python3 fig_lazo_cerrado.py [carpeta_salida]      # por defecto ./figuras
Genera fig_lazo_cerrado.png. El informe (docs/informe/README.md) la lee directo de docs/figuras.
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')  # dibujar a archivo, sin ventana
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'

# Misma paleta que figuras.py: brazo azul, pinza naranja, grises para lo demás.
SURF, INK, MUTED = '#fcfcfb', '#2b2b29', '#6b6a63'
C_ARM, C_GRIP, C_BASE = '#2a78d6', '#eb6834', '#a9a8a0'
C_FILL, C_FILL2, C_NOTE = '#eef3fa', '#f3f3f1', '#fff4e5'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'text.color': INK,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF,
                     'savefig.facecolor': SURF})


def caja(ax, cx, cy, w, h, titulo, texto, color=C_ARM, fill=C_FILL, ls='-'):
    """Bloque con título en negrita y un texto debajo; devuelve sus bordes (x0, x1, y0, y1)."""
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                boxstyle='round,pad=0.02,rounding_size=0.12',
                                fc=fill, ec=color, lw=1.6, ls=ls, zorder=2))
    ax.text(cx, cy + h / 2 - 0.22, titulo, ha='center', va='top', fontsize=9.5,
            fontweight='bold', color=color if color != C_BASE else INK, zorder=3)
    ax.text(cx, cy + h / 2 - 0.50, texto, ha='center', va='top', fontsize=8.3, color=INK,
            linespacing=1.35, zorder=3)
    return cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2


def flecha(ax, pts, etiqueta=None, color=INK, ls='-', lw=1.5, pos=0.5, dx=0.0, dy=0.12,
           ha='center', size=8.2):
    """Polilínea con punta de flecha al final y una etiqueta sobre el tramo 'pos' (0-1)."""
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, ls=ls, zorder=4,
                solid_capstyle='round')
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>', mutation_scale=13,
                                 color=color, lw=lw, ls=ls, zorder=4, shrinkA=0, shrinkB=0))
    if etiqueta:
        # Punto a lo largo de la polilínea donde va la etiqueta.
        import numpy as np
        p = np.array(pts, float)
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        d = pos * seg.sum()
        for i, s in enumerate(seg):
            if d <= s or i == len(seg) - 1:
                t = 0 if s == 0 else d / s
                x, y = p[i] + t * (p[i + 1] - p[i])
                break
            d -= s
        ax.text(x + dx, y + dy, etiqueta, ha=ha, va='bottom', fontsize=size, color=color,
                zorder=5, bbox=dict(boxstyle='round,pad=0.15', fc=SURF, ec='none', alpha=0.9))


def main():
    os.makedirs(OUT, exist_ok=True)
    fig, ax = plt.subplots(figsize=(16, 9.8))
    ax.set_xlim(0, 16.6)
    ax.set_ylim(0, 9.8)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('Lazo cerrado del brazo de Milo (como corre hoy en simulación)', loc='left',
                 fontsize=13, fontweight='bold', pad=10)

    Y = 7.3   # fila principal: de la orden al movimiento
    # ---------------------------------------------------------------- fila principal
    A = caja(ax, 1.45, Y, 2.3, 1.6, 'OBJETIVO',
             'pose de la pinza\n(x, y, z + orientación)\nmarcador de RViz o\n/arm_target_pose',
             color=C_GRIP, fill=C_NOTE)
    B = caja(ax, 4.55, Y, 2.8, 1.7, 'CINEMÁTICA INVERSA',
             'arm_ik_node\nDLS ponderado, 7 ejes\n(lift con peso 10)\n'
             '→ q* (7 valores) y duración T')
    C = caja(ax, 8.1, Y, 3.1, 1.7, 'CONTROLADOR',
             'arm_controller (JTC), 100 Hz\ninterpola q_ref(t) de q actual a q*\n'
             'compara q_ref con q medido:\ntolerancia de meta 0.02 rad / 5 mm')
    D = caja(ax, 11.65, Y, 2.9, 1.7, 'ACTUADORES',
             'sim: GazeboSystem\n(mando por posición: pone\ncada eje en q_ref, sin fuerzas)\n'
             'real: Arduino + 6 TB6600 (pendiente)')
    E = caja(ax, 14.95, Y, 2.5, 1.7, 'PLANTA',
             'brazo (6 ejes) + lift\n+ pinza\nsalida: pose real\nde la pinza', color=C_BASE,
             fill=C_FILL2)

    flecha(ax, [(A[1], Y), (B[0], Y)], 'pose\nobjetivo', dy=0.08)
    flecha(ax, [(B[1], Y), (C[0], Y)], 'q* (meta)\n+ T', dy=0.08)
    flecha(ax, [(C[1], Y), (D[0], Y)], 'q_ref(t)\n100 Hz', dy=0.08)
    flecha(ax, [(D[1], Y), (E[0], Y)], 'sim: SetPosition\nreal: pulsos\nSTEP/DIR', dy=0.08,
           size=7.8)

    # Resultado del controlador de vuelta a la IK (arriba, punteado gris).
    yr = Y + 1.25
    flecha(ax, [(C[0] + 0.3, C[3]), (C[0] + 0.3, yr), (B[1] - 0.3, yr), (B[1] - 0.3, B[3])],
           'resultado: éxito, o GOAL_TOLERANCE_VIOLATED si alguna articulación no llegó',
           color=MUTED, ls=(0, (4, 3)), pos=0.5, dy=0.06)

    # ---------------------------------------------------------------- realimentación de posición
    Yf = 4.55
    F = caja(ax, 11.65, Yf, 3.6, 1.35, 'SENSORES DE POSICIÓN',
             'joint_state_broadcaster → /joint_states (q medido, 100 Hz)\n'
             'sim: lee la posición que puso Gazebo (= q_ref)\nreal: encoders por eje (pendiente)')
    G = caja(ax, 6.0, Yf, 2.9, 1.1, 'robot_state_publisher',
             '/joint_states + URDF → TF:\ndónde está cada pieza y la pinza', color=C_BASE,
             fill=C_FILL2)

    # Planta -> sensores (baja por la derecha), sensores -> controlador (sube), sensores -> IK.
    flecha(ax, [(E[0] + 1.25, E[2]), (E[0] + 1.25, Yf), (F[1], Yf)],
           'ángulo real\nde cada eje', pos=0.35, dx=0.55, dy=0.0, ha='left')
    flecha(ax, [(F[0] + 1.0, F[3]), (F[0] + 1.0, C[2])], 'q medido', pos=0.5, dx=0.12, dy=0.0,
           ha='left')
    flecha(ax, [(F[0], Yf), (G[1], Yf)], '/joint_states', dy=0.08)
    flecha(ax, [(G[0] + 0.6, G[3]), (G[0] + 0.6, B[2])],
           'TF: frame del objetivo\ny pose actual de la pinza', pos=0.5, dx=0.12, dy=0.0,
           ha='left')
    flecha(ax, [(F[0] + 0.4, F[2]), (F[0] + 0.4, Yf - 1.05), (B[0] + 0.5, Yf - 1.05),
                (B[0] + 0.5, B[2])],
           'q actual: punto de partida de la IK (semilla) y de la trayectoria', pos=0.45, dy=0.06)

    # ------------------------------------------------- lazo de percepción (pendiente)
    Yp = 2.45
    H = caja(ax, 11.65, Yp, 3.6, 1.15, 'CÁMARA DE LA PINZA (lazo pendiente)',
             'Orbbec Gemini Plus, 15 Hz: color, profundidad, nube\n'
             '→ detección del objeto → pose del objeto', color=C_GRIP, fill=C_NOTE,
             ls=(0, (5, 3)))
    flecha(ax, [(E[0] + 1.9, E[2]), (E[0] + 1.9, Yp), (H[1], Yp)],
           'la cámara se mueve\ncon la pinza: ve la escena', color=C_GRIP, ls=(0, (5, 3)),
           pos=0.3, dx=-0.15, dy=0.0, ha='right', size=7.8)
    flecha(ax, [(H[0], Yp), (A[0] + 0.7, Yp), (A[0] + 0.7, A[2])],
           'pose del objeto → nuevo objetivo ("mirar y luego moverse", pendiente)',
           color=C_GRIP, ls=(0, (5, 3)), pos=0.45, dy=0.06)

    # ---------------------------------------------------------------- nota y leyenda
    nota = ('QUÉ CIERRA Y QUÉ NO (medido el 17-10-2026)\n'
            'En Gazebo, con mando por posición, el eje queda exactamente en q_ref: la posición\n'
            '"medida" es la orden misma, así que el lazo de posición se cierra sobre sí '
            'mismo.\n'
            'Un choque no frena el brazo: inclina la base 3° y la corre 3.5 cm, y el '
            'controlador\n'
            'reporta éxito. En el robot real sin encoders pasa lo mismo (el Arduino cuenta '
            'pulsos).\n'
            'El lazo se cierra de verdad con encoders en cada eje, o al menos con la IMU de la\n'
            'pinza comparada con la cinemática directa. Evitar choques: MoveIt 2 antes de mover.')
    ax.text(0.35, 1.72, nota, ha='left', va='top', fontsize=8.4, color=INK, linespacing=1.4,
            zorder=3, bbox=dict(boxstyle='round,pad=0.45', fc='#f1f3f6', ec='#d6d5ce'))
    leyenda = [((0, (1, 0)), INK, 'existe hoy (simulación)'),
               ((0, (4, 3)), MUTED, 'señal de estado (resultado)'),
               ((0, (5, 3)), C_GRIP, 'pendiente (robot real / percepción)')]
    for i, (ls, c, t) in enumerate(leyenda):
        x = 7.9 + i * 2.95   # en una fila, arriba a la derecha
        ax.plot([x, x + 0.5], [9.35, 9.35], color=c, lw=1.6, ls=ls)
        ax.text(x + 0.6, 9.35, t, va='center', fontsize=8.2, color=INK)

    fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.01)
    ruta = os.path.join(OUT, 'fig_lazo_cerrado.png')
    fig.savefig(ruta, dpi=120)
    print(ruta)


if __name__ == '__main__':
    main()
