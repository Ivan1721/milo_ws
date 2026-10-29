"""Dos figuras para el mensaje de actualización al equipo: qué cambió en el brazo y qué falta.

  fig_actualizacion_flujo.png         de la cámara a los motores, en vertical para el celular:
                                      azul = ya estaba en el mensaje anterior,
                                      verde (y la etiqueta NUEVO) = nuevo desde entonces,
                                      naranja punteado = pendiente (robot real / detección).
  fig_actualizacion_antes_ahora.png   tabla visual «antes → ahora» por tema.

Son esquemas: no leen el URDF ni necesitan ROS, solo matplotlib. Reusan la paleta y los bloques
de fig_lazo_cerrado.py, así que los colores significan lo mismo que en las figuras B y C del
informe (naranja punteado = pendiente, verde = lo que agrega MoveIt 2). La letra es más grande
que en B y C porque estas figuras se mandan por chat y se ven en una pantalla chica.
  python3 fig_actualizacion.py [carpeta_salida]      # por defecto ./figuras
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

from fig_lazo_cerrado import (  # noqa: E402
    C_ARM, C_FILL, C_GRIP, C_NOTE, INK, LABEL, MUTED, TEXT, flecha)

OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'
C_NEW, C_FILL_NEW = '#14915f', '#e9f7f1'     # verde: lo nuevo (el mismo de la figura C)
TEXT[C_NEW] = LABEL[C_NEW] = '#0f7a52'       # variante oscura para letra (4.8:1)
C_PAR, C_FILL_PAR, T_PAR = '#7a5cb8', '#f3effa', '#5b3f99'   # morado: investigación paralela
DASH = (0, (5, 3))


def caja(ax, cx, cy, w, h, titulo, texto, color, fill, ls='-', nuevo=False):
    """Bloque con título en negrita (y la etiqueta NUEVO si corresponde) y el texto debajo."""
    ax.add_patch(FancyBboxPatch((cx - w / 2, cy - h / 2), w, h,
                                boxstyle='round,pad=0.02,rounding_size=0.12',
                                fc=fill, ec=color, lw=1.8, ls=ls, zorder=2))
    ax.text(cx, cy + h / 2 - 0.3, titulo + ('  ·  NUEVO' if nuevo else ''), ha='center',
            va='top', fontsize=11.5, fontweight='bold', color=TEXT[color], zorder=3)
    ax.text(cx, cy + h / 2 - 1.05, texto, ha='center', va='top', fontsize=10, color=INK,
            linespacing=1.35, zorder=3)
    return cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2


def flujo():
    """De la cámara a los motores: dos columnas (simulación y robot real), de arriba abajo."""
    fig, ax = plt.subplots(figsize=(7, 12.8))
    ax.set_xlim(0, 19.6)
    ax.set_ylim(-6.8, 28.8)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.text(0.2, 28.5, 'El brazo ya planifica sin choques en simulación;\n'
            'el robot real sigue pendiente', va='top', fontsize=13, fontweight='bold')
    ax.text(2.1, 25.9 + 0.9, 'SIMULACIÓN: corre hoy', fontsize=11, fontweight='bold',
            color=TEXT[C_ARM])
    ax.text(10.7, 16.9, 'ROBOT REAL: pendiente', fontsize=11, fontweight='bold',
            color=TEXT[C_GRIP])

    W, H = 7.0, 2.6
    XL, XR = 5.6, 14.2
    ys = [24.0, 19.4, 14.8, 10.2, 5.6]
    cam = caja(ax, XL, ys[0], W, H, 'Cámara Gemini Plus', 'color, IR y profundidad\nen la pinza',
               C_ARM, C_FILL)
    det = caja(ax, XL, ys[1], W, H, 'Detector YOLO',
               'manija de la puerta\nentrenado en simulación',
               C_NEW, C_FILL_NEW, nuevo=True)
    mov = caja(ax, XL, ys[2], W, H, 'MoveIt 2', 'planifica sin choques\nKDL + OMPL + OctoMap',
               C_NEW, C_FILL_NEW, nuevo=True)
    ctl = caja(ax, XL, ys[3], W, H, 'arm_controller', 'ros2_control\nlift + joint_1..6',
               C_ARM, C_FILL)
    sim = caja(ax, XL, ys[4], W, H, 'Gazebo', 'brazo simulado\n(mando por posición)',
               C_ARM, C_FILL)
    obj = caja(ax, XR, ys[0], W, H, 'Objetivo', 'marcador en RViz o\n/arm_target_pose', C_ARM,
               C_FILL)
    sen = caja(ax, XR, ys[2], W, H, 'Sensores', 'IMU MPU-9250 (I²C)\nfinales de carrera',
               C_GRIP, C_NOTE, ls=DASH)
    hw = caja(ax, XR, ys[3], W, H, 'Interface de hardware',
              'plugin propio en C++\n(el mismo controlador)',
              C_GRIP, C_NOTE, ls=DASH)
    ard = caja(ax, XR, ys[4], W, H, 'Arduino Mega', 'AccelStepper + homing', C_GRIP, C_NOTE,
               ls=DASH)
    drv = caja(ax, XR, 1.0, W, H, '6 × TB6600', 'drivers de los motores', C_GRIP, C_NOTE,
               ls=DASH)
    mot = caja(ax, XR, -3.6, W, H, 'Motores', 'paso a paso\nNEMA 23 / NEMA 17', C_GRIP,
               C_NOTE, ls=DASH)

    sz = 9.5
    flecha(ax, [(XL, cam[2]), (XL, det[3])], 'imagen', C_ARM, dx=0.25, ha='left', size=sz)
    flecha(ax, [(XL, det[2]), (XL, mov[3])], 'pose del objeto\n(por cerrar)', C_GRIP, ls=DASH,
           dx=0.25, ha='left', size=sz)
    flecha(ax, [(XL, mov[2]), (XL, ctl[3])], 'trayectoria', C_NEW, dx=0.25, ha='left', size=sz)
    flecha(ax, [(XL, ctl[2]), (XL, sim[3])], None, C_ARM)
    # Nube de puntos → OctoMap, por el margen izquierdo (cámara → MoveIt).
    flecha(ax, [(cam[0], ys[0]), (0.9, ys[0]), (0.9, 15.6), (mov[0], 15.6)], None, C_NEW)
    ax.text(0.35, 19.8, 'nube de puntos → OctoMap (obstáculos)', rotation=90, ha='center',
            va='center', fontsize=sz, color=LABEL[C_NEW])
    # Posición medida de cada articulación (Gazebo → MoveIt), también por el margen izquierdo.
    flecha(ax, [(sim[0], ys[4]), (0.9, ys[4]), (0.9, 14.0), (mov[0], 14.0)], None, C_ARM)
    ax.text(0.35, 9.8, 'posición medida (/joint_states)', rotation=90, ha='center',
            va='center', fontsize=sz, color=LABEL[C_ARM])
    # Objetivo → MoveIt, por el pasillo entre las dos columnas.
    flecha(ax, [(obj[0], ys[0]), (9.9, ys[0]), (9.9, 15.6), (mov[1], 15.6)], None, C_ARM)
    # Lo mismo en el robot real: el controlador habla con la interface de hardware.
    flecha(ax, [(ctl[1], ys[3]), (hw[0], ys[3])], None, C_GRIP, ls=DASH)
    flecha(ax, [(XR, hw[2]), (XR, ard[3])], 'USB serie', C_GRIP, ls=DASH, dx=0.25, ha='left',
           size=sz)
    flecha(ax, [(XR, ard[2]), (XR, drv[3])], 'STEP / DIR / EN', C_GRIP, ls=DASH, dx=0.25,
           ha='left', size=sz)
    flecha(ax, [(XR, drv[2]), (XR, mot[3])], None, C_GRIP, ls=DASH)
    # Sensores → Arduino, por el margen derecho.
    flecha(ax, [(sen[1], ys[2]), (18.8, ys[2]), (18.8, ys[4]), (ard[1], ys[4])], None, C_GRIP,
           ls=DASH)
    ax.text(19.3, 10.2, 'I²C y homing', rotation=90, ha='center', va='center', fontsize=sz,
            color=LABEL[C_GRIP])

    for i, (txt, ec, fc, ls) in enumerate([
            ('ya estaba', C_ARM, C_FILL, '-'), ('NUEVO', C_NEW, C_FILL_NEW, '-'),
            ('pendiente', C_GRIP, C_NOTE, DASH)]):
        x = 1.0 + i * 6.3
        ax.add_patch(FancyBboxPatch((x, -6.4), 1.0, 0.7, boxstyle='round,pad=0.02', fc=fc,
                                    ec=ec, lw=1.8, ls=ls))
        ax.text(x + 1.3, -6.05, txt, va='center', fontsize=10.5)
    fig.tight_layout()
    ruta = os.path.join(OUT, 'fig_actualizacion_flujo.png')
    fig.savefig(ruta, dpi=130)
    plt.close(fig)
    print('escrito', ruta)


FILAS = [
    ('Mover el brazo', 'IK propia, sin revisar\nchoques',
     'MoveIt 2 planifica sin\nchoques (por defecto)'),
    ('Qué ve la cámara', 'imágenes de color,\nIR y profundidad',
     'además, la nube pasa a\nser obstáculos (OctoMap)'),
    ('Escena de prueba', 'una mesa con objetos',
     'mesa + puerta de prueba\nque se abre empujando'),
    ('Visión', 'sin detector',
     'YOLO de la manija: 60 de 60\n(en simulación, no en real)'),
    ('Pruebas', '6 pruebas automáticas',
     '11 pruebas + prueba de humo\nen Gazebo (~2 min)'),
    ('Docker', 'ROS + Gazebo', 'además MoveIt 2, YOLO\ny PyTorch'),
    ('Informe', 'glosario de 57 términos',
     'glosario de 64 + repasos de\nliteratura y tecnologías'),
]
PENDIENTES = ['Montaje físico del brazo', 'IMU en el URDF y su soporte',
              'Plano electrónico (Arduino + TB6600)', 'Medir topes y muñeca reales',
              'Pesar los reductores impresos', 'Driver del brazo real']


def antes_ahora():
    """Tabla vertical «antes → ahora» + pendientes + la investigación paralela."""
    fig, ax = plt.subplots(figsize=(7.2, 13.6))
    ax.set_xlim(0, 9)
    ax.set_ylim(-2.0, 15.0)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.text(0.2, 14.7, 'Brazo de Milo: del mensaje anterior a hoy', fontsize=14,
            fontweight='bold', va='top')
    ax.text(0.2, 14.05, 'rama brazo · 9 commits nuevos (del 10 al 19 de octubre)', fontsize=10,
            color=MUTED, va='top')
    ax.text(3.1, 13.35, 'ANTES', fontsize=10.5, fontweight='bold', color=TEXT[C_ARM],
            ha='center')
    ax.text(6.85, 13.35, 'AHORA', fontsize=10.5, fontweight='bold', color=TEXT[C_NEW],
            ha='center')

    h, y = 1.25, 13.1
    for tema, antes, ahora in FILAS:
        y0 = y - h
        ax.text(0.15, y0 + h / 2, tema.replace(' ', '\n', 1) if len(tema) > 12 else tema,
                fontsize=10, fontweight='bold', va='center', linespacing=1.2)
        ax.add_patch(FancyBboxPatch((1.9, y0 + 0.08), 2.5, h - 0.16, boxstyle='round,pad=0.02',
                                    fc=C_FILL, ec=C_ARM, lw=1.5))
        ax.text(3.15, y0 + h / 2, antes, fontsize=9.6, ha='center', va='center',
                linespacing=1.3)
        ax.annotate('', xy=(4.85, y0 + h / 2), xytext=(4.5, y0 + h / 2),
                    arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.4))
        ax.add_patch(FancyBboxPatch((4.95, y0 + 0.08), 3.85, h - 0.16, boxstyle='round,pad=0.02',
                                    fc=C_FILL_NEW, ec=C_NEW, lw=1.8))
        ax.text(6.875, y0 + h / 2, ahora, fontsize=9.6, ha='center', va='center',
                linespacing=1.3)
        y = y0

    ax.text(0.2, y - 0.4, 'Siguen pendientes (no cambiaron)', fontsize=11, fontweight='bold',
            color=TEXT[C_GRIP])
    for i, p in enumerate(PENDIENTES):
        col, fila = i % 2, i // 2
        x, yy = 0.2 + col * 4.4, y - 1.15 - fila * 0.78
        ax.add_patch(FancyBboxPatch((x, yy - 0.3), 4.2, 0.6, boxstyle='round,pad=0.02',
                                    fc=C_NOTE, ec=C_GRIP, lw=1.5, ls=DASH))
        ax.text(x + 0.15, yy, p, fontsize=9.4, va='center')
    y_par = y - 1.15 - 3 * 0.78 - 0.25

    ax.add_patch(FancyBboxPatch((0.2, y_par - 2.3), 8.6, 2.3, boxstyle='round,pad=0.02',
                                fc=C_FILL_PAR, ec=C_PAR, lw=1.6))
    ax.text(0.45, y_par - 0.3, 'Investigación paralela (no es parte del plan de Milo)',
            fontsize=10.5, fontweight='bold', color=T_PAR, va='top')
    ax.text(0.45, y_par - 0.85,
            'Alcanzabilidad y visibilidad de vistas con otro brazo\n'
            '(Piper + Pika), en otro repositorio. Milo le sirvió de\n'
            'plantilla: cámara en la pinza, MoveIt + OctoMap e\n'
            'imágenes etiquetadas solas. No se hace NBV en Milo.',
            fontsize=9.6, va='top', linespacing=1.4)
    fig.tight_layout()
    ruta = os.path.join(OUT, 'fig_actualizacion_antes_ahora.png')
    fig.savefig(ruta, dpi=130)
    plt.close(fig)
    print('escrito', ruta)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    flujo()
    antes_ahora()
