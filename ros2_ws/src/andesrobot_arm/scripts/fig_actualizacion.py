"""Dos figuras para el mensaje de actualización al equipo: qué cambió en el brazo y qué falta.

  fig_actualizacion_flujo.png   de la cámara a los motores, con tres colores:
                                azul = ya estaba en el mensaje anterior (19 de octubre),
                                verde = nuevo desde entonces (corre en simulación),
                                gris punteado = pendiente (robot real).
  fig_actualizacion_antes_ahora.png   tabla visual «antes → ahora» por tema, en vertical para
                                      que se lea bien en el celular.

Son esquemas: no leen el URDF ni necesitan ROS, solo matplotlib. Reusan la paleta y los bloques
de fig_lazo_cerrado.py (la misma que las figuras B y C del informe).
  python3 fig_actualizacion.py [carpeta_salida]      # por defecto ./figuras
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

from fig_lazo_cerrado import (  # noqa: E402
    C_ARM, C_BASE, C_FILL, C_FILL2, INK, MUTED, SURF, caja, flecha)

OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'
C_NEW, C_FILL_NEW = '#1baf7a', '#e9f7f1'   # verde: lo nuevo (el mismo de la figura C)


def flujo():
    """De la cámara a los motores: lo que ya estaba, lo nuevo y lo que falta del robot real."""
    fig, ax = plt.subplots(figsize=(16, 10.4))
    ax.set_xlim(0, 20.4)
    ax.set_ylim(-4.6, 9.6)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('Brazo de Milo: de la cámara a los motores', loc='left', fontsize=14,
                 fontweight='bold', pad=10)

    # Fila 1: lo que corre en simulación.
    Y, W, H = 6.0, 3.2, 1.5
    cam = caja(ax, 2.2, Y, W, H, 'Cámara Gemini Plus', 'color, IR y profundidad\nen la pinza', C_ARM, C_FILL)
    det = caja(ax, 6.7, Y, W, H, 'Detector YOLO', 'manija de la puerta\nentrenado en simulación',
               C_NEW, C_FILL_NEW)
    mov = caja(ax, 11.2, Y, W, H, 'MoveIt 2', 'planifica sin choques\n(KDL + OMPL + OctoMap)',
               C_NEW, C_FILL_NEW)
    ctl = caja(ax, 15.2, Y, 3.2, H, 'arm_controller', 'ros2_control\nlift + joint_1..6', C_ARM, C_FILL)
    sim = caja(ax, 19.0, Y, 2.6, H, 'Gazebo', 'brazo simulado', C_ARM, C_FILL)

    flecha(ax, [(cam[1], Y), (det[0], Y)], 'imagen', C_ARM)
    flecha(ax, [(det[1], Y), (mov[0], Y)], 'pose del objeto\n(por cerrar)', C_BASE, ls='--', dy=0.14)
    flecha(ax, [(mov[1], Y), (ctl[0], Y)], 'trayectoria', C_NEW)
    flecha(ax, [(ctl[1], Y), (sim[0], Y)], None, C_ARM)
    # Nube de puntos de la cámara → OctoMap de MoveIt (lo que ve la cámara pasa a ser obstáculo).
    flecha(ax, [(2.2, cam[2]), (2.2, 3.6), (11.2, 3.6), (11.2, mov[2])],
           'nube de puntos → OctoMap (obstáculos)', C_NEW, pos=0.5, dy=0.12)
    # Realimentación: posición medida de cada articulación.
    flecha(ax, [(19.0, sim[3]), (19.0, 8.4), (12.2, 8.4), (12.2, mov[3])],
           '/joint_states (posición medida)', C_ARM, pos=0.45, dy=0.12)
    # Objetivo.
    flecha(ax, [(10.2, 8.75), (10.2, mov[3])], None, C_ARM)
    ax.text(10.0, 8.95, 'objetivo: marcador en RViz o un topic (`/arm_target_pose`)',
            ha='right', va='center', fontsize=8.6, color=INK,
            bbox=dict(boxstyle='round,pad=0.25', fc=C_FILL, ec=C_ARM, lw=1.2))

    # Fila 2: robot real (pendiente). Corre de derecha a izquierda, desde el controlador.
    Y2 = 0.9
    hw = caja(ax, 15.2, Y2, 3.2, 1.5, 'Interface de hardware', 'plugin ros2_control\npropio (C++)',
              C_BASE, C_FILL2, ls='--')
    ard = caja(ax, 10.6, Y2, 3.7, 1.5, 'Arduino Mega', 'AccelStepper + homing',
               C_BASE, C_FILL2, ls='--')
    drv = caja(ax, 6.2, Y2, 3.2, 1.5, '6 × TB6600', 'drivers de los\nmotores', C_BASE, C_FILL2, ls='--')
    mot = caja(ax, 2.2, Y2, 3.0, 1.5, 'Motores', 'paso a paso\nNEMA 23 / 17', C_BASE, C_FILL2, ls='--')
    flecha(ax, [(ctl[1] - 0.8, ctl[2]), (ctl[1] - 0.8, hw[3])],
           'el mismo controlador', C_BASE, ls='--', pos=0.5, dx=0.1, ha='left')
    flecha(ax, [(hw[0], Y2), (ard[1], Y2)], 'USB serie', C_BASE, ls='--')
    flecha(ax, [(ard[0], Y2), (drv[1], Y2)], 'STEP / DIR / EN', C_BASE, ls='--')
    flecha(ax, [(drv[0], Y2), (mot[1], Y2)], None, C_BASE, ls='--')

    imu = caja(ax, 9.4, -1.9, 3.1, 1.3, 'IMU MPU-9250', 'bajo la cámara', C_BASE, C_FILL2, ls='--')
    fin = caja(ax, 13.0, -1.9, 3.1, 1.3, 'Finales de carrera', 'para el homing', C_BASE, C_FILL2, ls='--')
    flecha(ax, [(9.4, imu[3]), (9.4, ard[2])], 'I²C', C_BASE, ls='--', dx=0.1, ha='left')
    flecha(ax, [(12.0, fin[3]), (12.0, ard[2])], None, C_BASE, ls='--')
    ax.text(0.2, 2.45, 'ROBOT REAL: pendiente', fontsize=10, fontweight='bold', color=MUTED)
    ax.text(0.2, 8.95, 'SIMULACIÓN: corre hoy', fontsize=10, fontweight='bold', color=C_ARM)

    # Leyenda.
    for i, (txt, ec, fc, ls) in enumerate([
            ('ya estaba (mensaje anterior)', C_ARM, C_FILL, '-'),
            ('nuevo desde entonces', C_NEW, C_FILL_NEW, '-'),
            ('pendiente (robot real)', C_BASE, C_FILL2, '--')]):
        x = 0.3 + i * 5.6
        ax.add_patch(FancyBboxPatch((x, -4.45), 0.6, 0.36, boxstyle='round,pad=0.02',
                                    fc=fc, ec=ec, lw=1.6, ls=ls))
        ax.text(x + 0.8, -4.27, txt, va='center', fontsize=9.2)
    fig.tight_layout()
    ruta = os.path.join(OUT, 'fig_actualizacion_flujo.png')
    fig.savefig(ruta, dpi=110)
    plt.close(fig)
    print('escrito', ruta)


FILAS = [
    ('Mover el brazo', 'IK propia, sin revisar choques', 'MoveIt 2 planifica sin choques\n(es lo que corre por defecto)'),
    ('Qué ve la cámara', 'imágenes de color, IR y profundidad', 'además, la nube pasa a ser\nobstáculos (OctoMap)'),
    ('Escena de prueba', 'una mesa con objetos', 'mesa + puerta de prueba\nque se abre empujando'),
    ('Visión', 'sin detector', 'YOLO de la manija: 60 de 60 halladas\nen simulación (aún no en real)'),
    ('Pruebas', '6 pruebas automáticas', '11 pruebas + prueba de humo\nen Gazebo (~2 min, sin ventanas)'),
    ('Docker', 'ROS + Gazebo', 'además MoveIt 2, YOLO y PyTorch'),
    ('Informe', 'glosario de 57 términos', 'glosario de 64 + repasos de\nliteratura y de tecnologías'),
]
PENDIENTES = ['Montaje físico del brazo', 'IMU en el URDF y su soporte',
              'Plano electrónico (Arduino + 6 TB6600)', 'Medir topes y muñeca reales',
              'Pesar los reductores impresos', 'Driver del brazo real']


def antes_ahora():
    """Tabla vertical «antes → ahora» + pendientes + la investigación paralela."""
    fig, ax = plt.subplots(figsize=(9, 13.4))
    ax.set_xlim(0, 9)
    ax.set_ylim(-0.4, 13.6)
    ax.axis('off')
    ax.text(0.2, 13.15, 'Brazo de Milo: del mensaje anterior a hoy', fontsize=15,
            fontweight='bold')
    ax.text(0.2, 12.7, 'rama brazo · 9 commits nuevos (del 10 al 19 de octubre)', fontsize=10,
            color=MUTED)
    ax.text(3.0, 12.2, 'ANTES', fontsize=9.5, fontweight='bold', color=C_ARM, ha='center')
    ax.text(6.6, 12.2, 'AHORA', fontsize=9.5, fontweight='bold', color=C_NEW, ha='center')

    h, y = 1.0, 12.0
    for tema, antes, ahora in FILAS:
        y0 = y - h
        ax.text(0.2, y0 + h / 2, tema, fontsize=10, fontweight='bold', va='center')
        ax.add_patch(FancyBboxPatch((1.7, y0 + 0.08), 2.7, h - 0.16, boxstyle='round,pad=0.02',
                                    fc=C_FILL, ec=C_ARM, lw=1.3))
        ax.text(3.05, y0 + h / 2, antes, fontsize=8.8, ha='center', va='center')
        ax.annotate('', xy=(4.85, y0 + h / 2), xytext=(4.5, y0 + h / 2),
                    arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.4))
        ax.add_patch(FancyBboxPatch((4.95, y0 + 0.08), 3.85, h - 0.16, boxstyle='round,pad=0.02',
                                    fc=C_FILL_NEW, ec=C_NEW, lw=1.6))
        ax.text(6.875, y0 + h / 2, ahora, fontsize=8.8, ha='center', va='center')
        y = y0

    # Pendientes que siguen abiertos.
    ax.text(0.2, y - 0.35, 'Siguen pendientes (no cambiaron)', fontsize=10.5, fontweight='bold',
            color=MUTED)
    for i, p in enumerate(PENDIENTES):
        col, fila = i % 2, i // 2
        x, yy = 0.2 + col * 4.4, y - 1.0 - fila * 0.62
        ax.add_patch(FancyBboxPatch((x, yy - 0.22), 4.2, 0.46, boxstyle='round,pad=0.02',
                                    fc=C_FILL2, ec=C_BASE, lw=1.3, ls='--'))
        ax.text(x + 0.15, yy, p, fontsize=8.8, va='center')
    y_par = y - 1.0 - 3 * 0.62 - 0.45

    # Investigación paralela.
    ax.add_patch(FancyBboxPatch((0.2, y_par - 1.75), 8.6, 1.75, boxstyle='round,pad=0.02',
                                fc='#fff4e5', ec='#eb6834', lw=1.4))
    ax.text(0.4, y_par - 0.28, 'Investigación paralela (no es parte del plan de Milo)',
            fontsize=10, fontweight='bold', color='#eb6834')
    ax.text(0.4, y_par - 0.62,
            'Alcanzabilidad y visibilidad de vistas con otro brazo (Piper + Pika), en otro repositorio.\n'
            'Milo le sirvió de plantilla: cámara en la pinza, MoveIt 2 + OctoMap y\n'
            'datasets autoetiquetados. No se implementa NBV en Milo.',
            fontsize=8.8, va='top', linespacing=1.45)
    fig.tight_layout()
    ruta = os.path.join(OUT, 'fig_actualizacion_antes_ahora.png')
    fig.savefig(ruta, dpi=110)
    plt.close(fig)
    print('escrito', ruta)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    flujo()
    antes_ahora()
