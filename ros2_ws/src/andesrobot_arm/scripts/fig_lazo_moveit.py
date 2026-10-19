"""Figura del lazo cerrado del brazo CON MoveIt 2 (lo que corre con arm_sim moveit:=true).

Misma disposición que fig_lazo_cerrado.py (la figura B del informe), para comparar bloque a
bloque. Lo que cambia: en vez de la cinemática inversa sola, move_group arma una escena con el
robot, la mesa y lo que ve la cámara (OctoMap), resuelve la IK, planifica un camino completo sin
choques y le pone tiempo con rampas de aceleración; después vigila la ejecución. El lazo de
posición (controlador → actuadores → sensores) queda igual.
  azul    = igual que hoy
  verde   = nuevo con MoveIt 2
  naranja punteado = pendiente (robot real, detección del objeto)

Es un esquema: no lee el URDF ni necesita ROS, solo matplotlib. Reutiliza los bloques y flechas
de fig_lazo_cerrado.py (misma carpeta).
  python3 fig_lazo_moveit.py [carpeta_salida]      # por defecto ./figuras
Genera fig_lazo_moveit.png. El informe (docs/informe/README.md) la lee directo de docs/figuras.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib.pyplot as plt  # noqa: E402

from fig_lazo_cerrado import (  # noqa: E402
    C_ARM, C_BASE, C_FILL, C_FILL2, C_GRIP, C_NOTE, INK, MUTED, caja, flecha)

OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'
C_NEW, C_FILL_NEW = '#1baf7a', '#e9f7f1'   # verde: lo que agrega MoveIt 2


def main():
    os.makedirs(OUT, exist_ok=True)
    fig, ax = plt.subplots(figsize=(16, 9.8))
    ax.set_xlim(0, 16.6)
    ax.set_ylim(0, 9.8)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('Lazo cerrado del brazo de Milo con MoveIt 2 (arm_sim con moveit:=true)',
                 loc='left',
                 fontsize=13, fontweight='bold', pad=10)

    Y = 7.3   # fila principal
    # ---------------------------------------------------------------- fila principal
    A = caja(ax, 1.35, Y, 2.1, 1.7, 'OBJETIVO',
             'pose de la pinza\n(RViz, /arm_target_pose\no la pose del objeto\ndetectado)',
             color=C_GRIP, fill=C_NOTE)
    M = caja(ax, 5.35, Y, 4.9, 2.4, 'MOVEIT 2 (move_group)',
             'Escena: URDF + SRDF (pares que no chocan) + estado actual\n'
             '+ OctoMap de la cámara + la mesa\n'
             'IK: KDL o TRAC-IK  ·  Planificador: OMPL (RRT-Connect),\n'
             'revisa choques en cada tramo del camino\n'
             'Tiempo: TOTG / Ruckig (velocidad, aceleración y tirón)\n'
             '→ trayectoria completa (N puntos) sin choques',
             color=C_NEW, fill=C_FILL_NEW)
    C = caja(ax, 9.6, Y, 2.7, 1.7, 'CONTROLADOR',
             'arm_controller (JTC), 100 Hz\nsigue la trayectoria\npunto a punto\n'
             'tolerancias de meta y de camino')
    D = caja(ax, 12.6, Y, 2.5, 1.7, 'ACTUADORES',
             'sim: GazeboSystem\n(mando por posición)\nreal: Arduino + 6 TB6600\n(pendiente)')
    E = caja(ax, 15.15, Y, 2.2, 1.7, 'PLANTA',
             'brazo (6 ejes) + lift\n+ pinza\nsalida: pose real\nde la pinza', color=C_BASE,
             fill=C_FILL2)

    flecha(ax, [(A[1], Y), (M[0], Y)], 'pose\nobjetivo', dy=0.08)
    flecha(ax, [(M[1], Y), (C[0], Y)], 'trayectoria\n(FollowJoint-\nTrajectory)', color=C_NEW,
           dy=0.08, size=7.8)
    flecha(ax, [(C[1], Y), (D[0], Y)], 'q_ref(t)\n100 Hz', dy=0.08)
    flecha(ax, [(D[1], Y), (E[0], Y)], 'SetPosition /\nSTEP-DIR', dy=0.08, size=7.8)

    # Ejecución vigilada: el resultado del controlador vuelve a MoveIt (arriba, punteado).
    yr = M[3] + 0.35
    flecha(ax, [(C[0] + 0.3, C[3]), (C[0] + 0.3, yr), (M[1] - 0.3, yr), (M[1] - 0.3, M[3])],
           'avance y resultado de la ejecución: MoveIt vigila tiempo y tolerancias; '
           'si falla, aborta y puede replanificar', color=MUTED, ls=(0, (4, 3)), pos=0.5,
           dy=0.06)

    # ---------------------------------------------------------------- realimentación de posición
    Yf = 4.3
    F = caja(ax, 12.3, Yf, 3.3, 1.35, 'SENSORES DE POSICIÓN',
             'joint_state_broadcaster → /joint_states (100 Hz)\n'
             'sim: lee la posición que puso Gazebo (= q_ref)\nreal: encoders por eje (pendiente)')
    G = caja(ax, 5.0, Yf, 2.6, 1.1, 'robot_state_publisher',
             '/joint_states + URDF → TF', color=C_BASE, fill=C_FILL2)

    flecha(ax, [(E[0] + 1.1, E[2]), (E[0] + 1.1, Yf), (F[1], Yf)], 'ángulo real\nde cada eje',
           pos=0.35, dx=0.5, dy=0.0, ha='left')
    flecha(ax, [(F[0] + 0.15, F[3]), (F[0] + 0.15, C[2])], 'q medido', pos=0.5, dx=0.12,
           dy=0.0, ha='left')
    flecha(ax, [(F[0] + 0.95, F[3]), (F[0] + 0.95, 5.35), (M[1] - 0.6, 5.35), (M[1] - 0.6, M[2])],
           'estado actual de las articulaciones (planning scene monitor)', color=C_NEW,
           pos=0.55, dy=0.06)
    flecha(ax, [(F[0], Yf), (G[1], Yf)], '/joint_states', dy=0.08)
    flecha(ax, [(G[0] + 0.5, G[3]), (G[0] + 0.5, M[2])], 'TF', pos=0.5, dx=0.12, dy=0.0,
           ha='left')

    # ------------------------------------------------- percepción: obstáculos y objeto
    Yp = 2.15
    H = caja(ax, 12.3, Yp, 3.3, 1.35, 'CÁMARA DE LA PINZA',
             'Orbbec Gemini Plus, 15 Hz\nnube de puntos → OctoMap (obstáculos)\n'
             'detección del objeto → pose (pendiente)')
    flecha(ax, [(E[0] + 1.7, E[2]), (E[0] + 1.7, Yp), (H[1], Yp)],
           'la cámara va en la pinza:\nve la escena', pos=0.3, dx=-0.15, dy=0.0, ha='right',
           size=7.8)
    flecha(ax, [(H[0], Yp + 0.3), (M[0] + 0.4, Yp + 0.3), (M[0] + 0.4, M[2])],
           'nube de puntos → OctoMap: los obstáculos entran a la escena de planificación',
           color=C_NEW, pos=0.4, dy=0.06)
    flecha(ax, [(H[0], Yp - 0.35), (A[0] + 0.5, Yp - 0.35), (A[0] + 0.5, A[2])],
           'pose del objeto → nuevo objetivo (pendiente)', color=C_GRIP, ls=(0, (5, 3)),
           pos=0.4, dy=-0.33)

    # ---------------------------------------------------------------- nota y leyenda
    nota = ('QUÉ CAMBIA RESPECTO A LA FIGURA B\n'
            '1. Los choques se evitan ANTES de mover: la escena tiene el robot (URDF + SRDF), '
            'la mesa y lo que ve la cámara (OctoMap); el planificador descarta todo tramo que '
            'choque.\n'
            '2. Se planifica el camino completo, no solo la pose final, y se le pone tiempo con '
            'rampas de aceleración (Ruckig): apto para motores paso a paso.\n'
            '3. MoveIt vigila la ejecución: si el controlador no llega a tiempo o fuera de '
            'tolerancia, aborta y puede replanificar.\n'
            '4. El lazo de posición no cambia: en Gazebo sigue cerrándose sobre la orden y en '
            'el robot real necesita encoders. arm_moveit_node es el cliente de MoveIt; '
            'arm_ik_node queda para moveit:=false.')
    ax.text(0.35, 1.2, nota, ha='left', va='top', fontsize=8.3, color=INK, linespacing=1.4,
            zorder=3, bbox=dict(boxstyle='round,pad=0.45', fc='#f1f3f6', ec='#d6d5ce'))
    leyenda = [((0, (1, 0)), C_ARM, 'igual que sin MoveIt'),
               ((0, (1, 0)), C_NEW, 'lo que agrega MoveIt 2'),
               ((0, (4, 3)), MUTED, 'señal de estado'),
               ((0, (5, 3)), C_GRIP, 'pendiente (robot real / detección)')]
    for i, (ls, c, t) in enumerate(leyenda):
        x = 6.3 + i * 2.6
        ax.plot([x, x + 0.5], [9.35, 9.35], color=c, lw=1.8, ls=ls)
        ax.text(x + 0.6, 9.35, t, va='center', fontsize=8.2, color=INK)

    fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.01)
    ruta = os.path.join(OUT, 'fig_lazo_moveit.png')
    fig.savefig(ruta, dpi=120)
    print(ruta)


if __name__ == '__main__':
    main()
