"""Figura de notación: una articulación rotativa (R) y una prismática (P) de Milo, lado a lado.

Explica los símbolos de las fórmulas del informe (T_o,i, M_i(q_i), z_i, q_i, p_i, p_e y la columna
del jacobiano) sobre el robot real: las mallas 3D del URDF se dibujan casi transparentes, vistas
desde una esquina, y encima va el "esqueleto" (una línea por eslabón, de articulación a
articulación, que es lo que usan las cuentas). Así se ve a qué pieza corresponde cada símbolo.
  Izquierda: joint_2, el hombro (rotativa). Gris: lo que no se mueve con ella; azul: lo que gira.
  Derecha:   el lift (prismática). Gris: base y columna; azul: el carro y el brazo, que suben.
En los dos, la línea punteada es el esqueleto con q_i = 0, para ver qué cambió.

Las posiciones salen del URDF procesado con las funciones de figuras.py (mismas mallas y colores),
así que necesita el workspace compilado y cargado (dentro del contenedor):
  ./sim.sh shell
  source /ros2_ws/install/setup.bash
  cd /ros2_ws/src/andesrobot_arm
  python3 scripts/fig_articulaciones.py docs/figuras
  # y en el PC: cp ros2_ws/src/andesrobot_arm/docs/figuras/fig_articulaciones.png docs/informe/img/
Genera fig_articulaciones.png en la carpeta indicada (por defecto ./figuras).
"""
import os
import sys

import numpy as np

# figuras.py está en la misma carpeta: al importarlo procesa el xacro y carga las mallas.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import figuras as fg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.patches import FancyArrowPatch  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'

SURF, INK, MUTED = fg.SURF, fg.INK, fg.MUTED
C_ARM, C_GRIP, C_FIX = fg.C_ARM, fg.C_GRIP, '#8c8b84'
C_X, C_Y, C_Z = '#d23b3b', '#2e9d4f', '#2f6fe0'    # ejes de los frames, como RViz
C_T, C_K, C_V = '#7a52cc', '#eb6834', '#111111'    # traslación fija, eje y q, velocidad
ALPHA_MALLA = 0.12                                  # mallas casi transparentes

# Postura de ejemplo (la misma en los dos paneles): brazo hacia adelante, algo doblado.
POSTURA = {'joint_2': 0.6, 'joint_3': 1.1, 'joint_5': 0.6}
Q_LIFT = 0.30                                       # cuánto subió el lift en el panel P (m)

# Cadena del esqueleto, de la base del brazo a la pinza (nombres de joints del URDF).
CADENA = ['carriage_to_arm_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5',
          'joint_6', 'gripper_tcp_joint']
JOINT = {j['name']: j for j in fg.JOINTS}


class Camara:
    """Proyección ortográfica desde una esquina: azimut y elevación en grados, como matplotlib."""

    def __init__(self, azim, elev):
        a, e = np.radians(azim), np.radians(elev)
        self.c = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])  # al ojo
        self.r = np.array([-np.sin(a), np.cos(a), 0.0])                                # derecha
        self.u = np.cross(self.c, self.r)                                              # arriba

    def P(self, p):
        p = np.asarray(p, float)
        return np.stack([p @ self.r, p @ self.u], axis=-1)

    def prof(self, p):
        return np.asarray(p, float) @ self.c          # más grande = más cerca del ojo


def pos_joint(T, nombre):
    """Frame de la articulación (padre · origin, sin su movimiento): ahí viven p_i y su eje."""
    j = JOINT[nombre]
    return T[j['parent']] @ j['origin']


def esqueleto(T):
    return [pos_joint(T, n)[:3, 3] for n in CADENA]


def mallas(ax, cam, T, color_de):
    """Todas las mallas del robot con su color, casi transparentes, de atrás hacia adelante."""
    polys, cols, prof = [], [], []
    luz = cam.c * 0.6 + np.array([0, 0, 0.8])
    luz /= np.linalg.norm(luz)
    for link, t in fg.placed(T):
        color = color_de(link)
        if color is None:
            continue
        polys.append(cam.P(t))
        cols.append(fg.shade(color, fg.normals_of(t), luz))
        prof.append(cam.prof(t).mean(1))
    orden = np.argsort(np.concatenate(prof))
    cols = np.vstack(cols)[orden]
    cols = np.c_[cols, np.full(len(cols), ALPHA_MALLA)]
    ax.add_collection(PolyCollection(np.vstack(polys)[orden], facecolors=cols, linewidths=0,
                                     zorder=1))


# ----------------------------------------------------------------- dibujo 2D sobre la proyección
def flecha(ax, cam, a, b, color, lw=2.2, ms=14, ls='-', z=6):
    ax.add_patch(FancyArrowPatch(cam.P(a), cam.P(b), arrowstyle='-|>', mutation_scale=ms,
                                 color=color, lw=lw, linestyle=ls, zorder=z,
                                 shrinkA=0, shrinkB=0))


def linea(ax, cam, pts, color, lw=2.0, ls='-', z=4, alpha=1.0):
    q = cam.P(np.array(pts))
    ax.plot(q[:, 0], q[:, 1], color=color, lw=lw, ls=ls, zorder=z, alpha=alpha,
            solid_capstyle='round', dash_capstyle='round')


def texto(ax, cam, p, s, color=INK, dx=0.0, dy=0.0, size=10.5, ha='left', va='center'):
    u, v = cam.P(p)
    ax.text(u + dx, v + dy, s, color=color, fontsize=size, ha=ha, va=va, zorder=9,
            bbox=dict(boxstyle='round,pad=0.22', fc=SURF, ec='none', alpha=0.85))


def frame(ax, cam, T, nombre, largo=0.09, dx=0.0, dy=-0.05, ha='left'):
    o = T[:3, 3]
    for i, c in enumerate((C_X, C_Y, C_Z)):
        flecha(ax, cam, o, o + T[:3, i] * largo, c, lw=1.6, ms=9, z=7)
    if nombre:
        texto(ax, cam, o, nombre, color=MUTED, dx=dx, dy=dy, size=10, ha=ha)


def punto(ax, cam, p, color, s=40):
    ax.scatter(*cam.P(p), s=s, color=color, edgecolor=INK, linewidths=0.8, zorder=8)


def nota(ax, s):
    """Recuadro de explicación debajo del panel (fuera del dibujo, para no tapar nada)."""
    ax.text(0.0, -0.03, s, transform=ax.transAxes, fontsize=10, color=INK, va='top',
            ha='left', linespacing=1.45, zorder=10,
            bbox=dict(boxstyle='round,pad=0.5', fc='#f1f3f6', ec='#d6d5ce'))


def esqueleto_dibujo(ax, cam, pts, desde, fantasma=None):
    """Eslabones como líneas entre articulaciones: gris hasta 'desde', azul después."""
    linea(ax, cam, pts[:desde + 1], C_FIX, lw=4, z=4)
    linea(ax, cam, pts[desde:], C_ARM, lw=4, z=4)
    if fantasma is not None:
        linea(ax, cam, fantasma[desde:], C_ARM, lw=2.2, ls=(0, (2, 2)), z=3, alpha=0.7)
    for p in pts[1:-1]:
        punto(ax, cam, p, '#ffffff', s=22)


def encuadre(ax, cam, pts, margen=0.12):
    q = cam.P(np.array(pts))
    lo, hi = q.min(0) - margen, q.max(0) + margen
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.axis('off')


# ------------------------------------------------------------------------------------- paneles
def panel_R(ax):
    cam = Camara(azim=28, elev=16)      # desde adelante-izquierda: el hombro gira casi de frente
    T = fg.world(POSTURA)
    T0 = fg.world(dict(POSTURA, joint_2=0.0))
    i = CADENA.index('joint_2')

    def color_de(link):
        if link in ('arm_base_link_1', 'link_1_1', 'motor_joint_1', 'motor_joint_2'):
            return C_FIX                # no se mueven con joint_2
        if link.startswith(('link_', 'motor_joint_', 'gripper_servo')):
            return C_ARM
        if 'finger' in link:
            return C_GRIP
        return None                     # base, columna, carro: fuera de este panel

    mallas(ax, cam, T, color_de)
    pts, pts0 = esqueleto(T), esqueleto(T0)
    esqueleto_dibujo(ax, cam, pts, i, pts0)

    # {i-1}: el frame del eslabón anterior (link_1_1) y la traslación fija T_o hasta p_i.
    Tprev = T['link_1_1']
    Jf = pos_joint(T, 'joint_2')
    pi, pe = Jf[:3, 3], T['gripper_tcp'][:3, 3]
    frame(ax, cam, Tprev, r'$\{i-1\}$ (link_1)', dx=0.03, dy=-0.06)
    flecha(ax, cam, Tprev[:3, 3], pi, C_T, lw=2.2, ms=12, ls='--')
    texto(ax, cam, (Tprev[:3, 3] + pi) / 2, r'$T_{o,i}$: traslación fija', color=C_T,
          dx=0.05, dy=-0.02, size=10)
    punto(ax, cam, pi, INK, s=36)
    texto(ax, cam, pi, r'$\mathbf{p}_i$', dx=-0.03, dy=-0.06, ha='right', size=12)

    # Eje de giro z_i (en el mundo) y ángulo q_i entre el esqueleto en cero y el actual.
    z = Jf[:3, :3] @ (JOINT['joint_2']['axis'] / np.linalg.norm(JOINT['joint_2']['axis']))
    flecha(ax, cam, pi, pi + z * 0.22, C_K, lw=3.4, ms=18, z=7)
    texto(ax, cam, pi + z * 0.22, r'$\mathbf{z}_i$ eje de giro', color=C_K, dx=0.02,
          dy=-0.05, size=11.5)
    d0, d1 = pts0[i + 1] - pi, pts[i + 1] - pi
    d0, d1 = d0 - z * (d0 @ z), d1 - z * (d1 @ z)
    a = np.arctan2(np.cross(d0, d1) @ z, d0 @ d1)
    e1 = d0 / np.linalg.norm(d0)
    e2 = np.cross(z, e1)
    arc = [pi + 0.16 * (np.cos(t) * e1 + np.sin(t) * e2) for t in np.linspace(0, a, 30)]
    linea(ax, cam, arc[:-2], C_K, lw=2.0, z=7)
    flecha(ax, cam, arc[-3], arc[-1], C_K, lw=2.0, ms=12, z=7)
    texto(ax, cam, arc[len(arc) // 2], r'$q_i$ = ángulo (rad)', color=C_K, dx=0.03, dy=0.03,
          size=11.5)
    texto(ax, cam, pts0[-1], r'$q_i = 0$', color=MUTED, dx=0.03, dy=0.02, size=10)

    # Pinza, palanca y la velocidad que entra al jacobiano.
    linea(ax, cam, [pi, pe], INK, lw=1.2, ls=(0, (4, 3)), z=5)
    texto(ax, cam, pi + (pe - pi) * 0.55, r'$\mathbf{p}_e-\mathbf{p}_i$ (palanca)', color=INK,
          dx=0.03, dy=-0.05, size=10.5)
    punto(ax, cam, pe, C_GRIP, s=80)
    texto(ax, cam, pe, r'$\mathbf{p}_e$ (pinza)', dx=0.04, dy=0.05, size=12)
    v = np.cross(z, pe - pi) * 0.35
    flecha(ax, cam, pe, pe + v, C_V, lw=2.6, ms=16, z=8)
    texto(ax, cam, pe + v, r'$J_{v,i}=\mathbf{z}_i\times(\mathbf{p}_e-\mathbf{p}_i)$',
          color=C_V, dx=0.0, dy=-0.07, ha='center', size=11.5)
    encuadre(ax, cam, pts + pts0 + [pe + v, pi + z * 0.22])
    ax.set_title('Rotativa (R) · joint_2, el hombro', fontsize=13, fontweight='bold',
                 loc='left')
    nota(ax,
         r'$M_i(q_i)$: giro de $q_i$ alrededor de $\mathbf{z}_i$.' + '\n'
         'Gris: lo que no se mueve. Azul: lo que gira.\n'
         'La pinza recorre un círculo: su velocidad es\n'
         'perpendicular al eje y a la palanca. También\n'
         r'gira: $J_{\omega,i}=\mathbf{z}_i$.')


def panel_P(ax):
    cam = Camara(azim=-38, elev=14)     # desde adelante-derecha: se ve la columna y el carro
    T = fg.world(dict(POSTURA, vertical_lift_joint=Q_LIFT))
    T0 = fg.world(POSTURA)

    def color_de(link):
        if link in ('vertical_column_link_1', 'base_link', 'lidar_link_1',
                    'left_wheel_link_1', 'right_wheel_link_1', 'front_caster_link_1'):
            return C_FIX                # no se mueven con el lift
        if 'finger' in link:
            return C_GRIP
        if link.startswith(('link_', 'motor_joint_', 'gripper_servo', 'arm_base',
                            'lift_carriage')):
            return C_ARM
        return None

    mallas(ax, cam, T, color_de)
    # Esqueleto: de la columna (frame del lift) a la pinza.
    Tcol = T['vertical_column_link_1']
    Jf = pos_joint(T, 'vertical_lift_joint')
    pi = Jf[:3, 3]
    carro, carro0 = T['lift_carriage_link_1'][:3, 3], T0['lift_carriage_link_1'][:3, 3]
    pts = [pi, carro] + esqueleto(T)
    pts0 = [pi, carro0] + esqueleto(T0)
    linea(ax, cam, [Tcol[:3, 3], pi], C_FIX, lw=4, z=4)
    esqueleto_dibujo(ax, cam, pts, 0, pts0)
    pe, pe0 = T['gripper_tcp'][:3, 3], T0['gripper_tcp'][:3, 3]

    frame(ax, cam, Tcol, r'$\{i-1\}$ (columna)', dx=0.04, dy=-0.07)
    flecha(ax, cam, Tcol[:3, 3], pi, C_T, lw=2.2, ms=12, ls='--')
    texto(ax, cam, (Tcol[:3, 3] + pi) / 2, r'$T_{o,i}$: traslación fija', color=C_T,
          dx=-0.05, ha='right', size=10)
    punto(ax, cam, pi, INK, s=36)
    texto(ax, cam, pi, r'$\mathbf{p}_i$', dx=-0.04, dy=-0.04, ha='right', size=12)

    # Eje z_i (vertical) y el avance q_i = d, como cota entre el carro en cero y el actual.
    z = Jf[:3, :3] @ JOINT['vertical_lift_joint']['axis']
    lado = -cam.r * 0.16
    eje0 = pi + lado * 1.9 + z * 0.15
    flecha(ax, cam, eje0, eje0 + z * 0.35, C_K, lw=3.4, ms=18, z=7)
    texto(ax, cam, eje0 + z * 0.35, r'$\mathbf{z}_i$ eje de avance', color=C_K, dx=0.0,
          dy=0.06, ha='center', size=11.5)
    top = eje0 + z * 0.35
    a, b = carro0 + lado, carro + lado
    ax.annotate('', xy=cam.P(b), xytext=cam.P(a), zorder=8,
                arrowprops=dict(arrowstyle='<|-|>', color=C_K, lw=2.0, mutation_scale=13))
    linea(ax, cam, [carro0, a], MUTED, lw=0.8, z=5)
    linea(ax, cam, [carro, b], MUTED, lw=0.8, z=5)
    texto(ax, cam, (a + b) / 2, r'$q_i = d$ (m)', color=C_K, dx=-0.03, ha='right', size=11.5)
    texto(ax, cam, pe0, r'$q_i = 0$', color=MUTED, dx=0.03, dy=-0.03, size=10)

    punto(ax, cam, pe, C_GRIP, s=80)
    texto(ax, cam, pe, r'$\mathbf{p}_e$ (pinza)', dx=0.04, dy=-0.05, size=12)
    flecha(ax, cam, pe, pe + z * 0.3, C_V, lw=2.6, ms=16, z=8)
    texto(ax, cam, pe + z * 0.3, r'$J_{v,i}=\mathbf{z}_i$', color=C_V, dx=0.03, dy=0.02,
          size=11.5)
    base = fg.all_points(T, ['base_link', 'vertical_column_link_1'])
    encuadre(ax, cam, pts + pts0 + [top, pe + z * 0.3] + list(base[::200]))
    ax.set_title('Prismática (P) · el lift', fontsize=13, fontweight='bold', loc='left')
    nota(ax,
         r'$M_i(q_i)$: avance de $q_i$ a lo largo de $\mathbf{z}_i$.' + '\n'
         'Gris: base y columna, no se mueven. Azul: el\n'
         'carro y todo el brazo suben juntos. La pinza\n'
         'se mueve igual que el carro, sin importar la\n'
         r'palanca, y no gira: $J_{\omega,i}=\mathbf{0}$.')


def main():
    os.makedirs(OUT, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(14, 9.6), gridspec_kw={'width_ratios': [1.15, 1]})
    panel_R(axs[0])
    panel_P(axs[1])
    fig.text(0.5, 0.02,
             'Mallas del URDF casi transparentes; encima, el esqueleto que usan las cuentas '
             '(una línea por eslabón, círculos en las articulaciones). Línea punteada azul: '
             r'el esqueleto con $q_i = 0$.' + '\nEjes de cada frame: x rojo, y verde, z azul. '
             r'Naranja: eje $\mathbf{z}_i$ y variable $q_i$. Morado: traslación fija '
             r'$T_{o,i}$.',
             ha='center', fontsize=10, color=MUTED, linespacing=1.5)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.95, bottom=0.21, wspace=0.08)
    ruta = os.path.join(OUT, 'fig_articulaciones.png')
    fig.savefig(ruta, dpi=130)
    print(ruta)


if __name__ == '__main__':
    main()
