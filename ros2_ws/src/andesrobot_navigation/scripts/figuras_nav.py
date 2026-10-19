#!/usr/bin/env python3
"""Figuras del informe de navegación (docs/navegacion/img/).

Todo sale de los mismos datos que usa el robot: la casa del generador
(andesrobot_gazebo/tools/gen_casa_robocup.py), los lugares anotados
(andesrobot_slam/maps/casa_lugares.yaml) y los parámetros de safety_nav.yaml / nav2_params.yaml.
Si cambias algo de eso, vuelve a correr este script.

Uso (en tu PC o en ./sim.sh shell; necesita numpy, matplotlib y pyyaml):
  python3 ros2_ws/src/andesrobot_navigation/scripts/figuras_nav.py
"""

import importlib.util
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
REPO = os.path.abspath(os.path.join(SRC, '..', '..'))
OUT = os.path.join(REPO, 'docs', 'navegacion', 'img')
GEN = os.path.join(SRC, 'andesrobot_gazebo', 'tools', 'gen_casa_robocup.py')
LUGARES = os.path.join(SRC, 'andesrobot_slam', 'maps', 'casa_lugares.yaml')
SAFETY = os.path.join(SRC, 'andesrobot_safety', 'config', 'safety.yaml')
SAFETY_NAV = os.path.join(SRC, 'andesrobot_safety', 'config', 'safety_nav.yaml')
NAV2 = os.path.join(SRC, 'andesrobot_navigation', 'config', 'nav2_params.yaml')

# Lugares que anotó Aleja el 11-oct-2026 (por si todavía no está el .yaml en el repo)
LUGARES_RESPALDO = {
    'sofa': dict(x=-3.15, y=-4.03, yaw=-1.55),
    'dining_table': dict(x=1.572, y=-2.738, yaw=0.398),
    'table': dict(x=-3.943, y=-4.308, yaw=-1.828),
    'base': dict(x=-2.145, y=-1.770, yaw=-1.564),
    'indoor': dict(x=-6.644, y=-2.153, yaw=-0.259),
    'outdoor': dict(x=-5.998, y=-3.982, yaw=-3.171),
}

AZUL, NARANJO, GRIS, ROJO, VERDE = '#2f6fd6', '#e8822a', '#8a8a8a', '#c8402c', '#3a9a4a'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.titlesize': 11,
                     'axes.spines.top': False, 'axes.spines.right': False})


def cargar_generador():
    spec = importlib.util.spec_from_file_location('gen_casa', GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def params(path, nodo):
    with open(path) as fh:
        return yaml.safe_load(fh)[nodo]['ros__parameters']


def lugares():
    if os.path.exists(LUGARES):
        with open(LUGARES) as fh:
            d = yaml.safe_load(fh) or {}
        d = d.get('ubicaciones', d)
        return {k: v for k, v in d.items() if isinstance(v, dict)}, True
    return LUGARES_RESPALDO, False


def guardar(fig, nombre):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, nombre)
    fig.savefig(p, dpi=160, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print('  ', os.path.relpath(p, REPO))


def dibujar_prim(ax, g, p, lidar_z=None, **kw):
    """Dibuja una primitiva del generador (en coordenadas de mundo)."""
    cz = p['c'][2]
    hz = (p['size'][2] if p['kind'] == 'box' else p['h']) / 2
    ve_lidar = lidar_z is not None and cz - hz <= lidar_z <= cz + hz
    x, y = g.to_world(p['c'][0], p['c'][1])
    color = '#2b2b2b' if ve_lidar else '#c9c9c9'
    if p['kind'] == 'cyl':
        ax.add_patch(mp.Circle((x, y), p['r'], color=color, lw=0, **kw))
    else:
        sx, sy = p['size'][0], p['size'][1]
        t = matplotlib.transforms.Affine2D().rotate_around(x, y, p['yaw']) + ax.transData
        r = mp.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color=color, lw=0, **kw)
        r.set_transform(t)
        ax.add_patch(r)


def flecha_pose(ax, x, y, yaw, color, largo=0.55, **kw):
    ax.annotate('', xy=(x + largo * math.cos(yaw), y + largo * math.sin(yaw)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=2.2, mutation_scale=14), **kw)
    ax.plot(x, y, 'o', color=color, ms=6, zorder=5)


def huella(ax, x, y, yaw, color, **kw):
    fp = [(0.44, 0.25), (0.44, -0.25), (-0.12, -0.25), (-0.12, 0.25)]
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + a * c - b * s, y + a * s + b * c) for a, b in fp]
    ax.add_patch(mp.Polygon(pts, closed=True, fill=False, ec=color, lw=1.3, **kw))


# ------------------------------------------------------------------------------------------
def fig_casa_lugares(g):
    locs, del_archivo = lugares()
    fig, ax = plt.subplots(figsize=(8.4, 7.6))
    walls = g.wall_prims()
    for p in walls:
        dibujar_prim(ax, g, p, lidar_z=g.LIDAR_Z)
    for f in g.FURN:
        for p in g.furn_prims(f):
            dibujar_prim(ax, g, p, lidar_z=g.LIDAR_Z)
    h = g.L / 2
    ax.add_patch(mp.Rectangle((-h, -h), g.L, g.L, color='#f6ead2', zorder=-1, lw=0))
    # nombres de las salas
    q = g.L / 4  # centro de cada cuarto de la casa
    for nombre, (x, y) in (('Laundry', (-q, q + 1.8)), ('Bedroom', (q + 0.9, q + 1.8)),
                           ('Living room', (-q, -0.75)), ('Kitchen', (q + 1.3, -0.75))):
        ax.text(x, y, nombre, ha='center', color='#9a7a50', fontsize=12, fontweight='bold',
                alpha=0.85)
    # partida
    sx, sy = g.to_world(*g.SPAWN_UV)
    huella(ax, sx, sy, 0.0, ROJO, ls='--')
    ax.text(sx, sy - 0.55, 'partida\nde Milo', color=ROJO, ha='center', va='top', fontsize=9)
    # lugares anotados
    for n, d in locs.items():
        x, y, yaw = float(d['x']), float(d['y']), float(d.get('yaw', 0.0))
        huella(ax, x, y, yaw, AZUL, alpha=0.7)
        flecha_pose(ax, x, y, yaw, AZUL)
        ax.text(x + 0.18, y + 0.22, n, color=AZUL, fontsize=9.5, fontweight='bold',
                bbox=dict(fc='white', ec='none', alpha=0.75, pad=1.2))
    ax.set_aspect('equal')
    ax.set_xlim(-h - 1.6, h + 0.5)
    ax.set_ylim(-h - 0.5, h + 0.5)
    ax.set_xlabel('x del mapa [m]')
    ax.set_ylabel('y del mapa [m]')
    ax.grid(alpha=0.25)
    fuente = 'casa_lugares.yaml' if del_archivo else 'lugares anotados el 11-oct'
    ax.set_title(f'Casa RoboCup@Home y los lugares anotados ({fuente})')
    leyenda = [mp.Patch(color='#2b2b2b', label='lo que ve el lidar (a 0.33 m)'),
               mp.Patch(color='#c9c9c9', label='cubiertas de mesas/sillas (el lidar no las ve)'),
               plt.Line2D([], [], color=AZUL, marker='o', lw=2, label='lugar: pose (x, y) + yaw'),
               plt.Line2D([], [], color=ROJO, ls='--', lw=1.3, label='huella de Milo al partir')]
    ax.legend(handles=leyenda, loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=2,
              fontsize=8.5, frameon=False)
    guardar(fig, 'fig_casa_lugares.png')


def fig_huella_puertas():
    sp = params(SAFETY, 'safety_filter')
    sn = params(SAFETY_NAV, 'safety_filter')
    x0, x1, hw = sp['footprint_x_min'], sp['footprint_x_max'], sp['footprint_half_width']
    r = max(math.hypot(x, hw) for x in (x0, x1))
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.6))
    # (a) huella
    ax = axs[0]
    ax.add_patch(mp.Rectangle((x0, -hw), x1 - x0, 2 * hw, fc='#dbe6fb', ec=AZUL, lw=2))
    ax.add_patch(mp.Circle((0, 0), r, fill=False, ec=NARANJO, lw=2, ls='--'))
    ax.add_patch(mp.Circle((0, 0), r + sp['rotation_clearance'], fill=False, ec=ROJO, lw=1.2,
                           ls=':'))
    ax.plot([0, x1], [0, hw], color=NARANJO, lw=1.5)
    ax.text(x1 / 2 - 0.05, hw / 2 + 0.06, f'r = {r:.3f} m', color=NARANJO, rotation=29)
    ax.plot(0, 0, 'k+', ms=12, mew=2)
    ax.text(0.02, -0.07, 'base_footprint\n(eje de ruedas)', fontsize=8, va='top')
    ax.plot(0.35, 0, 's', color='k', ms=6)
    ax.text(0.35, 0.04, 'lidar', fontsize=8, ha='center')
    ax.annotate('', xy=(0.75, 0), xytext=(0.5, 0),
                arrowprops=dict(arrowstyle='-|>', color='k', lw=1.5))
    ax.text(0.62, 0.05, '+x\nadelante', fontsize=8, ha='center')
    ax.set_aspect('equal')
    ax.set_xlim(-0.8, 0.9)
    ax.set_ylim(-0.75, 0.75)
    ax.set_title('(a) Huella de Milo y círculo de giro')
    ax.text(-0.78, -0.72, f'rojo: r + holgura de giro ({sp["rotation_clearance"]:.2f} m)'
            f' = {r + sp["rotation_clearance"]:.2f} m', fontsize=8, color=ROJO)
    ax.grid(alpha=0.25)
    # (b) y (c) puertas
    for ax, ancho, titulo in ((axs[1], 0.90, '(b) Puerta real: 0.90 m'),
                              (axs[2], 1.40, '(c) Puerta de la sim: 1.40 m')):
        t = 0.15
        for s in (-1, 1):
            ax.add_patch(mp.Rectangle((-t / 2, s * ancho / 2 if s > 0 else -ancho / 2 - 1.0),
                                      t, 1.0, color='#555'))
        ax.add_patch(mp.Rectangle((x0, -hw), x1 - x0, 2 * hw, fc='#dbe6fb', ec=AZUL, lw=2))
        rr = r + sp['rotation_clearance']
        libre = ancho / 2
        ok = libre > rr
        ax.add_patch(mp.Circle((0, 0), rr, fill=False, ec=VERDE if ok else ROJO, lw=1.8,
                               ls=':'))
        ax.annotate('', xy=(0, ancho / 2), xytext=(0, -ancho / 2),
                    arrowprops=dict(arrowstyle='<->', color='k', lw=1))
        ax.text(-0.42, 0, f'{ancho:.2f} m', rotation=90, va='center', fontsize=9)
        msg = ('puede girar en la puerta' if ok
               else 'giro bloqueado: el marco\nqueda dentro del círculo')
        ax.set_xlabel(f'marco a {libre:.2f} m · círculo {rr:.2f} m\n{msg}', fontsize=9,
                      color=VERDE if ok else ROJO)
        ax.set_aspect('equal')
        ax.set_xlim(-0.9, 0.9)
        ax.set_ylim(-1.05, 1.05)
        ax.set_title(titulo)
        ax.grid(alpha=0.25)
    fig.suptitle('Por qué Milo se trababa en las puertas (filtro de seguridad, modo teleop)',
                 y=1.02)
    fig.text(0.5, -0.1, f'En modo navegación (safety_nav.yaml) la holgura baja a '
             f'{sn["rotation_clearance"]:.2f} m y solo se revisa al girar EN EL LUGAR.',
             ha='center', fontsize=9, color='#444')
    guardar(fig, 'fig_huella_puertas.png')


def fig_seguridad():
    sp = params(SAFETY, 'safety_filter')
    stop, slow, side = sp['stop_distance'], sp['slow_distance'], sp['side_margin']
    x0, x1, hw = sp['footprint_x_min'], sp['footprint_x_max'], sp['footprint_half_width']
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4), gridspec_kw={'width_ratios': [1.25, 1]})
    ax = axs[0]
    w = hw + side
    ax.add_patch(mp.Rectangle((x1 + stop, -w), slow - stop, 2 * w, color='#f9d8a8',
                              label=f'frena ({stop:.2f}–{slow:.2f} m)'))
    ax.add_patch(mp.Rectangle((x1, -w), stop, 2 * w, color='#f2a69a',
                              label=f'para (< {stop:.2f} m)'))
    ax.add_patch(mp.Rectangle((x0 - slow, -w), slow, 2 * w, color='#eeeeee',
                              label='atrás (solo si retrocede)'))
    ax.add_patch(mp.Rectangle((x0, -hw), x1 - x0, 2 * hw, fc='#dbe6fb', ec=AZUL, lw=2,
                              label='Milo'))
    ax.plot([x0 - slow, x1 + slow + 0.1], [w, w], 'k--', lw=0.8)
    ax.plot([x0 - slow, x1 + slow + 0.1], [-w, -w], 'k--', lw=0.8)
    ax.text(x1 + slow + 0.12, w, f'corredor ±{w:.2f} m', va='center', fontsize=8.5)
    ax.plot(0, 0, 'k+', ms=10, mew=2)
    ax.set_aspect('equal')
    ax.set_xlim(x0 - slow - 0.05, x1 + slow + 0.75)
    ax.set_ylim(-0.7, 0.7)
    ax.set_xlabel('x [m] (adelante →)')
    ax.set_title('(a) Zonas del filtro de seguridad (vista desde arriba)')
    ax.legend(loc='upper left', fontsize=8, ncol=2, framealpha=0.9)
    ax = axs[1]
    d = np.linspace(0, 1.0, 400)
    s = np.clip((d - stop) / (slow - stop), 0, 1)
    ax.plot(d, s * sp['max_linear'], color=AZUL, lw=2.5)
    ax.axvspan(0, stop, color='#f2a69a', alpha=0.6)
    ax.axvspan(stop, slow, color='#f9d8a8', alpha=0.6)
    ax.set_xlabel('distancia libre adelante, desde el borde de Milo [m]')
    ax.set_ylabel('velocidad máxima permitida [m/s]')
    ax.set_title('(b) Cuánto deja avanzar según la distancia')
    ax.grid(alpha=0.3)
    ax.text(0.72, sp['max_linear'] * 0.5, 'lineal entre\nstop y slow', fontsize=9)
    guardar(fig, 'fig_seguridad.png')


def fig_inflacion():
    with open(NAV2) as fh:
        gc = yaml.safe_load(fh)['global_costmap']['global_costmap']['ros__parameters']
    infl = gc['inflation_layer']
    k, R = infl['cost_scaling_factor'], infl['inflation_radius']
    pad = gc.get('footprint_padding', 0.0)
    r_ins = 0.12 + pad  # distancia del eje de ruedas al borde más cercano (la cola)
    r_cir = math.hypot(0.44 + pad, 0.25 + pad)
    d = np.linspace(0, R + 0.1, 500)
    c = np.where(d <= 0.0, 254, np.where(d <= r_ins, 253, 252 * np.exp(-k * (d - r_ins))))
    c = np.where(d > R, 0, c)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(d, c, color=NARANJO, lw=2.5)
    ax.axvline(r_ins, color=GRIS, ls=':')
    ax.text(r_ins + 0.005, 200, f'radio inscrito\n{r_ins:.2f} m', fontsize=8.5)
    ax.axvline(R, color=GRIS, ls='--')
    ax.text(R + 0.005, 120, f'inflation_radius\n{R:.2f} m', fontsize=8.5)
    ax.axvline(0.45, color=AZUL, ls='-.', lw=1)
    ax.text(0.30, 30, 'centro de una puerta\nde 0.90 m (0.45 m)', color=AZUL, fontsize=8.5)
    ax.set_xlabel('distancia a un obstáculo [m]')
    ax.set_ylabel('costo de la celda (0 libre … 254 obstáculo)')
    ax.set_title(f'Inflación del costmap: costo = 252·e^(−{k:g}·(d − r_inscrito))   '
                 f'(radio circunscrito {r_cir:.2f} m)')
    ax.grid(alpha=0.3)
    guardar(fig, 'fig_inflacion.png')


def fig_cuaternion():
    locs, _ = lugares()
    yaw = np.linspace(-math.pi, math.pi, 400)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(yaw, np.sin(yaw / 2), color=AZUL, lw=2.3, label='z = sin(yaw/2)')
    ax.plot(yaw, np.cos(yaw / 2), color=NARANJO, lw=2.3, label='w = cos(yaw/2)')
    vistos = []
    for n, d in locs.items():
        y = math.atan2(math.sin(float(d['yaw'])), math.cos(float(d['yaw'])))
        alto = 1.06 + 0.17 * sum(abs(y - v) < 0.45 for v in vistos)  # no encimar nombres
        vistos.append(y)
        ax.plot([y, y], [math.sin(y / 2), math.cos(y / 2)], color=GRIS, lw=0.8)
        ax.plot(y, math.sin(y / 2), 'o', color=AZUL, ms=5)
        ax.plot(y, math.cos(y / 2), 'o', color=NARANJO, ms=5)
        ax.text(y, alto, n, rotation=0, fontsize=8, ha='center', va='bottom')
    ax.set_xticks([-math.pi, -math.pi / 2, 0, math.pi / 2, math.pi])
    ax.set_xticklabels(['−π\n(atrás)', '−π/2\n(derecha)', '0\n(+x)', 'π/2\n(izquierda)',
                        'π\n(atrás)'])
    ax.set_ylim(-1.1, 1.75)
    ax.set_xlabel('yaw [rad] (hacia dónde mira Milo en el mapa)')
    ax.set_title('De yaw a cuaternión (giro solo en el plano: x = y = 0)')
    ax.legend(loc='lower right')
    ax.grid(alpha=0.3)
    guardar(fig, 'fig_cuaternion.png')


def main():
    print('Figuras en docs/navegacion/img/:')
    g = cargar_generador()
    fig_casa_lugares(g)
    fig_huella_puertas()
    fig_seguridad()
    fig_inflacion()
    fig_cuaternion()


if __name__ == '__main__':
    main()
