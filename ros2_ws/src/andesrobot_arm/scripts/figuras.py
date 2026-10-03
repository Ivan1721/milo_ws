"""Figuras y medidas del brazo de Milo, sacadas del URDF + las mallas STL (las de docs/BRAZO.md).

Genera en la carpeta de salida:
  fig_3d.png          vista 3D del robot
  fig_lateral.png     vista lateral (X-Z) con cotas
  fig_frontal.png     vista frontal (Y-Z) con cotas
  fig_cadena.png      posición de cada articulación del brazo en cero
  fig_alcance.png     alcance del punto de agarre para 3 alturas del lift
  medidas.json        las medidas que aparecen en las figuras
Las cotas se CALCULAN de las mallas, así que si cambia el xacro basta con volver a correrlo.

Uso (con el workspace compilado y cargado; necesita matplotlib):
  source install/setup.bash
  python3 src/andesrobot_arm/scripts/figuras.py [carpeta_salida]
"""
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')  # dibujar a archivo, sin ventana
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import xacro  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from andesrobot_arm.kinematics import (  # noqa: E402
    ArmKinematics, LIFT_BASE, LIFT_JOINT, axis_angle_to_matrix, make_pose)

DESC = get_package_share_directory('andesrobot_description')
OUT = sys.argv[1] if len(sys.argv) > 1 else 'figuras'

# Colores: brazo azul, pinza naranja, columna/lift gris oscuro, base gris claro.
SURF, INK, MUTED = '#fcfcfb', '#2b2b29', '#6b6a63'
C_ARM, C_GRIP, C_BASE, C_COL = '#2a78d6', '#eb6834', '#a9a8a0', '#7d7c74'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'text.color': INK,
                     'axes.labelcolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED,
                     'axes.edgecolor': '#d6d5ce', 'figure.facecolor': SURF,
                     'axes.facecolor': SURF, 'savefig.facecolor': SURF})

URDF = xacro.process_file(os.path.join(DESC, 'urdf', 'andesrobot.urdf.xacro'),
                          mappings={'lock_arm': 'false'}).toxml()
ROOT = ET.fromstring(URDF)


def floats(text, default):
    return np.array([float(v) for v in text.split()]) if text else np.array(default, float)


def origin_of(el):
    o = el.find('origin')
    return make_pose(floats(o.get('xyz') if o is not None else None, [0, 0, 0]),
                     floats(o.get('rpy') if o is not None else None, [0, 0, 0]))


def load_stl(path):
    """Triángulos (N, 3, 3) de un STL binario o ASCII."""
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        v = [list(map(float, ln.split()[1:])) for ln in data.decode().splitlines()
             if ln.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    return np.frombuffer(data[84:84 + n * 50], dtype=rec)['v'].reshape(-1, 3, 3).astype(float)


def box_tris(sx, sy, sz):
    c = np.array([[x, y, z] for x in (-sx / 2, sx / 2) for y in (-sy / 2, sy / 2)
                  for z in (-sz / 2, sz / 2)])
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return np.array([[c[a], c[b], c[d]] for a, b, _, d in quads]
                    + [[c[b], c[cc], c[d]] for _, b, cc, d in quads])


def cylinder_tris(r, length, n=24):
    t = np.linspace(0, 2 * np.pi, n + 1)
    ring = np.c_[r * np.cos(t), r * np.sin(t)]
    lo, hi = -length / 2, length / 2
    tris = []
    for i in range(n):
        a, b = ring[i], ring[i + 1]
        tris += [[[*a, lo], [*b, lo], [*b, hi]], [[*a, lo], [*b, hi], [*a, hi]],
                 [[0, 0, lo], [*b, lo], [*a, lo]], [[0, 0, hi], [*a, hi], [*b, hi]]]
    return np.array(tris)


# Geometría visual de cada link: lista de (transformación visual, triángulos en metros).
VISUALS = {}
for link in ROOT.findall('link'):
    for vis in link.findall('visual'):
        geo = vis.find('geometry')[0]
        if geo.tag == 'mesh':
            name = os.path.basename(geo.get('filename'))
            scale = floats(geo.get('scale'), [1, 1, 1])
            tris = load_stl(os.path.join(DESC, 'meshes', name)) * scale
        elif geo.tag == 'box':
            tris = box_tris(*floats(geo.get('size'), [0, 0, 0]))
        elif geo.tag == 'cylinder':
            tris = cylinder_tris(float(geo.get('radius')), float(geo.get('length')))
        else:
            continue
        VISUALS.setdefault(link.get('name'), []).append((origin_of(vis), tris))

JOINTS = [dict(name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
               child=j.find('child').get('link'), origin=origin_of(j),
               axis=floats(j.find('axis').get('xyz') if j.find('axis') is not None else None,
                           [1, 0, 0]))
          for j in ROOT.findall('joint')]


def world(q):
    """Pose de cada link en base_footprint para los valores articulares q (dict nombre->valor)."""
    T = {'base_footprint': np.eye(4)}
    changed = True
    while changed:
        changed = False
        for j in JOINTS:
            if j['parent'] in T and j['child'] not in T:
                M = np.eye(4)
                if j['type'] in ('revolute', 'continuous'):
                    M[:3, :3] = axis_angle_to_matrix(j['axis'] / np.linalg.norm(j['axis']),
                                                     q.get(j['name'], 0.0))
                elif j['type'] == 'prismatic':
                    M[:3, 3] = j['axis'] * q.get(j['name'], 0.0)
                T[j['child']] = T[j['parent']] @ j['origin'] @ M
                changed = True
    return T


def color_of(link):
    if 'finger' in link:
        return C_GRIP
    if link.startswith('link_') or link == 'arm_base_link_1':
        return C_ARM
    if link in ('vertical_column_link_1', 'lift_carriage_link_1'):
        return C_COL
    return C_BASE


def placed(T, links=None):
    """Triángulos de cada link puestos en el mundo: p_w = T_link · T_visual · v."""
    out = []
    for link, items in VISUALS.items():
        if links and link not in links:
            continue
        for V, tris in items:
            M = T[link] @ V
            out.append((link, tris @ M[:3, :3].T + M[:3, 3]))
    return out


def shade(hex_color, normals, light):
    base = np.array(matplotlib.colors.to_rgb(hex_color))
    k = 0.45 + 0.55 * np.clip(np.abs(normals @ light), 0, 1)
    return np.clip(base[None, :] * k[:, None] + (1 - k[:, None]) * 0.12, 0, 1)


def normals_of(t):
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)


def ortho(T, plane):
    """Proyección ortográfica (pintor: lo de atrás primero) en 'xz' (lateral) o 'yz' (frontal)."""
    i, j, depth = {'xz': (0, 2, 1), 'yz': (1, 2, 0)}[plane]
    light = np.zeros(3)
    light[depth], light[2] = -1.0, 0.35
    light /= np.linalg.norm(light)
    polys, cols, keys = [], [], []
    for link, t in placed(T):
        polys.append(t[:, :, [i, j]])
        cols.append(shade(color_of(link), normals_of(t), light))
        keys.append(-t[:, :, depth].mean(1) if plane == 'xz' else t[:, :, depth].mean(1))
    order = np.argsort(np.concatenate(keys))
    return np.vstack(polys)[order], np.vstack(cols)[order]


def all_points(T, links=None):
    return np.vstack([t.reshape(-1, 3) for _, t in placed(T, links)])


def dim(ax, p0, p1, text, off, vertical=False):
    """Cota con flechas entre p0 y p1, desplazada 'off' (en metros)."""
    (x0, y0), (x1, y1) = p0, p1
    kw = dict(arrowstyle='<|-|>', color=INK, lw=0.9, mutation_scale=8, shrinkA=0, shrinkB=0)
    if vertical:
        ax.annotate('', (x0 + off, y0), (x0 + off, y1), arrowprops=kw)
        ax.plot([x0, x0 + off * 1.15], [y0, y0], color=MUTED, lw=0.5)
        ax.plot([x1, x0 + off * 1.15], [y1, y1], color=MUTED, lw=0.5)
        ax.text(x0 + off + (0.015 if off > 0 else -0.015), (y0 + y1) / 2, text, rotation=90,
                ha='left' if off > 0 else 'right', va='center', fontsize=9)
    else:
        ax.annotate('', (x0, y0 + off), (x1, y0 + off), arrowprops=kw)
        ax.plot([x0, x0], [y0, y0 + off * 1.15], color=MUTED, lw=0.5)
        ax.plot([x1, x1], [y1, y0 + off * 1.15], color=MUTED, lw=0.5)
        ax.text((x0 + x1) / 2, y0 + off + (0.02 if off > 0 else -0.02), text, ha='center',
                va='bottom' if off > 0 else 'top', fontsize=9)


def style(ax, title):
    ax.set_title(title, fontsize=11, loc='left')
    ax.grid(color='#ecebe5', lw=0.6)
    ax.set_axisbelow(True)


def fig_3d(medidas):
    T = world({'joint_2': 0.35, 'joint_3': 0.9, 'joint_5': 0.6})
    fig = plt.figure(figsize=(7, 7.6))
    ax = fig.add_subplot(111, projection='3d', proj_type='ortho')
    light = np.array([0.4, -0.6, 0.7]) / np.linalg.norm([0.4, -0.6, 0.7])
    polys, cols = [], []
    for link, t in placed(T):
        polys.append(t)
        cols.append(shade(color_of(link), normals_of(t), light))
    ax.add_collection3d(Poly3DCollection(np.vstack(polys), facecolors=np.vstack(cols),
                                         linewidths=0))
    tcp = T['gripper_tcp'][:3, 3]
    ax.scatter(*tcp, color=INK, s=18, depthshade=False)
    ax.text(tcp[0] + 0.05, tcp[1], tcp[2] + 0.04, 'gripper_tcp', fontsize=9)
    ax.set_xlim(-0.45, 0.75)
    ax.set_ylim(-0.6, 0.6)
    ax.set_zlim(0, 1.9)
    ax.set_box_aspect((1.2, 1.2, 1.9))
    ax.view_init(elev=18, azim=-58)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.pane.set_facecolor(SURF)
        a.pane.set_edgecolor('#e4e3dc')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig_3d.png'), dpi=130)
    plt.close(fig)


def fig_vistas(medidas):
    T = world({})
    P = all_points(T)
    lo, hi = P.min(0), P.max(0)
    base_links = [k for k in VISUALS if color_of(k) == C_BASE]
    base_top = all_points(T, base_links)[:, 2].max()
    z_ab = T['arm_base_link_1'][2, 3]
    wheels = [all_points(T, [w]) for w in ('left_wheel_link_1', 'right_wheel_link_1')]
    y_wheels = [(w[:, 1].min() + w[:, 1].max()) / 2 for w in wheels]
    wheel_sep = abs(y_wheels[0] - y_wheels[1])
    medidas.update(largo=hi[0] - lo[0], x_min=lo[0], x_max=hi[0], ancho=hi[1] - lo[1],
                   altura=hi[2], altura_base=base_top, z_arm_base=z_ab,
                   separacion_ruedas=wheel_sep)

    for plane, fn, title in (('xz', 'fig_lateral.png', 'Vista lateral (plano X-Z)'),
                             ('yz', 'fig_frontal.png', 'Vista frontal (plano Y-Z)')):
        polys, cols = ortho(T, plane)
        fig, ax = plt.subplots(figsize=(6.2, 8.2))
        ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors='none'))
        ax.axhline(0, color=MUTED, lw=0.8)
        ax.set_aspect('equal')
        if plane == 'xz':
            ax.set_xlim(lo[0] - 0.3, hi[0] + 0.25)
            dim(ax, (lo[0], 0), (hi[0], 0), f'{hi[0] - lo[0]:.3f} m', -0.10)
            dim(ax, (lo[0], 0), (lo[0], hi[2]), f'{hi[2]:.3f} m (brazo vertical, lift en 0)',
                -0.12, vertical=True)
            dim(ax, (hi[0], 0), (hi[0], base_top), f'{base_top:.3f} m', 0.06, vertical=True)
            x_ab = T['arm_base_link_1'][0, 3]
            ax.plot([x_ab, hi[0] + 0.02], [z_ab, z_ab], color=MUTED, lw=0.6, ls='--')
            ax.text(hi[0] + 0.02, z_ab - 0.04, f'arm_base_link_1\nz = {z_ab:.3f} m', fontsize=8,
                    color=MUTED, ha='right', va='top')
            ax.set_xlabel('X (m), adelante →')
        else:
            ax.set_xlim(-0.55, 0.55)
            dim(ax, (lo[1], 0), (hi[1], 0), f'{hi[1] - lo[1]:.3f} m', -0.10)
            dim(ax, (min(y_wheels), 0.083), (max(y_wheels), 0.083),
                f'ruedas: {wheel_sep:.4f} m entre centros', 0.42)
            ax.set_xlabel('Y (m)  (+Y = izquierda del robot)')
        ax.set_ylim(-0.25, hi[2] + 0.13)
        ax.set_ylabel('Z (m)')
        style(ax, title)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, fn), dpi=130)
        plt.close(fig)


def fig_cadena(medidas):
    T = world({})
    names = ['arm_base_link_1', 'link_1_1', 'link_2_1', 'link_3_1', 'link_4_1', 'link_5_1',
             'link_6_1', 'gripper_tcp']
    labels = ['arm_base', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'TCP']
    pts = np.array([T[n][:3, 3] for n in names])
    # Eje de cada articulación en el mundo (q = 0), para la etiqueta.
    by_child = {j['child']: j for j in JOINTS}
    axes = {}
    for n, lab in zip(names[1:7], labels[1:7]):
        j = by_child[n]
        a = (T[j['parent']] @ j['origin'])[:3, :3] @ j['axis']
        k = int(np.argmax(np.abs(a)))
        axes[lab] = ('+' if a[k] > 0 else '−') + 'XYZ'[k]
    offs = {'arm_base': (0.03, -0.03, 'left'), 'joint_1': (-0.03, 0.0, 'right'),
            'joint_2': (0.03, -0.025, 'left'), 'joint_3': (0.03, 0.0, 'left'),
            'joint_4': (-0.03, 0.0, 'right'), 'joint_5': (0.03, -0.035, 'left'),
            'joint_6': (-0.03, 0.03, 'right'), 'TCP': (0.0, 0.035, 'center')}
    polys, cols = ortho(T, 'xz')
    fig, ax = plt.subplots(figsize=(6.4, 6.6))
    ax.add_collection(PolyCollection(polys, facecolors=np.clip(cols * 0.25 + 0.75, 0, 1),
                                     edgecolors='none'))
    ax.plot(pts[:, 0], pts[:, 2], '-', color=INK, lw=1.6, zorder=5)
    for p, lab in zip(pts, labels):
        is_joint = lab.startswith('joint')
        ax.scatter(p[0], p[2], s=46 if is_joint else 30, zorder=6, edgecolors=SURF,
                   color=C_ARM if is_joint else (C_GRIP if lab == 'TCP' else INK),
                   linewidths=1.5)
        dx, dz, ha = offs[lab]
        txt = lab + (f'  (eje {axes[lab]})' if is_joint else '')
        ax.text(p[0] + dx, p[2] + dz, txt, fontsize=8.5, zorder=7, ha=ha, va='center')
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    for k in (2, 3):
        mid = (pts[k] + pts[k + 1]) / 2
        ax.text(mid[0] - 0.035, mid[2], f'{seg[k]:.3f} m', ha='right', va='center', fontsize=9,
                color=C_ARM, fontweight='bold')
    cx = pts[:, 0].mean()
    ax.set_xlim(cx - 0.45, cx + 0.45)
    ax.set_ylim(pts[:, 2].min() - 0.07, pts[:, 2].max() + 0.08)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    style(ax, 'Cadena del brazo en posición cero (lift = 0)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig_cadena.png'), dpi=130)
    plt.close(fig)
    medidas['cadena'] = {lab: np.round(p, 4).tolist() for lab, p in zip(labels, pts)}
    medidas['tramos'] = np.round(seg, 4).tolist()
    medidas['ejes_mundo'] = axes


def fig_alcance(medidas):
    """Monte Carlo: configuraciones al azar dentro de los límites -> posición del TCP."""
    chain = ArmKinematics(URDF, base=LIFT_BASE)
    li = chain.joint_names.index(LIFT_JOINT)
    rng = np.random.default_rng(0)
    fig, axs = plt.subplots(1, 3, figsize=(11, 5.6), sharey=True)
    medidas['alcance'] = {}
    for ax, color, lift in zip(axs, SERIES, (-0.4, 0.0, 0.6)):
        Q = np.array([chain._random_q(rng) for _ in range(12000)])
        Q[:, li] = lift
        P = np.array([chain.fk(q)[:3, 3] for q in Q])
        P = P[P[:, 2] >= 0.0]
        polys, cols = ortho(world({LIFT_JOINT: lift}), 'xz')
        ax.scatter(P[:, 0], P[:, 2], s=1.6, color=color, alpha=0.35, linewidths=0,
                   rasterized=True, zorder=2)
        ax.add_collection(PolyCollection(polys, facecolors=np.clip(cols * 0.55 + 0.3, 0, 1),
                                         edgecolors='none', zorder=3))
        stats = dict(z_min=float(P[:, 2].min()), z_max=float(P[:, 2].max()),
                     r_max=float(np.hypot(P[:, 0], P[:, 1]).max()))
        medidas['alcance'][f'{lift:+.1f}'] = stats
        ax.axhline(0, color=INK, lw=0.9, zorder=4)
        ax.set_aspect('equal')
        ax.set_xlim(-0.95, 1.1)
        ax.set_ylim(-0.08, 2.5)
        ax.text(-0.9, 2.4, f"z: {stats['z_min']:.2f} a {stats['z_max']:.2f} m", fontsize=8.5,
                color=MUTED, va='top')
        ax.set_xlabel('X (m)')
        style(ax, f'lift = {lift:+.1f} m')
    axs[0].set_ylabel('Z (m)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig_alcance.png'), dpi=120)
    plt.close(fig)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    medidas = {}
    fig_3d(medidas)
    fig_vistas(medidas)
    fig_cadena(medidas)
    fig_alcance(medidas)
    json.dump(medidas, open(os.path.join(OUT, 'medidas.json'), 'w'), indent=1,
              default=float)
    print(json.dumps(medidas, indent=1, default=float))
