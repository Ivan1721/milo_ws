"""Exporta Milo y su escenario a un paquete para la web (three.js / Vite), sin ROS.

Genera en la carpeta de salida (por defecto /ros2_ws/escena_web, que en el PC es
~/milo_ws/ros2_ws/escena_web, fuera de git):
  milo.urdf      el robot como URDF normal (sin xacro, sin plugins de Gazebo ni ros2_control),
                 con el brazo móvil, la cámara de la pinza y las mallas con rutas relativas
                 (meshes/<archivo>.stl). Lo carga directo la librería urdf-loader de three.js.
  meshes/        las mallas STL que usa milo.urdf (en milímetros: el URDF las escala x0.001).
  robot.json     lo mismo que el URDF pero fácil de leer desde JavaScript: links con sus
                 visuales y colores, articulaciones con eje y límites, la cadena de la
                 cinemática inversa (base_footprint -> gripper_tcp) con sus parámetros, y los
                 sensores de la cámara (campo de visión, resolución, rango).
  escena.json    el escenario: la arena (muros, muebles, escombros, pilares), la mesa de prueba
                 con sus objetos y dónde aparece Milo. Cada objeto es una caja o un cilindro con
                 posición, orientación, tamaño y color.
Todo en las convenciones de ROS: metros, radianes, Z hacia arriba (ver docs/escena_web/README.md
para pasarlo a three.js, que usa Y hacia arriba).

Uso (dentro del contenedor, con el workspace compilado):
  ./sim.sh shell
  python3 /ros2_ws/src/andesrobot_arm/scripts/exportar_escena_web.py [carpeta_salida]
Si cambia el xacro o los mundos, se vuelve a correr y el paquete queda al día.
"""
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory

from andesrobot_arm.kinematics import ArmKinematics, LIFT_BASE

OUT = sys.argv[1] if len(sys.argv) > 1 else '/ros2_ws/escena_web'
DESC = get_package_share_directory('andesrobot_description')
GAZEBO = get_package_share_directory('andesrobot_gazebo')
ARM = get_package_share_directory('andesrobot_arm')

# Colores sugeridos para la web (los mismos de las figuras del informe). El URDF solo trae gris
# (silver) y negro (caster_black), que en un visor 3D dejan todo el robot del mismo color.
COLOR_ARM, COLOR_GRIP, COLOR_COL, COLOR_BASE = '#2a78d6', '#eb6834', '#7d7c74', '#a9a8a0'
# Parámetros de la cinemática inversa (andesrobot_arm/kinematics.py y arm_ik_node.py).
IK = {'metodo': 'mínimos cuadrados amortiguados (DLS) ponderados, con reinicios aleatorios',
      'amortiguamiento_lambda': 0.01, 'paso_maximo': 0.5, 'iteraciones_por_intento': 200,
      'reinicios': 20, 'tolerancia_posicion_m': 1e-4, 'tolerancia_rotacion_rad': 1e-3,
      'peso_lift': 10.0, 'peso_otras': 1.0,
      'velocidad_max_brazo_rad_s': 0.5, 'velocidad_max_lift_m_s': 0.1, 'duracion_minima_s': 1.0}


def floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def origin_dict(el):
    """<origin xyz rpy> -> {'xyz': [...], 'rpy': [...]} (ceros si no está)."""
    o = el.find('origin') if el is not None else None
    return {'xyz': floats(o.get('xyz') if o is not None else None, [0, 0, 0]),
            'rpy': floats(o.get('rpy') if o is not None else None, [0, 0, 0])}


def color_sugerido(link):
    if 'finger' in link or link.startswith('gripper_camera'):
        return COLOR_GRIP
    if link.startswith('link_') or link == 'arm_base_link_1':
        return COLOR_ARM
    if link in ('vertical_column_link_1', 'lift_carriage_link_1'):
        return COLOR_COL
    return COLOR_BASE


def procesar_urdf():
    """xacro -> URDF para la web: brazo móvil, con cámara, sin <gazebo> ni <ros2_control>."""
    doc = xacro.process_file(os.path.join(DESC, 'urdf', 'andesrobot.urdf.xacro'),
                             mappings={'lock_arm': 'false', 'gripper_camera': 'true'})
    root = ET.fromstring(doc.toxml())
    # Los sensores de la cámara viven en <gazebo>: se leen antes de borrar esos bloques.
    sensores = []
    for gz in root.findall('gazebo'):
        for s in gz.findall('sensor'):
            cam = s.find('camera')
            if cam is None:
                continue
            img = cam.find('image')
            clip = cam.find('clip')
            plugin = s.find('plugin')
            sensores.append({
                'nombre': s.get('name'), 'tipo': s.get('type'), 'link': gz.get('reference'),
                'frame_optico': plugin.findtext('frame_name') if plugin is not None else None,
                'fov_horizontal_rad': float(cam.findtext('horizontal_fov')),
                'ancho_px': int(img.findtext('width')), 'alto_px': int(img.findtext('height')),
                'formato': img.findtext('format'),
                'cerca_m': float(clip.findtext('near')), 'lejos_m': float(clip.findtext('far')),
                'profundidad_min_m': float(plugin.findtext('min_depth'))
                if plugin is not None and plugin.findtext('min_depth') else None,
                'profundidad_max_m': float(plugin.findtext('max_depth'))
                if plugin is not None and plugin.findtext('max_depth') else None,
                'hz': float(s.findtext('update_rate'))})
    for tag in ('gazebo', 'ros2_control', 'transmission'):
        for el in root.findall(tag):
            root.remove(el)
    # Mallas: file:///…/meshes/x.stl -> meshes/x.stl (relativa a milo.urdf) y se copian.
    os.makedirs(os.path.join(OUT, 'meshes'), exist_ok=True)
    for mesh in root.iter('mesh'):
        nombre = os.path.basename(mesh.get('filename'))
        shutil.copy(os.path.join(DESC, 'meshes', nombre), os.path.join(OUT, 'meshes', nombre))
        mesh.set('filename', 'meshes/' + nombre)
    ET.indent(root, space='  ')
    with open(os.path.join(OUT, 'milo.urdf'), 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0"?>\n<!-- Milo para la web: generado por '
                'andesrobot_arm/scripts/exportar_escena_web.py desde andesrobot.urdf.xacro. '
                'No editar a mano. -->\n' + ET.tostring(root, encoding='unicode') + '\n')
    return root, sensores


def robot_json(root, sensores):
    materiales = {m.get('name'): floats(m.find('color').get('rgba'), [1, 1, 1, 1])
                  for m in root.findall('material') if m.find('color') is not None}
    links = []
    for link in root.findall('link'):
        visuales = []
        for v in link.findall('visual'):
            g = v.find('geometry')[0]
            vis = {'origen': origin_dict(v), 'geometria': g.tag}
            if g.tag == 'mesh':
                vis.update(archivo=g.get('filename'), escala=floats(g.get('scale'), [1, 1, 1]))
            elif g.tag == 'box':
                vis['tamano'] = floats(g.get('size'), [0, 0, 0])
            elif g.tag == 'cylinder':
                vis.update(radio=float(g.get('radius')), largo=float(g.get('length')))
            mat = v.find('material')
            if mat is not None:
                vis['color_urdf'] = materiales.get(mat.get('name'))
            visuales.append(vis)
        m = link.find('inertial/mass')
        links.append({'nombre': link.get('name'), 'visuales': visuales,
                      'masa_kg': float(m.get('value')) if m is not None else None,
                      'color_sugerido': color_sugerido(link.get('name')) if visuales else None})
    joints = []
    for j in root.findall('joint'):
        lim, mim = j.find('limit'), j.find('mimic')
        joints.append({
            'nombre': j.get('name'), 'tipo': j.get('type'),
            'padre': j.find('parent').get('link'), 'hijo': j.find('child').get('link'),
            'origen': origin_dict(j),
            'eje': floats(j.find('axis').get('xyz') if j.find('axis') is not None else None,
                          [1, 0, 0]),
            'limite': None if lim is None else {
                'inferior': float(lim.get('lower', 0)), 'superior': float(lim.get('upper', 0)),
                'velocidad': float(lim.get('velocity', 0))},
            'copia_a': None if mim is None else {
                'articulacion': mim.get('joint'),
                'multiplicador': float(mim.get('multiplier', 1))}})
    # Cadena de la IK de 7 ejes, en el orden en que se recorre (lift + joint_1..6 + TCP fijo).
    arm = ArmKinematics(ET.tostring(root, encoding='unicode'), base=LIFT_BASE)
    por_nombre = {j['nombre']: j for j in joints}
    cadena = [{'nombre': j.name, 'tipo': j.type, 'origen': por_nombre[j.name]['origen'],
               'eje': [round(a, 6) for a in j.axis],
               'limite': [j.lower, j.upper] if j.lower is not None else None}
              for j in arm.chain]
    moviles = [j['nombre'] for j in joints if j['tipo'] in ('revolute', 'prismatic', 'continuous')
               and not j['copia_a'] and 'wheel' not in j['nombre']]
    return {
        'convenciones': {'unidades': 'metros y radianes', 'ejes': 'REP-103: +X adelante, '
                         '+Y izquierda, +Z arriba', 'rpy': 'R = Rz(yaw) Ry(pitch) Rx(roll)',
                         'mallas': 'STL en milímetros (escala 0.001 en el URDF)',
                         'raiz': 'base_footprint (suelo, bajo el eje de las ruedas)'},
        'urdf': 'milo.urdf', 'links': links, 'joints': joints,
        'articulaciones_moviles': moviles,
        'pinza': {'articulacion': 'right_finger_joint', 'cerrada_m': -0.007, 'abierta_m': 0.016,
                  'nota': 'left_finger_joint copia a right_finger_joint con multiplicador -1'},
        'tcp': 'gripper_tcp',
        'cinematica_inversa': {'base': LIFT_BASE, 'punta': 'gripper_tcp', 'cadena': cadena,
                               'parametros': IK},
        'camara': {'modelo': 'Orbbec Gemini Plus', 'sensores': sensores},
    }


def pose(text):
    v = floats(text, [0, 0, 0, 0, 0, 0])
    return {'xyz': v[:3], 'rpy': v[3:]}


def geometria(g):
    if g.tag == 'box':
        return {'tipo': 'box', 'tamano': floats(g.findtext('size'), [0, 0, 0])}
    if g.tag == 'cylinder':
        return {'tipo': 'cylinder', 'radio': float(g.findtext('radius')),
                'largo': float(g.findtext('length'))}
    return {'tipo': g.tag}


def objetos_sdf(model, categoria):
    """Cada <visual> de un <model> SDF como objeto: pose final = pose del modelo + del link + del
    visual (aquí solo se suman, porque los modelos de estos mundos no giran en roll/pitch)."""
    out = []
    mp = pose(model.findtext('pose'))
    for link in model.findall('link'):
        lp = pose(link.findtext('pose'))
        for v in link.findall('visual'):
            vp = pose(v.findtext('pose'))
            mat = v.find('material')
            yaw = mp['rpy'][2]
            c, s = math.cos(yaw), math.sin(yaw)
            # Posición del link + visual, girada por el yaw del modelo, más la posición del modelo.
            px = lp['xyz'][0] + vp['xyz'][0]
            py = lp['xyz'][1] + vp['xyz'][1]
            xyz = [round(mp['xyz'][0] + c * px - s * py, 4),
                   round(mp['xyz'][1] + s * px + c * py, 4),
                   round(mp['xyz'][2] + lp['xyz'][2] + vp['xyz'][2], 4)]
            rpy = [0.0, 0.0, round(yaw + lp['rpy'][2] + vp['rpy'][2], 4)]
            # Nombre: modelo[/link][/visual] (el visual solo si el link tiene varios).
            nombre = model.get('name')
            if link.get('name') != 'link':
                nombre += '/' + link.get('name')
            if len(link.findall('visual')) > 1:
                nombre += '/' + v.get('name')
            out.append({
                'nombre': nombre,
                'categoria': categoria,
                'estatico': (model.findtext('static') or 'false').strip() == 'true',
                'pose': {'xyz': xyz, 'rpy': rpy},
                'geometria': geometria(v.find('geometry')[0]),
                'color': floats(mat.findtext('diffuse'), [0.7, 0.7, 0.7, 1]) if mat is not None
                else [0.7, 0.7, 0.7, 1]})
    return out


def categoria_de(nombre):
    for pref, cat in (('muro', 'muro'), ('pilar', 'pilar'), ('escombro', 'escombro'),
                      ('escritorio', 'mueble'), ('estante', 'mueble'), ('barrera', 'barrera')):
        if nombre.startswith(pref):
            return cat
    return 'objeto'


def escena_json():
    world_file = os.path.join(GAZEBO, 'worlds', 'andesrobot_arena.world')
    world = ET.parse(world_file).getroot().find('world')
    objetos = []
    for m in world.findall('model'):
        objetos += objetos_sdf(m, categoria_de(m.get('name')))
    mesa = ET.parse(os.path.join(ARM, 'worlds', 'mesa_prueba.sdf')).getroot().find('model')
    mesa_obj = objetos_sdf(mesa, 'mesa_prueba')
    return {
        'convenciones': 'metros y radianes, +Z arriba; pose = centro de la pieza',
        'suelo': {'tamano_m': [100, 100], 'color': [0.8, 0.8, 0.8, 1]},
        'luz': {'tipo': 'sol', 'direccion': [-0.5, 0.1, -0.9], 'nota': 'model://sun de Gazebo'},
        'arena': {'tamano_m': [24.0, 18.0], 'centro': [0, 0], 'objetos': objetos,
                  'plano': 'andesrobot_gazebo/worlds/andesrobot_arena.png'},
        'mesa_prueba': {'objetos': mesa_obj,
                        'nota': 'la pone arm_sim.launch.py (mesa:=true); la mesa no se mueve, '
                                'los objetos de encima sí'},
        'spawn_robot': {'xyz': [0, 0, 0.02], 'yaw': 0.0,
                        'nota': 'Milo aparece en (0, 0) mirando a +X'},
    }


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    root, sensores = procesar_urdf()
    with open(os.path.join(OUT, 'robot.json'), 'w', encoding='utf-8') as f:
        json.dump(robot_json(root, sensores), f, ensure_ascii=False, indent=1)
    esc = escena_json()
    with open(os.path.join(OUT, 'escena.json'), 'w', encoding='utf-8') as f:
        json.dump(esc, f, ensure_ascii=False, indent=1)
    n_mallas = len(os.listdir(os.path.join(OUT, 'meshes')))
    print('Paquete web en %s: milo.urdf, %d mallas, robot.json, escena.json (%d objetos en la '
          'arena, %d en la mesa)' % (OUT, n_mallas, len(esc['arena']['objetos']),
                                     len(esc['mesa_prueba']['objetos'])))
