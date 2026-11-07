"""Catálogo de objetos 3D para la mesa (config/objetos.yaml), sin ROS: se prueba solo.

Lo usan tres programas:
  scripts/descargar_objetos.py   baja cada objeto a ros2_ws/modelos/gazebo/<carpeta>/, arregla su
                                 material, le pone masa e inercia y anota sus medidas (milo.json)
  launch/arm_sim.launch.py       objetos:=taza,platano  ->  qué objetos y dónde ponerlos en la mesa
  scripts/evaluar_objetos.py     pone cada objeto solo en la mesa y prueba el detector

Medidas: la mesa de mesa_objetos.sdf tiene el tablero a ALTO_MESA m, de x = 0.70 a 1.30 m y de
y = −0.40 a 0.40 m (base_footprint). El brazo llega hasta x ~ 0.89 m a esa altura, así que la
primera fila de objetos va en x = 0.80.
"""
import re

import numpy as np

ALTO_MESA = 0.72
FILAS_X = (0.80, 0.95)        # la segunda fila queda fuera del alcance del brazo, pero se ve
ANCHO_FILA = 0.56             # de y = −0.28 a 0.28: deja 12 cm al borde de la mesa
POR_FILA = 5
# objetos:=true pone los dos del escenario original (sim_cup_bowl).
ORIGINALES = ('taza_blanca', 'bowl_blanco')


def leer_catalogo(texto_yaml):
    """Lista de objetos (diccionarios) del YAML del catálogo."""
    import yaml
    return yaml.safe_load(texto_yaml)['objetos']


def carpeta(objeto):
    """Nombre de la carpeta (y del modelo de Gazebo) de un objeto: sin espacios, porque Gazebo
    busca "model://<carpeta>" y un espacio rompe la ruta."""
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', objeto['nombre']).strip('_')


def elegir(catalogo, pedido):
    """
    Objetos pedidos en el argumento objetos:= del launch.

    pedido  'false' o '' = ninguno; 'true' = los dos originales; si no, una lista separada por
            comas de ids (taza, platano) o de clases (cup = el primero de esa clase del catálogo)
    Devuelve la lista de objetos del catálogo, sin repetir. Un nombre que no existe da ValueError
    con los ids que sí hay.
    """
    pedido = (pedido or '').strip()
    if pedido in ('', 'false'):
        return []
    if pedido == 'true':
        nombres = ORIGINALES
    else:
        nombres = [p.strip() for p in pedido.split(',') if p.strip()]
    elegidos = []
    for nombre in nombres:
        objeto = next((o for o in catalogo if o['id'] == nombre), None)
        if objeto is None:
            objeto = next((o for o in catalogo if o['clase'] == nombre), None)
        if objeto is None:
            ids = ', '.join(o['id'] for o in catalogo)
            raise ValueError(f'No hay un objeto "{nombre}" en config/objetos.yaml (hay: {ids})')
        if objeto not in elegidos:
            elegidos.append(objeto)
    return elegidos


def posiciones_en_mesa(n):
    """(x, y) de n objetos sobre la mesa: filas de hasta POR_FILA, repartidos a lo ancho."""
    if n > POR_FILA * len(FILAS_X):
        raise ValueError(f'En la mesa caben hasta {POR_FILA * len(FILAS_X)} objetos (pediste {n})')
    puntos = []
    for fila, x in enumerate(FILAS_X):
        k = min(POR_FILA, n - fila * POR_FILA)
        if k <= 0:
            break
        ys = [0.0] if k == 1 else np.linspace(-ANCHO_FILA / 2, ANCHO_FILA / 2, k)
        puntos += [(x, float(y)) for y in ys]
    return puntos


def vertices_obj(texto):
    """Vértices (N x 3) de una malla .obj (las líneas "v x y z")."""
    filas = [linea.split()[1:4] for linea in texto.splitlines() if linea.startswith('v ')]
    return np.array(filas, float).reshape(-1, 3)


def medidas(vertices):
    """Caja que encierra la malla: {'min': [x, y, z], 'max': [...], 'tam': [...]} en metros."""
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    return {'min': lo.round(4).tolist(), 'max': hi.round(4).tolist(),
            'tam': (hi - lo).round(4).tolist()}


def inercia_caja(masa, tam):
    """Momentos de inercia (ixx, iyy, izz) de una caja maciza de lados tam = (a, b, c)."""
    a, b, c = (max(float(v), 0.01) for v in tam)
    return (masa * (b * b + c * c) / 12.0, masa * (a * a + c * c) / 12.0,
            masa * (a * a + b * b) / 12.0)


def xml_inercial(masa, tam, centro):
    """Bloque <inertial> de SDF con la masa en el centro de la caja de la malla."""
    ixx, iyy, izz = inercia_caja(masa, tam)
    x, y, z = centro
    return (f'<inertial><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose><mass>{masa}</mass>'
            f'<inertia><ixx>{ixx:.3e}</ixx><iyy>{iyy:.3e}</iyy><izz>{izz:.3e}</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def corregir_uris(sdf, nombre_carpeta):
    """Todas las mallas del model.sdf como model://<carpeta>/...: las relativas ("meshes/a.dae")
    y las que apuntan a otro nombre de carpeta ("model://coke_can/meshes/a.dae")."""
    def cambiar(m):
        ruta = m.group(1).strip()
        if ruta.startswith(('http://', 'https://')):
            return m.group(0)
        if ruta.startswith('model://'):
            # model://<otra carpeta>/meshes/a.dae -> se queda con "meshes/a.dae"
            partes = ruta[len('model://'):].split('/', 1)
            ruta = partes[1] if len(partes) == 2 else ''
        return f'<uri>model://{nombre_carpeta}/{ruta}</uri>'
    return re.sub(r'<uri>\s*(.*?)\s*</uri>', cambiar, sdf)


def altura_de_spawn(medida):
    """z a la que poner el origen del modelo para que su base quede 3 mm sobre el tablero (los
    modelos de Google tienen el origen en la base; los de YCB, en cualquier parte)."""
    zmin = medida['min'][2] if medida else 0.0
    return round(ALTO_MESA - zmin + 0.003, 4)
