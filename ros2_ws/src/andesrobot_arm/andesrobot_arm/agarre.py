"""Geometría para agarrar desde arriba un objeto de la mesa (sin ROS: se prueba sola).

Es lo que usa agarre_node (fases 4 y 5 del plan de agarre, P15 en docs/PENDIENTES.md). Parte de
los puntos 3D que la cámara de profundidad ve dentro de la caja de YOLO (como objetos.py, pero
con muchos rayos repartidos por toda la caja, no solo el centro):

  1. Separar la mesa del objeto (`analizar_objeto`): dentro de la caja hay puntos de los dos.
     La altura de la mesa es la altura donde más puntos se juntan (el tablero es la superficie
     dominante alrededor de un objeto chico); los puntos que quedan más de `margen` (4 mm) por
     encima son del objeto.
  2. Medir el objeto con esos puntos: el EJE LARGO con PCA (análisis de componentes
     principales: la dirección en que los puntos están más repartidos), el largo y el ancho
     (cuánto ocupa a lo largo del eje y cruzado a él, `extension`), el centro (punto medio de
     esa extensión) y el alto sobre la mesa.
  3. Elegir el agarre (`pose_desde_arriba`, `altura_de_agarre`): la pinza baja vertical, con los
     dedos cerrando CRUZADOS al eje largo (así aprietan el lado angosto, el que cabe), y el TCP a
     una altura en que las puntas de los dedos quedan justo sobre la mesa.
  4. Cuánto cerrar (`objetivo_dedos`): cada dedo `apriete` (2 mm) más adentro que el ancho, como
     en scripts/prueba_levantar.py: así cada dedo empuja ~20 N y el objeto queda centrado.

Convenciones: metros, todo en base_footprint (+X adelante, +Y izquierda, +Z arriba). La pinza
apunta con su eje −X local y los dedos se abren a lo largo de su eje Y local (ver manija.py).
"""
import numpy as np

# Pinza EBG-20 en el URDF: apertura = 0.017 + q_derecho − q_izquierdo, con los dedos en espejo
# (q_izquierdo = −q_derecho). Abre de 3 a 49 mm.
CARAS_EN_CERO = 0.017                # distancia entre las caras de los dedos con q = 0 [m]
ABIERTA, CERRADA = 0.016, -0.007     # límites de right_finger_joint [m]
# La punta de los dedos sobresale 20.4 mm más allá del TCP, a lo largo del eje de aproximación
# (−X local), medido en las mallas de colisión del URDF el 7-nov. Con la pinza hacia abajo es lo
# que baja por debajo del TCP.
PUNTA_DEDO = 0.0204


def altura_de_la_mesa(z, paso=0.003):
    """La altura [m] donde más puntos se juntan: el centro del intervalo de `paso` con más
    puntos, afinado con la mediana de los puntos de ese intervalo y sus vecinos.
    No sirve un percentil bajo: con el objeto en el borde, la caja de YOLO incluye el canto de la
    mesa (vertical, más abajo) y el percentil cae ahí (medido el 7-nov: mesa a 0.705 m en vez
    de 0.72, y el tablero contado como objeto: 64 mm de ancho en vez de 27)."""
    z = np.asarray(z, float)
    bordes = np.arange(z.min(), z.max() + 2 * paso, paso)
    cuenta, bordes = np.histogram(z, bordes)
    i = int(np.argmax(cuenta))
    cerca = z[(z >= bordes[max(i - 1, 0)]) & (z < bordes[min(i + 2, len(bordes) - 1)])]
    return float(np.median(cerca))


def extension(valores, paso=0.003, minimo=2):
    """(desde, hasta) [m] que ocupan unos valores: se reparten en intervalos de `paso` y se
    toman el primero y el último con al menos `minimo` valores (un punto suelto no cuenta), por
    sus centros. No depende de cuántos puntos hay en cada parte, como sí un percentil: la parte
    del objeto más cercana a la cámara tiene más puntos."""
    v = np.asarray(valores, float)
    bordes = np.arange(v.min(), v.max() + 2 * paso, paso)
    cuenta, bordes = np.histogram(v, bordes)
    ocupados = np.flatnonzero(cuenta >= minimo)
    if len(ocupados) == 0:
        return float(v.min()), float(v.max())
    return (float(bordes[ocupados[0]] + paso / 2), float(bordes[ocupados[-1]] + paso / 2))


def analizar_objeto(puntos, margen=0.004, minimo=20):
    """
    Separa la mesa del objeto y lo mide.

    puntos  N x 3 (o lista con None, que se descartan): lo que tocaron los rayos de la caja
    margen  cuánto por encima de la mesa empieza el objeto [m]
    Devuelve un dict con mesa_z, centro (x, y, z), eje (unitario, horizontal, el eje largo),
    largo, ancho, alto y n (puntos del objeto), o None si hay menos de `minimo` puntos de objeto.
    """
    p = np.array([q for q in puntos if q is not None], float).reshape(-1, 3)
    if len(p) < minimo:
        return None
    mesa_z = altura_de_la_mesa(p[:, 2])
    obj = p[(p[:, 2] > mesa_z + margen) & (p[:, 2] < mesa_z + 0.5)]
    if len(obj) < minimo:
        return None
    xy = obj[:, :2]
    centro_xy = np.median(xy, axis=0)   # provisorio, para el PCA
    # PCA en el plano de la mesa: el vector propio de la covarianza con el valor más grande es la
    # dirección en que el objeto es más largo.
    cov = np.cov((xy - centro_xy).T)
    valores, vectores = np.linalg.eigh(cov)
    eje = vectores[:, int(np.argmax(valores))]
    # Que el eje apunte siempre "hacia adelante" (+X): el objeto girado 180° es el mismo.
    if eje[0] < 0 or (abs(eje[0]) < 1e-9 and eje[1] < 0):
        eje = -eje
    cruzado = np.array([-eje[1], eje[0]])
    a_lo_largo = (xy - centro_xy) @ eje
    de_lado = (xy - centro_xy) @ cruzado
    l_min, l_max = extension(a_lo_largo)
    d_min, d_max = extension(de_lado)
    largo, ancho = float(l_max - l_min), float(d_max - d_min)
    # El centro es el punto medio de esa extensión, no la mediana de los puntos: la parte más
    # cercana a la cámara tiene más puntos y corre la mediana hacia ella (medido el 7-nov: 2.3 cm
    # hacia Milo en un tenedor; agarrado ahí, quedó colgando inclinado 29°).
    centro_xy = centro_xy + (l_min + l_max) / 2 * eje + (d_min + d_max) / 2 * cruzado
    alto = float(np.percentile(obj[:, 2], 95) - mesa_z)
    centro = np.array([centro_xy[0], centro_xy[1], mesa_z + alto / 2])
    return {'mesa_z': mesa_z, 'centro': centro, 'eje': np.array([eje[0], eje[1], 0.0]),
            'largo': largo, 'ancho': ancho, 'alto': alto, 'n': len(obj)}


def altura_de_agarre(mesa_z, alto, holgura=0.005):
    """
    Altura del TCP para agarrar desde arriba [m].

    Las puntas de los dedos bajan PUNTA_DEDO (20.4 mm) por debajo del TCP, así que el TCP no puede
    ir a menos de mesa + 20.4 mm + holgura sin que los dedos toquen la mesa (MoveIt tampoco lo
    deja: medido, baja en línea recta hasta 25 mm sobre la mesa y no a 20). Si el objeto es más
    alto, el TCP va a la mitad de su altura (agarre por el medio).
    Ejemplo: tenedor de 16 mm de alto sobre una mesa a 0.72 m -> 0.72 + 0.0254 = 0.7454 m; las
    puntas quedan 5 mm sobre la mesa y los dedos aprietan los 11 mm de arriba del tenedor.
    """
    return mesa_z + max(PUNTA_DEDO + holgura, alto / 2.0)


def pose_desde_arriba(centro, eje, z):
    """
    (posición, giro 3x3) del TCP para agarrar desde arriba.

    La pinza apunta hacia abajo (su eje −X local = −Z del mundo, o sea X local = +Z) y los dedos
    (eje Y local) quedan cruzados al eje largo del objeto. Entre los dos sentidos posibles del Y
    local se elige el que deja el Z local (el "arriba" de la cámara de la muñeca) apuntando
    lejos de Milo: con esa elección la IK deja joint_6 cerca de 0 (medido: 0.13 rad en x = 0.8 m),
    que es lo que exige arm_moveit_node (|joint_6| <= 90°, para que la imagen no quede de cabeza).
    """
    eje = np.asarray(eje, float)
    eje = eje / np.linalg.norm(eje)
    x_local = np.array([0.0, 0.0, 1.0])
    y_local = np.array([-eje[1], eje[0], 0.0])        # horizontal, cruzado al eje largo
    z_local = np.cross(x_local, y_local)
    afuera = np.array([centro[0], centro[1], 0.0])    # de Milo hacia el objeto
    if np.linalg.norm(afuera) > 1e-6 and z_local @ afuera < 0:
        y_local, z_local = -y_local, -z_local
    giro = np.column_stack([x_local, y_local, z_local])
    return np.array([centro[0], centro[1], z], float), giro


def objetivo_dedos(ancho, apriete=0.002):
    """
    Posición de right_finger_joint para apretar un objeto de `ancho` [m] (el izquierdo va en
    espejo, −q). Cada dedo se manda `apriete` más adentro que la cara del objeto: con la ganancia
    p = 10000 N/m de gripper_controller eso son 20 N, y si el objeto se corre hacia un lado el
    otro dedo empuja más y lo devuelve (ver scripts/prueba_levantar.py). Se recorta a los
    límites del dedo.
    Ejemplo: 27 mm -> (0.027 − 0.017) / 2 − 0.002 = 0.003 m.
    """
    return float(np.clip((ancho - CARAS_EN_CERO) / 2.0 - apriete, CERRADA, ABIERTA))


def apertura(q_derecho, q_izquierdo):
    """Distancia entre las caras de los dedos [m]."""
    return CARAS_EN_CERO + q_derecho - q_izquierdo


def hay_objeto(apertura_medida, objetivo, minimo=0.002):
    """¿Los dedos se frenaron contra algo? Si quedaron al menos `minimo` [m] más abiertos de lo
    que se les pidió, algo los detiene: el objeto. Si llegaron a lo pedido, cerraron en vacío."""
    pedida = CARAS_EN_CERO + 2.0 * objetivo
    return apertura_medida - pedida >= minimo


def promediar(analisis):
    """Combina varias mediciones del mismo objeto (de `analizar_objeto`): mediana de cada dato.
    El eje se promedia con signo alineado al primero."""
    base = analisis[0]['eje']
    ejes = np.array([a['eje'] if a['eje'] @ base >= 0 else -a['eje'] for a in analisis])
    eje = np.median(ejes, axis=0)
    eje = eje / np.linalg.norm(eje)
    r = {k: float(np.median([a[k] for a in analisis]))
         for k in ('mesa_z', 'largo', 'ancho', 'alto')}
    r['centro'] = np.median([a['centro'] for a in analisis], axis=0)
    r['eje'] = eje
    r['n'] = int(sum(a['n'] for a in analisis))
    return r


def estable(analisis, tol=0.01):
    """¿Los centros de estas mediciones caen todos a menos de `tol` [m] de su mediana?"""
    centros = np.array([a['centro'] for a in analisis])
    return bool(np.all(np.linalg.norm(centros - np.median(centros, axis=0), axis=1) <= tol))
