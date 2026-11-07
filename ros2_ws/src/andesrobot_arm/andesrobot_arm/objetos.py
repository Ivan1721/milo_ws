"""Geometría para llevar la pinza frente a un objeto detectado por YOLO (sin ROS: se prueba sola).

Es la versión para objetos de manija.py. El modelo de objetos (modelos/objetos/best.pt, 27 clases
del reglamento: cup, bowl, bottle...) es de DETECCIÓN: solo da una caja (rectángulo) por objeto,
sin keypoints. Para mover el brazo hace falta un punto 3D y una dirección de llegada:

  1. De todas las cajas se elige la de la clase pedida con más confianza (`elegir_deteccion`).
  2. Se toman píxeles de la parte central de la caja (`pixeles_centrales`): los bordes suelen
     tener fondo (la mesa, la pared). Cada píxel a color es un rayo que se sigue hasta la
     superficie que mide la profundidad (`manija.punto_en_rayo`, la misma función de la manija).
  3. El punto del objeto es la mediana de esos puntos (`punto_del_objeto`): la mediana no se
     mueve si unos pocos rayos tocan el fondo. Es un punto de la CARA del objeto que mira a la
     cámara, no su centro (la cámara no ve el otro lado).
  4. La pinza llega desde el lado del robot, en horizontal o un poco desde arriba
     (`direccion_de_llegada`), y la secuencia en dos tramos es la misma de la manija
     (`manija.Acercamiento`: 30 cm, nueva detección, y después `distancia_final`).

Convenciones: puntos en metros en un mismo frame (base_footprint); la pinza apunta con su eje −X
local (ver manija.py).
"""
import numpy as np


def elegir_deteccion(cajas, confianzas, clases, nombres, clase=''):
    """
    La mejor detección de una clase.

    cajas       N x 4, en píxeles (x1, y1, x2, y2), como `boxes.xyxy` de Ultralytics
    confianzas  N valores entre 0 y 1; clases: N índices; nombres: índice -> nombre del modelo
    clase       nombre buscado ('cup'); vacío = la de más confianza de cualquier clase
    Devuelve (caja, confianza, nombre) o None si no hay ninguna de esa clase.
    """
    mejor = None
    for caja, conf, c in zip(cajas, confianzas, clases):
        nombre = nombres[int(c)]
        if clase and nombre != clase:
            continue
        if mejor is None or conf > mejor[1]:
            mejor = (np.asarray(caja, float), float(conf), nombre)
    return mejor


def pixeles_centrales(caja, fraccion=0.5, n=5):
    """Grilla de n x n píxeles (u, v) dentro de la parte central de la caja: `fraccion` del ancho y
    del alto, alrededor del centro."""
    x1, y1, x2, y2 = (float(v) for v in caja)
    cu, cv = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    mu, mv = fraccion * (x2 - x1) / 2.0, fraccion * (y2 - y1) / 2.0
    us = np.linspace(cu - mu, cu + mu, n)
    vs = np.linspace(cv - mv, cv + mv, n)
    return [(u, v) for v in vs for u in us]


def punto_del_objeto(puntos, minimo=5):
    """Mediana (coordenada por coordenada) de los puntos 3D que tocaron el objeto, o None si
    quedaron menos de `minimo` (casi toda la caja sin profundidad)."""
    puntos = [p for p in puntos if p is not None]
    if len(puntos) < minimo:
        return None
    return np.median(np.array(puntos, float), axis=0)


def direccion_de_llegada(desde, punto, inclinacion=0.0):
    """
    Dirección (unitaria) en que la pinza llega al objeto: la horizontal que va de `desde` (la
    cámara o el TCP) al objeto, inclinada `inclinacion` [rad] hacia abajo. Con 0 la pinza llega
    de frente; con 0.3-0.5 rad llega un poco desde arriba, lo que deja más espacio sobre la mesa.
    """
    d = np.asarray(punto, float) - np.asarray(desde, float)
    d[2] = 0.0
    largo = np.linalg.norm(d)
    if largo < 1e-6:
        raise ValueError('el objeto está justo debajo: no hay una dirección horizontal de llegada')
    d = d / largo
    return np.array([d[0] * np.cos(inclinacion), d[1] * np.cos(inclinacion),
                     -np.sin(inclinacion)])


def validar_objeto(punto, alcance=(0.25, 1.2), altura=(0.05, 1.8)):
    """Problemas de un punto detectado (vacío = tiene sentido): distancia horizontal desde Milo
    (base_footprint) y altura dentro de lo que el brazo puede alcanzar."""
    problemas = []
    horizontal = float(np.hypot(punto[0], punto[1]))
    if not alcance[0] <= horizontal <= alcance[1]:
        problemas.append(f'está a {horizontal:.2f} m de Milo (se esperan '
                         f'{alcance[0]:.2f}-{alcance[1]:.2f} m)')
    if not altura[0] <= punto[2] <= altura[1]:
        problemas.append(f'está a {punto[2]:.2f} m de altura (se esperan '
                         f'{altura[0]:.2f}-{altura[1]:.2f} m)')
    return problemas
