"""Cuentas de la ventana de sliders (sliders_node), sin ROS ni ventana, para poder probarlas.

En palabras simples: cada slider es el ángulo de UNA articulación. Al soltarlo, sliders_node arma
una trayectoria de un solo punto ("llega a este ángulo en T segundos") y se la manda al mismo
controlador que usa MoveIt (/arm_controller/follow_joint_trajectory). Aquí están las cuentas:

  - limites_de_urdf: los topes de cada articulación, leídos del URDF que publica
    robot_state_publisher en /robot_description (así los sliders no dejan pedir más que el URDF).
  - duracion: cuánto tiempo darle al movimiento para que ninguna articulación pase de
    `velocidad` [rad/s] (o [m/s] para el lift). Despacio a propósito: es para probar motores.
  - acotar, a_segundos_nanos, dedos: pasar del slider (grados, o mm para el lift y los dedos)
    a lo que espera ROS (rad o m).
"""
import math
import xml.etree.ElementTree as ET

# Los dedos de la pinza: abierta 0.016 m, cerrada -0.007 m por dedo (como en la simulación y en
# brazo_real.py). El izquierdo va en espejo: [q, -q].
DEDO_ABIERTA = 0.016
DEDO_CERRADA = -0.007


def limites_de_urdf(texto, nombres):
    """{articulación: (inferior, superior)} de las articulaciones `nombres` en el URDF `texto`.

    Las 'continuous' (sin topes) quedan en ±π. Si una no está en el URDF, falta en el resultado
    (el nodo usa entonces ±π o lo que diga su parámetro)."""
    raiz = ET.fromstring(texto)
    limites = {}
    # Solo los <joint> hijos directos de <robot>: el bloque <ros2_control> también tiene
    # <joint name="..."> (sin <limit>) y no debe pisar los topes de verdad.
    for j in raiz.findall('joint'):
        nombre = j.get('name')
        if nombre not in nombres:
            continue
        lim = j.find('limit')
        if j.get('type') == 'continuous' or lim is None:
            limites[nombre] = (-math.pi, math.pi)
        else:
            limites[nombre] = (float(lim.get('lower', -math.pi)),
                               float(lim.get('upper', math.pi)))
    return limites


def acotar(valor, inferior, superior):
    return min(max(valor, inferior), superior)


def duracion(actual, meta, velocidad, minimo=1.0):
    """Segundos para ir de `actual` a `meta` (listas del mismo largo) sin que ninguna pase de
    `velocidad` (rad/s o m/s; puede ser una lista, una por articulación). Nunca menos de `minimo`.

    Ejemplo: joint_2 de 0 a 30° (0.52 rad) a 0.1 rad/s -> 5.2 s."""
    if not isinstance(velocidad, (list, tuple)):
        velocidad = [velocidad] * len(meta)
    t = max((abs(m - a) / v for a, m, v in zip(actual, meta, velocidad) if v > 0), default=0.0)
    return max(t, minimo)


def a_segundos_nanos(t):
    """Duración de ROS (sec, nanosec) a partir de segundos."""
    sec = int(t)
    return sec, int(round((t - sec) * 1e9))


def dedos(q):
    """Posiciones de los dos dedos para una apertura `q` [m] del derecho: [q, -q], acotada."""
    q = acotar(q, DEDO_CERRADA, DEDO_ABIERTA)
    return [q, -q]
