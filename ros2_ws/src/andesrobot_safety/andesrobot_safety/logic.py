"""Lógica pura del filtro de seguridad (sin ROS, testeable).

Todo en el frame base_footprint: +X adelante, +Y izquierda, origen = centro del eje de ruedas.

Por qué está separado de safety_filter.py: aquí solo hay matemática (recibe números, devuelve
números). Así se puede probar con pytest sin tener ROS corriendo (ver test/test_logic.py).

Vista desde arriba (x hacia la derecha = adelante de Milo):

              corredor (ancho = robot + side_margin a cada lado)
     ┌──────────────┬─────────────────────┬──────────────┐
     │ zona trasera │       ROBOT         │ zona delant. │
     └──────────────┴─────────────────────┴──────────────┘
                 x_min (-0.12)     (0,0)  x_max (0.44)
  Solo cuentan los obstáculos DENTRO del corredor y en la dirección en que se mueve.
"""
# math: funciones matemáticas (hypot, inf). dataclass: forma corta de definir una clase de datos.
import math
from dataclasses import dataclass

# numpy: operar con miles de puntos del lidar a la vez (mucho más rápido que un for).
import numpy as np


# @dataclass crea automáticamente el constructor: SafetyParams(stop_distance=0.5, ...).
# Los valores de aquí son los por defecto; safety_filter.py los reemplaza con los de safety.yaml.
@dataclass
class SafetyParams:
    # Caja del robot (base_footprint), medida de las mallas de Milo
    footprint_x_min: float = -0.12
    footprint_x_max: float = 0.44
    footprint_half_width: float = 0.25
    # Zonas
    stop_distance: float = 0.3      # [m] desde el borde del robot: no avanza más
    slow_distance: float = 0.7      # [m] empieza a frenar de a poco
    side_margin: float = 0.10       # [m] holgura lateral del corredor
    rotation_clearance: float = 0.15  # [m] holgura mínima alrededor para girar en el lugar
    # Límites
    max_linear: float = 0.4         # [m/s]
    max_angular: float = 1.0        # [rad/s]


def robot_radius(p: SafetyParams) -> float:
    """Radio del círculo que barre el robot al girar en el lugar."""
    # Al girar en el lugar (alrededor del eje de ruedas), la parte que más se aleja del centro
    # es una de las esquinas de la caja. Probamos la esquina de adelante y la de atrás.
    xs = (p.footprint_x_min, p.footprint_x_max)
    # hypot(x, y) = distancia desde (0,0) a (x, y) = raíz(x² + y²). Nos quedamos con la mayor.
    return max(math.hypot(x, p.footprint_half_width) for x in xs)


def corridor_distances(pts: np.ndarray, p: SafetyParams):
    """Distancia libre adelante y atrás (desde el borde del robot) dentro del corredor."""
    # Sin puntos (nada a la vista): libre infinito adelante y atrás.
    if pts.size == 0:
        return math.inf, math.inf
    # pts es una tabla de N filas x 2 columnas: [:, 0] = todas las x, [:, 1] = todas las y.
    x, y = pts[:, 0], pts[:, 1]
    # in_lane = arreglo de True/False: ¿cada punto está dentro del ancho del corredor?
    in_lane = np.abs(y) <= p.footprint_half_width + p.side_margin
    # Puntos del corredor que están delante de la punta del robot, y su distancia a esa punta.
    # (El "&" combina condiciones punto a punto; x[...] se queda solo con los True.)
    front = x[in_lane & (x > p.footprint_x_max)] - p.footprint_x_max
    # Igual para atrás: puntos detrás de la cola del robot y su distancia a la cola.
    rear = p.footprint_x_min - x[in_lane & (x < p.footprint_x_min)]
    # La distancia libre es la del punto más cercano. Si no hay ninguno, infinito.
    return (float(front.min()) if front.size else math.inf,
            float(rear.min()) if rear.size else math.inf)


def remove_self(pts: np.ndarray, p: SafetyParams) -> np.ndarray:
    """Descarta puntos que caen sobre el propio robot (p. ej. la columna)."""
    if pts.size == 0:
        return pts
    # inside = True para los puntos dentro de la caja del robot (+2 cm de margen):
    # son lecturas del propio Milo (la columna), no obstáculos.
    inside = ((pts[:, 0] >= p.footprint_x_min - 0.02) & (pts[:, 0] <= p.footprint_x_max + 0.02)
              & (np.abs(pts[:, 1]) <= p.footprint_half_width + 0.02))
    # "~" invierte True/False: devolvemos solo los puntos que NO están dentro.
    return pts[~inside]


def scale_for(d: float, p: SafetyParams) -> float:
    """1 = libre, 0 = parar; lineal entre stop_distance y slow_distance."""
    # Muy cerca: velocidad x 0 (parar).
    if d <= p.stop_distance:
        return 0.0
    # Lejos: velocidad x 1 (sin cambios).
    if d >= p.slow_distance:
        return 1.0
    # En medio: proporcional. Ej. stop 0.3, slow 0.7, d = 0.5 -> (0.5-0.3)/(0.7-0.3) = 0.5
    return (d - p.stop_distance) / (p.slow_distance - p.stop_distance)


def filter_cmd(v: float, w: float, pts: np.ndarray, p: SafetyParams):
    """Devuelve (v, w, estado) seguros. pts: Nx2 en base_footprint, sin el robot."""
    # v = velocidad lineal pedida (m/s, + adelante), w = velocidad de giro (rad/s, + izquierda).
    # 1) Recortar a los máximos: min/max deja el valor entre -max y +max.
    v = max(-p.max_linear, min(p.max_linear, v))
    w = max(-p.max_angular, min(p.max_angular, w))
    # 2) Cuánto espacio libre hay adelante y atrás.
    d_front, d_rear = corridor_distances(pts, p)
    # Texto que se publica en /safety/state para saber qué está haciendo el filtro.
    state = 'libre'

    # 3) Si va hacia adelante, solo importa lo de adelante...
    if v > 0.0:
        s = scale_for(d_front, p)
        if s < 1.0:
            # "A if condición else B": texto distinto si paró del todo o solo está frenando.
            state = 'PARADO adelante (%.2f m)' % d_front if s == 0.0 else 'frenando adelante (%.2f m)' % d_front
        v *= s
    # ...y si va hacia atrás, solo lo de atrás. Así siempre puede alejarse de un obstáculo.
    elif v < 0.0:
        s = scale_for(d_rear, p)
        if s < 1.0:
            state = 'PARADO atrás (%.2f m)' % d_rear if s == 0.0 else 'frenando atrás (%.2f m)' % d_rear
        v *= s

    # 4) Giro: si hay algo dentro del círculo que barre el robot al girar (+ holgura),
    #    no dejar girar (la esquina del robot lo golpearía).
    if w != 0.0 and pts.size:
        # Distancia de cada punto al centro del robot, y la menor de todas.
        r_min = float(np.hypot(pts[:, 0], pts[:, 1]).min())
        if r_min < robot_radius(p) + p.rotation_clearance:
            w = 0.0
            state += ' | giro bloqueado (%.2f m)' % r_min
    # Devolver las velocidades seguras y el estado.
    return v, w, state
