# Tests de la lógica del filtro de seguridad (logic.py), sin ROS.
# Correr:  cd ros2_ws/src/andesrobot_safety && python3 -m pytest -q test
# pytest ejecuta cada función que empieza con "test_"; si un "assert" es falso, el test falla.
import math

import numpy as np

from andesrobot_safety.logic import SafetyParams, filter_cmd, remove_self

# Parámetros por defecto (los mismos valores que safety.yaml).
P = SafetyParams()


# Crea un "muro" de prueba: n puntos en línea, todos con la misma x, con y entre y0 e y1.
def wall_at(x, y0=-1.0, y1=1.0, n=50):
    return np.column_stack((np.full(n, x), np.linspace(y0, y1, n)))


# Sin obstáculos y pidiendo 1 m/s: debe dejar avanzar pero recortado al máximo (0.4).
def test_libre_avanza_y_satura():
    v, w, s = filter_cmd(1.0, 0.0, np.zeros((0, 2)), P)
    assert v == P.max_linear and s == 'libre'


# Muro 5 cm más cerca que stop_distance: no debe avanzar.
def test_para_en_stop_distance():
    pts = wall_at(P.footprint_x_max + P.stop_distance - 0.05)
    v, w, s = filter_cmd(0.3, 0.0, pts, P)
    assert v == 0.0 and 'PARADO adelante' in s


# Muro justo a la mitad de la zona lenta: debe ir a la mitad de la velocidad (0.4 -> 0.2).
def test_frena_en_zona_lenta():
    pts = wall_at(P.footprint_x_max + (P.stop_distance + P.slow_distance) / 2)
    v, _, _ = filter_cmd(0.4, 0.0, pts, P)
    assert math.isclose(v, 0.2, rel_tol=1e-6)


# Muro pegado adelante pero pidiendo retroceder: debe dejarlo (para salir).
def test_puede_retroceder_con_muro_adelante():
    pts = wall_at(P.footprint_x_max + 0.1)
    v, _, _ = filter_cmd(-0.2, 0.0, pts, P)
    assert v == -0.2


# Muro cerca atrás y pidiendo retroceder: no debe dejarlo.
def test_para_atras():
    pts = wall_at(P.footprint_x_min - 0.2)
    v, _, s = filter_cmd(-0.2, 0.0, pts, P)
    assert v == 0.0 and 'atrás' in s


def test_pasa_puerta_de_1m():
    # marcos de puerta a ±0.5 m, a lo largo de 2 m delante: fuera del corredor (±0.35)
    xs = np.linspace(0.5, 2.5, 40)
    pts = np.vstack([np.column_stack((xs, np.full(40, 0.5))), np.column_stack((xs, np.full(40, -0.5)))])
    v, _, _ = filter_cmd(0.3, 0.0, pts, P)
    assert v == 0.3


# Un punto a 0.6 m al costado (dentro del círculo de giro): no debe dejar girar.
def test_giro_bloqueado_si_algo_muy_cerca():
    pts = np.array([[0.0, 0.6]])                     # a 0.6 m del centro, al lado
    _, w, s = filter_cmd(0.0, 0.8, pts, P)
    assert w == 0.0 and 'giro bloqueado' in s


# Muro lejos adelante: debe dejar girar.
def test_giro_permitido_con_espacio():
    pts = wall_at(P.footprint_x_max + 1.2)
    _, w, _ = filter_cmd(0.0, 0.8, pts, P)
    assert w == 0.8


# Puntos sobre la columna del propio robot: deben descartarse.
def test_ignora_columna_propia():
    pts = np.array([[0.0, 0.0], [-0.03, 0.02]])      # puntos sobre la columna
    assert remove_self(pts, P).size == 0
