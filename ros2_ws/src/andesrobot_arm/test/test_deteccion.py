"""Pruebas de deteccion.py (las cuentas de deteccion_node), sin ROS ni modelo.

python3 -m pytest -q test/test_deteccion.py
"""
import numpy as np
import pytest

from andesrobot_arm.deteccion import distancia_en_caja, escalar_caja, leer_clases, punto_3d


def test_escalar_caja_de_color_a_profundidad():
    """Color 640x480 -> profundidad 640x400: solo cambian las y."""
    assert escalar_caja((100, 240, 200, 480), (640, 480), (640, 400)) == \
        pytest.approx((100, 200, 200, 400))


def test_distancia_mediana_del_centro_de_la_caja():
    """Piso a 600 mm y un objeto a 450 mm que llena el centro de la caja; los bordes de la caja
    (piso) y los píxeles sin dato (0) no cambian la mediana."""
    prof = np.full((400, 640), 600, np.uint16)
    prof[150:250, 300:340] = 450
    prof[190:200, 310:320] = 0
    assert distancia_en_caja(prof, (290, 140, 350, 260)) == pytest.approx(0.45)


def test_distancia_sin_datos():
    prof = np.zeros((400, 640), np.uint16)
    assert distancia_en_caja(prof, (10, 10, 50, 50)) is None


def test_caja_fuera_de_la_imagen_se_recorta():
    prof = np.full((400, 640), 500, np.uint16)
    assert distancia_en_caja(prof, (600, 380, 700, 450)) == pytest.approx(0.5)


def test_punto_3d():
    """Con la K de la Gemini real: el centro óptico queda en x = y = 0; 100 px a la derecha a
    1 m son 100 / fx metros."""
    K = [[455.1, 0, 327.0], [0, 455.1, 243.9], [0, 0, 1]]
    assert punto_3d(327.0, 243.9, 1.0, K) == pytest.approx((0, 0, 1))
    x, y, z = punto_3d(427.0, 243.9, 1.0, K)
    assert x == pytest.approx(100 / 455.1) and y == pytest.approx(0) and z == 1.0


def test_leer_clases():
    assert leer_clases(' can, cup ,, bottle ') == ['can', 'cup', 'bottle']
