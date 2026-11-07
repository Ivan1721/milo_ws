# Pruebas de objetos.py: de la caja de YOLO a un punto 3D y una dirección de llegada, con una
# escena sintética (un objeto plano delante de una pared) de geometría conocida (sin ROS).
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_objetos.py
import numpy as np
import pytest

from andesrobot_arm.manija import Acercamiento, matriz_k, punto_en_rayo, rayo
from andesrobot_arm.objetos import (
    direccion_de_llegada, elegir_deteccion, pixeles_centrales, punto_del_objeto, validar_objeto)

K = matriz_k(640, 480, np.radians(71.0))
NOMBRES = {0: 'bag', 3: 'bowl', 10: 'cup'}


def escena(caja, z_objeto=0.6, z_fondo=1.5):
    """Profundidad 640x480: pared a z_fondo y un objeto a z_objeto ocupando `caja` (píxeles)."""
    d = np.full((480, 640), z_fondo)
    x1, y1, x2, y2 = caja
    d[y1:y2, x1:x2] = z_objeto
    return d


def test_elige_la_clase_pedida_con_mas_confianza():
    cajas = [[0, 0, 10, 10], [5, 5, 20, 20], [1, 1, 2, 2]]
    caja, conf, nombre = elegir_deteccion(cajas, [0.9, 0.6, 0.8], [3, 10, 10], NOMBRES, 'cup')
    assert nombre == 'cup' and conf == pytest.approx(0.8) and list(caja) == [1, 1, 2, 2]


def test_sin_clase_elige_la_de_mas_confianza_y_none_si_no_esta():
    assert elegir_deteccion([[0, 0, 1, 1]], [0.5], [3], NOMBRES)[2] == 'bowl'
    assert elegir_deteccion([[0, 0, 1, 1]], [0.5], [3], NOMBRES, 'cup') is None


def test_pixeles_centrales_quedan_dentro_de_la_caja():
    pix = np.array(pixeles_centrales([100, 200, 200, 300], fraccion=0.5, n=5))
    assert len(pix) == 25
    assert pix[:, 0].min() == pytest.approx(125) and pix[:, 0].max() == pytest.approx(175)
    assert pix[:, 1].min() == pytest.approx(225) and pix[:, 1].max() == pytest.approx(275)


def test_punto_del_objeto_ignora_los_rayos_que_tocan_el_fondo():
    # La caja de YOLO es más grande que el objeto (como pasa en la práctica): los rayos de los
    # bordes tocan la pared, pero la mediana queda sobre el objeto.
    objeto = (300, 220, 340, 260)
    d = escena(objeto)
    pix = pixeles_centrales([295, 215, 345, 265], fraccion=0.8, n=7)
    puntos = [punto_en_rayo(d, K, rayo(u, v, K), np.eye(3), np.zeros(3)) for u, v in pix]
    p = punto_del_objeto(puntos)
    assert p[2] == pytest.approx(0.6, abs=0.01)


def test_punto_del_objeto_necesita_suficientes_puntos():
    assert punto_del_objeto([None, None, np.zeros(3)]) is None


def test_direccion_de_llegada_horizontal_e_inclinada():
    d = direccion_de_llegada([0.2, 0.0, 1.0], [0.8, 0.0, 0.78])
    assert np.allclose(d, [1, 0, 0])
    d = direccion_de_llegada([0.2, 0.0, 1.0], [0.8, 0.6, 0.78], inclinacion=np.radians(30))
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert d[2] == pytest.approx(-0.5)
    assert d[0] == pytest.approx(d[1])
    with pytest.raises(ValueError):
        direccion_de_llegada([0.5, 0.0, 1.0], [0.5, 0.0, 0.7])


def test_validar_objeto():
    assert validar_objeto([0.8, 0.1, 0.78]) == []
    assert len(validar_objeto([3.0, 0.0, 0.78])) == 1
    assert len(validar_objeto([0.8, 0.0, 2.5])) == 1


def test_acercamiento_de_la_manija_sirve_para_objetos():
    # Misma secuencia de manija.py: 3 detecciones estables -> 30 cm; llegar -> otra vez -> 10 cm.
    a = Acercamiento((0.30, 0.10), tol_llegada=0.01)
    punto, d = np.array([0.8, 0.1, 0.78]), direccion_de_llegada([0.2, 0.0, 1.0], [0.8, 0.1, 0.78])
    for _ in range(2):
        assert a.deteccion(punto, d) is None
    posicion, giro = a.deteccion(punto, d)
    assert np.linalg.norm(posicion - punto) == pytest.approx(0.30)
    assert np.allclose(-giro[:, 0], d)          # la pinza apunta con su −X hacia el objeto
    assert a.tcp(posicion) and a.fase == 'buscar'
    for _ in range(3):
        objetivo = a.deteccion(punto, d)
    assert np.linalg.norm(objetivo[0] - punto) == pytest.approx(0.10)
