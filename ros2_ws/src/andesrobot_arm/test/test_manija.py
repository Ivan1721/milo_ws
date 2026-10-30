# Pruebas de manija.py: de píxeles de la cámara a puntos 3D y a movimientos, con una puerta
# sintética de geometría conocida (sin ROS ni Gazebo).
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_manija.py
import numpy as np
import pytest

from andesrobot_arm.kinematics import matrix_to_quaternion
from andesrobot_arm.manija import (
    arco_palanca, deproyectar, matriz_k, normal_de_la_puerta, orientacion_pinza,
    pose_previa, punto_en_plano, punto_en_rayo, puntos_anillo, rayo, validar_manija)

# Cámara de profundidad 640x400 (67.9°) y cámara a color 640x480 (71°), como en la simulación.
K_PROF = matriz_k(640, 400, np.radians(67.9))
K_COLOR = matriz_k(640, 480, np.radians(71.0))
# La cámara a color está 2.5 cm a la derecha de la de profundidad (en la real hay una
# distancia así entre las dos).
R, T = np.eye(3), np.array([0.025, 0.0, 0.0])
DISTANCIA = 1.0   # la puerta es un plano z = 1 m en el frame de profundidad


def puerta_sintetica(huecos=()):
    """Profundidad de un plano a 1 m; `huecos` = rectángulos (u0, u1, v0, v1) sin dato."""
    profundidad = np.full((400, 640), DISTANCIA)
    for u0, u1, v0, v1 in huecos:
        profundidad[v0:v1, u0:u1] = 0.0
    return profundidad


def pixel_de_color(punto_prof):
    """Píxel de la cámara a color donde se ve un punto dado en el frame de profundidad."""
    p = R.T @ (punto_prof - T)
    return K_COLOR[0, 0] * p[0] / p[2] + K_COLOR[0, 2], K_COLOR[1, 1] * p[1] / p[2] + K_COLOR[1, 2]


def test_deproyectar_y_rayo_son_coherentes():
    punto = deproyectar(400, 150, 2.0, K_PROF)
    assert np.allclose(rayo(400, 150, K_PROF) * 2.0 / rayo(400, 150, K_PROF)[2], punto)


def test_punto_en_rayo_recupera_un_punto_de_la_puerta():
    verdadero = np.array([0.12, -0.07, DISTANCIA])
    u, v = pixel_de_color(verdadero)
    hallado = punto_en_rayo(puerta_sintetica(), K_PROF, rayo(u, v, K_COLOR), R, T)
    assert hallado is not None and np.linalg.norm(hallado - verdadero) < 0.005


def test_punto_en_rayo_con_un_hueco_de_profundidad_da_none():
    verdadero = np.array([0.12, -0.07, DISTANCIA])
    u, v = pixel_de_color(verdadero)
    hueco = (int(K_PROF[0, 2] + 0.12 / DISTANCIA * K_PROF[0, 0]) - 40,) * 2
    profundidad = puerta_sintetica([(hueco[0], hueco[0] + 80, 100, 300)])
    assert punto_en_rayo(profundidad, K_PROF, rayo(u, v, K_COLOR), R, T) is None


def test_plano_de_la_puerta_y_punto_de_respaldo():
    # La profundidad falla sobre la manija: se ajusta el plano con el anillo de alrededor.
    centro = (int(K_PROF[0, 2] + 0.12 * K_PROF[0, 0]), int(K_PROF[1, 2] - 0.07 * K_PROF[1, 1]))
    hueco = (centro[0] - 30, centro[0] + 30, centro[1] - 30, centro[1] + 30)
    profundidad = puerta_sintetica([hueco])
    puntos = puntos_anillo(profundidad, K_PROF, centro, radio_exterior=80, radio_interior=35)
    normal, punto, error = normal_de_la_puerta(puntos)
    assert np.allclose(normal, [0, 0, 1], atol=1e-6) and error < 1e-9
    verdadero = np.array([0.12, -0.07, DISTANCIA])
    u, v = pixel_de_color(verdadero)
    d = R @ rayo(u, v, K_COLOR)
    hallado = punto_en_plano(T, d, normal, punto, saliente=0.05)
    # Cae en la puerta, corrido 5 cm hacia la cámara (la manija sobresale de la hoja).
    assert np.allclose(hallado, verdadero - [0, 0, 0.05], atol=0.005)


def test_la_normal_apunta_hacia_la_puerta_aunque_el_ajuste_la_de_vuelta():
    z = np.random.default_rng(0).uniform(-0.3, 0.3, (200, 2))
    puntos = np.column_stack([z, np.full(200, 1.5)])
    normal = normal_de_la_puerta(puntos)[0]
    assert normal[2] > 0.99


def test_pocos_puntos_no_alcanzan_para_un_plano():
    assert normal_de_la_puerta(np.zeros((5, 3))) is None


def test_validar_manija():
    normal = np.array([1.0, 0, 0])                       # puerta en el plano y-z
    eje = np.array([0.75, -0.38, 1.0])
    assert validar_manija(eje, eje + [0, -0.07, 0], eje + [0, -0.12, 0], normal) == []
    assert 'mide' in validar_manija(eje, eje, eje + [0, -0.30, 0], normal)[0]
    fuera = validar_manija(eje, eje + [0, -0.2, 0], eje + [0, -0.12, 0], normal)
    assert any('entre' in p for p in fuera)
    assert any('plano' in p for p in validar_manija(eje, eje, eje + [0.12, 0, 0], normal))


def test_orientacion_de_la_pinza_coincide_con_la_pose_de_ejemplo_de_brazo_md():
    # Mirando a +X, el ejemplo de docs/BRAZO.md usa el cuaternión (0, 0, 1, 0).
    cuaternion = matrix_to_quaternion(orientacion_pinza([1, 0, 0]))
    assert np.allclose(np.abs(cuaternion), [0, 0, 1, 0], atol=1e-9)
    giro = orientacion_pinza([1, 0, 0])
    assert np.allclose(giro @ [-1, 0, 0], [1, 0, 0])       # su eje −X local apunta a la puerta


def test_pose_previa_queda_antes_de_la_manija():
    posicion, giro = pose_previa([0.75, -0.38, 1.0], [1, 0, 0], distancia=0.30)
    assert np.allclose(posicion, [0.45, -0.38, 1.0])
    assert abs(np.linalg.det(giro) - 1.0) < 1e-9


def test_aproximacion_vertical_es_un_error():
    with pytest.raises(ValueError):
        orientacion_pinza([0, 0, 1])


def test_el_arco_baja_la_palanca_y_conserva_el_radio():
    eje = np.array([0.75, -0.38, 1.0])
    agarre = eje + [0, -0.07, 0]                         # palanca horizontal hacia la derecha
    for sentido in (1.0, -1.0):                          # da igual hacia qué lado apunte
        camino = arco_palanca(eje, eje + sentido * np.array([0, 0.07, 0]), [1, 0, 0],
                              np.radians(40), pasos=8)
        assert len(camino) == 9 and np.allclose(camino[0], eje + sentido * np.array([0, 0.07, 0]))
        assert camino[-1][2] < camino[0][2] - 0.03       # terminó más abajo
        assert np.all(np.diff(camino[:, 2]) < 0)         # y bajó todo el camino
        assert np.allclose(np.linalg.norm(camino - eje, axis=1), 0.07)
        assert np.allclose(camino[:, 0], 0.75)           # sigue en el plano de la puerta
    assert agarre is not None


def test_arco_con_palanca_vertical_es_un_error():
    with pytest.raises(ValueError):
        arco_palanca([0, 0, 1.0], [0, 0, 1.07], [1, 0, 0], np.radians(30))
