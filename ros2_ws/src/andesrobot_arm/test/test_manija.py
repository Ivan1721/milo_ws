# Pruebas de manija.py: de píxeles de la cámara a puntos 3D y a movimientos, con una puerta
# sintética de geometría conocida (sin ROS ni Gazebo).
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_manija.py
import numpy as np
import pytest

from andesrobot_arm.kinematics import matrix_to_quaternion
from andesrobot_arm.manija import (
    Acercamiento, arco_palanca, corregir_saliente, deproyectar, distancia_desde_tcp, matriz_k,
    normal_de_la_puerta, orientacion_pinza, pose_previa, punto_en_plano, punto_en_rayo,
    puntos_anillo, rayo, validar_manija)

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
    # El píxel muestra la manija, que sobresale 5 cm de la hoja (hacia la cámara).
    verdadero = np.array([0.12, -0.07, DISTANCIA - 0.05])
    u, v = pixel_de_color(verdadero)
    d = R @ rayo(u, v, K_COLOR)
    hallado = punto_en_plano(T, d, normal, punto, saliente=0.05)
    assert np.allclose(hallado, verdadero, atol=1e-6)


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


# Puerta frente a Milo, como en puerta_prueba.sdf: la normal apunta a +X (hacia donde se empuja)
# y el punto de agarre queda a 1 m de altura.
AGARRE = np.array([0.75, -0.38, 1.0])
NORMAL = np.array([1.0, 0.0, 0.0])


def test_distancia_desde_tcp_separa_avance_y_corrimientos():
    # Pinza a 30 cm de la manija, mirándola, pero 2 cm a la izquierda (+Y del mundo) y 1 cm abajo.
    posicion, giro = pose_previa(AGARRE, NORMAL, 0.30)
    posicion = posicion + np.array([0.0, 0.02, -0.01])
    d = distancia_desde_tcp(posicion, giro, AGARRE)
    assert d['adelante'] == pytest.approx(0.30)
    # La manija queda 2 cm a −Y del mundo respecto de la pinza; el eje Y de la pinza apunta a −Y
    # del mundo (orientacion_pinza), así que para la pinza son +2 cm. Y queda 1 cm más arriba.
    assert d['lateral'] == pytest.approx(0.02)
    assert d['vertical'] == pytest.approx(0.01)
    assert d['total'] == pytest.approx(np.sqrt(0.30**2 + 0.02**2 + 0.01**2))
    # Una manija detrás de la pinza da un avance negativo.
    assert distancia_desde_tcp(AGARRE + 0.1 * NORMAL, giro, AGARRE)['adelante'] < 0


def test_acercamiento_en_dos_tramos():
    a = Acercamiento(distancias=(0.30, 0.05), n_estables=3)
    rng = np.random.default_rng(0)
    vista = [AGARRE + rng.normal(0, 0.003, 3) for _ in range(3)]
    assert a.deteccion(vista[0], NORMAL) is None and a.deteccion(vista[1], NORMAL) is None
    posicion, giro = a.deteccion(vista[2], NORMAL)
    assert a.fase == 'yendo'
    assert distancia_desde_tcp(posicion, giro, AGARRE)['adelante'] == pytest.approx(0.30, abs=0.01)
    assert a.deteccion(AGARRE, NORMAL) is None          # mientras va, no manda otra cosa
    assert not a.tcp(posicion + [0.0, 0.0, 0.05])       # todavía a 5 cm del objetivo
    assert a.tcp(posicion) and a.fase == 'buscar' and a.tramo == 1
    for _ in range(2):
        assert a.deteccion(AGARRE, NORMAL) is None
    posicion, giro = a.deteccion(AGARRE, NORMAL)
    assert distancia_desde_tcp(posicion, giro, AGARRE)['adelante'] == pytest.approx(0.05)
    assert a.tcp(posicion) and a.fase == 'listo'
    assert a.deteccion(AGARRE, NORMAL) is None


def test_acercamiento_espera_detecciones_estables_y_reintenta():
    a = Acercamiento(n_estables=3, tol_estable=0.02)
    a.deteccion(AGARRE, NORMAL)
    a.deteccion(AGARRE + [0.0, 0.10, 0.0], NORMAL)      # una detección que saltó 10 cm
    assert a.deteccion(AGARRE, NORMAL) is None
    assert a.deteccion(AGARRE, NORMAL) is None          # el salto sigue entre las 3 últimas
    assert a.deteccion(AGARRE, NORMAL) is not None      # ya salió: tres estables
    a.reintentar()                                      # MoveIt no llegó
    assert a.fase == 'buscar' and a.tramo == 0 and a.vistas == []


def test_corregir_saliente_adelanta_los_puntos_que_cayeron_en_la_hoja():
    # Cámara en (0.40, -0.30, 1.10) mirando la manija de AGARRE en diagonal: el rayo que pasa por
    # la manija sigue hasta la hoja (x = 0.80) si no la toca.
    camara = np.array([0.40, -0.30, 1.10])
    direccion = (AGARRE - camara) / np.linalg.norm(AGARRE - camara)
    plano = np.array([0.80, 0.0, 1.0])                  # un punto de la hoja
    s = (0.80 - camara[0]) / direccion[0]
    en_la_hoja = camara + s * direccion                 # lo que midió la profundidad
    corregido = corregir_saliente(en_la_hoja, camara, direccion, NORMAL, plano, 0.05)
    assert np.allclose(corregido, AGARRE)               # sobre el mismo rayo, en la manija
    detras = en_la_hoja + 0.01 * direccion              # ruido: 1 cm más lejos
    assert np.allclose(corregir_saliente(detras, camara, direccion, NORMAL, plano, 0.05), AGARRE)
    en_la_manija = AGARRE + 0.002 * direccion           # la profundidad sí midió la manija
    assert np.allclose(corregir_saliente(en_la_manija, camara, direccion, NORMAL, plano, 0.05),
                       en_la_manija)
