"""Pruebas de agarre.py (geometría del agarre desde arriba) con una mesa y un objeto sintéticos.

No necesitan ROS: python3 -m pytest -q test/test_agarre.py
El "objeto" es una barra como el tenedor (190 x 27 mm, 16 mm de alto) sobre una mesa a 0.72 m,
vista como una nube de puntos: la tapa del objeto y la mesa alrededor (lo que daría la grilla de
rayos dentro de la caja de YOLO).
"""
import numpy as np
import pytest

from andesrobot_arm.agarre import (
    ABIERTA, CERRADA, PUNTA_DEDO, altura_de_agarre, analizar_objeto, apertura, estable,
    hay_objeto, objetivo_dedos, pose_desde_arriba, promediar)
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix
from andesrobot_arm.manija import orientacion_pinza

MESA = 0.72


def nube(centro=(0.80, 0.0), angulo=0.0, largo=0.19, ancho=0.027, alto=0.016, paso=0.003):
    """Puntos de la mesa (cuadrado de 30 cm) y de la tapa de una barra girada `angulo` [rad]."""
    eje = np.array([np.cos(angulo), np.sin(angulo)])
    cruzado = np.array([-eje[1], eje[0]])
    s = np.arange(-0.15, 0.15, paso)
    puntos = []
    for a in s:
        for b in s:
            xy = np.array(centro) + a * np.array([1.0, 0.0]) + b * np.array([0.0, 1.0])
            rel = xy - np.array(centro)
            encima = abs(rel @ eje) <= largo / 2 and abs(rel @ cruzado) <= ancho / 2
            puntos.append(np.array([xy[0], xy[1], MESA + (alto if encima else 0.0)]))
    return puntos


@pytest.mark.parametrize('angulo', [0.0, 0.5, -1.0, np.pi / 2])
def test_analizar_mide_la_barra(angulo):
    m = analizar_objeto(nube(angulo=angulo))
    assert m is not None
    assert m['mesa_z'] == pytest.approx(MESA, abs=1e-6)
    assert m['centro'][:2] == pytest.approx([0.80, 0.0], abs=0.003)
    assert m['alto'] == pytest.approx(0.016, abs=0.002)
    # Ancho y largo por percentiles 5-95: un poco menos que la medida real.
    assert 0.018 <= m['ancho'] <= 0.030
    assert 0.15 <= m['largo'] <= 0.20
    # El eje largo, con signo libre (la barra girada 180° es la misma).
    real = np.array([np.cos(angulo), np.sin(angulo), 0.0])
    assert abs(m['eje'] @ real) == pytest.approx(1.0, abs=0.01)


def test_analizar_con_el_canto_de_la_mesa_en_la_caja():
    # Objeto en el borde: la caja incluye el canto delantero de la mesa (vertical, de 0.68 a
    # 0.72 m) y algo del suelo. La mesa tiene que seguir saliendo a 0.72 y el ancho, el del objeto.
    canto = [np.array([0.70, y, z]) for y in np.arange(-0.15, 0.15, 0.003)
             for z in np.arange(0.68, 0.72, 0.003)]
    suelo = [np.array([0.65, y, 0.0]) for y in np.arange(-0.15, 0.15, 0.01)]
    m = analizar_objeto(nube() + canto + suelo)
    assert m['mesa_z'] == pytest.approx(MESA, abs=0.002)
    assert 0.018 <= m['ancho'] <= 0.030
    assert m['alto'] == pytest.approx(0.016, abs=0.003)


def test_centro_sin_sesgo_por_densidad():
    # La mitad cercana a la cámara se ve con 4 veces más puntos (más cerca = más densa). El
    # centro debe seguir siendo el del objeto, no correrse hacia la parte densa.
    puntos = nube(paso=0.004)
    densa = [p for p in nube(paso=0.002) if p[2] > MESA + 0.004 and p[0] < 0.80]
    m = analizar_objeto(puntos + densa)
    assert m['centro'][0] == pytest.approx(0.80, abs=0.006)


def test_analizar_sin_objeto_o_con_pocos_puntos():
    solo_mesa = nube(ancho=0.0, largo=0.0)
    assert analizar_objeto(solo_mesa) is None
    assert analizar_objeto([None, np.array([0.8, 0.0, MESA])]) is None


def test_analizar_ignora_rayos_perdidos():
    puntos = nube() + [None] * 50
    assert analizar_objeto(puntos) is not None


def test_altura_de_agarre():
    # Objeto bajo: el TCP queda a la punta del dedo + holgura sobre la mesa (no la toca).
    assert altura_de_agarre(MESA, 0.016) == pytest.approx(MESA + PUNTA_DEDO + 0.005)
    # Objeto alto: el TCP va a la mitad de su altura.
    assert altura_de_agarre(MESA, 0.08) == pytest.approx(MESA + 0.04)


@pytest.mark.parametrize('angulo', [0.0, 0.7, -0.7, np.pi / 2])
def test_pose_desde_arriba(angulo):
    eje = np.array([np.cos(angulo), np.sin(angulo), 0.0])
    centro = np.array([0.80, 0.05, 0.736])
    pos, giro = pose_desde_arriba(centro, eje, 0.7325)
    assert pos == pytest.approx([0.80, 0.05, 0.7325])
    x, y, z = giro[:, 0], giro[:, 1], giro[:, 2]
    assert x == pytest.approx([0.0, 0.0, 1.0])          # la pinza (−X local) apunta abajo
    assert y @ eje == pytest.approx(0.0, abs=1e-9)      # dedos cruzados al eje largo
    assert y[2] == pytest.approx(0.0, abs=1e-9)
    assert giro.T @ giro == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(giro) == pytest.approx(1.0)    # sistema derecho (es una rotación)
    assert z @ np.array([0.80, 0.05, 0.0]) > 0          # "arriba" de la cámara, lejos de Milo


def test_objetivo_dedos():
    # 27 mm: cada cara a 13.5 mm del centro; la de q = 0 está a 8.5 -> 5 mm, menos 2 de apriete.
    assert objetivo_dedos(0.027) == pytest.approx(0.003)
    assert objetivo_dedos(0.001) == CERRADA     # más angosto de lo que cierra: cerrar del todo
    assert objetivo_dedos(0.2) == ABIERTA       # más ancho de lo que abre: recortado


def test_apertura_y_hay_objeto():
    assert apertura(0.0, 0.0) == pytest.approx(0.017)
    assert apertura(0.005, -0.005) == pytest.approx(0.027)
    q = objetivo_dedos(0.027)                   # se piden 23 mm de apertura
    assert hay_objeto(0.027, q)                 # se frenaron en los 27 mm del objeto
    assert not hay_objeto(0.0235, q)            # llegaron a lo pedido: no había nada
    assert not hay_objeto(apertura(CERRADA, -CERRADA), CERRADA)


def test_estable_y_promediar():
    a = analizar_objeto(nube(centro=(0.80, 0.0)))
    b = analizar_objeto(nube(centro=(0.804, 0.0)))
    lejos = analizar_objeto(nube(centro=(0.85, 0.0)))
    assert estable([a, b])
    assert not estable([a, b, lejos])
    # Un eje con el signo dado vuelta no debe anular el promedio.
    b_vuelta = dict(b, eje=-b['eje'])
    m = promediar([a, b_vuelta, a])
    assert abs(m['eje'] @ a['eje']) == pytest.approx(1.0, abs=1e-6)
    assert m['centro'][0] == pytest.approx(0.80, abs=0.003)


@pytest.mark.parametrize('direccion', [(0.47, 0.0, -0.88), (0.9, 0.2, -0.4), (1.0, 0.0, 0.0),
                                       (0.0, 1.0, -1.0)])
def test_cuaternion_de_giros_de_180_grados(direccion):
    # Una pinza hacia adelante y abajo es un giro de ~180° (w = 0). Antes del 7-nov
    # matrix_to_quaternion sacaba los signos del ruido numérico y la pose quedaba al revés.
    giro = orientacion_pinza(direccion)
    q = matrix_to_quaternion(giro)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert quaternion_to_matrix(*q)[:3, :3] == pytest.approx(giro, abs=1e-9)


@pytest.mark.parametrize('angulo', [0.0, 0.7, np.pi / 2, 2.5])
def test_cuaternion_de_la_pose_desde_arriba(angulo):
    _, giro = pose_desde_arriba([0.8, 0.0, 0.73], [np.cos(angulo), np.sin(angulo), 0.0], 0.73)
    q = matrix_to_quaternion(giro)
    assert quaternion_to_matrix(*q)[:3, :3] == pytest.approx(giro, abs=1e-9)
