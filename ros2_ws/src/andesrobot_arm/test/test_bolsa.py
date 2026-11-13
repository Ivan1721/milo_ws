"""Pruebas de bolsa.py (encontrar el asa de una bolsa) con bolsas sintéticas. BORRADOR, P17.

No necesitan ROS: python3 -m pytest -q test/test_bolsa.py
La "bolsa" es como la de papel de referencia (315 x 160 x 410 mm), vista como nube de puntos: sus
cuatro paredes, el suelo (o la mesa) alrededor y las asas, cada asa una U invertida de 12 cm de
ancho que sobresale 9 cm de la boca, con 6 mm de grosor. Es lo que darían los puntos de la máscara
de la bolsa con su profundidad, sin el ruido ni los huecos de una cámara real.
"""
import numpy as np
import pytest

from andesrobot_arm.agarre import PUNTA_DEDO, estable
from andesrobot_arm.bolsa import (
    altura_de_la_boca, buscar_asa, en_la_mascara, nube_de_profundidad, pose_de_agarre,
    promediar_asas, separar_asas)

ANCHO, FONDO, ALTO = 0.315, 0.16, 0.41
ALTO_ASA, ANCHO_ASA, GROSOR = 0.09, 0.12, 0.006


def asa(y_pared, paso=0.004):
    """Puntos de una U invertida en el plano x = y_pared (coordenada a lo largo del fondo), en el
    frame de la bolsa (x = fondo, y = ancho, z desde la boca)."""
    puntos = []
    for g in np.arange(-GROSOR / 2, GROSOR / 2 + 1e-9, 0.002):
        for z in np.arange(0.0, ALTO_ASA, paso):           # las dos patas
            puntos += [[y_pared + g, -ANCHO_ASA / 2, z], [y_pared + g, ANCHO_ASA / 2, z]]
        for y in np.arange(-ANCHO_ASA / 2, ANCHO_ASA / 2 + 1e-9, paso):   # el tramo de arriba
            puntos.append([y_pared + g, y, ALTO_ASA])
    return np.array(puntos)


def bolsa(centro=(0.60, 0.0), angulo=0.0, piso=0.0, asas='dos', paso=0.005):
    """(puntos de la bolsa, puntos del suelo alrededor) en base_footprint. La bolsa está girada
    `angulo` [rad] en Z; su fondo (16 cm) va a lo largo de X cuando angulo = 0.
    asas: 'dos' (una en cada pared, a 14 cm), 'juntas' (las dos en el medio), 'ninguna'."""
    paredes = []
    for z in np.arange(0.0, ALTO, paso):
        for y in np.arange(-ANCHO / 2, ANCHO / 2, paso):
            paredes += [[-FONDO / 2, y, z], [FONDO / 2, y, z]]
        for x in np.arange(-FONDO / 2, FONDO / 2, paso):
            paredes += [[x, -ANCHO / 2, z], [x, ANCHO / 2, z]]
    p = [np.array(paredes)]
    if asas == 'dos':
        p += [asa(-0.07) + [0, 0, ALTO], asa(0.07) + [0, 0, ALTO]]
    elif asas == 'juntas':
        p += [asa(0.0) + [0, 0, ALTO]]
    p = np.vstack(p)
    c, s = np.cos(angulo), np.sin(angulo)
    giro = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    p = p @ giro.T + [centro[0], centro[1], piso]
    g = np.arange(-0.30, 0.30, 0.01)
    suelo = np.array([[centro[0] + a, centro[1] + b, piso] for a in g for b in g
                      if abs(a) > FONDO / 2 + 0.02 or abs(b) > ANCHO / 2 + 0.02])
    return p, suelo


@pytest.mark.parametrize('piso', [0.0, 0.72])
def test_boca_de_la_bolsa(piso):
    p, _ = bolsa(piso=piso)
    assert altura_de_la_boca(p[:, 2]) == pytest.approx(piso + ALTO, abs=0.012)


@pytest.mark.parametrize('piso', [0.0, 0.72])
def test_encuentra_el_asa_mas_cercana(piso):
    """Bolsa en el suelo o en la mesa: se elige el asa de la pared cercana a Milo (x = 0.53)."""
    p, _ = bolsa(piso=piso)
    a, problema = buscar_asa(p)
    assert problema == ''
    assert a['n_asas'] == 2
    assert a['centro'][0] == pytest.approx(0.53, abs=0.005)
    assert a['centro'][1] == pytest.approx(0.0, abs=0.01)
    assert a['centro'][2] == pytest.approx(piso + ALTO + ALTO_ASA, abs=0.01)
    assert abs(a['eje'][1]) == pytest.approx(1.0, abs=0.01)       # el asa va a lo largo de Y
    assert a['alto'] == pytest.approx(ALTO_ASA, abs=0.015)
    assert a['largo'] == pytest.approx(ANCHO_ASA, abs=0.01)
    assert a['grosor'] < 0.012


def test_bolsa_girada():
    p, _ = bolsa(centro=(0.55, 0.10), angulo=np.radians(30))
    a, problema = buscar_asa(p)
    assert problema == ''
    eje = np.array([-np.sin(np.radians(30)), np.cos(np.radians(30)), 0.0])
    assert abs(a['eje'] @ eje) == pytest.approx(1.0, abs=0.01)


def test_asas_juntas_son_una():
    p, _ = bolsa(asas='juntas')
    a, problema = buscar_asa(p)
    assert problema == '' and a['n_asas'] == 1
    assert a['centro'][0] == pytest.approx(0.60, abs=0.005)


def test_mascara_sin_asa_usa_toda_la_imagen():
    """Si el modelo dejó el asa fuera de la máscara, se busca en todos los puntos, dentro de la
    huella de la bolsa (el suelo de alrededor no cuenta)."""
    p, suelo = bolsa()
    cuerpo = p[p[:, 2] <= ALTO]
    a, problema = buscar_asa(cuerpo, todos=np.vstack([p, suelo]))
    assert problema == ''
    assert a['centro'][2] == pytest.approx(ALTO + ALTO_ASA, abs=0.01)
    assert buscar_asa(cuerpo)[0] is None                 # solo con la máscara, no hay asa


def test_sin_asa_o_tumbada():
    p, _ = bolsa(asas='ninguna')
    a, problema = buscar_asa(p)
    assert a is None and 'sobresale' in problema


def test_algo_lejos_sobre_la_boca_no_es_asa():
    """Un objeto a la altura del asa pero fuera de la huella de la bolsa (un mueble detrás) no se
    cuenta como asa."""
    p, _ = bolsa(asas='ninguna')
    mueble = np.array([[0.90, y, ALTO + 0.05] for y in np.arange(-0.1, 0.1, 0.005)])
    assert buscar_asa(p, todos=np.vstack([p, mueble]))[0] is None


def test_asa_baja_se_rechaza():
    p, _ = bolsa()
    p = p[p[:, 2] <= ALTO + 0.015]                        # asas aplastadas: 15 mm sobre la boca
    a, problema = buscar_asa(p, minimo=5)
    assert a is None


def test_separar_asas():
    dos = np.vstack([asa(-0.07), asa(0.07)])
    assert len(separar_asas(dos)) == 2
    assert len(separar_asas(asa(0.0))) == 1


def test_pose_de_agarre():
    p, _ = bolsa()
    a, _ = buscar_asa(p)
    posicion, giro = pose_de_agarre(a, metido=0.012)
    assert posicion[2] == pytest.approx(a['centro'][2] + PUNTA_DEDO - 0.012)
    assert giro[:, 0] == pytest.approx([0.0, 0.0, 1.0])      # la pinza apunta hacia abajo (−X)
    assert abs(giro[:, 1] @ a['eje']) < 1e-9                  # los dedos cierran cruzados al asa
    assert np.linalg.det(giro) == pytest.approx(1.0)


def test_promediar_y_estable():
    p, _ = bolsa()
    a, _ = buscar_asa(p)
    b = dict(a, centro=a['centro'] + [0.004, 0, 0], eje=-a['eje'])
    m = promediar_asas([a, b, a])
    assert m['centro'] == pytest.approx(a['centro'])
    assert abs(m['eje'] @ a['eje']) == pytest.approx(1.0)
    assert estable([a, b, a], tol=0.01)


def test_nube_y_mascara():
    """Una pared plana a 1 m: la nube sale a 1 m, y con la máscara de la mitad izquierda de la
    imagen quedan los puntos con x < 0 (X óptico = a la derecha). Con las cámaras corridas 2.5 cm,
    la mitad se corre 2.5 cm."""
    k = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
    prof = np.full((480, 640), 1.0)
    prof[:10, :] = 0.0                                     # filas sin dato
    p = nube_de_profundidad(prof, k, paso=4)
    assert np.allclose(p[:, 2], 1.0)
    assert len(p) == (480 - 12) // 4 * 640 // 4
    mascara = np.zeros((480, 640), bool)
    mascara[:, :320] = True
    dentro = en_la_mascara(p, mascara, k, np.eye(3), np.zeros(3))
    assert np.all(p[dentro, 0] < 0.001) and np.all(p[~dentro, 0] >= -0.001)
    dentro = en_la_mascara(p, mascara, k, np.eye(3), np.array([0.025, 0, 0]))
    centro = np.abs(p[:, 0]) < 0.5         # los del borde caen fuera de la imagen a color
    assert np.all(p[dentro, 0] < 0.026)
    assert np.all(p[~dentro & centro, 0] >= 0.024)
