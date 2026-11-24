"""Pruebas de bolsa.py (tomar una bolsa por el asa a partir de los 3 puntos clave del modelo). P17.

No necesitan ROS: python3 -m pytest -q test/test_bolsa.py
La escena sintética imita la bolsa de tela cruda del equipo (asas.json: 10 x 36 x 38 cm, asa en
arco de 20 cm entre bases que sobresale 24 cm, cinta de 25 x 4 mm) parada en el suelo frente a
Milo, vista por una cámara como la de la muñeca desde ~0.5 m. La imagen de profundidad se DIBUJA
punto por punto (cada pixel guarda lo más cercano que cae en él, como una cámara real), y los
puntos clave son la proyección de la punta y las bases verdaderas, corridos unos píxeles como
los daría el modelo. Sin el ruido ni los huecos de una cámara real.
"""
import numpy as np
import pytest

from andesrobot_arm.agarre import PUNTA_DEDO, estable
from andesrobot_arm.bolsa import (
    asa_de_puntos, elegir_asa, nube_de_profundidad, pose_de_agarre, promediar_asas, proyectar,
    punto_del_pixel)

FONDO, ANCHO, ALTO = 0.10, 0.36, 0.38          # bolsa de tela cruda [m]
R_ASA, H_ASA, CINTA, GRUESO = 0.10, 0.24, 0.025, 0.004
K = np.array([[554.0, 0.0, 320.0], [0.0, 554.0, 240.0], [0.0, 0.0, 1.0]])   # 640 x 480, 60°


def arco(x_pared, paso=0.001):
    """Puntos (superficie densa) de un asa en arco en el plano x = x_pared, frame de la bolsa
    (x = fondo, y = ancho, z desde el piso). Devuelve (puntos, base izquierda, punta, base
    derecha). La base "izquierda" es la de +Y (la izquierda de Milo) como la vería la cámara."""
    puntos = []
    for t in np.arange(0.0, np.pi + 1e-9, paso / max(R_ASA, H_ASA)):
        c = np.array([x_pared, R_ASA * np.cos(t), ALTO + H_ASA * np.sin(t)])
        for g in np.arange(-GRUESO / 2, GRUESO / 2 + 1e-9, 0.002):
            for w in np.arange(-CINTA / 2, CINTA / 2 + 1e-9, 0.002):
                # la cinta: ancha en el plano del arco (hacia afuera del arco), delgada en x
                normal = np.array([0.0, np.cos(t), np.sin(t) * R_ASA / H_ASA])
                normal /= np.linalg.norm(normal)
                puntos.append(c + [g, 0, 0] + w * normal * 0.5)
    izq = np.array([x_pared, R_ASA, ALTO])
    der = np.array([x_pared, -R_ASA, ALTO])
    punta = np.array([x_pared, 0.0, ALTO + H_ASA])
    return np.array(puntos), izq, punta, der


def escena(centro=(0.60, 0.0), angulo=0.0, asa='parada'):
    """Puntos del mundo (base_footprint) y los 3 puntos clave verdaderos del asa de la pared
    cercana a Milo. asa: 'parada', 'doblada' (caída 70° hacia la bolsa) o 'ninguna'."""
    p = []
    for x in np.arange(-0.5, 0.5, 0.003):                  # suelo alrededor
        for y in np.arange(-0.5, 0.5, 0.003):
            p.append([centro[0] + x, centro[1] + y, 0.0])
    caja = []
    for z in np.arange(0.0, ALTO, 0.003):                  # paredes
        for y in np.arange(-ANCHO / 2, ANCHO / 2, 0.003):
            caja += [[-FONDO / 2, y, z], [FONDO / 2, y, z]]
        for x in np.arange(-FONDO / 2, FONDO / 2, 0.003):
            caja += [[x, -ANCHO / 2, z], [x, ANCHO / 2, z]]
    caja = [np.array(caja)]
    clave = None
    if asa != 'ninguna':
        for x_pared in (-FONDO / 2, FONDO / 2):
            puntos, izq, punta, der = arco(x_pared)
            if asa == 'doblada':           # girada alrededor de la línea de las bases
                a = np.radians(70.0) * np.sign(-x_pared)
                giro = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
                eje = np.array([x_pared, 0.0, ALTO])
                puntos = (puntos - eje) @ giro.T + eje
                punta = giro @ (punta - eje) + eje
            caja.append(puntos)
            if x_pared < 0:                # la pared cercana a Milo (x menor)
                clave = (izq, punta, der)
    caja = np.vstack(caja)
    c, s = np.cos(angulo), np.sin(angulo)
    giro_z = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    mover = np.array([centro[0], centro[1], 0.0])
    caja = caja @ giro_z.T + mover
    if clave is not None:
        clave = tuple(giro_z @ q + mover for q in clave)
    return np.vstack([np.array(p), caja]), clave


def camara(posicion=(0.30, 0.0, 0.95), mira=(0.60, 0.0, 0.45)):
    """(rot, pos) de la cámara óptica (Z adelante, X derecha, Y abajo) en base_footprint."""
    z = np.asarray(mira, float) - posicion
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z]), np.asarray(posicion, float)


def dibujar(puntos_mundo, rot, pos, alto=480, ancho=640):
    """Imagen de profundidad: cada píxel guarda la z más cercana que cae en él (z-buffer)."""
    pc = (puntos_mundo - pos) @ rot
    pc = pc[pc[:, 2] > 0.05]
    u = np.round(K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]).astype(int)
    v = np.round(K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]).astype(int)
    ok = (u >= 0) & (u < ancho) & (v >= 0) & (v < alto)
    img = np.full(alto * ancho, np.inf)
    np.minimum.at(img, v[ok] * ancho + u[ok], pc[ok, 2])
    img[np.isinf(img)] = 0.0
    return img.reshape(alto, ancho)


def pixel(p_mundo, rot, pos, corrido=(0.0, 0.0)):
    pc = rot.T @ (p_mundo - pos)
    return (K[0, 0] * pc[0] / pc[2] + K[0, 2] + corrido[0],
            K[1, 1] * pc[1] / pc[2] + K[1, 2] + corrido[1])


def medir(mundo, clave, rot, pos, corrido=(0.0, 0.0)):
    """Lo que hace bolsa_node con una imagen: nube, proyección y los 3 puntos 3D -> asa."""
    nube = nube_de_profundidad(dibujar(mundo, rot, pos), K)
    uvz = proyectar(nube, K, np.eye(3), np.zeros(3))      # color = profundidad en la prueba
    puntos = [punto_del_pixel(nube, uvz, pixel(q, rot, pos, corrido)) for q in clave]
    assert all(p is not None for p in puntos)
    return asa_de_puntos(*[rot @ p + pos for p in puntos])


@pytest.fixture(scope='module')
def parada():
    return escena()


def test_asa_parada(parada):
    """Bolsa a 0.60 m: el vértice sale a menos de 1 cm del verdadero y el eje va a lo largo de Y
    (el ancho de la bolsa), aunque el modelo se equivoque 2 px en la punta."""
    mundo, clave = parada
    rot, pos = camara()
    a, problema = medir(mundo, clave, rot, pos, corrido=(2.0, -2.0))
    assert problema == ''
    assert np.linalg.norm(a['centro'] - clave[1]) < 0.01
    assert abs(a['eje'][1]) == pytest.approx(1.0, abs=0.02)
    assert a['alto'] == pytest.approx(H_ASA, abs=0.02)
    assert a['largo'] == pytest.approx(2 * R_ASA, abs=0.02)


def test_la_capa_de_adelante_es_el_asa(parada):
    """Con el píxel exacto de la punta corrido 3 px, el píxel mismo puede ver el suelo de atrás
    (a ~0.9 m); punto_del_pixel igual devuelve el asa (~0.5 m), no el suelo."""
    mundo, clave = parada
    rot, pos = camara()
    nube = nube_de_profundidad(dibujar(mundo, rot, pos), K)
    uvz = proyectar(nube, K, np.eye(3), np.zeros(3))
    p = punto_del_pixel(nube, uvz, pixel(clave[1], rot, pos, (0.0, -3.0)))
    assert np.linalg.norm(rot @ p + pos - clave[1]) < 0.01
    lejos = punto_del_pixel(nube, uvz, (1000.0, 1000.0))      # fuera de la imagen
    assert lejos is None


def test_bolsa_girada():
    mundo, clave = escena(centro=(0.55, 0.08), angulo=np.radians(30))
    rot, pos = camara(mira=(0.55, 0.08, 0.45))
    a, problema = medir(mundo, clave, rot, pos)
    assert problema == ''
    eje = np.array([-np.sin(np.radians(30)), np.cos(np.radians(30)), 0.0])
    assert abs(a['eje'] @ eje) == pytest.approx(1.0, abs=0.02)
    assert np.linalg.norm(a['centro'] - clave[1]) < 0.01


def test_asa_doblada_se_rechaza():
    mundo, clave = escena(asa='doblada')
    rot, pos = camara()
    a, problema = medir(mundo, clave, rot, pos)
    assert a is None and 'doblada' in problema


def test_validaciones():
    izq, punta, der = [0.6, 0.1, 0.38], [0.6, 0.0, 0.62], [0.6, -0.1, 0.38]
    assert asa_de_puntos(izq, punta, der)[1] == ''
    assert 'sobresale' in asa_de_puntos(izq, [0.6, 0.0, 0.395], der)[1]     # aplastada
    assert 'bases' in asa_de_puntos([0.6, 0.01, 0.38], punta, [0.6, -0.01, 0.38])[1]
    assert 'desnivel' in asa_de_puntos(izq, punta, [0.6, -0.1, 0.20])[1]    # tumbada
    # Una base 8 cm más arriba en el arco (lo hace el modelo): sirve, y la boca es la más baja.
    a, problema = asa_de_puntos([0.6, 0.09, 0.46], punta, der)
    assert problema == '' and a['boca_z'] == pytest.approx(0.38)
    assert asa_de_puntos(izq, punta, izq)[0] is None


def test_elegir_la_mas_cercana():
    cerca = asa_de_puntos([0.55, 0.1, 0.38], [0.55, 0.0, 0.62], [0.55, -0.1, 0.38])[0]
    lejos = asa_de_puntos([0.65, 0.1, 0.38], [0.65, 0.0, 0.62], [0.65, -0.1, 0.38])[0]
    e = elegir_asa([lejos, cerca])
    assert e['centro'][0] == pytest.approx(0.55) and e['n_asas'] == 2
    assert elegir_asa([]) is None


def test_pose_de_agarre(parada):
    mundo, clave = parada
    a, _ = asa_de_puntos(*clave)
    posicion, giro = pose_de_agarre(a, metido=0.012)
    assert posicion[2] == pytest.approx(a['centro'][2] + PUNTA_DEDO - 0.012)
    assert giro[:, 0] == pytest.approx([0.0, 0.0, 1.0])      # la pinza apunta hacia abajo (−X)
    assert abs(giro[:, 1] @ a['eje']) < 1e-9                  # los dedos cierran cruzados al asa
    assert np.linalg.det(giro) == pytest.approx(1.0)


def test_promediar_y_estable():
    a, _ = asa_de_puntos([0.55, 0.1, 0.38], [0.55, 0.0, 0.62], [0.55, -0.1, 0.38])
    b = dict(a, centro=a['centro'] + [0.004, 0, 0], eje=-a['eje'])
    m = promediar_asas([a, b, a])
    assert m['centro'] == pytest.approx(a['centro'])
    assert abs(m['eje'] @ a['eje']) == pytest.approx(1.0)
    assert estable([a, b, a], tol=0.01)


def test_nube_y_proyeccion():
    """Una pared plana a 1 m: la nube sale a 1 m y cada punto vuelve a su píxel. Con las cámaras
    separadas 2.5 cm a lo largo de X, la imagen a color ve todo corrido 0.025 · 554 / 1 = 13.85
    px."""
    prof = np.full((480, 640), 1.0)
    prof[:10, :] = 0.0                                     # filas sin dato
    p = nube_de_profundidad(prof, K, paso=4)
    assert np.allclose(p[:, 2], 1.0)
    assert len(p) == (480 - 12) // 4 * 640 // 4
    uvz = proyectar(p, K, np.eye(3), np.zeros(3))
    assert uvz[0, 0] == pytest.approx(0.0) and uvz[0, 1] == pytest.approx(12.0)
    corrida = proyectar(p, K, np.eye(3), np.array([-0.025, 0.0, 0.0]))
    assert corrida[0, 0] - uvz[0, 0] == pytest.approx(0.025 * 554.0)
