"""Pruebas de grupos.py (grupos de objetos para evaluar el detector), sin ROS ni Gazebo.

python3 -m pytest -q test/test_grupos.py
Las cajas de los objetos son inventadas, con medidas parecidas a las del catálogo (un plato de
27 cm, una jarra de 28 cm de alto, un sombrero de 40 cm); la cámara y las detecciones de YOLO,
también.
"""
from pathlib import Path

import numpy as np
import pytest

from andesrobot_arm.catalogo_objetos import ALTO_MESA, leer_catalogo
from andesrobot_arm.grupos import (
    BORDE_DELANTERO, BORDE_TRASERO, GRUPOS, HUECO, MEDIO_ANCHO, acomodar, asignar, caja_de_sdf,
    centro_en_mesa, esquinas_en_mesa, iou, matriz_de_tf, normalizar, proyectar, rectangulo,
    resumir)

CATALOGO = Path(__file__).resolve().parent.parent / 'config' / 'objetos.yaml'


def caja(x, y, z, centro=(0.0, 0.0)):
    """Caja de x * y * z con la base en z = 0 y su centro en `centro` (como un modelo de YCB, que
    no tiene el origen al medio)."""
    cx, cy = centro
    return {'min': [cx - x / 2, cy - y / 2, 0.0], 'max': [cx + x / 2, cy + y / 2, z]}


def huellas(cajas, lugares):
    """Rectángulo (x0, x1, y0, y1) que ocupa cada objeto visto desde arriba, ya puesto."""
    salida = []
    for c, (x, y, giro) in zip(cajas, lugares):
        lo, hi = np.array(c['min']), np.array(c['max'])
        esquinas = np.array([[a, b] for a in (lo[0], hi[0]) for b in (lo[1], hi[1])])
        r = np.array([[np.cos(giro), -np.sin(giro)], [np.sin(giro), np.cos(giro)]])
        p = esquinas @ r.T + [x, y]
        salida.append((p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()))
    return salida


def revisar(cajas, lugares):
    """Que todos queden sobre la mesa y no se pisen."""
    h = huellas(cajas, lugares)
    for x0, x1, y0, y1 in h:
        assert BORDE_DELANTERO - 1e-6 <= x0 and x1 <= BORDE_TRASERO + 1e-6
        assert -MEDIO_ANCHO - 1e-6 <= y0 and y1 <= MEDIO_ANCHO + 1e-6
    for i in range(len(h)):
        for j in range(i + 1, len(h)):
            separados_x = h[i][1] + HUECO - 1e-6 <= h[j][0] or h[j][1] + HUECO - 1e-6 <= h[i][0]
            separados_y = h[i][3] + HUECO - 1e-6 <= h[j][2] or h[j][3] + HUECO - 1e-6 <= h[i][2]
            assert separados_x or separados_y, (i, j)


def test_grupos_existen_en_el_catalogo():
    catalogo = {o['id']: o for o in leer_catalogo(CATALOGO.read_text())}
    # los 5 por tema no repiten objetos; 'equipo' repite algunos (es otra prueba)
    todos = [i for g, ids in GRUPOS.items() if g != 'equipo' for i in ids]
    assert len(GRUPOS) == 6 and len(todos) == len(set(todos))
    todos += GRUPOS['equipo']
    assert all(i in catalogo for i in todos)


def test_alias_del_modelo_de_9_clases():
    assert normalizar('redbull') == 'redbull_can' and normalizar('rubik') == 'rubik_cube'
    assert normalizar('cup') == 'cup'


def test_cocina_bajos_adelante():
    """Plato y bandeja (bajos) adelante; jarra (alta) atrás; nada se pisa."""
    cajas = [caja(0.13, 0.10, 0.11), caja(0.16, 0.16, 0.08), caja(0.27, 0.27, 0.04),
             caja(0.17, 0.31, 0.03), caja(0.24, 0.17, 0.28, centro=(0.02, -0.01))]
    lugares = acomodar(cajas)
    revisar(cajas, lugares)
    x = [centro_en_mesa(lugar, c)[0] for lugar, c in zip(lugares, cajas)]
    assert x[2] < 1.0 and x[3] < 1.0 and x[4] > 1.0


def test_gira_lo_que_no_cabe_derecho():
    """Con un sombrero de 38 x 40 cm atrás, el martillo (29 cm de largo) solo cabe adelante si
    se gira y queda a lo ancho de la mesa."""
    cajas = [caja(0.16, 0.14, 0.09), caja(0.29, 0.06, 0.10), caja(0.14, 0.05, 0.21),
             caja(0.38, 0.40, 0.17), caja(0.20, 0.26, 0.35)]
    lugares = acomodar(cajas)
    revisar(cajas, lugares)
    assert lugares[1][2] == pytest.approx(np.pi / 2)


def test_uno_solo_y_demasiados():
    lugar, = acomodar([caja(0.1, 0.1, 0.1)])
    assert lugar[1] == pytest.approx(0.0)
    with pytest.raises(ValueError):
        acomodar([caja(0.5, 0.5, 0.1)] * 3)


def test_centro_en_mesa_con_giro():
    """Un modelo con el centro de su caja corrido 5 cm en x: girado 90°, el centro queda corrido
    5 cm en y."""
    c = caja(0.10, 0.04, 0.20, centro=(0.05, 0.0))
    assert centro_en_mesa((0.8, 0.0, 0.0), c) == pytest.approx([0.85, 0.0, ALTO_MESA + 0.103])
    assert centro_en_mesa((0.8, 0.0, np.pi / 2), c) == pytest.approx(
        [0.80, 0.05, ALTO_MESA + 0.103])


def test_matriz_de_tf():
    m = matriz_de_tf((1, 2, 3), (0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)))   # 90° en Z
    assert m @ [1, 0, 0, 1] == pytest.approx([1, 3, 3, 1])


def camara_mirando_x():
    """Cámara en el origen mirando hacia +X (frame óptico: Z adelante, X derecha = −Y del mundo,
    Y abajo = −Z del mundo): la matriz mundo -> cámara."""
    m = np.eye(4)
    m[:3, :3] = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    return m


K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])


def test_proyectar():
    puntos = [[1.0, 0.0, 0.0], [1.0, -0.1, 0.0], [-1.0, 0.0, 0.0], [1.0, 2.0, 0.0]]
    uv, vis = proyectar(puntos, camara_mirando_x(), K, 640, 480)
    assert uv[0] == pytest.approx([320, 240])
    assert uv[1] == pytest.approx([370, 240])          # −Y del mundo = a la derecha en la foto
    assert list(vis) == [True, True, False, False]     # detrás y fuera de la imagen: no


def test_esquinas_y_rectangulo():
    c = caja(0.10, 0.04, 0.20)
    e = esquinas_en_mesa((0.8, 0.0, np.pi / 2), c)
    assert e.min(axis=0) == pytest.approx([0.78, -0.05, ALTO_MESA + 0.003])
    assert e.max(axis=0) == pytest.approx([0.82, 0.05, ALTO_MESA + 0.203])
    uv = np.array([[10, 20], [700, 30], [np.nan, np.nan], [50, -5]], float)
    assert rectangulo(uv, 640, 480) == pytest.approx([10, 0, 640, 30])


def test_iou():
    assert iou([0, 0, 10, 10], [0, 0, 10, 10]) == pytest.approx(1.0)
    assert iou([0, 0, 10, 10], [5, 0, 15, 10]) == pytest.approx(1 / 3)
    assert iou([0, 0, 1, 1], [5, 5, 6, 6]) == 0.0


def test_asignar():
    """Taza bien encontrada; bowl con una caja de `cup` encima (la confusión); lata con el
    nombre corto del modelo de 9 clases; el último no estaba en la foto."""
    clases = ['cup', 'bowl', 'redbull_can', 'cup']
    rects = [[80, 80, 120, 120], [200, 200, 400, 400], [490, 90, 510, 110], [0, 0, 0, 0]]
    vis = [True, True, True, False]
    cajas = [[82, 80, 120, 122], [210, 190, 400, 390], [488, 90, 510, 112]]
    r = asignar(clases, rects, vis, cajas, ['cup', 'cup', 'redbull'], [0.8, 0.3, 0.6])
    assert r[0] == {'visible': True, 'correcta': 0.8, 'otra': None}
    assert r[1]['correcta'] is None and r[1]['otra'] == ('cup', 0.3)
    assert r[2]['correcta'] == pytest.approx(0.6)
    assert r[3] == {'visible': False, 'correcta': None, 'otra': None}
    assert asignar(['cup'], rects[:1], [True], [], [], [])[0]['otra'] is None


def test_caja_sobre_media_mesa_no_cuenta():
    """Una caja `clothing` del tamaño de la foto no se superpone bien con nadie: no cuenta."""
    rects = [[80, 80, 120, 120], [280, 280, 320, 320]]
    r = asignar(['cup', 'fork'], rects, [True, True], [[0, 0, 640, 480], [80, 80, 120, 120]],
                ['clothing', 'cup'], [0.3, 0.7])
    assert r[0]['correcta'] == pytest.approx(0.7)
    assert r[1] == {'visible': True, 'correcta': None, 'otra': None}


def test_objeto_grande_con_otro_delante():
    """La bolsa (grande, atrás) tiene el Rubik delante, dentro de su rectángulo: la caja `bag`
    es de la bolsa y la caja chica `card`, del Rubik."""
    rects = [[0, 0, 200, 300], [20, 200, 80, 260]]
    r = asignar(['bag', 'rubik_cube'], rects, [True, True],
                [[5, 0, 210, 290], [20, 198, 82, 262]], ['bag', 'card'], [0.5, 0.5])
    assert r[0]['correcta'] == pytest.approx(0.5)
    assert r[1]['otra'] == ('card', 0.5)


def test_resumir():
    vistas = [{'visible': True, 'correcta': 0.4, 'otra': None},
              {'visible': True, 'correcta': None, 'otra': ('plate', 0.3)},
              {'visible': False, 'correcta': None, 'otra': None},
              {'visible': True, 'correcta': 0.7, 'otra': None}]
    assert resumir(vistas) == {'vistas': 3, 'aciertos': 2, 'mejor': 0.7,
                               'confusiones': {'plate': 1}}


def test_caja_de_sdf():
    """La tarjeta del equipo: una caja de 4 x 85.6 x 54 mm con la base en z = 0."""
    sdf = ('<model><link name="l"><visual name="v"><pose>0 0 0.027 0 0 0</pose><geometry><box>'
           '<size>0.004 0.0856 0.054</size></box></geometry></visual><collision name="c"><pose>'
           '0 0 0.027 0 0 0</pose><geometry><box><size>0.004 0.0856 0.054</size></box>'
           '</geometry></collision></link></model>')
    c = caja_de_sdf(sdf)
    assert c['min'] == pytest.approx([-0.002, -0.0428, 0.0])
    assert c['max'] == pytest.approx([0.002, 0.0428, 0.054])
