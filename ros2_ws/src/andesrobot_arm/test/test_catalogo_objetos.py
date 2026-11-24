# Pruebas de catalogo_objetos.py y de config/objetos.yaml: elegir objetos, repartirlos en la mesa,
# medir mallas y armar el SDF (sin ROS ni Gazebo).
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_catalogo_objetos.py
import os

import numpy as np
import pytest

from andesrobot_arm.catalogo_objetos import (
    ALTO_MESA, altura_de_spawn, carpeta, corregir_uris, elegir, inercia_caja, leer_catalogo,
    medidas, posiciones_en_mesa, tenir_mtl, vertices_obj)

CATALOGO = os.path.join(os.path.dirname(__file__), '..', 'config', 'objetos.yaml')
CLASES = {'bag', 'book', 'bottle', 'bowl', 'box', 'card', 'cereal_box', 'cleaning_supply',
          'clothing', 'colgate_box', 'cup', 'drink', 'electronic_device', 'food', 'fork', 'fruit',
          'key', 'knife', 'pitcher', 'plate', 'pringles', 'redbull_can', 'rubik_cube', 'spoon',
          'tool', 'tray', 'writing_tool'}


@pytest.fixture(scope='module')
def catalogo():
    with open(CATALOGO) as f:
        return leer_catalogo(f.read())


def test_catalogo_bien_formado(catalogo):
    ids = [o['id'] for o in catalogo]
    assert len(ids) == len(set(ids)), 'ids repetidos'
    for o in catalogo:
        assert o['clase'] in CLASES, o['id']
        assert o['fuente'] in ('fuel', 'ycb', 'equipo'), o['id']
        assert 0 < o['masa'] < 2, o['id']
        assert o['fuente'] != 'fuel' or o.get('dueno'), o['id']
        # Lo que se republica en el release tiene que tener licencia y autor anotados.
        assert not o.get('release') or (o.get('licencia') and o.get('autor')), o['id']
    # Las 24 clases que tienen modelo (faltan colgate_box, key y drink; redbull_can llegó con
    # los modelos del equipo el 9-nov).
    sin_modelo = {'colgate_box', 'key', 'drink'}
    assert CLASES - {o['clase'] for o in catalogo} == sin_modelo


def test_elegir_por_id_por_clase_y_true(catalogo):
    assert [o['id'] for o in elegir(catalogo, 'true')] == ['taza_blanca', 'bowl_blanco']
    assert elegir(catalogo, 'false') == [] and elegir(catalogo, '') == []
    # Por clase toma el primero de esa clase; un repetido no se pone dos veces.
    assert [o['id'] for o in elegir(catalogo, 'platano, fruit,cup,platano')] == [
        'platano', 'taza_blanca']
    # Con dos modelos de una clase, la clase elige el primero del catálogo: el del equipo.
    assert [o['id'] for o in elegir(catalogo, 'pringles,card,bag')] == [
        'pringles_equipo', 'tarjeta_equipo', 'bolsa_equipo']
    with pytest.raises(ValueError, match='hay:'):
        elegir(catalogo, 'llave')


def test_posiciones_en_mesa():
    assert posiciones_en_mesa(1) == [(0.80, 0.0)]
    p = posiciones_en_mesa(7)
    assert [x for x, _ in p] == [0.80] * 5 + [0.95] * 2
    ys = [y for _, y in p[:5]]
    assert ys[0] == pytest.approx(-0.28) and ys[-1] == pytest.approx(0.28)
    assert all(abs(y) <= 0.28 + 1e-9 for _, y in p)
    with pytest.raises(ValueError):
        posiciones_en_mesa(11)


def test_medidas_y_altura_de_spawn():
    texto = '# malla\nv -0.05 -0.02 -0.10\nv 0.05 0.02 0.15\nvt 0.1 0.2\nf 1 2 2\n'
    m = medidas(vertices_obj(texto))
    assert m['tam'] == pytest.approx([0.10, 0.04, 0.25])
    # El origen queda 10 cm sobre la base: hay que ponerlo 10 cm (+3 mm) sobre el tablero.
    assert altura_de_spawn(m) == pytest.approx(ALTO_MESA + 0.10 + 0.003)
    assert altura_de_spawn(None) == pytest.approx(ALTO_MESA + 0.003)


def test_inercia_caja():
    ixx, iyy, izz = inercia_caja(1.2, (0.1, 0.2, 0.3))
    assert ixx == pytest.approx(1.2 * (0.04 + 0.09) / 12)
    assert izz == pytest.approx(1.2 * (0.01 + 0.04) / 12)
    assert np.all(np.array(inercia_caja(0.02, (0.2, 0.0, 0.0))) > 0)   # nunca cero


def test_carpeta_y_uris():
    assert carpeta({'nombre': 'Coke Can'}) == 'Coke_Can'
    sdf = ('<uri>meshes/a.dae</uri><uri>model://coke_can/meshes/b.dae</uri>'
           '<uri> https://fuel/x.dae </uri>')
    assert corregir_uris(sdf, 'Coke_Can') == (
        '<uri>model://Coke_Can/meshes/a.dae</uri><uri>model://Coke_Can/meshes/b.dae</uri>'
        '<uri> https://fuel/x.dae </uri>')


def test_carpeta_propia_y_color(catalogo):
    """Dos versiones del mismo modelo (la bolsa de tela crema y café papel) en carpetas distintas;
    tenir_mtl cambia el color difuso y el ambiente, y agrega el que falte."""
    tela = next(o for o in catalogo if o['id'] == 'bolsa_tela')
    papel = next(o for o in catalogo if o['id'] == 'bolsa_papel')
    assert tela['nombre'] == papel['nombre'] and carpeta(tela) != carpeta(papel)
    assert len({carpeta(o) for o in catalogo}) == len(catalogo)
    mtl = tenir_mtl('newmtl m\nKa 0.4 0.4 0.4\nKd 0.9 0.9 0.8\nNs 1\n', (0.7, 0.5, 0.3))
    assert 'Kd 0.700000 0.500000 0.300000' in mtl and 'Ka 0.700000 0.500000 0.300000' in mtl
    assert 'Ka 0.700000' in tenir_mtl('newmtl m\nKd 1 1 1\n', (0.7, 0.5, 0.3))
