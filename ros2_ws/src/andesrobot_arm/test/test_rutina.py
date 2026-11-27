"""Pruebas de rutina.py (grabar y reproducir), sin ROS ni ventana.

python3 -m pytest -q test/test_rutina.py
"""
import math

import pytest

from andesrobot_arm.rutina import cargar, guardar, nuevo_punto, tiempos, tramos

ABIERTA, CERRADA = 0.016, -0.007


def p(a1, garra, nombre=''):
    return nuevo_punto([a1, 0, 0, 0, 0, 0], garra, nombre)


def test_tramos_agarrar_y_llevar():
    """Arriba, bajar, cerrar en el punto bajo, subir, ir a dejarlo, abrir: el brazo llega a cada
    punto donde cambia la garra antes de moverla."""
    rutina = [p(0.0, ABIERTA, 'arriba'), p(0.1, ABIERTA, 'abajo'), p(0.1, CERRADA, 'cerrar'),
              p(0.0, CERRADA, 'subir'), p(0.5, CERRADA, 'dejar'), p(0.5, ABIERTA, 'soltar')]
    pasos = tramos(rutina, garra_inicial=ABIERTA)
    assert [tipo for tipo, _ in pasos] == ['brazo', 'garra', 'brazo', 'garra']
    assert [q[0] for q in pasos[0][1]] == [0.0, 0.1, 0.1]
    assert pasos[1][1] == CERRADA
    assert [q[0] for q in pasos[2][1]] == [0.0, 0.5, 0.5]
    assert pasos[3][1] == ABIERTA


def test_tramos_sin_cambios_de_garra():
    pasos = tramos([p(0.0, ABIERTA), p(0.2, ABIERTA)], garra_inicial=ABIERTA)
    assert pasos == [('brazo', [[0.0, 0, 0, 0, 0, 0], [0.2, 0, 0, 0, 0, 0]])]


def test_tramos_cambio_pequeno_no_cuenta():
    """Menos de 1 mm de diferencia (ruido de la medición) no es un cambio de garra."""
    pasos = tramos([p(0.0, ABIERTA - 0.0005)], garra_inicial=ABIERTA)
    assert [tipo for tipo, _ in pasos] == ['brazo']


def test_tiempos_acumulados():
    """0 -> 30° -> 30° a 0.1 rad/s: el primer tramo 5.24 s, el segundo (sin moverse) el mínimo."""
    t = tiempos([0] * 6, [[math.radians(30), 0, 0, 0, 0, 0]] * 2, 0.1, minimo=1.0)
    assert t[0] == pytest.approx(math.radians(30) / 0.1)
    assert t[1] == pytest.approx(t[0] + 1.0)


def test_guardar_y_cargar(tmp_path):
    rutina = [p(0.1, ABIERTA, 'arriba'), p(0.2, CERRADA, 'cerrar')]
    ruta = tmp_path / 'r.yaml'
    guardar(ruta, rutina, nota='prueba')
    assert cargar(ruta) == rutina


def test_punto_con_angulos_de_mas():
    with pytest.raises(ValueError):
        nuevo_punto([0] * 7, ABIERTA)
