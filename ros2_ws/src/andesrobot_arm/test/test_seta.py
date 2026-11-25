"""Pruebas de seta.py (seta de emergencia por GPIO), sin ROS ni Jetson: lecturas inventadas.

python3 -m pytest -q test/test_seta.py
"""
import time

from andesrobot_arm.seta import Antirrebote, Seta


def test_arranca_presionada_hasta_leer_un_uno():
    """Falla segura: sin ninguna lectura buena, el brazo no se mueve; un 0 suelto tampoco
    la suelta."""
    f = Antirrebote(antirrebote=3, fallo=1.0, ahora=0.0)
    assert f.presionada
    assert f.leer('0', 0.05)
    assert not f.leer('1', 0.10)
    assert f.estado == 'suelta'


def test_antirrebote():
    """Hacen falta 3 ceros seguidos para darla por presionada; un 1 en medio reinicia."""
    f = Antirrebote(antirrebote=3, fallo=1.0, ahora=0.0)
    f.leer('1', 0.0)
    assert not f.leer('0', 0.05) and not f.leer('0', 0.10)
    assert not f.leer('1', 0.15)                  # ruido: vuelve a contar
    assert not f.leer('0', 0.20) and not f.leer('0', 0.25)
    assert f.leer('0', 0.30)
    assert f.estado == 'PRESIONADA'
    assert not f.leer('1', 0.35)                  # soltarla: una lectura en 1 basta


def test_sin_lectura_es_presionada():
    """Si el pin no se lee durante `fallo` s, se toma como presionada."""
    f = Antirrebote(antirrebote=3, fallo=1.0, ahora=0.0)
    f.leer('1', 0.0)
    assert not f.leer(None, 0.5)                  # todavía dentro del margen
    assert f.leer(None, 1.2)
    assert f.estado == 'SIN LECTURA'


def test_hilo_avisa_los_cambios():
    """El hilo llama al_cambiar al empezar y en cada cambio, no en cada lectura."""
    lecturas = iter(['1'] * 3 + ['0'] * 4 + ['1'] * 50)
    cambios = []
    s = Seta(periodo=0.005, antirrebote=3, leer=lambda chip, linea: (next(lecturas, '1'), ''),
             al_cambiar=lambda presionada, estado, error: cambios.append(presionada))
    s.start()
    time.sleep(0.4)
    s.parar()
    s.join(1.0)
    assert cambios == [False, True, False]
