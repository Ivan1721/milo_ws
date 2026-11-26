"""Pruebas de brazo_real.py (conversiones y protocolo del Arduino del brazo), sin ROS ni Arduino.

python3 -m pytest -q test/test_brazo_real.py
El ArduinoFalso hace lo mismo que firmware/brazo_mega/brazo_mega.ino; aquí se le pasa un reloj
falso para poder "adelantar" el tiempo.
"""
import math
import time

import pytest

from andesrobot_arm.brazo_real import (
    DEDO_ABIERTA, DEDO_CERRADA, ArduinoFalso, Articulacion, cmd_ir, cmd_mover, dedo_a_servo,
    interpolar, leer_config, leer_estado, servo_a_dedo)


def test_pasos_por_grado():
    """200 pasos x 1/8 x 38.4 = 61440 pasos por vuelta: 170.67 por grado."""
    a = Articulacion('joint_1')
    assert a.pasos_por_rad * math.pi / 180 == pytest.approx(170.6667, abs=1e-3)
    assert a.a_pasos(math.radians(90)) == 15360
    assert a.a_rad(15360) == pytest.approx(math.pi / 2)


def test_invertir_y_cero():
    a = Articulacion('joint_2', invertir=True, cero=0.5)
    assert a.a_pasos(0.5) == 0
    assert a.a_pasos(0.6) < 0
    assert a.a_rad(a.a_pasos(1.234)) == pytest.approx(1.234, abs=1e-4)


def test_leer_config():
    arts = leer_config({'comun': {'micropasos': 16},
                        'articulaciones': {'joint_3': {'invertir': True, 'micropasos': 4}}})
    assert [a.nombre for a in arts] == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5',
                                        'joint_6']
    assert arts[0].micropasos == 16 and arts[2].micropasos == 4 and arts[2].invertir


def test_comandos_y_estado():
    assert cmd_ir([1, -2, 3, 0, 0, 5], 50.4) == 'G 50 1 -2 3 0 0 5'
    assert cmd_mover(1, -171) == 'M 1 -171'
    with pytest.raises(ValueError):
        cmd_mover(7, 10)
    with pytest.raises(ValueError):
        cmd_ir([1, 2], 50)
    assert leer_estado('E 1 2 3 4 5 -6 1 -1') == ([1, 2, 3, 4, 5, -6], True, None)
    assert leer_estado('E 0 0 0 0 0 0 0 90')[2] == 90
    with pytest.raises(ValueError):
        leer_estado('OK G')


def test_garra():
    """Abierta -> ángulo de abierta, cerrada -> de cerrada, y de vuelta; fuera del rango se
    queda en el extremo."""
    assert dedo_a_servo(DEDO_ABIERTA, 40, 110) == 40
    assert dedo_a_servo(DEDO_CERRADA, 40, 110) == 110
    assert dedo_a_servo(0.5, 40, 110) == 40
    assert servo_a_dedo(75, 40, 110) == pytest.approx((DEDO_ABIERTA + DEDO_CERRADA) / 2)


def test_interpolar():
    t, p = [0.0, 1.0, 3.0], [[0.0, 0.0], [1.0, 2.0], [1.0, 0.0]]
    assert interpolar(t, p, -1) == [0.0, 0.0]
    assert interpolar(t, p, 0.5) == pytest.approx([0.5, 1.0])
    assert interpolar(t, p, 2.0) == pytest.approx([1.0, 1.0])
    assert interpolar(t, p, 9) == [1.0, 0.0]


class Reloj:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_arduino_falso_g_llegan_juntas():
    """G 1000 ms, movimientos lentos (100 y 50 pasos/s, casi sin rampa): las dos llegan a la vez,
    ~al segundo, y a los 0.4 s van por ~40 % del camino."""
    r = Reloj()
    a = ArduinoFalso(reloj=r)
    assert a.responder('P') == 'PONG brazo_mega v5'
    assert a.responder('G 1000 100 -50 0 0 0 0') == 'OK G'
    r.t = 0.4
    pos, mov, _ = leer_estado(a.responder('E'))
    assert mov and 36 <= pos[0] <= 41 and -21 <= pos[1] <= -18
    r.t = 0.8
    a.responder('E')                    # (cualquier comando mantiene vivo el perro guardián)
    r.t = 1.1
    pos, mov, _ = leer_estado(a.responder('E'))
    assert pos[:2] == [100, -50] and not mov


def velocidades(a, r, hasta, dt=0.01):
    """Muestrea la posición de J1 cada dt hasta `hasta` s; devuelve (tiempos, posiciones)."""
    ts, ps = [], []
    while r.t < hasta:
        r.t += dt
        ts.append(r.t)
        ps.append(leer_estado(a.responder('E'))[0][0])
    return ts, ps


def test_arduino_falso_rampa_trapezoidal():
    """M de 4000 pasos a 2000 pasos/s con 4000 pasos/s²: acelera 0.5 s, sigue 1.5 s a 2000/s,
    frena 0.5 s (total 2.5 s). La velocidad nunca cambia más que la aceleración permite."""
    r = Reloj()
    a = ArduinoFalso(reloj=r)
    a.responder('V 2000')
    a.responder('M 1 4000')
    ts, ps = velocidades(a, r, 3.0)
    # Velocidad y aceleración en ventanas de 50 ms (la posición viene en pasos enteros: con
    # ventanas de 10 ms, un paso de diferencia ya parece un cambio de 100 pasos/s).
    vs = [(ps[k + 5] - ps[k]) / 0.05 for k in range(0, len(ps) - 5, 5)]
    acel = max(abs(v1 - v0) / 0.05 for v0, v1 in zip(vs, vs[1:]))
    assert acel < 4000 * 1.3
    assert max(vs) == pytest.approx(2000, abs=100)   # (una ventana puede tener un tick de más)
    llegada = next(t for t, p in zip(ts, ps) if p == 4000)
    assert llegada == pytest.approx(2.5, abs=0.05)
    assert vs[2] == pytest.approx(4000 * 0.125, abs=60)       # ventana 0.10-0.15 s: ~500/s


def test_arduino_falso_cambio_de_sentido():
    """Yendo a 2000 pasos/s hacia +8000, se le pide ir a -8000: primero frena (sigue avanzando
    ~2000²/(2·4000) = 500 pasos), después da la vuelta y llega."""
    r = Reloj()
    a = ArduinoFalso(reloj=r)
    a.responder('V 2000')
    a.responder('M 1 8000')
    r.t = 1.0
    antes = leer_estado(a.responder('E'))[0][0]
    a.responder('M 1 -16000')           # relativo al destino 8000 -> -8000
    ts, ps = velocidades(a, r, 12.0)
    assert max(ps) - antes == pytest.approx(500, abs=40)
    assert ps[-1] == -8000


def test_arduino_falso_perro_guardian_y_m():
    """Sin comandos por más de 0.5 s, un movimiento de G se para (en 0.5 s, acelerando a 4000
    pasos/s², recorre ~500 pasos); uno de M no."""
    r = Reloj()
    a = ArduinoFalso(reloj=r)
    a.responder('G 10000 20000 0 0 0 0 0')
    r.t = 2.0
    pos, mov, _ = leer_estado(a.responder('E'))
    assert not mov and pos[0] == pytest.approx(500, abs=20)
    assert a.responder('M 2 300') == 'OK M 2 300'
    r.t = 10.0
    pos, mov, _ = leer_estado(a.responder('E'))
    assert pos[1] == 300 and not mov


def test_arduino_falso_errores_z_y_servo():
    a = ArduinoFalso(reloj=Reloj())
    assert a.responder('G 0 1 2 3 4 5 6').startswith('ERR')
    assert a.responder('M 9 1').startswith('ERR')
    assert a.responder('Q').startswith('ERR')
    assert a.responder('A 50').startswith('ERR') and a.responder('A 3000') == 'OK A 3000'
    # v5: servo de 270° manejado de 0 a 297°; más que eso se acota.
    assert a.responder('S 200') == 'OK S 200'
    assert a.responder('S 400') == 'OK S 297'
    assert leer_estado(a.responder('E'))[2] == 297
    assert a.responder('D') == 'OK D'
    assert a.responder('Z') == 'OK Z'


def test_enlace_con_puerto_virtual():
    """Enlace por un puerto serie virtual (pty) con el ArduinoFalso del otro lado."""
    import os
    import threading

    from andesrobot_arm.brazo_real import Enlace

    maestro, esclavo = os.openpty()
    falso = ArduinoFalso()
    vivo = [True]

    def arduino():
        os.write(maestro, b'LISTO brazo_mega v1\n')
        resto = b''
        while vivo[0]:
            try:
                resto += os.read(maestro, 256)
            except OSError:
                return
            while b'\n' in resto:
                linea, resto = resto.split(b'\n', 1)
                os.write(maestro, (falso.responder(linea.decode()) + '\n').encode())

    hilo = threading.Thread(target=arduino, daemon=True)
    enlace = Enlace(os.ttyname(esclavo))
    hilo.start()
    try:
        assert enlace.esperar_listo(2.0) == 'LISTO brazo_mega v1'
        assert enlace.preguntar('P', 1.0) == 'PONG brazo_mega v5'
        assert enlace.preguntar('M 3 10', 1.0) == 'OK M 3 10'
        assert leer_estado(enlace.preguntar('E', 1.0))[0] is not None
    finally:
        vivo[0] = False
        enlace.cerrar()
        os.close(esclavo)
        os.close(maestro)


def test_conectar_distingue_reinicio():
    """Un Arduino que ya estaba andando contesta P enseguida (no se reinició); uno que se está
    reiniciando calla ~0.5 s, saluda con LISTO y recién ahí contesta (se reinició)."""
    import os
    import threading

    from andesrobot_arm.brazo_real import Enlace

    for demora, reinicio_esperado in ((0.0, False), (0.6, True)):
        maestro, esclavo = os.openpty()
        os.set_blocking(maestro, False)
        falso = ArduinoFalso()
        vivo = [True]

        def arduino():
            inicio = time.monotonic()
            saludo = demora == 0.0           # sin demora no hay saludo que mandar
            resto = b''
            while vivo[0]:
                if not saludo and time.monotonic() - inicio > demora:
                    os.write(maestro, b'LISTO brazo_mega v1\n')
                    saludo = True
                try:
                    resto += os.read(maestro, 256)
                except (BlockingIOError, OSError):
                    time.sleep(0.005)
                    continue
                while b'\n' in resto:
                    linea, resto = resto.split(b'\n', 1)
                    if saludo:               # mientras "se reinicia" no contesta
                        os.write(maestro, (falso.responder(linea.decode()) + '\n').encode())

        enlace = Enlace(os.ttyname(esclavo))
        hilo = threading.Thread(target=arduino, daemon=True)
        hilo.start()
        try:
            respuesta, reinicio = enlace.conectar(espera=2.0)
            assert respuesta == 'PONG brazo_mega v5'
            assert reinicio == reinicio_esperado
        finally:
            vivo[0] = False
            hilo.join(1.0)
            enlace.cerrar()
            os.close(esclavo)
            os.close(maestro)
