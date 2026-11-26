#!/usr/bin/env python3
"""Pruebas exigentes del Arduino del brazo (firmware/brazo_mega v2 o v3), SIN el brazo conectado.

Para correrlas solo con el Arduino: sin drivers ni motores, los pulsos no mueven nada. CON el
brazo conectado NO: estas pruebas mueven las 6 articulaciones miles de pasos a toda velocidad.

Qué mide (cada prueba dice OK o FALLA y sus números):
  1. protocolo    comandos mal escritos, líneas largas, fines de línea \\r\\n: una respuesta por
                  línea, ERR cuando corresponde, y el Arduino sigue contestando después
  2. latencia     ida y vuelta de P y E (lo que tarda una pregunta)
  3. M            cada articulación sola: llega a los pasos pedidos y en el tiempo esperado
                  (con la rampa: acelera, sigue, frena)
  4. G            las 6 a la vez con distancias distintas: llegan exactas, y si se pasan al
                  frenar, no más que la distancia de frenado
 10. rampa        la velocidad medida sube y baja como mucho a la aceleración pedida
 11. sentido      yendo rápido, se le pide volver: frena (sigue v²/2a pasos) y vuelve
  5. carga        las 6 a la velocidad máxima mientras se pregunta E a 50 Hz
  6. perro        G largo y el PC se calla: se tiene que parar a los ~0.5 s y quedarse quieto
  7. seguimiento  como ROS: 30 s de senos en las 6 articulaciones, G + E a 20 Hz: error de
                  seguimiento, respuestas perdidas o ERR
  8. ráfaga       30 comandos seguidos sin esperar respuesta (el buffer de entrada del Arduino
                  es de 64 bytes): ¿se pierde alguno?
  9. X, Z, servo  parar en movimiento, cero, el servo y soltarlo

Uso: python3 firmware/prueba_exigente.py [--puerto /dev/ttyUSB0] [--solo 1 4 7]
"""
import argparse
import math
import os
import statistics
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, '..', 'ros2_ws', 'src', 'andesrobot_arm'))
from andesrobot_arm.brazo_real import Enlace, cmd_ir, leer_estado  # noqa: E402

VEL_MAX = 2000           # [pasos/s] la del firmware
ACEL = 4000              # [pasos/s²] la rampa del firmware al encender


MARGEN_FRENO = 0.9       # el firmware frena como si la aceleración fuera 10 % menor


def trapecio(pasos, vel=VEL_MAX, acel=ACEL):
    """Tiempo ideal [s] de un movimiento trapezoidal como el del firmware: acelera a `acel`,
    sigue a `vel` y frena a MARGEN_FRENO · acel."""
    frena = MARGEN_FRENO * acel
    d_sube, d_baja = vel * vel / (2 * acel), vel * vel / (2 * frena)
    if pasos >= d_sube + d_baja:
        return vel / acel + vel / frena + (pasos - d_sube - d_baja) / vel
    pico = math.sqrt(2 * pasos * acel * frena / (acel + frena))     # triángulo: no llega a vel
    return pico / acel + pico / frena


RESULTADOS = []


def anotar(nombre, bien, detalle):
    RESULTADOS.append((nombre, bien, detalle))
    print(f'  {"OK   " if bien else "FALLA"} {nombre}: {detalle}', flush=True)


def estado(e):
    """(posiciones, se mueve, servo) o lanza ValueError."""
    return leer_estado(e.preguntar('E', 0.5))


def esperar_quieto(e, limite):
    """Pregunta E hasta que nada se mueva. Devuelve (posiciones, segundos que tardó)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < limite:
        pos, mov, _ = estado(e)
        if not mov:
            return pos, time.monotonic() - t0
        time.sleep(0.005)
    e.preguntar('X')
    raise TimeoutError(f'no terminó en {limite} s')


def a_cero(e):
    """Vuelve todo a 0 (G) y espera."""
    e.preguntar('V 2000')
    pos, _, _ = estado(e)
    lejos = max(abs(p) for p in pos)
    if lejos:
        e.preguntar(cmd_ir([0] * 6, max(1, 1000 * lejos / VEL_MAX)))
        # (G tiene perro guardián: se pregunta E seguido mientras tanto)
        esperar_quieto(e, 2 * trapecio(lejos) + 3)


def p_protocolo(e):
    casos = [('G', 'ERR'), ('G 10 1 2 3', 'ERR'), ('G -5 1 2 3 4 5 6', 'ERR'),
             ('G 0 1 2 3 4 5 6', 'ERR'), ('M 0 5', 'ERR'), ('M 7 5', 'ERR'), ('M 1', 'ERR'),
             ('V 0', 'ERR'), ('S', 'ERR'), ('xyz', 'ERR'), ('p', 'ERR'),
             ('M 1 abc', 'ERR'), ('A 50', 'ERR'), ('P', 'PONG')]
    malos = []
    for comando, espera in casos:
        r = e.preguntar(comando, 0.5)
        if not r or not r.startswith(espera):
            malos.append(f'{comando!r} -> {r!r}')
    anotar('protocolo: comandos mal escritos', not malos, f'{len(casos)} casos' +
           (f'; mal: {malos}' if malos else ''))
    # \r\n (Windows / Monitor Serie con "Ambos NL & CR")
    os.write(e.fd, b'P\r\n')
    r = e.leer_linea(0.5)
    anotar('protocolo: fin de línea \\r\\n', (r or '').startswith('PONG brazo_mega'), repr(r))
    # Línea demasiado larga: tiene que dar UNA respuesta de error y seguir funcionando.
    import termios
    termios.tcflush(e.fd, termios.TCIFLUSH)
    e.resto = b''
    os.write(e.fd, b'M 1 ' + b'9' * 150 + b'\n')
    respuestas = []
    while True:
        r = e.leer_linea(0.4)
        if r is None:
            break
        respuestas.append(r)
    sigue = e.preguntar('P', 0.5)
    anotar('protocolo: línea de 154 caracteres', len(respuestas) == 1 and
           respuestas[0].startswith('ERR') and (sigue or '').startswith('PONG brazo_mega'),
           f'respuestas {respuestas}, después P -> {sigue!r}')
    pos, _, _ = estado(e)
    anotar('protocolo: no se movió nada', not any(pos), f'posiciones {pos}')


def p_latencia(e):
    for comando in ('P', 'E'):
        tiempos = []
        for _ in range(300):
            t0 = time.perf_counter()
            r = e.preguntar(comando, 0.5)
            tiempos.append((time.perf_counter() - t0) * 1000)
            if r is None:
                tiempos[-1] = float('inf')
        perdidas = sum(1 for t in tiempos if t == float('inf'))
        validos = sorted(t for t in tiempos if t != float('inf'))
        p95 = validos[int(0.95 * len(validos)) - 1]
        anotar(f'latencia {comando} (300 veces)', perdidas == 0 and p95 < 20,
               f'media {statistics.mean(validos):.1f} ms, p95 {p95:.1f} ms, '
               f'máx {validos[-1]:.1f} ms, sin respuesta {perdidas}')


def p_m(e):
    e.preguntar('V 2000')
    pasos = 4000                                    # 2 s a 2000 pasos/s
    for j in range(1, 7):
        antes = estado(e)[0][j - 1]
        e.preguntar(f'M {j} {pasos}')
        pos, t = esperar_quieto(e, 10)
        ida = pos[j - 1] - antes
        e.preguntar(f'M {j} {-pasos}')
        pos2, t2 = esperar_quieto(e, 10)
        otras = [p for k, p in enumerate(pos2) if k != j - 1]
        ideal = trapecio(pasos)
        bien = (ida == pasos and pos2[j - 1] == antes and not any(otras) and
                abs(t - ideal) < 0.15 and abs(t2 - ideal) < 0.15)
        anotar(f'M J{j} ±{pasos} pasos a {VEL_MAX}/s', bien,
               f'ida {ida:+d} en {t:.3f} s, vuelta a {pos2[j - 1]} en {t2:.3f} s '
               f'(ideal con rampa {ideal:.3f} s)')


def p_g(e):
    """Un solo G de 2 s hacia lejos (no como ROS, que manda uno cada 50 ms): con la rampa no
    llega justo a los 2 s; tiene que llegar exacto, y pasarse como mucho lo que tarda en frenar."""
    a_cero(e)
    metas = [3600, -2400, 1200, -600, 300, -3600]
    e.preguntar(cmd_ir(metas, 2000))
    t0 = time.monotonic()
    llegada = [None] * 6
    pasado = [0] * 6
    while time.monotonic() - t0 < 8:
        pos, mov, _ = estado(e)
        t = time.monotonic() - t0
        for j in range(6):
            mas_alla = (pos[j] - metas[j]) * (1 if metas[j] > 0 else -1)
            pasado[j] = max(pasado[j], mas_alla)
            if llegada[j] is None and pos[j] == metas[j]:
                llegada[j] = t
        if not mov:
            break
        time.sleep(0.01)
    frenado = VEL_MAX ** 2 / (2 * ACEL)
    bien = pos == metas and all(x is not None for x in llegada) and max(pasado) <= frenado
    anotar('G: 6 articulaciones, distancias distintas, un G de 2 s', bien,
           'llegadas ' + ', '.join('-' if x is None else f'{x:.2f}' for x in llegada) +
           f' s; se pasaron como mucho {max(pasado)} pasos (frenado {frenado:.0f}); '
           f'final {pos}')


def p_carga(e):
    a_cero(e)
    pasos = 6000                                     # las 6 a la vez con M, a 2000/s
    e.preguntar('V 2000')
    for j in range(1, 7):
        e.preguntar(f'M {j} {pasos if j % 2 else -pasos}')
    t0 = time.monotonic()
    lat = []
    while True:
        t1 = time.perf_counter()
        pos, mov, _ = estado(e)
        lat.append((time.perf_counter() - t1) * 1000)
        if not mov or time.monotonic() - t0 > 12:
            break
        time.sleep(0.02)
    t = time.monotonic() - t0
    ideal = trapecio(pasos)
    bien = pos == [pasos, -pasos, pasos, -pasos, pasos, -pasos] and abs(t - ideal) < 0.1 * ideal
    anotar('carga: 6 a 2000 pasos/s + E a 50 Hz', bien,
           f'{t:.3f} s (ideal con rampa {ideal:.3f}), E máx {max(lat):.1f} ms, posiciones {pos}')


def p_perro(e):
    a_cero(e)
    e.preguntar(cmd_ir([20000] * 6, 10000))         # 2000 pasos/s durante 10 s
    time.sleep(1.2)                                  # el PC se calla
    pos1, mov1, _ = estado(e)
    time.sleep(0.5)
    pos2, mov2, _ = estado(e)
    esperado = 0.5 * ACEL * 0.5 ** 2                # 0.5 s acelerando desde 0
    bien = not mov1 and pos1 == pos2 and abs(pos1[0] - esperado) < 0.2 * esperado
    anotar('perro guardián (G y silencio)', bien,
           f'paró en {pos1[0]} pasos (~{esperado:.0f} esperado), quieto después: {pos1 == pos2}')


def p_seguimiento(e, duracion=30.0, hz=20.0):
    a_cero(e)
    periodo = 1.0 / hz
    amp = [3000, 2500, 2000, 1500, 1000, 3000]      # [pasos]
    frec = [0.05, 0.07, 0.11, 0.13, 0.17, 0.2]       # [Hz]: hasta ~3770 pasos/s pico en J6
    # (J6 a 0.2 Hz con 3000 pasos pide 2π·0.2·3000 = 3770 pasos/s: más que VEL_MAX, a propósito,
    # para ver cuánto se atrasa cuando se le pide de más)

    def meta(t):
        # a·(1 - cos): empieza quieta en 0, como una trayectoria de MoveIt (un seno empieza a
        # toda velocidad y con la rampa ninguna articulación puede seguirlo en el arranque).
        return [a * (1 - math.cos(2 * math.pi * f * t)) for a, f in zip(amp, frec)]

    t0 = time.monotonic()
    errores, perdidas, err_g, vueltas, retraso = [], 0, 0, 0, []
    while (t := time.monotonic() - t0) < duracion:
        r = e.preguntar(cmd_ir([round(x) for x in meta(t + periodo)], periodo * 1000), 0.3)
        if r != 'OK G':
            err_g += 1
        try:
            pos, _, _ = estado(e)
            ahora = meta(time.monotonic() - t0)
            errores.append([abs(p - m) for p, m in zip(pos, ahora)])
        except ValueError:
            perdidas += 1
        vueltas += 1
        resto = periodo - ((time.monotonic() - t0) - t)
        if resto < 0:
            retraso.append(-resto)
        time.sleep(max(0.0, resto))
    peor = [max(fila[j] for fila in errores) for j in range(6)]
    bien = err_g == 0 and perdidas == 0 and max(peor[:5]) < 150
    anotar(f'seguimiento tipo ROS {duracion:.0f} s a {hz:.0f} Hz', bien,
           f'{vueltas} ciclos, G con error {err_g}, E perdidos {perdidas}, ciclos atrasados '
           f'{len(retraso)}; error máx por articulación [pasos]: '
           + ', '.join(f'J{j + 1} {p:.0f}' for j, p in enumerate(peor))
           + ' (J6 pide más de VEL_MAX a propósito)')
    e.preguntar('X')


def p_rafaga(e):
    import termios
    a_cero(e)
    termios.tcflush(e.fd, termios.TCIFLUSH)
    e.resto = b''
    comandos = [cmd_ir([k * 10] * 6, 50) for k in range(1, 31)]   # 30 líneas de ~40 bytes
    os.write(e.fd, ('\n'.join(comandos) + '\n').encode())
    respuestas = []
    while True:
        r = e.leer_linea(0.5)
        if r is None:
            break
        respuestas.append(r)
    ok = sum(1 for r in respuestas if r == 'OK G')
    anotar('ráfaga de 30 G sin esperar (~1.2 kB de una vez)', ok == 30,
           f'{ok} de 30 contestaron OK G; otras respuestas: '
           f'{[r for r in respuestas if r != "OK G"][:3]}')
    e.preguntar('X')


def p_x_z_servo(e):
    a_cero(e)
    e.preguntar('M 1 8000')
    time.sleep(0.5)
    r = e.preguntar('X')
    p1, mov, _ = estado(e)
    time.sleep(0.3)
    p2, _, _ = estado(e)
    anotar('X para en seco', r == 'OK X' and not mov and p1 == p2 and 0 < p1[0] < 8000,
           f'paró en {p1[0]} pasos')
    r_z = e.preguntar('Z')
    p3, _, _ = estado(e)
    e.preguntar('M 2 4000')
    r_zm = e.preguntar('Z')
    e.preguntar('X')
    anotar('Z: cero quieto, ERR moviéndose', r_z == 'OK Z' and not any(p3) and
           (r_zm or '').startswith('ERR'), f'{r_z!r}, posiciones {p3}, moviéndose {r_zm!r}')
    bien = True
    for g in (0, 45, 90, 135, 180, 200, -5):
        r = e.preguntar(f'S {g}')
        bien &= r == f'OK S {min(297, max(0, g))}'
    servo = estado(e)[2]
    r = e.preguntar('D')
    suelto = estado(e)[2]
    anotar('servo: ángulos y límites, soltar', bien and servo == 0 and r == 'OK D' and
           suelto is None, f'último {servo}, después de D {suelto}')


def muestrear(e, segundos, cada=0.01):
    """(t, posición de J1) cada `cada` s, hasta que se quede quieto o pasen `segundos`. (Sin
    pausa, ~220 preguntas por segundo, contestar le quita ~10 % de velocidad al Arduino; ROS
    pregunta 20 veces por segundo.)"""
    t0 = time.monotonic()
    datos = []
    while (t := time.monotonic() - t0) < segundos:
        pos, mov, _ = estado(e)
        datos.append((t, pos[0]))
        if not mov and t > 0.2:
            break
        time.sleep(cada)
    return datos


def velocidades(datos, ventana=0.05):
    """Velocidad media [pasos/s] en ventanas de `ventana` s: [(t medio, v)]."""
    salida, k = [], 0
    while k < len(datos):
        t0, p0 = datos[k]
        m = k
        while m < len(datos) and datos[m][0] - t0 < ventana:
            m += 1
        if m >= len(datos):
            break
        t1, p1 = datos[m]
        salida.append(((t0 + t1) / 2, (p1 - p0) / (t1 - t0)))
        k = m
    return salida


def p_rampa(e):
    a_cero(e)
    e.preguntar('V 2000')
    e.preguntar('M 1 8000')
    datos = muestrear(e, trapecio(8000) + 3)
    vs = velocidades(datos)
    acel = [abs(v1 - v0) / (t1 - t0) for (t0, v0), (t1, v1) in zip(vs, vs[1:])]
    llegada = next((t for t, p in datos if p == 8000), None)
    pico = max(v for _, v in vs)
    bien = (max(acel) < 1.3 * ACEL and abs(pico - VEL_MAX) < 120 and llegada is not None and
            abs(llegada - trapecio(8000)) < 0.15)
    anotar('rampa: M de 8000 pasos medida cada ~15 ms', bien,
           f'aceleración máx medida {max(acel):.0f} pasos/s² (pedida {ACEL}), velocidad pico '
           f'{pico:.0f}, llegó en {llegada and round(llegada, 3)} s (ideal '
           f'{trapecio(8000):.3f}); velocidad a los 0.25 s ~'
           f'{min(vs, key=lambda x: abs(x[0] - 0.25))[1]:.0f} (ideal {ACEL * 0.25:.0f})')
    a_cero(e)


def p_sentido(e):
    a_cero(e)
    e.preguntar('V 2000')
    e.preguntar('M 1 8000')
    time.sleep(1.0)
    antes = estado(e)[0][0]
    e.preguntar('M 1 -16000')                       # relativo al destino: va a -8000
    datos = muestrear(e, 15)
    maximo = max(p for _, p in datos)
    final = estado(e)[0][0]
    frenado = VEL_MAX ** 2 / (2 * ACEL)
    bien = abs((maximo - antes) - frenado) < 0.15 * frenado and final == -8000
    anotar('cambio de sentido a 2000 pasos/s', bien,
           f'siguió {maximo - antes} pasos al frenar (ideal {frenado:.0f}), final {final}')
    a_cero(e)


def p_aceleracion(e):
    bien = e.preguntar('A 2000') == 'OK A 2000'
    a_cero(e)
    e.preguntar('V 2000')
    e.preguntar('M 1 4000')
    _, t = esperar_quieto(e, 10)
    bien &= abs(t - trapecio(4000, acel=2000)) < 0.15
    bien &= e.preguntar(f'A {ACEL}') == f'OK A {ACEL}'
    anotar('comando A (aceleración 2000)', bien,
           f'M de 4000 en {t:.3f} s (ideal con 2000 pasos/s²: '
           f'{trapecio(4000, acel=2000):.3f} s); vuelta a {ACEL}')
    a_cero(e)


PRUEBAS = {1: p_protocolo, 2: p_latencia, 3: p_m, 4: p_g, 5: p_carga, 6: p_perro,
           7: p_seguimiento, 8: p_rafaga, 9: p_x_z_servo, 10: p_rampa, 11: p_sentido,
           12: p_aceleracion}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--puerto', default='/dev/ttyUSB0')
    ap.add_argument('--solo', type=int, nargs='*', help='números de las pruebas a correr')
    a = ap.parse_args()
    e = Enlace(a.puerto)
    r, reinicio = e.conectar()
    print(f'{a.puerto}: {r} (reinicio: {reinicio})', flush=True)
    if not (r or '').startswith('PONG brazo_mega v'):
        sys.exit(f'No contesta brazo_mega ({r!r}): cargar firmware/brazo_mega')
    try:
        for n, prueba in PRUEBAS.items():
            if a.solo and n not in a.solo:
                continue
            print(f'{n}. {prueba.__name__[2:]}', flush=True)
            try:
                prueba(e)
            except (TimeoutError, ValueError) as error:
                anotar(prueba.__name__[2:], False, f'se cortó: {error}')
                e.preguntar('X')
        a_cero(e)
    finally:
        e.preguntar('X')
        e.cerrar()
    malas = [r for r in RESULTADOS if not r[1]]
    print(f'\n{len(RESULTADOS) - len(malas)} de {len(RESULTADOS)} OK')
    sys.exit(1 if malas else 0)


if __name__ == '__main__':
    main()
