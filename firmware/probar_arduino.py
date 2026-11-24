#!/usr/bin/env python3
"""Prueba el Arduino Mega del brazo (firmware/brazo_mega v2) desde el PC, sin ROS.

Sin opciones solo revisa la comunicación (no mueve nada): espera el "LISTO", manda P (¿estás
ahí?), E (estado) y un comando que no existe (tiene que dar ERR).

Para probar las articulaciones UNA POR UNA (lo primero con el brazo real, con alguien al lado
listo para cortar la fuente de 24 V):
  python3 firmware/probar_arduino.py --articulacion 1 --grados 5             # J1 +5° y para
  python3 firmware/probar_arduino.py --articulacion 1 --grados 5 --volver    # +5° y vuelve a 0
  python3 firmware/probar_arduino.py --articulacion 3 --grados -2 --vel 100  # más lento
  python3 firmware/probar_arduino.py --servo 90 60 90 --suelta               # servo de la garra
  python3 firmware/probar_arduino.py --acel 2000 --articulacion 2 --grados 2 # rampa más suave
  python3 firmware/probar_arduino.py --cero        # el brazo está en su pose cero: contar 0 ahí
Los grados se pasan a pasos con los mismos números que usa ROS (ros2_ws/src/andesrobot_arm/
config/brazo_real.yaml y andesrobot_arm/brazo_real.py): si la articulación gira al revés de lo
esperado, poner invertir: true en ese archivo; si gira más o menos de lo pedido, revisar
micropasos (los interruptores del TB6600) y reduccion. Para mover sin convertir: --pasos.

Usa solo la biblioteca estándar de Python (y PyYAML, que viene con Ubuntu), sin pyserial.
Termina con código 0 si todo respondió bien, 1 si no.
"""
import argparse
import math
import os
import sys
import time

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
PAQUETE = os.path.join(AQUI, '..', 'ros2_ws', 'src', 'andesrobot_arm')
sys.path.insert(0, PAQUETE)
from andesrobot_arm.brazo_real import (  # noqa: E402
    Enlace, cmd_mover, leer_config, leer_estado)

PUERTOS = ('/dev/arduino_brazo', '/dev/ttyUSB0', '/dev/ttyACM0')
CONFIG = os.path.join(PAQUETE, 'config', 'brazo_real.yaml')


def preguntar(enlace, comando, espera=0.5):
    respuesta = enlace.preguntar(comando, espera)
    print(f'  -> {comando:14s} <- {respuesta}')
    return respuesta


def esperar_quieto(enlace, limite):
    """Pide el estado hasta que el Arduino diga que nada se mueve. Devuelve las posiciones."""
    fin = time.time() + limite
    while time.time() < fin:
        try:
            pos, mov, _ = leer_estado(enlace.preguntar('E', 0.5))
        except ValueError:
            continue
        if not mov:
            return pos
        time.sleep(0.2)
    preguntar(enlace, 'X')
    raise TimeoutError(f'no terminó en {limite:.0f} s: se mandó X (parar)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--puerto', help=f'por defecto el primero que exista de {PUERTOS}')
    ap.add_argument('--articulacion', type=int, choices=range(1, 7), help='J1..J6')
    ap.add_argument('--grados', type=float, help='cuánto mover la articulación [grados]')
    ap.add_argument('--pasos', type=int, help='o cuánto moverla en pasos, sin convertir')
    ap.add_argument('--volver', action='store_true', help='después, volver a donde estaba')
    ap.add_argument('--vel', type=int, help='[pasos/s] velocidad de la prueba (V; firmware: 200)')
    ap.add_argument('--acel', type=int,
                    help='[pasos/s²] aceleración de la rampa (A; firmware: 4000). Queda puesta '
                    'hasta que el Arduino se reinicie')
    ap.add_argument('--cero', action='store_true',
                    help='la posición actual pasa a ser 0 en las 6 (Z; solo con todo quieto)')
    ap.add_argument('--servo', type=int, nargs='*', default=[], help='ángulos del servo [grados]')
    ap.add_argument('--pausa', type=float, default=1.5, help='segundos entre ángulos del servo')
    ap.add_argument('--suelta', action='store_true', help='soltar el servo al final (D)')
    a = ap.parse_args()
    puerto = a.puerto or next((p for p in PUERTOS if os.path.exists(p)), None)
    if not puerto:
        sys.exit(f'No encuentro el Arduino en {PUERTOS}. ¿Está conectado? (ls -l /dev/ttyUSB*; '
                 'si aparece y desaparece, ver brltty en docker/udev/README.md)')
    print(f'Puerto: {puerto}')
    try:
        enlace = Enlace(puerto)
    except PermissionError:
        sys.exit(f'Sin permiso para {puerto}: el usuario tiene que estar en el grupo dialout, o '
                 'instalar la regla docker/udev/99-milo.rules')
    bien = True
    try:
        r, reinicio = enlace.conectar()
        print(f'  -> P              <- {r}   (' + ('se reinició: cuenta desde 0' if reinicio
                                                   else 'no se reinició: sigue su cuenta') + ')')
        bien &= bool(r and r.startswith('PONG brazo_mega'))
        if r and not r.startswith('PONG brazo_mega'):
            print('  (no es brazo_mega: cargar firmware/brazo_mega/brazo_mega.ino)')
        r = preguntar(enlace, 'E')
        bien &= bool(r and r.startswith('E '))
        if a.cero:
            bien &= preguntar(enlace, 'Z') == 'OK Z'
        if a.acel:
            bien &= (preguntar(enlace, f'A {a.acel}') or '').startswith('OK A')
        if a.articulacion:
            j = a.articulacion
            if a.pasos is not None:
                pasos = a.pasos
            elif a.grados is not None:
                with open(CONFIG) as f:
                    art = leer_config(yaml.safe_load(f))[j - 1]
                # Movimiento relativo: la misma cuenta que Articulacion.a_pasos, sin el cero.
                signo = -1 if art.invertir else 1
                pasos = int(round(signo * math.radians(a.grados) * art.pasos_por_rad))
                print(f'  J{j}: {a.grados}° = {pasos} pasos '
                      f'({math.radians(1) * art.pasos_por_rad:.2f} '
                      f'pasos/°, micropasos {art.micropasos}, reducción {art.reduccion}, '
                      f'invertir {art.invertir})')
            else:
                sys.exit('Con --articulacion va --grados o --pasos')
            if a.vel:
                bien &= (preguntar(enlace, f'V {a.vel}') or '').startswith('OK V')
            vel = a.vel or 200
            limite = abs(pasos) / vel + 5          # (+ el tiempo de la rampa, de sobra)
            antes = leer_estado(enlace.preguntar('E'))[0][j - 1]
            bien &= (preguntar(enlace, cmd_mover(j, pasos)) or '').startswith('OK M')
            pos = esperar_quieto(enlace, limite)
            print(f'  J{j}: de {antes} a {pos[j - 1]} pasos (pedido {pasos:+d})')
            bien &= pos[j - 1] - antes == pasos
            if a.volver:
                bien &= (preguntar(enlace, cmd_mover(j, -pasos)) or '').startswith('OK M')
                pos = esperar_quieto(enlace, limite)
                print(f'  J{j}: de vuelta en {pos[j - 1]} pasos')
                bien &= pos[j - 1] == antes
        for angulo in a.servo:
            r = preguntar(enlace, f'S {angulo}')
            bien &= bool(r and r.startswith('OK S'))
            time.sleep(a.pausa)
        if a.suelta:
            bien &= preguntar(enlace, 'D') == 'OK D'
        r = preguntar(enlace, 'Q')                 # un comando que no existe: debe dar ERR
        bien &= bool(r and r.startswith('ERR'))
    except (TimeoutError, ValueError) as error:
        print(f'  {error}')
        bien = False
    finally:
        enlace.cerrar()
    print('Todo OK' if bien else 'FALLÓ: alguna respuesta no fue la esperada')
    sys.exit(0 if bien else 1)


if __name__ == '__main__':
    main()
