"""Arduino de mentira para probar brazo_real_node y MoveIt sin el Arduino ni los motores.

Crea un puerto serie virtual (un pty: un par de "archivos" conectados como un cable) y del otro
lado contesta igual que firmware/brazo_mega/brazo_mega.ino (brazo_real.ArduinoFalso: mismos
comandos y los motores "se mueven" con el tiempo). Deja un enlace fijo al puerto, por defecto
/tmp/arduino_falso, para pasárselo al nodo:

  ros2 run andesrobot_arm arduino_falso                       # terminal 1
  ros2 launch andesrobot_arm brazo_real.launch.py puerto:=/tmp/arduino_falso   # terminal 2

(En el PC, fuera del contenedor, también sirve para firmware/probar_arduino.py --puerto
/tmp/arduino_falso: python3 -m andesrobot_arm.arduino_falso desde ros2_ws/src/andesrobot_arm.)

Escribe en pantalla cada comando y su respuesta (salvo los E, que son 20 por segundo) si se le
pasa --ver.
"""
import argparse
import os
import time
import tty

from andesrobot_arm.brazo_real import ArduinoFalso


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--enlace', default='/tmp/arduino_falso', help='nombre fijo del puerto')
    ap.add_argument('--ver', action='store_true', help='mostrar los comandos (menos E)')
    a = ap.parse_args()
    maestro, esclavo = os.openpty()
    # Modo crudo en el lado del nodo desde ya: un pty nuevo trae eco, y el saludo volvía de
    # vuelta como si fuera un comando.
    tty.setraw(esclavo)
    ruta = os.ttyname(esclavo)
    if os.path.islink(a.enlace) or os.path.exists(a.enlace):
        os.remove(a.enlace)
    os.symlink(ruta, a.enlace)
    print(f'Arduino falso en {a.enlace} -> {ruta} (Ctrl+C para terminar)', flush=True)
    falso = ArduinoFalso()
    os.set_blocking(maestro, False)
    resto, hablo, ultimo_listo = b'', False, 0.0
    try:
        while True:
            # Hasta el primer comando, saluda cada medio segundo (el de verdad saluda cada vez que
            # se abre el puerto, porque se reinicia; aquí no se sabe cuándo lo abren).
            if not hablo and time.monotonic() - ultimo_listo > 0.5:
                os.write(maestro, b'LISTO brazo_mega v3 (falso)\n')
                ultimo_listo = time.monotonic()
            try:
                trozo = os.read(maestro, 512)
            except BlockingIOError:
                trozo = b''
            except OSError:
                trozo = b''
            if not trozo:
                time.sleep(0.002)
                continue
            resto += trozo
            while b'\n' in resto:
                linea, resto = resto.split(b'\n', 1)
                texto = linea.decode(errors='replace').strip()
                if not texto:
                    continue
                hablo = True
                respuesta = falso.responder(texto)
                if respuesta is not None:
                    os.write(maestro, (respuesta + '\n').encode())
                if a.ver and not texto.startswith('E'):
                    print(f'  {texto:40s} -> {respuesta}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.islink(a.enlace):
            os.remove(a.enlace)


if __name__ == '__main__':
    main()
