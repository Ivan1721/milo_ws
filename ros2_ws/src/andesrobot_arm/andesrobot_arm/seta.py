"""Seta de emergencia leída por GPIO de la Jetson, sin ROS (la usa brazo_real_node).

En palabras simples: la seta es el botón rojo grande de parada. Es la MISMA de la base y la misma
que lee el driver del control de Xbox (ros2_ws/src/drivers/brazo_driver): un botón normalmente
cerrado (NC) entre el pin 1 (3,3 V) y el pin 7 (GPIO09, gpiochip0 línea 144) del header de 40
pines de la Jetson. Suelta, el contacto está cerrado y el pin lee 1; presionada (o con el cable
cortado) lee 0. Por eso un cable suelto también para el brazo: "falla segura".

Se lee con el programa `gpioget` (paquete gpiod), que toma el pin solo un instante: así la base,
el driver de Xbox y este nodo pueden leerlo a la vez (si coinciden, uno sale "ocupado" y
reintenta).

Reglas (las mismas del driver del equipo):
  - antirrebote: hacen falta `antirrebote` lecturas seguidas en 0 para darla por presionada (una
    lectura suelta en 0 puede ser ruido); una sola lectura en 1 la da por suelta.
  - al arrancar se toma como presionada hasta la primera lectura en 1.
  - si el pin no se puede leer durante `fallo` segundos (gpioget no existe, sin permisos, cable
    del header suelto...), se toma como PRESIONADA.

Ojo: solo sirve si quien manda al Arduino lee la seta. El puerto serie del Arduino lo usa un solo
programa a la vez (brazo_real_node O el driver de Xbox), y los dos la leen; los scripts de prueba
de firmware/ (probar_arduino.py, prueba_exigente.py) no. Para que pare pase lo que pase en el PC,
la seta tendría que ir también a un pin del Arduino (firmware/README.md).
"""
import subprocess
import threading
import time


class Antirrebote:
    """El estado de la seta a partir de las lecturas del pin ('0', '1' o None si falló)."""

    def __init__(self, antirrebote=3, fallo=1.0, ahora=None):
        self.antirrebote = max(1, int(antirrebote))
        self.fallo = fallo
        self.ultimo_ok = time.monotonic() if ahora is None else ahora
        self.ceros = 0
        # Hasta la primera lectura buena se toma como presionada (falla segura: si nunca se
        # puede leer, el brazo nunca se mueve).
        self.presionada = True
        self.estado = 'leyendo...'

    def leer(self, valor, ahora):
        """Agrega una lectura y devuelve si está presionada."""
        if valor == '1':                      # NC: suelta = 1 (basta una lectura)
            self.ultimo_ok = ahora
            self.ceros = 0
            self.presionada = False
            self.estado = 'suelta'
        elif valor == '0':                    # presionada: `antirrebote` lecturas seguidas
            self.ultimo_ok = ahora
            self.ceros += 1
            if self.ceros >= self.antirrebote:
                self.presionada = True
                self.estado = 'PRESIONADA'
        elif ahora - self.ultimo_ok > self.fallo:
            self.presionada = True
            self.estado = 'SIN LECTURA'
        return self.presionada


def gpioget(chip, linea):
    """Lee un pin con gpioget. Devuelve ('0' | '1' | None, mensaje de error)."""
    try:
        r = subprocess.run(['gpioget', chip, str(linea)], capture_output=True, text=True,
                           timeout=0.5)
    except (OSError, subprocess.TimeoutExpired) as error:
        return None, str(error)
    valor = r.stdout.strip()
    return (valor if valor in ('0', '1') else None), r.stderr.strip()


class Seta(threading.Thread):
    """Hilo que lee la seta cada `periodo` s. `al_cambiar(presionada, estado, error)` se llama
    cada vez que cambia (desde este hilo)."""

    def __init__(self, chip='gpiochip0', linea=144, periodo=0.05, fallo=1.0, antirrebote=3,
                 al_cambiar=None, leer=gpioget):
        super().__init__(daemon=True)
        self.chip, self.linea, self.periodo = chip, linea, periodo
        self.filtro = Antirrebote(antirrebote, fallo)
        self.al_cambiar = al_cambiar
        self.leer_pin = leer
        self.seguir = True

    @property
    def presionada(self):
        return self.filtro.presionada

    @property
    def estado(self):
        return self.filtro.estado

    def run(self):
        antes = None
        while self.seguir:
            valor, error = self.leer_pin(self.chip, self.linea)
            ahora = self.filtro.leer(valor, time.monotonic())
            if ahora != antes and self.al_cambiar is not None:
                self.al_cambiar(ahora, self.filtro.estado, error)
            antes = ahora
            time.sleep(self.periodo)

    def parar(self):
        self.seguir = False
