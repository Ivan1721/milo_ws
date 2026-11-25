"""El brazo real por el Arduino Mega: conversiones y protocolo, sin ROS (se prueba solo).

Lo usa brazo_real_node.py (el nodo ROS que reemplaza a los controladores de Gazebo) y lo prueba
test/test_brazo_real.py. El Arduino (firmware/brazo_mega/brazo_mega.ino) solo cuenta PASOS; aquí se
pasa de radianes (lo que usan ROS y MoveIt) a pasos y al revés, se arman las líneas de comando y
se leen sus respuestas.

Cuántos pasos es un radián: el motor da `pasos_motor` pasos por vuelta (200 en un motor de 1.8°),
el TB6600 divide cada paso en `micropasos` (sus interruptores: 1, 2, 4, 8, 16, 32) y la caja
reductora hace que el eje de la articulación gire `reduccion` veces menos que el motor (38.4).
Entonces una vuelta de la articulación (2π rad) son pasos_motor x micropasos x reduccion pasos:

    pasos = signo * (q - cero) * pasos_motor * micropasos * reduccion / (2π)

con q el ángulo de la articulación [rad], `cero` el ángulo en que el Arduino cuenta 0 (la pose
en que se encendió o se mandó Z) y `signo` = -1 si el motor gira al revés que el URDF.
Ejemplo: 200 x 8 x 38.4 / 360 = 170.7 pasos por grado.

También está ArduinoFalso: hace lo mismo que el firmware v3 (mismas respuestas y movimiento en el
tiempo, con la misma rampa de aceleración) para probar el nodo y MoveIt sin el Arduino ni los
motores (ros2 run andesrobot_arm arduino_falso).
"""
import math
import os
import termios
import time
from dataclasses import dataclass

ARTICULACIONES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
LIFT = 'vertical_lift_joint'
DEDOS = ('right_finger_joint', 'left_finger_joint')
# Posición de los dedos (la del dedo derecho; el izquierdo es la misma con el signo cambiado),
# igual que en la simulación (agarre.py): abierta 0.016 m, cerrada -0.007 m.
DEDO_ABIERTA = 0.016
DEDO_CERRADA = -0.007


@dataclass
class Articulacion:
    """Datos de un motor (de config/brazo_real.yaml)."""
    nombre: str
    pasos_motor: int = 200
    micropasos: int = 8
    reduccion: float = 38.4
    invertir: bool = False
    cero: float = 0.0          # [rad] ángulo de la articulación cuando el Arduino cuenta 0

    @property
    def pasos_por_rad(self):
        return self.pasos_motor * self.micropasos * self.reduccion / (2.0 * math.pi)

    def a_pasos(self, q):
        """Ángulo [rad] -> pasos (entero, lo que entiende el Arduino)."""
        signo = -1.0 if self.invertir else 1.0
        return int(round(signo * (q - self.cero) * self.pasos_por_rad))

    def a_rad(self, pasos):
        """Pasos -> ángulo [rad]."""
        signo = -1.0 if self.invertir else 1.0
        return self.cero + signo * pasos / self.pasos_por_rad


def leer_config(datos):
    """Las 6 Articulacion a partir del diccionario de brazo_real.yaml (`articulaciones`: una
    entrada por joint_1..joint_6; lo que falte toma los valores de `comun`)."""
    comun = datos.get('comun', {})
    salida = []
    for nombre in ARTICULACIONES:
        propio = dict(comun, **datos.get('articulaciones', {}).get(nombre, {}))
        salida.append(Articulacion(nombre=nombre, **propio))
    return salida


# --- Protocolo (ver el comienzo de brazo_mega.ino) ---

def cmd_ir(pasos, ms):
    """Línea G: ir a esas 6 posiciones [pasos] llegando en ms milisegundos."""
    if len(pasos) != len(ARTICULACIONES):
        raise ValueError('G necesita 6 posiciones')
    return 'G %d %s' % (max(1, int(round(ms))), ' '.join(str(int(p)) for p in pasos))


def cmd_mover(articulacion, pasos):
    """Línea M: mover solo la articulación (1..6) esos pasos, lento."""
    if not 1 <= articulacion <= len(ARTICULACIONES):
        raise ValueError('articulación de 1 a 6')
    return 'M %d %d' % (articulacion, int(pasos))


def leer_estado(linea):
    """Respuesta de E -> (6 posiciones [pasos], se mueve, ángulo del servo o None). ValueError si
    la línea no es un estado."""
    partes = (linea or '').split()
    if len(partes) != 9 or partes[0] != 'E':
        raise ValueError(f'no es un estado: {linea!r}')
    numeros = [int(x) for x in partes[1:]]
    servo = numeros[7]
    return numeros[:6], numeros[6] == 1, (None if servo < 0 else servo)


class Enlace:
    """El puerto serie hacia el Arduino, con la biblioteca estándar (termios, sin pyserial).

    Se abre en modo "crudo" (sin eco ni traducción de fines de línea), 8 bits sin paridad, a
    115200 baudios, y SIN HUPCL: por defecto Linux baja la señal DTR al cerrar el puerto, y en el
    Mega eso reinicia el Arduino la próxima vez que se abre, que vuelve a contar 0 pasos donde
    esté el brazo (medido el 23-nov: J1 en 162 pasos -> 0 al reabrir). Sin HUPCL el Arduino solo
    se reinicia la primera vez después de enchufarlo. `conectar` dice si se reinició.
    """

    def __init__(self, ruta, baudios=termios.B115200):
        self.ruta = ruta
        self.fd = os.open(ruta, os.O_RDWR | os.O_NOCTTY)
        iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(self.fd)
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL          # sin HUPCL (ver arriba)
        cc[termios.VMIN] = 0          # read() vuelve aunque no haya nada...
        cc[termios.VTIME] = 0         # ...enseguida (se espera con time en leer_linea)
        termios.tcsetattr(self.fd, termios.TCSANOW, [0, 0, cflag, 0, baudios, baudios, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.resto = b''

    def leer_linea(self, espera=0.5):
        """Una línea (sin el fin de línea) o None si no llega en `espera` s."""
        fin = time.monotonic() + espera
        while True:
            if b'\n' in self.resto:
                linea, self.resto = self.resto.split(b'\n', 1)
                return linea.decode(errors='replace').strip()
            if time.monotonic() > fin:
                return None
            trozo = os.read(self.fd, 256)
            if trozo:
                self.resto += trozo
            else:
                time.sleep(0.002)

    def conectar(self, espera=4.0):
        """Primer contacto: (respuesta a P, se_reinicio). Si el Arduino contesta enseguida, no se
        reinició (sigue contando donde estaba); si no, se está reiniciando: se espera su LISTO
        y se pregunta de nuevo (cuenta desde 0)."""
        respuesta = self.preguntar('P', 0.3)
        if respuesta and respuesta.startswith('PONG'):
            return respuesta, False
        self.esperar_listo(espera)
        return self.preguntar('P', 1.0), True

    def esperar_listo(self, espera=4.0):
        """Espera el "LISTO ..." del arranque. Devuelve la línea o None."""
        fin = time.monotonic() + espera
        while time.monotonic() < fin:
            linea = self.leer_linea(fin - time.monotonic())
            if linea and linea.startswith('LISTO'):
                return linea
        return None

    def preguntar(self, comando, espera=0.3):
        """Manda una línea y devuelve la respuesta (None si no llega). Descarta antes lo que
        hubiera quedado sin leer."""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.resto = b''
        os.write(self.fd, (comando + '\n').encode())
        fin = time.monotonic() + espera
        while True:
            linea = self.leer_linea(max(0.0, fin - time.monotonic()))
            # Un "LISTO" suelto (el Arduino recién reiniciado) no es la respuesta: se salta.
            if linea is None or not linea.startswith('LISTO'):
                return linea

    def cerrar(self):
        os.close(self.fd)


# --- Garra: posición de los dedos <-> ángulo del servo ---

def dedo_a_servo(q, angulo_abierta, angulo_cerrada):
    """Posición del dedo derecho [m] -> ángulo del servo [grados], recta entre los dos ángulos
    medidos (abierta = DEDO_ABIERTA, cerrada = DEDO_CERRADA), sin salirse de ellos."""
    f = (q - DEDO_CERRADA) / (DEDO_ABIERTA - DEDO_CERRADA)
    f = min(1.0, max(0.0, f))
    return int(round(angulo_cerrada + f * (angulo_abierta - angulo_cerrada)))


def servo_a_dedo(angulo, angulo_abierta, angulo_cerrada):
    """Ángulo del servo -> posición del dedo derecho [m] (lo inverso de dedo_a_servo)."""
    if angulo_abierta == angulo_cerrada:
        return DEDO_ABIERTA
    f = (angulo - angulo_cerrada) / (angulo_abierta - angulo_cerrada)
    f = min(1.0, max(0.0, f))
    return DEDO_CERRADA + f * (DEDO_ABIERTA - DEDO_CERRADA)


# --- Trayectorias (lo que manda MoveIt) ---

def interpolar(tiempos, puntos, t):
    """Posición de la trayectoria en el instante t [s]: recta entre los dos puntos que lo rodean.

    tiempos  instantes de cada punto [s] (time_from_start), crecientes
    puntos   lista de listas: la posición de cada articulación en cada punto
    Antes del primero da el primero; después del último, el último.
    """
    if t <= tiempos[0]:
        return list(puntos[0])
    for k in range(1, len(tiempos)):
        if t <= tiempos[k]:
            t0, t1 = tiempos[k - 1], tiempos[k]
            f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return [a + f * (b - a) for a, b in zip(puntos[k - 1], puntos[k])]
    return list(puntos[-1])


# --- Arduino de mentira, para probar sin hardware ---

class ArduinoFalso:
    """Contesta como brazo_mega.ino v2 y mueve sus 6 contadores de pasos con el tiempo, con la
    misma rampa de aceleración (mismas reglas, recalculadas cada TICK s; ver el comienzo del
    .ino). `reloj` es una función que da segundos (time.monotonic por defecto; en las pruebas se
    le pasa uno falso)."""

    VEL_MAX = 2000.0
    VEL_INICIO = 100.0
    VEL_PARADA = 400.0
    MARGEN_FRENO = 0.9
    TICK = 0.002
    G_MIN = 0.02
    WATCHDOG = 0.5

    def __init__(self, reloj=time.monotonic):
        self.reloj = reloj
        n = len(ARTICULACIONES)
        self.pos = [0.0] * n
        self.destino = [0] * n
        self.vel = [0.0] * n
        self.dir = [1] * n
        self.por_m = [False] * n
        self.plazo = [0.0] * n
        self.vel_prueba = 200.0
        self.acel = 4000.0
        self.servo = -1
        self.modo_g = False
        self.t = reloj()
        self.ultimo_comando = self.t

    def _tick(self, ahora, dt):
        """Un tick: rampa (como rampas() del .ino) y avance continuo."""
        for j in range(len(self.pos)):
            falta = self.destino[j] - self.pos[j]
            if abs(falta) < 0.5 and self.vel[j] <= self.VEL_PARADA:
                self.pos[j] = float(self.destino[j])
                self.vel[j] = 0.0
                continue
            lado = 1 if falta > 0 else -1
            if lado != self.dir[j] and self.vel[j] > self.VEL_INICIO:
                objetivo = 0.0
            else:
                if self.vel[j] <= self.VEL_INICIO:
                    self.dir[j] = lado
                d = abs(falta)
                frenable = math.sqrt(2.0 * self.MARGEN_FRENO * self.acel * d)
                if self.por_m[j]:
                    objetivo = min(self.vel_prueba, frenable)
                else:
                    queda = self.plazo[j] - ahora
                    objetivo = d / max(queda, self.G_MIN)
                    if queda <= 0:
                        objetivo = min(objetivo, frenable)
                objetivo = min(objetivo, self.VEL_MAX)
            dv = self.acel * dt
            v = self.vel[j]
            self.vel[j] = min(objetivo, v + dv) if v < objetivo else max(objetivo, v - dv)
            paso = self.dir[j] * self.vel[j] * dt
            nueva = self.pos[j] + paso
            # despacio, no se pasa del destino (el .ino no da pasos de más a baja velocidad)
            if self.vel[j] <= self.VEL_PARADA and (self.destino[j] - self.pos[j]) * \
                    (self.destino[j] - nueva) <= 0:
                nueva = float(self.destino[j])
            self.pos[j] = nueva

    def avanzar(self):
        """Simula, tick por tick, desde la última vez hasta ahora."""
        ahora = self.reloj()
        while self.t + self.TICK <= ahora:
            self.t += self.TICK
            if self.modo_g and self.t - self.ultimo_comando > self.WATCHDOG and self.moviendo():
                self.parar()
            self._tick(self.t, self.TICK)
            if not self.moviendo():
                self.modo_g = False

    def parar(self):
        self.destino = [int(round(p)) for p in self.pos]
        self.pos = [float(d) for d in self.destino]
        self.vel = [0.0] * len(self.vel)
        self.modo_g = False

    def moviendo(self):
        return any(abs(d - p) >= 0.5 or v > 0 for d, p, v in zip(self.destino, self.pos, self.vel))

    def responder(self, linea):
        """Una línea de comando -> la línea de respuesta (sin el fin de línea)."""
        self.avanzar()
        self.ultimo_comando = self.reloj()
        partes = linea.strip().split()
        if not partes:
            return None
        c, args = partes[0], partes[1:]
        try:
            nums = [int(x) for x in args]
        except ValueError:
            return 'ERR numero mal escrito'
        if c == 'P':
            return 'PONG brazo_mega v3'
        if c == 'E':
            return 'E ' + ' '.join(str(int(round(p))) for p in self.pos) + \
                f' {1 if self.moviendo() else 0} {self.servo}'
        if c == 'G':
            if len(nums) != 7 or nums[0] <= 0:
                return 'ERR uso: G ms p1 p2 p3 p4 p5 p6'
            plazo = self.reloj() + nums[0] / 1000.0
            for j, d in enumerate(nums[1:]):
                self.destino[j] = d
                self.plazo[j] = plazo
                self.por_m[j] = False
            self.modo_g = True
            return 'OK G'
        if c == 'M':
            if len(nums) != 2 or not 1 <= nums[0] <= 6:
                return 'ERR uso: M j pasos  (j = 1..6)'
            j = nums[0] - 1
            base = self.destino[j] if self.vel[j] > 0 else int(round(self.pos[j]))
            self.destino[j] = base + nums[1]
            self.por_m[j] = True
            return f'OK M {nums[0]} {self.destino[j]}'
        if c == 'V' and len(nums) == 1 and nums[0] >= 1:
            self.vel_prueba = min(self.VEL_MAX, float(nums[0]))
            return f'OK V {int(self.vel_prueba)}'
        if c == 'A' and len(nums) == 1 and nums[0] >= 100:
            self.acel = min(100000.0, float(nums[0]))
            return f'OK A {int(self.acel)}'
        if c == 'A':
            return 'ERR uso: A pasos_por_s2  (100 o mas)'
        if c == 'X':
            self.parar()
            return 'OK X'
        if c == 'Z':
            if self.moviendo():
                return 'ERR Z solo con el brazo quieto (X primero)'
            self.pos = [0.0] * len(self.pos)
            self.destino = [0] * len(self.destino)
            return 'OK Z'
        if c == 'S' and len(nums) == 1:
            self.servo = min(180, max(0, nums[0]))
            return f'OK S {self.servo}'
        if c == 'D':
            self.servo = -1
            return 'OK D'
        return 'ERR comando desconocido: ' + linea.strip()
