"""Driver del brazo de Milo con el control de Xbox (arm_driver_node).

Habla con el Arduino Mega del brazo (firmware/brazo_mega, protocolo de texto a 115200) y abre una
ventana con el estado de las 6 articulaciones. Con el control de Xbox:

  - A (botón ``btn_cambiar``): selecciona la articulación siguiente (J1 -> J2 -> ... -> J6 -> J1).
  - Joystick izquierdo, arriba/abajo (``eje``): mueve la articulación seleccionada; cuanto más se
    empuja, más rápido (hasta ``vel_max`` pasos/s). Al soltarlo frena con la rampa.
  - Y (``btn_home``): con el brazo quieto, la pose actual pasa a ser el nuevo home (todo en 0).
  - B (``btn_parar``): parada en seco de todo (emergencia).
  - Gatillo derecho RT (``eje_deadman``): HOMBRE MUERTO. El brazo solo se mueve mientras está
    apretado (más de ``umbral_deadman``); al soltarlo frena con la rampa.
  - Joystick derecho, arriba/abajo (``eje_servo``): mueve el servo de la garra (pin 9 del Mega,
    comando S del firmware). Es por velocidad: cuanto más se empuja, más rápido cambia el ángulo
    (hasta ``servo_vel`` °/s) y al soltarlo el servo se queda donde está. También necesita RT
    apretado y se bloquea con la seta. Límites: ``servo_min`` .. ``servo_max``. El servo queda
    suelto al encender; el primer movimiento parte de ``servo_inicial``.

Seta de emergencia física: la misma del driver de la hoverboard (botón NC entre el pin 1 y el
pin 7 del header de la Jetson, gpiochip0 línea 144), leída con gpioget a 20 Hz: suelta = 1,
presionada o cable cortado = 0 (con antirrebote de ``estop_gpio_debounce`` lecturas). Al
presionarla, parada en seco (X) y el brazo queda bloqueado. Para volver a mover: soltar la seta
y además soltar el gatillo (rearme: así no arranca solo si el gatillo quedó apretado). Si el pin
no se puede leer durante ``estop_gpio_fail`` s, se toma como presionada (falla segura).

Cómo mueve: unas 20 veces por segundo lee la posición (comando E) y, mientras el joystick esté
empujado, manda un G con la articulación seleccionada un poco adelante (velocidad x ``adelanto``)
y las demás donde están. Si este programa se cae, el perro guardián del Arduino para el brazo.
Al soltar el joystick manda un último G con el punto de frenado (v²/2a) y plazo ya vencido: el
firmware frena con su rampa para llegar justo ahí (sin pasarse y volver).

El puerto se abre SIN HUPCL (como brazo_real.Enlace de andesrobot_arm): con HUPCL, Linux baja la
señal DTR al cerrar el puerto y el Mega se reinicia la próxima vez que se abre, perdiendo la
cuenta de pasos (y el home). Así el Arduino solo se reinicia la primera vez después de
enchufarlo; la ventana avisa si se reinició.

Integrado en milo_ws (25-nov-2026) desde la entrega del equipo (brazo_milo.zip): se agregó la
el puerto sin HUPCL y el aviso de reinicio. La garra con el joystick derecho es del equipo
(segunda entrega, brazo_milo_servo.zip, 25-nov-2026), que reemplazó a la de LB/RB que se había
agregado aquí; lo demás quedó igual.

Los números de botones y ejes son los del control conectado por Bluetooth con hid-generic
(/dev/input/js0); si alguno no coincide, la ventana muestra el número de cada botón que se pulsa
y se cambian con los parámetros.
"""

import array
import fcntl
import math
import os
import select
import struct
import subprocess
import termios
import threading
import time
import tkinter as tk

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

N = 6
JS_EVENT = struct.Struct('IhBB')      # tiempo [ms], valor, tipo, número
JS_BOTON, JS_EJE, JS_INIT = 0x01, 0x02, 0x80
MARGEN_FRENO = 0.9                    # el mismo del firmware


class Joystick(threading.Thread):
    """Lee /dev/input/js0 en segundo plano; se reconecta solo si el control se apaga."""

    def __init__(self, ruta, al_pulsar):
        super().__init__(daemon=True)
        self.ruta = ruta
        self.al_pulsar = al_pulsar
        self.ejes = {}
        self.nombre = ''
        self.conectado = False

    def run(self):
        while True:
            try:
                with open(self.ruta, 'rb', buffering=0) as f:
                    self.nombre = self._leer_nombre(f)
                    self.conectado = True
                    self.ejes = {}
                    while True:
                        if not select.select([f], [], [], 0.5)[0]:
                            continue
                        datos = f.read(JS_EVENT.size)
                        if not datos:
                            raise OSError('sin datos')
                        _, valor, tipo, numero = JS_EVENT.unpack(datos)
                        if tipo & JS_EJE:
                            self.ejes[numero] = valor / 32767.0
                        elif tipo & JS_BOTON and valor == 1 and not tipo & JS_INIT:
                            self.al_pulsar(numero)
            except OSError:
                pass
            self.conectado = False
            self.ejes = {}
            time.sleep(1.0)

    @staticmethod
    def _leer_nombre(f):
        try:
            buf = array.array('B', [0] * 128)
            fcntl.ioctl(f, (2 << 30) | (128 << 16) | (ord('j') << 8) | 0x13, buf)  # JSIOCGNAME
            return buf.tobytes().split(b'\0', 1)[0].decode(errors='replace')
        except OSError:
            return 'joystick'

    def eje(self, numero):
        return self.ejes.get(numero, 0.0)


class BrazoDriver(Node):

    def __init__(self):
        super().__init__('arm_driver_node')
        p = self.declare_parameter
        # Por id: es el CH340 del Mega del brazo (el de la hoverboard es usb-1a86_USB_Serial).
        p('port', '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0')
        p('joystick', '/dev/input/js0')
        p('vel_max', 2000.0)          # [pasos/s] con el joystick a fondo (tope del firmware: 2000)
        p('aceleracion', 4000.0)      # [pasos/s²] se le manda al firmware al conectar (comando A)
        p('zona_muerta', 0.15)
        p('adelanto', 0.2)            # [s] qué tan adelante va el destino de cada G
        p('periodo', 0.05)            # [s] lazo de control (20 Hz)
        p('eje', 1)                   # joystick izquierdo vertical
        p('invertir_eje', True)       # en js0 arriba es negativo: arriba = sentido positivo
        p('btn_cambiar', 0)           # A
        p('btn_parar', 1)             # B
        p('btn_home', 4)              # Y
        # Hombre muerto: gatillo derecho RT (eje 4 = ABS_GAS; suelto -1, a fondo +1). LT es el 5.
        p('eje_deadman', 4)
        p('umbral_deadman', 0.5)      # 0 = suelto, 1 = a fondo
        # Servo de la garra (pin 9 del Mega) con el joystick derecho vertical (eje 3; el 2 es el
        # horizontal).
        p('eje_servo', 3)
        p('invertir_eje_servo', True)  # arriba = ángulo mayor
        p('servo_vel', 90.0)          # [°/s] con el joystick a fondo
        p('servo_min', 0)             # [°] (el firmware además limita a 0..180)
        p('servo_max', 180)
        p('servo_inicial', 90)        # [°] desde dónde parte si el servo estaba suelto
        # Seta física (como en hoverboard_bridge). estop_gpio_line < 0 la desactiva.
        p('estop_gpio_chip', 'gpiochip0')
        p('estop_gpio_line', 144)     # pin 7 del header = GPIO09 = PAC.06
        p('estop_gpio_period', 0.05)  # [s]
        p('estop_gpio_fail', 1.0)     # [s] sin poder leer el pin -> presionada
        p('estop_gpio_debounce', 3)   # lecturas seguidas en 0 para darla por presionada
        # Solo para mostrar grados en la ventana: 200 pasos x 1/8 micropaso x 38.4:1 / 360°.
        p('pasos_por_grado', [170.667] * N)
        p('nombres', ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in (
            'port', 'joystick', 'vel_max', 'aceleracion', 'zona_muerta', 'adelanto', 'periodo',
            'eje', 'invertir_eje', 'btn_cambiar', 'btn_parar', 'btn_home', 'pasos_por_grado',
            'nombres', 'eje_deadman', 'umbral_deadman', 'estop_gpio_chip', 'estop_gpio_line',
            'estop_gpio_period', 'estop_gpio_fail', 'estop_gpio_debounce', 'eje_servo',
            'invertir_eje_servo', 'servo_vel', 'servo_min', 'servo_max', 'servo_inicial')}

        self.pub = self.create_publisher(JointState, '/arm/joint_steps', 10)

        self.lock = threading.Lock()  # estado compartido con la ventana
        self.ser_lock = threading.Lock()  # una pregunta/respuesta a la vez por el puerto
        self.ser = None
        self.pos = [0] * N
        self.moviendo = False
        self.sel = 0
        self.v_est = [0.0] * N        # velocidad estimada [pasos/s] (sigue la rampa del firmware)
        self.activa = None            # articulación que se está moviendo con el joystick
        self.pedidos = []             # botones pulsados, los atiende el lazo de control
        self.ultimo_boton = None
        self.mensaje = 'Conectando...'
        self.estado_conexion = 'desconectado'
        self.seta = False             # seta presionada AHORA (la lee el hilo _leer_seta)
        self.seta_estado = 'leyendo...'
        self.bloqueado = False        # parada por la seta, hasta el rearme
        self.servo = -1               # ángulo del servo según el Arduino (-1 = suelto)
        self.servo_obj = None         # [°] ángulo pedido con el joystick derecho (float)

        if self.cfg['estop_gpio_line'] >= 0:
            threading.Thread(target=self._leer_seta, daemon=True).start()
        else:
            self.seta_estado = 'DESACTIVADA'
            self.get_logger().warn('Seta de emergencia DESACTIVADA (estop_gpio_line < 0)')

        self.js = Joystick(self.cfg['joystick'], self._boton)
        self.js.start()
        threading.Thread(target=self._lazo, daemon=True).start()

    # ---------------------------------------------------------------- serie

    def _conectar(self):
        try:
            ser = serial.Serial(self.cfg['port'], 115200, timeout=0.3)
            # Sin HUPCL: al CERRAR el puerto DTR queda arriba y la próxima apertura no reinicia
            # el Mega (ver arriba). La primera apertura después de enchufarlo sí lo reinicia.
            atributos = termios.tcgetattr(ser.fd)
            atributos[2] &= ~termios.HUPCL
            termios.tcsetattr(ser.fd, termios.TCSANOW, atributos)
        except (serial.SerialException, termios.error) as e:
            self._msg(f'No se pudo abrir {self.cfg["port"]}: {e}')
            return None
        ser.reset_input_buffer()
        self.ser = ser
        # ¿Se reinició? Si contesta P enseguida, no (sigue con su cuenta de pasos). Si no, se está
        # reiniciando: esperar su "LISTO" (~2 s) y preguntar de nuevo.
        r = self._cmd('P')
        reinicio = not r.startswith('PONG')
        if reinicio:
            fin = time.monotonic() + 4.0
            while time.monotonic() < fin:
                with self.ser_lock:
                    linea = ser.readline().decode(errors='replace').strip()
                if linea.startswith('LISTO'):
                    break
            r = self._cmd('P')
        if not r.startswith('PONG brazo_mega'):
            self._msg(f'El Arduino no contesta como brazo_mega (contestó {r!r})')
            ser.close()
            self.ser = None
            return None
        self._cmd(f'A {int(self.cfg["aceleracion"])}')
        self.estado_conexion = r.replace('PONG ', '')
        if not r.startswith('PONG brazo_mega v3'):
            self.get_logger().warn(
                f'El Arduino tiene {self.estado_conexion}: con el cableado de positivo común de '
                'los TB6600 hace falta la v3 (firmware/cargar_firmware.sh).')
        if reinicio:
            self._msg('El Arduino se REINICIÓ: cuenta 0 pasos donde esté el brazo. Si no está en '
                      'su pose cero, llévalo con el control y pulsa Y.')
        else:
            self._msg('Listo (el Arduino siguió con su cuenta). A: articulación · joystick izq.: '
                      'mover · der.: garra · Y: home · B: parar')
        self.get_logger().info(f'Conectado a {self.cfg["port"]} ({self.estado_conexion}, '
                               f'{"se reinició" if reinicio else "sin reiniciar"})')
        return ser

    def _cmd(self, linea):
        with self.ser_lock:
            self.ser.reset_input_buffer()
            self.ser.write((linea + '\n').encode())
            r = self.ser.readline().decode(errors='replace').strip()
            # Un "LISTO" suelto (el Arduino recién reiniciado) no es la respuesta: leer otra.
            if r.startswith('LISTO'):
                r = self.ser.readline().decode(errors='replace').strip()
            return r

    # ---------------------------------------------------------------- control

    def _boton(self, numero):
        with self.lock:
            self.ultimo_boton = numero
            self.pedidos.append(numero)

    def _msg(self, texto):
        with self.lock:
            self.mensaje = texto

    def _leer_seta(self):
        """Hilo: lee el pin de la seta con gpioget (igual que hoverboard_bridge).

        gpioget toma el pin solo un instante, así este driver, el de la hoverboard y boton.sh
        pueden leerlo a la vez; si coinciden, sale "ocupado" y se reintenta.
        """
        chip, linea = self.cfg['estop_gpio_chip'], str(self.cfg['estop_gpio_line'])
        debounce = max(1, int(self.cfg['estop_gpio_debounce']))
        ultimo_ok = time.monotonic()
        ceros = 0
        while rclpy.ok():
            try:
                r = subprocess.run(['gpioget', chip, linea], capture_output=True, text=True,
                                   timeout=0.5)
                valor, err = r.stdout.strip(), r.stderr.strip()
            except (OSError, subprocess.TimeoutExpired) as e:
                valor, err = '', str(e)
            if valor in ('0', '1'):
                ultimo_ok = time.monotonic()
                ceros = ceros + 1 if valor == '0' else 0   # NC: suelta = 1, presionada = 0
                self.seta = ceros >= debounce
                self.seta_estado = 'PRESIONADA' if self.seta else 'suelta'
            elif time.monotonic() - ultimo_ok > self.cfg['estop_gpio_fail']:
                if not self.seta:
                    self.get_logger().error(f'No se puede leer la seta ({chip} línea {linea}): '
                                            f'{err or valor!r} -> se toma como presionada')
                self.seta = True
                self.seta_estado = 'SIN LECTURA'
            time.sleep(self.cfg['estop_gpio_period'])

    def _deadman(self):
        """Gatillo del hombre muerto, 0 (suelto) .. 1 (a fondo). Sin datos = suelto."""
        return (self.js.ejes.get(self.cfg['eje_deadman'], -1.0) + 1.0) / 2.0

    def _deadman_ok(self):
        return self._deadman() > self.cfg['umbral_deadman']

    def _stick(self):
        return self._eje_con_zona(self.cfg['eje'], self.cfg['invertir_eje'])

    def _stick_servo(self):
        return self._eje_con_zona(self.cfg['eje_servo'], self.cfg['invertir_eje_servo'])

    def _eje_con_zona(self, numero, invertir):
        x = self.js.eje(numero)
        if invertir:
            x = -x
        zm = self.cfg['zona_muerta']
        if abs(x) < zm:
            return 0.0
        return math.copysign((abs(x) - zm) / (1.0 - zm), x)

    def _lazo(self):
        periodo = self.cfg['periodo']
        while rclpy.ok():
            if self.ser is None:
                self.estado_conexion = 'desconectado'
                if self._conectar() is None:
                    time.sleep(2.0)
                    continue
            t0 = time.monotonic()
            try:
                self._paso(periodo)
            except Exception as e:
                if not rclpy.ok():
                    return                # cerrando el programa
                if not isinstance(e, (serial.SerialException, OSError, ValueError)):
                    raise
                self.get_logger().error(f'Serie: {e}')
                self._msg(f'Se perdió el Arduino: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.activa = None
                self.v_est = [0.0] * N
                self.servo_obj = None     # el Mega se reinicia al reconectar: servo suelto
            time.sleep(max(0.0, periodo - (time.monotonic() - t0)))

    def _leer_estado(self):
        r = self._cmd('E')
        partes = r.split()
        if len(partes) != N + 3 or partes[0] != 'E':
            raise ValueError(f'respuesta rara a E: {r!r}')
        pos = [int(v) for v in partes[1:N + 1]]
        servo = int(partes[N + 2])
        with self.lock:
            self.pos = pos
            self.moviendo = partes[N + 1] == '1'
            self.servo = servo
        if servo < 0:
            self.servo_obj = None     # suelto (por ejemplo, el Mega se reinició)
        return pos

    def _paso(self, dt):
        pos = self._leer_estado()
        with self.lock:
            pedidos, self.pedidos = self.pedidos, []

        stick = self._stick()
        stick_servo = self._stick_servo()
        deadman = self._deadman_ok()

        # Seta: al presionarla, parada en seco y bloqueo hasta soltarla + soltar el gatillo.
        if self.seta and not self.bloqueado:
            self._cmd('X')
            self.activa = None
            self.v_est = [0.0] * N
            self.bloqueado = True
            self.get_logger().warn('Seta presionada: brazo detenido')
            self._msg('PARADA DE EMERGENCIA (seta). Para seguir: suelta la seta y el gatillo RT.')
        elif self.bloqueado and not self.seta:
            if deadman:
                self._msg('Seta suelta. Suelta el gatillo RT para rearmar.')
            else:
                self.bloqueado = False
                self.get_logger().info('Seta suelta y gatillo suelto: rearmado')
                self._msg('Rearmado. Aprieta RT y mueve el joystick.')
        if self.bloqueado or not deadman:
            stick = 0.0               # sin hombre muerto (o con la seta) no se mueve nada
            stick_servo = 0.0

        for b in pedidos:
            if b == self.cfg['btn_parar']:
                self._cmd('X')
                self.activa = None
                self.v_est = [0.0] * N
                self._msg('PARADA (B). Todo quieto donde está.')
            elif b == self.cfg['btn_cambiar']:
                with self.lock:
                    self.sel = (self.sel + 1) % N
                self._msg(f'Seleccionada {self.cfg["nombres"][self.sel]}')
            elif b == self.cfg['btn_home']:
                if self.moviendo or self.activa is not None:
                    self._msg('Home: primero suelta el joystick y espera a que se detenga.')
                else:
                    r = self._cmd('Z')
                    if r == 'OK Z':
                        pos = self._leer_estado()
                        self._msg('Nuevo HOME: la pose actual es ahora 0 en todas las '
                                  'articulaciones.')
                        self.get_logger().info('Nuevo home (Z)')
                    else:
                        self._msg(f'No se pudo fijar el home: {r}')

        a = self.cfg['aceleracion']
        sel = self.sel
        # Si se cambió de articulación con la anterior todavía frenando, esperar a que pare.
        otra_frenando = self.activa is not None and self.activa != sel
        if stick != 0.0 and not (otra_frenando and self.moviendo):
            if otra_frenando:
                self.v_est[self.activa] = 0.0
            v = stick * self.cfg['vel_max']
            # La velocidad real sube/baja con la rampa del firmware: seguirla para el frenado.
            dv = a * dt
            self.v_est[sel] = max(min(v, self.v_est[sel] + dv), self.v_est[sel] - dv)
            destino = list(pos)
            destino[sel] = pos[sel] + int(round(v * self.cfg['adelanto']))
            self._g(int(self.cfg['adelanto'] * 1000), destino)
            self.activa = sel
        elif self.activa is not None:
            # Soltó el joystick: destino = donde alcanza a frenar, con el plazo ya vencido (1 ms)
            # para que el firmware frene "para llegar justo".
            j = self.activa
            v = self.v_est[j]
            destino = list(pos)
            destino[j] = pos[j] + int(math.copysign(v * v / (2 * MARGEN_FRENO * a), v))
            self._g(1, destino)
            self.activa = None
            self.v_est[j] = 0.0

        self._mover_servo(stick_servo, dt)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.cfg['nombres'])
        msg.position = [float(x) for x in pos]
        self.pub.publish(msg)

    def _mover_servo(self, stick, dt):
        """Joystick derecho: cambia el ángulo del servo a stick x servo_vel °/s (comando S).

        Solo se manda un S cuando cambia el ángulo entero, así no se llena el puerto.
        """
        if stick == 0.0:
            return
        lo, hi = self.cfg['servo_min'], self.cfg['servo_max']
        if self.servo_obj is None:    # suelto: parte del ángulo inicial
            self.servo_obj = float(self.servo if self.servo >= 0 else self.cfg['servo_inicial'])
        self.servo_obj = min(hi, max(lo, self.servo_obj + stick * self.cfg['servo_vel'] * dt))
        angulo = int(round(self.servo_obj))
        if angulo == self.servo:
            return
        r = self._cmd(f'S {angulo}')
        if r.startswith('OK S'):
            with self.lock:
                self.servo = int(r.split()[2])
        else:
            self._msg(f'El Arduino rechazó el S: {r}')

    def _g(self, ms, destino):
        r = self._cmd('G ' + str(max(1, ms)) + ' ' + ' '.join(str(d) for d in destino))
        if r != 'OK G':
            self._msg(f'El Arduino rechazó el G: {r}')

    def cerrar(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self._cmd('X')
                self.ser.close()
            except Exception:
                pass


class Ventana:

    def __init__(self, nodo):
        self.n = nodo
        self.root = tk.Tk()
        self.root.title('Brazo de Milo')
        self.root.configure(bg='#1e1e1e', padx=16, pady=12)
        fuente = ('DejaVu Sans', 13)
        grande = ('DejaVu Sans Mono', 16, 'bold')

        self.lbl_conexion = tk.Label(self.root, font=fuente, bg='#1e1e1e', fg='#bbbbbb',
                                     anchor='w')
        self.lbl_conexion.grid(row=0, column=0, columnspan=4, sticky='we')

        self.filas = []
        for j in range(N):
            nombre = tk.Label(self.root, text=nodo.cfg['nombres'][j], font=grande, width=4)
            pasos = tk.Label(self.root, font=grande, width=10, anchor='e')
            grados = tk.Label(self.root, font=grande, width=10, anchor='e')
            barra = tk.Canvas(self.root, width=220, height=20, highlightthickness=0, bg='#2b2b2b')
            for c, w in enumerate((nombre, pasos, grados, barra)):
                w.grid(row=j + 1, column=c, padx=4, pady=3, sticky='we')
            self.filas.append((nombre, pasos, grados, barra))

        # Servo de la garra (joystick derecho).
        tk.Label(self.root, text='Garra', font=grande, width=4, bg='#2b2b2b', fg='#cccccc').grid(
            row=N + 1, column=0, padx=4, pady=(8, 3), sticky='we')
        self.lbl_servo = tk.Label(self.root, font=grande, width=21, anchor='e', bg='#2b2b2b',
                                  fg='#cccccc')
        self.lbl_servo.grid(row=N + 1, column=1, columnspan=2, padx=4, pady=(8, 3), sticky='we')
        self.barra_servo = tk.Canvas(self.root, width=220, height=20, highlightthickness=0,
                                     bg='#2b2b2b')
        self.barra_servo.grid(row=N + 1, column=3, padx=4, pady=(8, 3), sticky='we')

        self.lbl_msg = tk.Label(self.root, font=fuente, bg='#1e1e1e', fg='#ffd166', anchor='w',
                                wraplength=640, justify='left')
        self.lbl_msg.grid(row=N + 2, column=0, columnspan=4, sticky='we', pady=(10, 2))
        self.lbl_seguridad = tk.Label(self.root, font=('DejaVu Sans', 14, 'bold'), anchor='w',
                                      padx=8, pady=4)
        self.lbl_seguridad.grid(row=N + 5, column=0, columnspan=4, sticky='we', pady=(8, 0))
        ayuda = ('Mantén RT (hombre muerto) + joystick izq. ↑↓: mover · joystick der. ↑↓: garra'
                 '\nA: cambiar articulación   ·   Y: fijar HOME (todo en 0)   ·   B: parar')
        tk.Label(self.root, text=ayuda, font=('DejaVu Sans', 11), bg='#1e1e1e', fg='#888888',
                 anchor='w', justify='left').grid(row=N + 3, column=0, columnspan=4, sticky='we')
        self.lbl_js = tk.Label(self.root, font=('DejaVu Sans', 10), bg='#1e1e1e', fg='#666666',
                               anchor='w')
        self.lbl_js.grid(row=N + 4, column=0, columnspan=4, sticky='we')
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self._refrescar()

    def _refrescar(self):
        n = self.n
        if not rclpy.ok():            # Ctrl+C en la terminal: cerrar la ventana también
            self.root.quit()
            return
        with n.lock:
            pos, sel, msg, ub = list(n.pos), n.sel, n.mensaje, n.ultimo_boton
            servo = n.servo
        stick = n._stick()
        dm = n._deadman()
        dm_ok = dm > n.cfg['umbral_deadman']
        if n.bloqueado or n.seta:
            self.lbl_seguridad.config(text=f'⛔ PARADA DE EMERGENCIA · seta: {n.seta_estado}',
                                      bg='#c1121f', fg='white')
        elif dm_ok:
            self.lbl_seguridad.config(text='● RT apretado: puede moverse   ·   seta: '
                                           f'{n.seta_estado}',
                                      bg='#2a9d3f', fg='white')
        else:
            self.lbl_seguridad.config(text=f'○ Aprieta RT para mover   ·   seta: {n.seta_estado}',
                                      bg='#3a3a3a', fg='#dddddd')
        stick_servo = n._stick_servo()
        if not dm_ok or n.bloqueado:
            stick = 0.0
            stick_servo = 0.0
        self.lbl_servo.config(text=f'{servo:d}°' if servo >= 0 else 'suelto')
        b = self.barra_servo
        b.delete('all')
        if servo >= 0:            # ángulo actual dentro de servo_min..servo_max
            lo, hi = n.cfg['servo_min'], n.cfg['servo_max']
            x = 5 + 210 * (servo - lo) / max(1, hi - lo)
            b.create_rectangle(5, 6, x, 14, fill='#2d6cdf', width=0)
        if stick_servo != 0.0:
            b.create_rectangle(110, 15, 110 + stick_servo * 105, 19, fill='#06d6a0', width=0)
        control = n.js.nombre if n.js.conectado else 'NO conectado'
        self.lbl_conexion.config(text=f'Arduino: {n.estado_conexion}   ·   Control: {control}')
        for j, (nombre, pasos, grados, barra) in enumerate(self.filas):
            activa = j == sel
            fondo = '#2d6cdf' if activa else '#2b2b2b'
            for w in (nombre, pasos, grados):
                w.config(bg=fondo, fg='white' if activa else '#cccccc')
            pasos.config(text=f'{pos[j]:d} p')
            grados.config(text=f'{pos[j] / n.cfg["pasos_por_grado"][j]:+.1f}°')
            barra.delete('all')
            if activa and stick != 0.0:
                medio = 110
                barra.create_rectangle(medio, 3, medio + stick * 105, 17, fill='#06d6a0', width=0)
            barra.create_line(110, 0, 110, 20, fill='#555555')
        self.lbl_msg.config(text=msg)
        self.lbl_js.config(text=f'último botón pulsado: {ub if ub is not None else "-"}   '
                                f'·   eje {n.cfg["eje"]} = {n._stick():+.2f}   '
                                f'·   eje {n.cfg["eje_servo"]} = {n._stick_servo():+.2f}   '
                                f'·   RT = {dm:.2f}')
        self.root.after(50, self._refrescar)


def main():
    rclpy.init()
    nodo = BrazoDriver()

    def girar():
        try:
            rclpy.spin(nodo)
        except ExternalShutdownException:
            pass
    hilo = threading.Thread(target=girar, daemon=True)
    hilo.start()
    if not os.environ.get('DISPLAY'):
        os.environ['DISPLAY'] = ':0'
    try:
        Ventana(nodo).root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Orden de cierre: parar el brazo, apagar rclpy (corta el spin y los lazos) y esperar a
        # que el hilo del spin salga ANTES de destruir el nodo. Destruirlo con el spin corriendo
        # abortaba el proceso ("terminate called without an active exception", 24-nov).
        nodo.cerrar()
        if rclpy.ok():
            rclpy.shutdown()
        hilo.join(timeout=2.0)
        time.sleep(0.2)               # que el lazo de control y la seta vean rclpy.ok() == False
        nodo.destroy_node()


if __name__ == '__main__':
    main()
