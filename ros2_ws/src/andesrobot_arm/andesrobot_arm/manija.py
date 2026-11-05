"""Geometría de la manija de la puerta: de los keypoints de YOLO a una pose 3D y a un movimiento
(sin ROS: se prueba sola con imágenes de profundidad sintéticas).

El detector (scripts/entrenar_manija.py) da, sobre la imagen a color, tres puntos de la manija
de palanca: `eje` (la roseta, donde gira), `agarre` (a ~7 cm del eje) y `punta` (el extremo de la
palanca). Para mover el brazo hay que pasar esos píxeles a puntos 3D, saber hacia dónde mira la
puerta y calcular por dónde mover la pinza. Los pasos, cada uno una función de este archivo:

  1. Un píxel de la imagen a color es un RAYO (todos los puntos que se ven en ese píxel). La
     profundidad está en OTRA cámara (otra resolución, otro campo de visión y, en la cámara real,
     unos centímetros al lado): se camina por el rayo hasta que toca la superficie que mide la
     profundidad (`punto_en_rayo`).
  2. La manija es metálica y brillante: la profundidad suele fallar justo encima de ella. Para esos
     casos se ajusta un plano a la puerta con los píxeles de alrededor (`normal_de_la_puerta`) y
     el punto es donde el rayo cruza ese plano, corrido hacia la cámara lo que sobresale la manija
     (`punto_en_plano`). Y si el rayo pasó al lado de la palanca y midió la hoja de detrás, el
     punto se adelanta lo que sobresale la manija (`corregir_saliente`).
  3. Con los tres puntos y la normal de la puerta se comprueba que la detección tenga sentido
     (largo de la palanca, que esté en el plano de la puerta: `validar_manija`) y se calcula la
     pose previa de la pinza (`pose_previa`) y el arco que recorre la palanca al bajarla
     (`arco_palanca`).
  4. Para llevar la pinza hasta la manija: cuánto le falta al efector final (`distancia_desde_tcp`)
     y la secuencia de acercamiento en dos tramos, primero a la pose previa y después, con una
     detección nueva tomada desde ahí, a pocos centímetros de la manija (`Acercamiento`).

Convenciones: puntos en metros; la normal de la puerta apunta DEL robot HACIA la puerta (la
dirección en que se empuja). La pinza (gripper_tcp) apunta con su eje −X local: con la pinza
mirando a +X del mundo, su orientación es la rotación de 180° en Z, cuaternión (0, 0, 1, 0), la
misma de la pose de ejemplo de docs/BRAZO.md.
"""
import numpy as np

# Largo de la palanca de las puertas de prueba (generar_dataset_manija.py las varía entre 10 y 14
# cm) y cuánto sobresale del plano de la hoja (la manija de puerta_prueba.sdf está a 5 cm).
LARGO_PALANCA = (0.08, 0.18)
SALIENTE = 0.05


def matriz_k(ancho, alto, fov_horizontal):
    """Matriz de la cámara (pinhole) de una cámara simulada: se da el campo de visión [rad]."""
    fx = (ancho / 2.0) / np.tan(fov_horizontal / 2.0)
    return np.array([[fx, 0.0, ancho / 2.0], [0.0, fx, alto / 2.0], [0.0, 0.0, 1.0]])


def rayo(u, v, k):
    """Dirección (unitaria) del rayo que pasa por el píxel (u, v), en el frame de la cámara
    (óptico: Z adelante, X a la derecha, Y abajo)."""
    d = np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])
    return d / np.linalg.norm(d)


def deproyectar(u, v, z, k):
    """Punto 3D [m] de un píxel con profundidad z (distancia a lo largo del eje óptico)."""
    return np.array([(u - k[0, 2]) / k[0, 0] * z, (v - k[1, 2]) / k[1, 1] * z, z])


def punto_en_rayo(profundidad, k_prof, d_color, r, t, rango=(0.25, 2.5), paso=0.005, tol=0.01):
    """
    Dónde toca la superficie medida por la cámara de profundidad un rayo de la cámara a color.

    profundidad  imagen alto x ancho en metros (0 o NaN = sin dato)
    k_prof       matriz de la cámara de profundidad
    d_color      dirección del rayo en el frame de la cámara a color (de `rayo`)
    r, t         pasan del frame de color al de profundidad: p_prof = r · p_color + t
    rango        distancias [m] a recorrer (la profundidad de la Gemini Plus: 0.25-2.5 m)
    Devuelve el punto en el frame de la cámara de profundidad, o None si el rayo nunca cruza una
    superficie con dato (por ejemplo, un hueco de la profundidad sobre una manija brillante).
    """
    direccion = r @ d_color
    s = np.arange(rango[0], rango[1], paso)
    p = t[None, :] + s[:, None] * direccion[None, :]
    z = p[:, 2]
    delante = z > 1e-3
    u = np.full(len(s), -1.0)
    v = np.full(len(s), -1.0)
    u[delante] = k_prof[0, 0] * p[delante, 0] / z[delante] + k_prof[0, 2]
    v[delante] = k_prof[1, 1] * p[delante, 1] / z[delante] + k_prof[1, 2]
    alto, ancho = profundidad.shape
    ui, vi = np.round(u).astype(int), np.round(v).astype(int)
    dentro = delante & (ui >= 0) & (ui < ancho) & (vi >= 0) & (vi < alto)
    medida = np.full(len(s), np.nan)
    medida[dentro] = profundidad[vi[dentro], ui[dentro]]
    valido = dentro & np.isfinite(medida) & (medida > 0)
    # diferencia > 0: el punto del rayo ya quedó detrás de la superficie que mide la profundidad.
    diferencia = np.where(valido, z - medida, np.nan)
    golpes = np.flatnonzero(valido & (diferencia >= -tol))
    if len(golpes) == 0:
        return None
    i = golpes[0]
    previos = np.flatnonzero(valido[:i])
    if len(previos) and diferencia[previos[-1]] < -tol:
        # Entre la última muestra delante de la superficie y la primera detrás: interpolar.
        j = previos[-1]
        a = -diferencia[j] / (diferencia[i] - diferencia[j])
        return p[j] + a * (p[i] - p[j])
    return p[i]


def puntos_anillo(profundidad, k, centro_uv, radio_exterior, radio_interior):
    """Puntos 3D de los píxeles con dato entre dos radios [px] alrededor de un píxel (el anillo
    deja fuera la manija misma y toma la hoja de la puerta que la rodea)."""
    alto, ancho = profundidad.shape
    cu, cv = centro_uv
    us, vs = np.meshgrid(np.arange(ancho), np.arange(alto))
    distancia = np.hypot(us - cu, vs - cv)
    marca = (distancia <= radio_exterior) & (distancia >= radio_interior)
    z = profundidad[marca]
    bueno = np.isfinite(z) & (z > 0)
    u, v, z = us[marca][bueno], vs[marca][bueno], z[bueno]
    return np.column_stack([(u - k[0, 2]) / k[0, 0] * z, (v - k[1, 2]) / k[1, 1] * z, z])


def normal_de_la_puerta(puntos, origen_camara=(0.0, 0.0, 0.0), minimo=30):
    """
    Plano que mejor pasa por unos puntos 3D (mínimos cuadrados, con SVD).
    Devuelve (normal, punto del plano, error medio): la normal apunta alejándose de la cámara
    (hacia la puerta). None si hay menos de `minimo` puntos.
    """
    puntos = np.asarray(puntos, float)
    if len(puntos) < minimo:
        return None
    centro = puntos.mean(axis=0)
    # La normal es la dirección en que los puntos varían menos: el último vector singular.
    normal = np.linalg.svd(puntos - centro, full_matrices=False)[2][-1]
    if normal @ (centro - np.asarray(origen_camara, float)) < 0:
        normal = -normal
    error = float(np.abs((puntos - centro) @ normal).mean())
    return normal, centro, error


def punto_en_plano(origen, direccion, normal, punto_plano, saliente=SALIENTE):
    """
    Dónde cruza un rayo el plano de la manija: el de la puerta corrido `saliente` hacia el robot
    (la manija sobresale de la hoja). Se corre el plano y no el punto, porque el píxel muestra la
    manija: el punto buscado está sobre el mismo rayo, más cerca. None si el rayo es casi
    paralelo al plano o el cruce queda detrás.
    """
    origen, direccion = np.asarray(origen, float), np.asarray(direccion, float)
    normal = np.asarray(normal, float)
    denominador = direccion @ normal
    if abs(denominador) < 1e-6:
        return None
    plano_manija = np.asarray(punto_plano, float) - saliente * normal
    s = ((plano_manija - origen) @ normal) / denominador
    if s <= 0:
        return None
    return origen + s * direccion


def corregir_saliente(punto, origen, direccion, normal, punto_plano, saliente=SALIENTE,
                      tol=0.015):
    """
    La palanca es delgada: el rayo de un keypoint puede pasar al lado y medir la hoja de la puerta
    que está detrás, `saliente` más lejos (o el keypoint cae un par de píxeles fuera de la
    palanca). Si el punto quedó sobre el plano de la puerta (a menos de `tol`) o detrás, se lo
    reemplaza por el cruce del mismo rayo (`origen`, `direccion`) con el plano de la manija
    (`punto_en_plano`). Un punto que ya está delante del plano (la profundidad sí midió la
    manija) no se toca.
    """
    punto = np.asarray(punto, float)
    altura = float((punto - np.asarray(punto_plano, float)) @ np.asarray(normal, float))
    if altura < -tol:          # altura > 0: detrás del plano de la puerta
        return punto
    corregido = punto_en_plano(origen, direccion, normal, punto_plano, saliente)
    return punto if corregido is None else corregido


def validar_manija(eje, agarre, punta, normal):
    """
    Lista de problemas de una detección (vacía = tiene sentido):
    el largo de la palanca debe estar en LARGO_PALANCA, el agarre debe quedar entre el eje y la
    punta, y la palanca debe estar en el plano de la puerta (perpendicular a la normal, con 25°
    de tolerancia).
    """
    problemas = []
    palanca = np.asarray(punta, float) - np.asarray(eje, float)
    largo = float(np.linalg.norm(palanca))
    if not LARGO_PALANCA[0] <= largo <= LARGO_PALANCA[1]:
        problemas.append(f'la palanca mide {largo * 100:.1f} cm (se esperan '
                         f'{LARGO_PALANCA[0] * 100:.0f}-{LARGO_PALANCA[1] * 100:.0f} cm)')
        return problemas
    avance = float((np.asarray(agarre, float) - np.asarray(eje, float)) @ palanca) / largo ** 2
    if not 0.0 < avance < 1.0:
        problemas.append('el punto de agarre no está entre el eje y la punta')
    seno = abs(float(palanca @ normal)) / largo
    if seno > np.sin(np.radians(25.0)):
        problemas.append('la palanca no está en el plano de la puerta')
    return problemas


def orientacion_pinza(aproximacion, arriba=(0.0, 0.0, 1.0)):
    """
    Matriz de rotación de gripper_tcp para que la pinza apunte (con su eje −X local) en la
    dirección `aproximacion` y su eje Z local quede lo más cerca posible de `arriba`.
    """
    a = np.asarray(aproximacion, float)
    a = a / np.linalg.norm(a)
    arriba = np.asarray(arriba, float)
    z = arriba - (arriba @ a) * a          # la parte de "arriba" perpendicular a la aproximación
    if np.linalg.norm(z) < 1e-6:
        raise ValueError('la dirección de aproximación es paralela a "arriba"')
    z = z / np.linalg.norm(z)
    x = -a
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def pose_previa(agarre, normal, distancia=0.30, arriba=(0.0, 0.0, 1.0)):
    """Pose (posición, matriz de rotación) de la pinza, a `distancia` antes de la manija, mirando
    hacia la puerta. Desde ahí la profundidad de la cámara todavía mide (0.25 m o más)."""
    posicion = np.asarray(agarre, float) - distancia * np.asarray(normal, float)
    return posicion, orientacion_pinza(normal, arriba)


def arco_palanca(eje, punto, normal, angulo, pasos=10):
    """
    Camino de un punto de la palanca (el de agarre, por ejemplo) al bajarla `angulo` [rad]:
    gira alrededor del eje de la roseta, que es perpendicular a la puerta (dirección `normal`).
    Devuelve pasos+1 puntos, desde el actual hasta el final. El sentido de giro es el que baja la
    palanca; con la palanca vertical no se sabe cuál es, y da un error.
    """
    eje, punto = np.asarray(eje, float), np.asarray(punto, float)
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    v = punto - eje
    # Al girar +theta alrededor de n, el punto sube a razón de (n × v)_z: se elige el signo
    # que lo baja.
    subida = float(np.cross(n, v)[2])
    if abs(subida) < 1e-6:
        raise ValueError('la palanca está vertical: no se sabe hacia dónde bajarla')
    signo = -1.0 if subida > 0 else 1.0
    camino = []
    for theta in np.linspace(0.0, signo * angulo, pasos + 1):
        # Fórmula de Rodrigues: v girado theta alrededor de n.
        giro = (v * np.cos(theta) + np.cross(n, v) * np.sin(theta)
                + n * (n @ v) * (1.0 - np.cos(theta)))
        camino.append(eje + giro)
    return np.array(camino)


def distancia_desde_tcp(posicion_tcp, giro_tcp, punto):
    """
    Cuánto le falta a la pinza (gripper_tcp) para llegar a `punto`, todo en el mismo frame.

    posicion_tcp, giro_tcp  pose del TCP (vector de 3 y matriz de rotación 3x3), de TF
    Devuelve un diccionario en metros:
      total      distancia en línea recta
      adelante   lo que falta a lo largo de la dirección en que apunta la pinza (su eje −X local);
                 negativo = el punto quedó detrás de la pinza
      lateral    corrimiento a lo largo del eje Y local de la pinza
      vertical   corrimiento a lo largo del eje Z local de la pinza
    Con lateral y vertical cerca de 0, basta avanzar en línea recta para llegar.
    """
    v = np.asarray(giro_tcp, float).T @ (np.asarray(punto, float)
                                         - np.asarray(posicion_tcp, float))
    return {'total': float(np.linalg.norm(v)), 'adelante': float(-v[0]),
            'lateral': float(v[1]), 'vertical': float(v[2])}


class Acercamiento:
    """
    Secuencia para llevar la pinza hasta la manija, sin ROS (manija_node le pasa las detecciones
    y la posición del TCP, y manda a MoveIt lo que esta clase devuelve).

    Por qué en tramos: la profundidad de la cámara solo mide desde 0.25 m, así que el último tramo
    se hace sin ver. Con `distancias` = (0.30, 0.05):
      tramo 0: con la manija vista desde lejos, ir a 30 cm de ella, mirándola;
      tramo 1: desde ahí la cámara la ve mejor (más cerca y de frente): con esa detección nueva, ir
               a 5 cm (la pinza todavía no la toca; empujar es el paso siguiente).
    En cada tramo hacen falta `n_estables` detecciones seguidas que no se separen más de
    `tol_estable` (una detección suelta puede estar mal); se usa su promedio. El tramo termina
    cuando el TCP queda a menos de `tol_llegada` del objetivo.

    Fases: 'buscar' (juntando detecciones), 'yendo' (esperando que el brazo llegue), 'listo'.
    """

    def __init__(self, distancias=(0.30, 0.05), n_estables=3, tol_estable=0.02,
                 tol_llegada=0.02):
        self.distancias = tuple(distancias)
        self.n_estables, self.tol_estable, self.tol_llegada = n_estables, tol_estable, tol_llegada
        self.tramo = 0
        self.fase = 'buscar'
        self.vistas = []
        self.objetivo = self.agarre = self.normal = None
        self.dispersion = None   # cuánto se separaron las últimas detecciones [m], para el log

    def deteccion(self, agarre, normal):
        """
        Una detección válida de la manija (punto de agarre y normal de la puerta).
        Devuelve (posición, rotación) de la pinza a mandar al brazo, o None si todavía no hay que
        mandar nada (faltan detecciones estables, o el brazo ya va en camino).
        """
        if self.fase != 'buscar':
            return None
        self.vistas.append((np.asarray(agarre, float), np.asarray(normal, float)))
        self.vistas = self.vistas[-self.n_estables:]
        if len(self.vistas) < self.n_estables:
            return None
        puntos = np.array([a for a, _ in self.vistas])
        centro = puntos.mean(axis=0)
        self.dispersion = float(np.linalg.norm(puntos - centro, axis=1).max())
        if self.dispersion > self.tol_estable:
            return None    # todavía saltan: se espera la próxima (la más vieja sale de la lista)
        normal = np.array([n for _, n in self.vistas]).mean(axis=0)
        self.agarre, self.normal = centro, normal / np.linalg.norm(normal)
        posicion, giro = pose_previa(self.agarre, self.normal, self.distancias[self.tramo])
        self.objetivo, self.fase, self.vistas = posicion, 'yendo', []
        return posicion, giro

    def tcp(self, posicion):
        """Posición actual del TCP. Devuelve True si con ella se terminó un tramo."""
        if self.fase != 'yendo':
            return False
        if np.linalg.norm(np.asarray(posicion, float) - self.objetivo) > self.tol_llegada:
            return False
        self.tramo += 1
        self.fase = 'listo' if self.tramo >= len(self.distancias) else 'buscar'
        return True

    def reintentar(self):
        """El brazo no llegó (MoveIt no encontró camino, o tardó demasiado): se vuelve a buscar
        la manija en el mismo tramo, con detecciones nuevas."""
        if self.fase == 'yendo':
            self.fase, self.vistas = 'buscar', []
