"""Genera imágenes etiquetadas de la manija de la puerta de prueba, con la cámara de la pinza.

Para qué: los detectores preentrenados (YOLOE, YOLOv8 de Open Images) encuentran la región de la
manija en la imagen simulada, pero no la reconocen (19-10-2026: clase equivocada, confianza
< 0.06). Hay que afinar un modelo, y para eso hacen falta imágenes etiquetadas. En la simulación
no hace falta anotar a mano: se sabe exactamente dónde está la manija (la geometría del SDF + el
ángulo de la bisagra y de la manija en /puerta/joint_states) y dónde está la cámara (TF),
así que se proyecta y se escribe la etiqueta sola.

Cómo: mueve la pinza (con MoveIt, por /arm_target_pose) a vistas al azar alrededor de la manija,
espera a que llegue, saca una foto a color y escribe tres etiquetas en formato YOLO:
  labels_det/   caja                                    (para YOLOv8/11 detección)
  labels_seg/   polígono de la manija                   (para YOLOv8/11-seg, máscaras)
  labels_pose/  caja + 3 keypoints con su visibilidad   (para YOLOv8/11-pose)
Keypoints (en el frame de la manija, ver worlds/puerta_prueba.sdf):
  0 eje     centro de la roseta, cara delantera (donde gira la palanca)
  1 agarre  a 7 cm del eje sobre la palanca (donde cerrar la pinza)
  2 punta   extremo de la palanca (12 cm)
Con eje -> punta sale la dirección de la palanca; con eje y agarre, dónde y hacia dónde bajarla.

Variar la escena (--variar K, "domain randomization"): si todas las fotos muestran la misma puerta
marrón con la misma luz, el modelo aprende ESA puerta y no reconoce otra. Con --variar K, cada K
imágenes se borra la puerta de Gazebo y se pone una variante al azar, hecha a partir del mismo
worlds/puerta_prueba.sdf:
  - colores y texturas de hoja, marco, muros y manija (manija plateada, dorada, negra...)
  - largo y grosor de la palanca y radio de la roseta (las etiquetas usan esas medidas)
  - una luz puntual de color, posición e intensidad al azar del lado de Milo
  - la hoja entreabierta (0 a 0.3 rad) y la manija bajada (0 a 0.6 rad): se cambia el punto de
    reposo de sus resortes (<spring_reference>), así quedan quietas en ese ángulo
La primera tanda usa la puerta original. meta.csv anota qué variante tiene cada imagen.

Uso (con la simulación de la puerta corriendo, en otra terminal):
  ./sim.sh shell
  ros2 launch andesrobot_arm arm_sim.launch.py puerta:=true octomap:=false       # terminal 1
  cd /ros2_ws/src/andesrobot_arm/scripts                                          # terminal 2
  python3 generar_dataset_manija.py --n 300 --variar 10
Deja todo en ros2_ws/datasets/manija_sim/ (no va a git). Para entrenar hace falta una carpeta
train/ y otra val/ (dos corridas con distinta --semilla, --salida .../train y .../val); después
scripts/entrenar_manija.py arma las carpetas que pide Ultralytics y entrena.

Cuidados con la base (medido el 19-10-2026: en una corrida de 300 imágenes la base se corrió
0.41 m hacia atrás y giró 24°, y al final MoveIt fallaba 1 de cada 3 vistas porque la puerta
quedaba lejos). En Gazebo el brazo mandado por posición no frena al chocar: empuja toda la base.
Por eso el generador:
  - salta, sin mandarla, una vista que deja el TCP a menos de --margen de la cara de la hoja;
  - le pide a arm_moveit_node movimientos más suaves (--aceleracion, la fracción de la
    aceleración máxima; los arranques bruscos de un brazo pesado sobre una base liviana también
    la mueven);
  - mide cuánto se movió la base desde el comienzo: avisa a los 5 cm o 3°, y PARA a --deriva_max
    (hay que relanzar la simulación para que Milo vuelva a su lugar).
Las etiquetas siguen bien aunque la base se mueva (todo se calcula con la TF del momento); el
costo es que se alcanzan menos vistas.

Supuestos: la puerta está en el origen del mundo (como la pone arm_sim) y el frame "odom"
coincide con el mundo de Gazebo (el plugin diff_drive lo saca de la pose real del robot, por eso
las etiquetas siguen calzando aunque la base resbale). Una vista que MoveIt rechaza o
que no llega en 25 s se salta.
"""
import argparse
import math
import os
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from geometry_msgs.msg import PoseStamped
from PIL import Image as PILImage
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.kinematics import matrix_to_quaternion, quaternion_to_matrix

# ---------------- Geometría de la puerta (copiada de worlds/puerta_prueba.sdf) ----------------
BISAGRA = np.array([0.80, 0.45, 0.0])          # origen del link "hoja" (en la bisagra), mundo
MANIJA_EN_HOJA = np.array([-0.05, -0.83, 1.0])  # origen del link "manija" respecto de la hoja
# Medidas de la manija original: palanca de 12 cm y 2 cm de grosor, roseta de 2.5 cm de radio.
MANIJA_ORIGINAL = {'largo': 0.12, 'grosor': 0.02, 'radio': 0.025}
NOMBRES_KP = ['eje', 'agarre', 'punta']


def keypoints(m):
    """Keypoints en el frame de la manija, para las medidas m (largo de la palanca, etc.)."""
    return np.array([[0.0, 0.0, 0.0],                      # 0 eje
                     [-0.005, 0.07 / 0.12 * m['largo'], 0.0],  # 1 agarre (7 cm en la original)
                     [-0.005, m['largo'], 0.0]])          # 2 punta


def esquinas_manija(m):
    """Puntos del contorno 3D de la manija (palanca + roseta), en el frame de la manija."""
    pts = []
    # Palanca: caja de grosor x largo x grosor centrada en (-0.005, largo / 2, 0).
    g, lg = m['grosor'] / 2, m['largo'] / 2
    for dx in (-g, g):
        for dy in (-lg, lg):
            for dz in (-g, g):
                pts.append([-0.005 + dx, lg + dy, dz])
    # Roseta: cilindro de largo 0.03 a lo largo de X, centrado en x = 0.015.
    for a in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        for x in (0.0, 0.03):
            pts.append([x, m['radio'] * np.cos(a), m['radio'] * np.sin(a)])
    return np.array(pts)


# ---------------- Variantes de la puerta (domain randomization) ----------------
# Texturas que trae Gazebo (/usr/share/gazebo-11/media/materials/scripts/gazebo.material).
TEXTURAS_HOJA = ['Gazebo/Wood', 'Gazebo/WoodPallet', 'Gazebo/WoodFloor', 'Gazebo/White',
                 'Gazebo/Grey', 'Gazebo/PaintedWall']
TEXTURAS_MURO = ['Gazebo/PaintedWall', 'Gazebo/Bricks', 'Gazebo/White', 'Gazebo/Grey']
# Colores de manija (RGB).
COLORES_MANIJA = {'plateado': (0.75, 0.75, 0.77), 'acero': (0.45, 0.46, 0.48),
                  'dorado': (0.80, 0.65, 0.30), 'bronce': (0.50, 0.35, 0.20),
                  'negro': (0.07, 0.07, 0.07), 'blanco': (0.92, 0.92, 0.90)}


def poner_material(visual, rng, texturas, rgb=None):
    """Cambiar el <material> de un <visual>: una textura de Gazebo o un color al azar."""
    viejo = visual.find('material')
    if viejo is not None:
        visual.remove(viejo)
    mat = ET.SubElement(visual, 'material')
    if rgb is None and texturas and rng.random() < 0.5:
        script = ET.SubElement(mat, 'script')
        ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'
        ET.SubElement(script, 'name').text = str(rng.choice(texturas))
        return
    c = np.clip(np.array(rgb if rgb is not None else rng.uniform(0.1, 0.95, 3))
                * rng.uniform(0.8, 1.2), 0, 1)
    txt = ' '.join(f'{v:.3f}' for v in c) + ' 1'
    ET.SubElement(mat, 'ambient').text = txt
    ET.SubElement(mat, 'diffuse').text = txt
    brillo = rng.uniform(0.0, 1.0)
    ET.SubElement(mat, 'specular').text = f'{brillo:.2f} {brillo:.2f} {brillo:.2f} 1'


def sdf_original(texto):
    """puerta_prueba.sdf tal cual (sin comentarios). Devuelve (xml, medidas, descripción)."""
    return ET.tostring(ET.fromstring(texto), encoding='unicode'), dict(MANIJA_ORIGINAL), 'original'


def sdf_variante(texto, rng):
    """Una variante al azar de puerta_prueba.sdf. Devuelve (xml, medidas, descripción)."""
    raiz = ET.fromstring(texto)   # (ElementTree descarta los comentarios: mejor para spawnear)
    modelo = raiz.find('model')
    links = {lk.get('name'): lk for lk in modelo.findall('link')}
    joints = {j.get('name'): j for j in modelo.findall('joint')}
    # Colores y texturas.
    for vis in links['marco'].findall('visual'):
        es_muro = vis.get('name').startswith('muro')
        poner_material(vis, rng, TEXTURAS_MURO if es_muro else TEXTURAS_HOJA)
    poner_material(links['hoja'].find('visual'), rng, TEXTURAS_HOJA)
    color_manija = str(rng.choice(list(COLORES_MANIJA)))
    for vis in links['manija'].findall('visual'):
        poner_material(vis, rng, None, COLORES_MANIJA[color_manija])
    # Medidas de la manija: se cambian igual en <visual> y <collision>.
    m = {'largo': rng.uniform(0.10, 0.14), 'grosor': rng.uniform(0.015, 0.022),
         'radio': rng.uniform(0.02, 0.03)}
    for el in links['manija']:
        nombre = el.get('name') or ''
        if nombre.startswith('palanca'):
            el.find('pose').text = f"-0.005 {m['largo'] / 2:.4f} 0 0 0 0"
            el.find('geometry/box/size').text = f"{m['grosor']:.4f} {m['largo']:.4f} " \
                f"{m['grosor']:.4f}"
        elif nombre.startswith('roseta'):
            el.find('geometry/cylinder/radius').text = f"{m['radio']:.4f}"
    # Ángulos: el punto de reposo de los resortes. Más rígidos que en la puerta original para
    # que la fricción de la bisagra no la deje lejos del ángulo pedido.
    bisagra = rng.uniform(0.0, 0.3) if rng.random() < 0.5 else 0.0
    manija = rng.uniform(-0.6, 0.0) if rng.random() < 0.5 else 0.0
    for nombre, ref, k in (('bisagra', bisagra, 30.0), ('eje_manija', manija, 5.0)):
        din = joints[nombre].find('axis/dynamics')
        din.find('spring_reference').text = f'{ref:.3f}'
        din.find('spring_stiffness').text = str(k)
    # Una luz puntual del lado de Milo (el marco está en x = 0.80: x relativo negativo). Se suma
    # a la luz del mundo: más fuerte que esto deja la imagen casi blanca (probado).
    luz = ET.SubElement(links['marco'], 'light', name='luz_variante', type='point')
    pos = (rng.uniform(-0.9, -0.1), rng.uniform(-1.0, 1.0), rng.uniform(1.2, 2.3))
    ET.SubElement(luz, 'pose').text = '%.2f %.2f %.2f 0 0 0' % pos
    col = rng.uniform(0.0, 0.5) * np.array([1.0, *rng.uniform(0.8, 1.0, 2)])
    ET.SubElement(luz, 'diffuse').text = ' '.join(f'{v:.2f}' for v in col) + ' 1'
    ET.SubElement(luz, 'specular').text = '0.2 0.2 0.2 1'
    aten = ET.SubElement(luz, 'attenuation')
    ET.SubElement(aten, 'range').text = '6'
    ET.SubElement(aten, 'constant').text = '1.0'
    ET.SubElement(aten, 'linear').text = '0.1'
    ET.SubElement(aten, 'quadratic').text = '0.02'
    ET.SubElement(luz, 'cast_shadows').text = 'false'
    desc = f"largo={m['largo']:.3f} grosor={m['grosor']:.3f} radio={m['radio']:.3f} " \
        f'bisagra={bisagra:.2f} manija={manija:.2f} color_manija={color_manija} ' \
        f'luz={col[0]:.2f}'
    return ET.tostring(raiz, encoding='unicode'), m, desc


def rot(eje, ang):
    c, s = math.cos(ang), math.sin(ang)
    if eje == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pose_manija(ang_bisagra, ang_manija):
    """Transformación 4x4 mundo <- manija para los ángulos actuales de la bisagra y la manija."""
    t_hoja = np.eye(4)
    t_hoja[:3, :3] = rot('z', ang_bisagra)
    t_hoja[:3, 3] = BISAGRA
    t_man = np.eye(4)
    t_man[:3, :3] = rot('x', ang_manija)
    t_man[:3, 3] = MANIJA_EN_HOJA
    return t_hoja @ t_man


def distancia_a_hoja(p, ang_bisagra):
    """Distancia [m] del punto p (mundo) a la cara de la hoja del lado de Milo (+ = delante)."""
    r = rot('z', ang_bisagra)
    normal = r @ np.array([-1.0, 0.0, 0.0])            # la cara mira hacia Milo (-X)
    punto = BISAGRA + r @ np.array([-0.02, 0.0, 0.0])  # la hoja mide 4 cm de espesor
    return float(normal @ (p - punto))


def tf_a_matriz(tf):
    m = np.eye(4)
    q = tf.transform.rotation
    m[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    t = tf.transform.translation
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def mirar_hacia(ojo, objetivo, giro):
    """Pose 4x4 de un frame óptico (Z adelante, X derecha, Y abajo) en 'ojo' mirando 'objetivo'.

    giro = rotación [rad] alrededor del eje óptico, para variar la orientación de la imagen.
    """
    z = objetivo - ojo
    z /= np.linalg.norm(z)
    arriba = np.array([0.0, 0.0, 1.0])
    x = np.cross(-arriba, z)          # derecha de la imagen (Y óptico apunta hacia abajo)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([0.0, -1.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r = np.column_stack([x, y, z]) @ rot('z', giro)
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = ojo
    return m


class Generador(Node):

    def __init__(self, args):
        super().__init__('generar_dataset_manija')
        self.args = args
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self, spin_thread=True)
        self.img = self.info = None
        self.ang = {}
        self.create_subscription(Image, '/gripper_camera/color/image_raw',
                                 lambda m: setattr(self, 'img', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/puerta/joint_states',
                                 lambda m: self.ang.update(zip(m.name, m.position)), 10)
        self.pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)
        self.rng = np.random.default_rng(args.semilla)
        self.medidas = dict(MANIJA_ORIGINAL)
        self.variante_hecha = False
        self.puerta = 'puerta_prueba'   # nombre del modelo en Gazebo (lo pone arm_sim)
        self.n_variantes = 0
        self.variante = 'original'
        self.borrar = self.create_client(DeleteEntity, '/delete_entity')
        self.spawnear = self.create_client(SpawnEntity, '/spawn_entity')
        self.listar = self.create_client(GetModelList, '/get_model_list')
        self.parametros = self.create_client(SetParameters, '/arm_moveit_node/set_parameters')
        with open(os.path.join(get_package_share_directory('andesrobot_arm'), 'worlds',
                               'puerta_prueba.sdf')) as f:
            self.sdf = f.read()

    def llamar(self, cliente, req, seg=15):
        """Llamar a un servicio y esperar la respuesta (None si no contesta)."""
        if not cliente.wait_for_service(timeout_sec=seg):
            return None
        fut = cliente.call_async(req)
        self.esperar(seg, fut.done)
        return fut.result() if fut.done() else None

    def suavizar_brazo(self):
        """Bajar la aceleración de los movimientos de arm_moveit_node (parámetro en vivo)."""
        valor = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                               double_value=self.args.aceleracion)
        res = self.llamar(self.parametros, SetParameters.Request(
            parameters=[Parameter(name='acceleration_scaling', value=valor)]), seg=5)
        if res is None or not res.results[0].successful:
            self.get_logger().warn('No pude cambiar acceleration_scaling de arm_moveit_node '
                                   '(¿moveit:=false?): sigue con la de siempre.')

    def deriva(self, t_inicio):
        """Cuánto se movió la base desde t_inicio: (distancia en el piso [m], giro [grados])."""
        t = self.lookup('odom', 'base_footprint')
        if t is None:
            return 0.0, 0.0
        d = np.linalg.norm(t[:2, 3] - t_inicio[:2, 3])
        giro = math.atan2(t[1, 0], t[0, 0]) - math.atan2(t_inicio[1, 0], t_inicio[0, 0])
        return float(d), abs(math.degrees(math.atan2(math.sin(giro), math.cos(giro))))

    def modelos_puerta(self):
        """Nombres de los modelos de Gazebo que son una puerta (puerta_prueba, puerta_v...)."""
        res = self.llamar(self.listar, GetModelList.Request(), seg=5)
        return [m for m in (res.model_names if res else []) if m.startswith('puerta')]

    def nueva_variante(self, original=False):
        """Borrar la puerta de Gazebo y poner una variante al azar (o la original) ahí mismo.

        Cada puerta nueva lleva otro nombre (puerta_v<pid>_<n>): por precaución, para que Gazebo
        no confunda la puerta nueva con la recién borrada si el borrado todavía no terminó.
        """
        log = self.get_logger()
        xml, medidas, desc = sdf_original(self.sdf) if original else \
            sdf_variante(self.sdf, self.rng)
        # Borrar la de esta corrida, o cualquiera que haya dejado una corrida anterior.
        for viejo in self.modelos_puerta():
            self.llamar(self.borrar, DeleteEntity.Request(name=viejo), seg=5)
        self.esperar(1.0)
        self.n_variantes += 1
        self.puerta = f'puerta_v{os.getpid()}_{self.n_variantes}'
        res = self.llamar(self.spawnear, SpawnEntity.Request(name=self.puerta, xml=xml))
        if res is None or not res.success:
            log.error(f'No se pudo poner la variante: {res.status_message if res else "?"}')
            return False
        self.medidas, self.variante = medidas, desc
        # Esperar a que lleguen los ángulos de la puerta nueva y a que los resortes la lleven
        # al ángulo pedido (con la amortiguación de la bisagra tarda unos segundos).
        self.ang = {}
        self.esperar(10, lambda: 'bisagra' in self.ang)
        self.esperar(6.0)
        log.info(f'Variante nueva: {desc}')
        return 'bisagra' in self.ang

    def esperar(self, seg, cond=lambda: False):
        t0 = time.time()
        while time.time() - t0 < seg and not cond():
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def lookup(self, padre, hijo):
        try:
            return tf_a_matriz(self.tf.lookup_transform(padre, hijo, Time()))
        except TransformException:
            return None

    def vista_al_azar(self, t_manija):
        """Pose del TCP para que la cámara mire la manija desde un punto al azar."""
        centro = t_manija[:3, :3] @ keypoints(self.medidas)[1] + t_manija[:3, 3]   # agarre
        d = self.rng.uniform(self.args.dmin, self.args.dmax)
        # Dirección desde la manija hacia la cámara: del lado de Milo (-X), ±40° a los lados,
        # -15° a +45° en altura.
        az = math.radians(180 + self.rng.uniform(-40, 40))
        el = math.radians(self.rng.uniform(-15, 45))
        ojo = centro + d * np.array([math.cos(el) * math.cos(az),
                                     math.cos(el) * math.sin(az), math.sin(el)])
        apunte = centro + self.rng.normal(0, 0.03, 3)   # no siempre al centro de la imagen
        return mirar_hacia(ojo, apunte, self.rng.uniform(-0.3, 0.3))

    def proyectar(self, pts_mundo, t_mundo_cam):
        """Puntos del mundo -> píxeles (u, v) y profundidad, con la matriz K de camera_info."""
        k = np.array(self.info.k).reshape(3, 3)
        cam = np.linalg.inv(t_mundo_cam)
        p = (cam[:3, :3] @ pts_mundo.T).T + cam[:3, 3]
        uv = (k @ p.T).T
        return uv[:, :2] / uv[:, 2:3], p[:, 2]

    def correr(self):
        a = self.args
        carpetas = {t: os.path.join(a.salida, t) for t in
                    ('images', 'labels_det', 'labels_seg', 'labels_pose')}
        for c in carpetas.values():
            os.makedirs(c, exist_ok=True)
        log = self.get_logger()
        if not self.esperar(20, lambda: self.img is not None and self.info is not None
                            and 'bisagra' in self.ang):
            log.error('No llegan la cámara o /puerta/joint_states: ¿arm_sim con puerta:=true?')
            return
        # Del TCP a la cámara (fijo): pasar de "dónde quiero la cámara" a "dónde va el TCP".
        t_tcp_cam = None
        while t_tcp_cam is None:
            t_tcp_cam = self.lookup('gripper_tcp', 'gripper_camera_color_optical_frame')
            self.esperar(0.2)
        # Partir siempre de la puerta original: una corrida anterior pudo dejar una variante.
        if a.variar and not self.nueva_variante(original=True):
            return
        self.suavizar_brazo()
        t_base0 = None
        while t_base0 is None:
            t_base0 = self.lookup('odom', 'base_footprint')
            self.esperar(0.2)
        avisado = False
        hechas = intentos = cerca = 0
        meta = open(os.path.join(a.salida, 'meta.csv'), 'a')
        while hechas < a.n and intentos < a.n * 3:
            intentos += 1
            # Cada --variar imágenes, otra puerta (la primera tanda es la original).
            if a.variar and hechas and hechas % a.variar == 0 and not self.variante_hecha:
                if not self.nueva_variante():
                    break
                self.variante_hecha = True
            d, giro = self.deriva(t_base0)
            if d > a.deriva_max or giro > 3 * a.deriva_max / 0.05:
                log.error(f'La base se movió {d:.2f} m y {giro:.0f}° desde el comienzo: paro. '
                          'Relanzar la simulación (vuelve a su lugar) y seguir con --inicio '
                          f'{a.inicio + hechas}.')
                break
            if not avisado and (d > 0.05 or giro > 3):
                log.warn(f'La base se movió {d:.2f} m y {giro:.0f}° desde el comienzo.')
                avisado = True
            t_man = pose_manija(self.ang['bisagra'], self.ang['eje_manija'])
            t_cam_deseada = self.vista_al_azar(t_man)
            t_tcp = t_cam_deseada @ np.linalg.inv(t_tcp_cam)
            if distancia_a_hoja(t_tcp[:3, 3], self.ang['bisagra']) < a.margen:
                # El TCP quedaría casi tocando la hoja: ni se intenta (y no cuenta como intento,
                # no cuesta nada sortear otra).
                intentos -= 1
                cerca += 1
                if cerca > 50 * a.n:
                    log.error('Casi todas las vistas quedan pegadas a la hoja: bajar --margen.')
                    break
                continue
            # La meta va en base_footprint: la escena de MoveIt no conoce "odom".
            t_bf_odom = self.lookup('base_footprint', 'odom')
            if t_bf_odom is None:
                continue
            t_meta = t_bf_odom @ t_tcp
            msg = PoseStamped()
            msg.header.frame_id = 'base_footprint'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t_meta[:3, 3]
            q = matrix_to_quaternion(t_meta[:3, :3])
            msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
            msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
            self.pub.publish(msg)

            def llego():
                actual = self.lookup('odom', 'gripper_tcp')
                return actual is not None and np.linalg.norm(actual[:3, 3] - t_tcp[:3, 3]) < 0.01
            if not self.esperar(25, llego):
                log.info(f'vista {intentos}: no se alcanzó (MoveIt la rechazó o no llegó)')
                continue
            self.esperar(1.0)   # que se asiente y llegue una imagen nueva
            self.img = None
            self.esperar(2, lambda: self.img is not None)
            t_cam = self.lookup('odom', 'gripper_camera_color_optical_frame')
            if self.img is None or t_cam is None:
                continue
            t_man = pose_manija(self.ang['bisagra'], self.ang['eje_manija'])
            w, h = self.info.width, self.info.height
            uv_cont, z_cont = self.proyectar(
                (t_man[:3, :3] @ esquinas_manija(self.medidas).T).T + t_man[:3, 3], t_cam)
            kp = keypoints(self.medidas)
            uv_kp, z_kp = self.proyectar((t_man[:3, :3] @ kp.T).T + t_man[:3, 3], t_cam)
            if (z_cont <= 0.05).any():
                continue
            dentro = (uv_cont[:, 0] >= 0) & (uv_cont[:, 0] < w) & (uv_cont[:, 1] >= 0) & \
                (uv_cont[:, 1] < h)
            if dentro.mean() < 0.5:   # la manija casi fuera de la imagen: no sirve
                continue
            # Caja y polígono (envolvente convexa del contorno proyectado), dentro de la imagen.
            uv_c = np.clip(uv_cont, [0, 0], [w - 1, h - 1])
            x0, y0 = uv_c.min(0)
            x1, y1 = uv_c.max(0)
            casco = envolvente(uv_c)
            nombre = f'manija_{a.inicio + hechas:05d}'
            PILImage.fromarray(image_to_array(self.img)).save(
                os.path.join(carpetas['images'], nombre + '.png'))
            caja = f'{(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} {(x1 - x0) / w:.6f} ' \
                f'{(y1 - y0) / h:.6f}'
            with open(os.path.join(carpetas['labels_det'], nombre + '.txt'), 'w') as f:
                f.write(f'0 {caja}\n')
            with open(os.path.join(carpetas['labels_seg'], nombre + '.txt'), 'w') as f:
                f.write('0 ' + ' '.join(f'{u / w:.6f} {v / h:.6f}' for u, v in casco) + '\n')
            kps = []
            for (u, v), z in zip(uv_kp, z_kp):
                vis = 2 if (0 <= u < w and 0 <= v < h and z > 0) else 0
                kps.append(f'{u / w:.6f} {v / h:.6f} {vis}' if vis else '0 0 0')
            with open(os.path.join(carpetas['labels_pose'], nombre + '.txt'), 'w') as f:
                f.write(f'0 {caja} ' + ' '.join(kps) + '\n')
            meta.write(f'{nombre},{self.variante}\n')
            meta.flush()
            hechas += 1
            self.variante_hecha = False
            log.info(f'{nombre}: distancia {np.linalg.norm(t_cam[:3, 3] - t_man[:3, 3]):.2f} m')
        meta.close()
        log.info(f'Listo: {hechas} imágenes en {a.salida} ({intentos} vistas intentadas, '
                 f'{cerca} descartadas por quedar pegadas a la hoja).')


def envolvente(p):
    """Envolvente convexa 2D (monotone chain), en orden: el polígono de la máscara."""
    p = sorted(map(tuple, p))
    if len(p) < 3:
        return p

    def cruz(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    inf, sup = [], []
    for q in p:
        while len(inf) >= 2 and cruz(inf[-2], inf[-1], q) <= 0:
            inf.pop()
        inf.append(q)
    for q in reversed(p):
        while len(sup) >= 2 and cruz(sup[-2], sup[-1], q) <= 0:
            sup.pop()
        sup.append(q)
    return inf[:-1] + sup[:-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--n', type=int, default=50, help='imágenes a generar')
    ap.add_argument('--salida', default='/ros2_ws/datasets/manija_sim')
    ap.add_argument('--dmin', type=float, default=0.35, help='distancia mínima a la manija [m]')
    ap.add_argument('--dmax', type=float, default=0.80, help='distancia máxima a la manija [m]')
    ap.add_argument('--semilla', type=int, default=0)
    ap.add_argument('--variar', type=int, default=0,
                    help='cada cuántas imágenes poner otra variante de la puerta (0 = nunca)')
    ap.add_argument('--margen', type=float, default=0.12,
                    help='distancia mínima del TCP a la cara de la hoja [m]')
    ap.add_argument('--aceleracion', type=float, default=0.2,
                    help='fracción de la aceleración máxima del brazo (en el nodo: 0.5)')
    ap.add_argument('--deriva_max', type=float, default=0.15,
                    help='parar si la base se movió más que esto [m] (o 3° por cada 5 cm)')
    ap.add_argument('--inicio', type=int, default=0,
                    help='número de la primera imagen (para agregar a un dataset que ya existe)')
    args = ap.parse_args()
    rclpy.init()
    nodo = Generador(args)
    try:
        nodo.correr()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
