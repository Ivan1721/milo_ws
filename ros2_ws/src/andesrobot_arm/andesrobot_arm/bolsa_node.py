"""Nodo bolsa_node: toma una bolsa por el asa (del suelo o una mesa) y la levanta. BORRADOR, P17.

  ros2 run andesrobot_arm bolsa_node --ros-args -p modelo:=/ros2_ws/modelos/bolsa/best.pt

ESTADO: esqueleto escrito el 13-nov-2026 esperando el modelo de segmentación del equipo de visión
(una clase, `bag`, entrenado con la bolsa de papel de supermercado). La geometría (bolsa.py) está
probada con nubes sintéticas (test/test_bolsa.py); el nodo NO se ha corrido nunca: falta el modelo
y una bolsa con asa en Gazebo (la `bolsa_equipo` del catálogo es una caja sin asas). Las vistas y
la pose `retraer` de abajo son estimaciones sin probar. Guía en docs/robocup/README.md §8.1.

Es agarre_node (la misma clase, AgarreNode: mismos movimientos, misma pinza, mismos parámetros)
con otra visión: en vez de pedirle a objeto_node una caja y medidas, corre AQUÍ el modelo de
segmentación y busca el asa por geometría (ver bolsa.py):

  1. MIRAR: abre la pinza y prueba las `vistas` una por una (primero para una bolsa en el suelo,
     después para una en la mesa) hasta ver la bolsa.
  2. ENCONTRAR: en cada imagen nueva, el modelo da la máscara de la bolsa (la detección `bag` más
     segura); la profundidad la convierte en puntos 3D; bolsa.buscar_asa encuentra la boca y el
     asa que sobresale (la más cercana a Milo si se ven dos). Con `n_mediciones` (3) seguidas
     que coinciden (vértice a menos de `tol_estable`), sigue. Si el asa no cumple (muy baja,
     muy gruesa...), lo anota y sigue mirando.
  3. PREVIA: la pinza va `sobre` (10 cm) encima del vértice del asa, apuntando hacia abajo, con los
     dedos cruzados al asa (MoveIt, camino libre de choques).
  4. BAJAR en línea recta hasta que el asa quede `metido` (12 mm) adentro de los dedos.
  5. CERRAR del todo: el asa mide pocos mm, así que no se usa "ancho − apriete" como con el
     tenedor; los 20 N de la pinza la aprietan. Se comprueba que los dedos se frenaron (si no,
     abre, sube y termina con error). OJO: hay_objeto pide que se frenen `minimo_frenado` (2 mm)
     antes de cerrar del todo (3 mm entre caras), o sea un asa de 5 mm o más; con un asa más
     delgada hay que bajar `minimo_frenado` (y aceptar más falsos "hay objeto").
  6. UNIR (si `unir`): una caja del tamaño de la bolsa (`medidas_bolsa`) colgando bajo la pinza en
     la escena de MoveIt, para que el brazo no la lleve contra el robot al retraer.
  7. LEVANTAR `subir` (15 cm: la bolsa cuelga del asa y despega del suelo) en línea recta y
     RETRAER sin girar la pinza (mantener_orientacion, en línea recta).
  8. Revisar que los dedos sigan frenados.

Para probarlo en la simulación (cuando exista la bolsa con asa en el catálogo):
  ros2 launch andesrobot_arm arm_sim.launch.py mesa:=false octomap:=false    # bolsa en el suelo
  ros2 run andesrobot_arm bolsa_node --ros-args -p modelo:=<ruta al .pt>
Parámetros propios (además de los de agarre_node): modelo, clase, paso, usar_toda_la_imagen,
metido, minimo_frenado, unir, medidas_bolsa.
"""
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, PlanningScene
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from andesrobot_arm.agarre import ABIERTA, CERRADA, apertura, estable, hay_objeto
from andesrobot_arm.agarre_node import FRAME, TCP, TOCAN, AgarreNode, Fallo
from andesrobot_arm.arm_moveit_node import caja
from andesrobot_arm.bolsa import (
    buscar_asa, en_la_mascara, nube_de_profundidad, pose_de_agarre, promediar_asas)
from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.manija_node import profundidad_en_metros

# Valores por defecto distintos de los de agarre_node (con -p se cambian igual).
DEFECTOS = {
    # Vistas, de a 6 números: TCP (x, y, z) y punto al que apunta la pinza (x, y, z), en
    # base_footprint. ESTIMADAS, SIN PROBAR. Las dos primeras, para una bolsa en el suelo frente a
    # Milo (x ~ 0.6 m: el asa queda a ~0.5 m de altura); la tercera, para una en la mesa de prueba
    # (x ~ 0.8 m, asa a ~1.2 m). Como en agarre_node, apuntan un poco ANTES del asa: la cámara va
    # sobre la pinza y mira por encima de ella.
    'vistas': [0.35, 0.0, 0.85, 0.55, 0.0, 0.40,
               0.30, 0.0, 0.95, 0.55, 0.0, 0.45,
               0.45, 0.0, 1.35, 0.74, 0.0, 1.05],
    'confianza': 0.5,              # del modelo de segmentación (a ajustar cuando llegue)
    'subir': 0.15,                 # la bolsa cuelga del asa: con 15 cm despega del suelo
    'retraer': [0.55, 0.0, 1.05],  # más lejos del cuerpo que con el tenedor: la bolsa cuelga
    'tiempo_por_vista': 15.0,
}


class BolsaNode(AgarreNode):
    def __init__(self):
        super().__init__({'id': 'bolsa', 'clase': 'bag'}, nombre='bolsa_node', defectos=DEFECTOS)
        self.declare_parameter('modelo', '/ros2_ws/modelos/bolsa/best.pt')
        self.declare_parameter('clase', 'bag')           # el nombre de la clase en el modelo
        self.declare_parameter('paso', 2)                # un píxel de profundidad de cada 2
        # El asa se busca en TODOS los puntos de la imagen que caen sobre la bolsa vista desde
        # arriba, no solo en la máscara: así sirve aunque el equipo haya dibujado las máscaras sin
        # las asas. False = solo los puntos de la máscara.
        self.declare_parameter('usar_toda_la_imagen', True)
        self.declare_parameter('metido', 0.012)          # asa adentro de los dedos [m]
        self.declare_parameter('minimo_frenado', 0.002)  # ver el paso 5 arriba
        self.declare_parameter('unir', True)
        self.declare_parameter('medidas_bolsa', [0.315, 0.16, 0.41])   # ancho, fondo, alto [m]
        for n in ('modelo', 'clase', 'paso', 'usar_toda_la_imagen', 'metido', 'minimo_frenado',
                  'unir', 'medidas_bolsa'):
            self.p[n] = self.get_parameter(n).value

        self.color = self.profundidad = self.k_color = self.k_prof = None
        self.modelo = None
        self.ultima = None          # marca de tiempo de la última imagen procesada
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/gripper_camera/color/image_raw', self.al_color, s)
        self.create_subscription(Image, '/gripper_camera/depth/image_raw', self.al_prof, s)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info', self.al_kc, s)
        self.create_subscription(CameraInfo, '/gripper_camera/depth/camera_info', self.al_kp, s)
        # Para ver en RViz dónde encontró el asa (Pose, frame base_footprint): la pose de agarre.
        self.pub_asa = self.create_publisher(PoseStamped, '/bolsa/asa', 1)

    # ------------------------------------------------------------------ cámara
    def al_color(self, m):
        self.color = m

    def al_prof(self, m):
        self.profundidad = m

    def al_kc(self, m):
        self.k_color = np.array(m.k).reshape(3, 3)

    def al_kp(self, m):
        self.k_prof = np.array(m.k).reshape(3, 3)

    def asegurar_vision(self):
        """La visión corre en este mismo nodo (no se arranca objeto_node): se carga el modelo."""
        try:
            from ultralytics import YOLO
            self.modelo = YOLO(self.p['modelo'])
        except Exception as error:   # sin ultralytics, o sin el archivo de pesos
            raise Fallo(f'no pude cargar el modelo {self.p["modelo"]}: {error}')
        if self.p['clase'] not in self.modelo.names.values():
            raise Fallo(f'el modelo no tiene la clase "{self.p["clase"]}" (tiene: '
                        + ', '.join(self.modelo.names.values()) + ')')
        if getattr(self.modelo, 'task', 'segment') != 'segment':
            raise Fallo(f'el modelo es de tarea "{self.modelo.task}", no de segmentación')

    def mascara(self):
        """Máscara (alto x ancho, True/False) de la bolsa más segura en la imagen a color, o
        None."""
        imagen = image_to_array(self.color)[:, :, ::-1]   # RGB -> BGR (ver objeto_node)
        # retina_masks: máscaras del tamaño de la imagen original (si no, del tamaño de entrada
        # del modelo, 640 px).
        r = self.modelo.predict(imagen, conf=self.p['confianza'], retina_masks=True,
                                verbose=False)[0]
        if r.masks is None or len(r.boxes) == 0:
            return None
        nombres = self.modelo.names
        indices = [i for i, c in enumerate(r.boxes.cls.cpu().numpy())
                   if nombres[int(c)] == self.p['clase']]
        if not indices:
            return None
        conf = r.boxes.conf.cpu().numpy()
        i = max(indices, key=lambda j: conf[j])
        m = r.masks.data[i].cpu().numpy() > 0.5
        alto, ancho = imagen.shape[:2]
        if m.shape != (alto, ancho):   # por si no vino del tamaño original: vecino más cercano
            filas = np.arange(alto) * m.shape[0] // alto
            columnas = np.arange(ancho) * m.shape[1] // ancho
            m = m[filas][:, columnas]
        return m

    def medir_asa(self):
        """Busca el asa en la imagen actual: (asa, problema) como bolsa.buscar_asa."""
        m = self.mascara()
        if m is None:
            return None, f'no veo {self.p["clase"]}'
        frame_color, frame_prof = self.color.header.frame_id, self.profundidad.header.frame_id
        c2p = self.transformacion(frame_prof, frame_color)
        r, t = self.transformacion(FRAME, frame_prof)
        nube = nube_de_profundidad(profundidad_en_metros(self.profundidad), self.k_prof,
                                   int(self.p['paso']))
        dentro = en_la_mascara(nube, m, self.k_color, *c2p)
        base = nube @ r.T + t
        return buscar_asa(base[dentro], base if self.p['usar_toda_la_imagen'] else None)

    def encontrar_aqui(self):
        """Como en agarre_node, pero midiendo el asa aquí: espera `n_mediciones` seguidas con el
        vértice estable y devuelve su mediana, o None si no lo logra en `tiempo_por_vista`."""
        n = int(self.p['n_mediciones'])
        asas = []
        desde = self.get_clock().now()
        fin = time.time() + self.p['tiempo_por_vista']
        while time.time() < fin:
            self.esperar(0.1)
            if any(x is None for x in (self.color, self.profundidad, self.k_color, self.k_prof)):
                continue
            sello = Time.from_msg(self.color.header.stamp)
            if sello < desde or sello == self.ultima:   # imagen vieja o ya procesada
                continue
            self.ultima = sello
            asa, problema = self.medir_asa()
            if asa is None:
                self.get_logger().info(f'   {problema}', throttle_duration_sec=3.0)
                continue
            asas.append(asa)
            self.publicar_asa(asa)
            if len(asas) >= n and estable(asas[-n:], self.p['tol_estable']):
                return promediar_asas(asas[-n:])
        return None

    def publicar_asa(self, asa):
        posicion, giro = pose_de_agarre(asa, self.p['metido'])
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = FRAME, self.get_clock().now().to_msg()
        msg.pose = self.pose(posicion, giro)
        self.pub_asa.publish(msg)

    # ------------------------------------------------------------------ escena de MoveIt
    def unir_bolsa(self, asa, z_tcp):
        """Caja del tamaño de la bolsa colgando bajo la pinza. En el frame del TCP (pinza hacia
        abajo): X local = arriba, Y local = entre los dedos (el fondo de la bolsa, cruzado al
        asa), Z local = a lo largo del asa (el ancho de la bolsa). Su centro queda bajo la boca,
        a la mitad del alto de la bolsa."""
        ancho, fondo, alto = self.p['medidas_bolsa']
        bajo_tcp = (z_tcp - asa['boca_z']) + alto / 2.0
        adjunto = AttachedCollisionObject()
        adjunto.link_name = TCP
        adjunto.object = caja('bolsa_agarrada', (alto, fondo, ancho), (-bajo_tcp, 0.0, 0.0), TCP)
        adjunto.touch_links = TOCAN
        escena = PlanningScene(is_diff=True)
        escena.robot_state.is_diff = True
        escena.robot_state.attached_collision_objects.append(adjunto)
        self.escena.publish(escena)
        self.esperar(0.5)

    # ------------------------------------------------------------------ secuencia
    def correr(self):
        log = self.get_logger()
        self.esperar(2.0)   # TF, cámara y /joint_states
        if self.dedos is None:
            raise Fallo('no llega /joint_states con los dedos: ¿está corriendo arm_sim?')
        self.asegurar_vision()
        self.alejarse_de_la_mesa()

        log.info('1-2. Mirar y encontrar: abro la pinza y busco el asa de la bolsa')
        self.pinza(ABIERTA)
        asa = self.encontrar()
        v = asa['centro']
        log.info(f'   ¡Asa encontrada! Vértice en ({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f}), '
                 f'{asa["alto"] * 100:.0f} cm sobre la boca (a {asa["boca_z"]:.2f} m); mide '
                 f'{asa["largo"] * 100:.0f} cm de largo y {asa["grosor"] * 1000:.0f} mm de '
                 'grosor; '
                 f'{asa["n_asas"]} asa(s) a la vista')

        agarre, giro = pose_de_agarre(asa, self.p['metido'])
        previa = agarre + np.array([0.0, 0.0, self.p['sobre']])
        log.info(f'3. Previa: {self.p["sobre"] * 100:.0f} cm sobre el asa, apuntando hacia abajo')
        self.mover(self.pose(previa, giro), 'Previa')

        log.info(f'4. Bajar en línea recta hasta z = {agarre[2]:.3f} m (el asa queda '
                 f'{self.p["metido"] * 1000:.0f} mm adentro de los dedos)')
        self.linea_recta(self.pose(agarre, giro), 'Bajar')

        log.info('5. Cerrar la pinza del todo')
        abierta_en = self.pinza(CERRADA)
        if not hay_objeto(abierta_en, CERRADA, self.p['minimo_frenado']):
            self.pinza(ABIERTA)
            self.linea_recta(self.pose(previa, giro), 'Subir sin bolsa')
            raise Fallo(f'los dedos cerraron hasta {abierta_en * 1000:.1f} mm: no agarré el asa '
                        '(¿se movió la bolsa? ¿el asa es más delgada que minimo_frenado?)')
        log.info(f'   los dedos se frenaron a {abierta_en * 1000:.1f} mm: tengo el asa')

        if self.p['unir']:
            log.info('6. Unir la bolsa a la pinza en la escena de MoveIt')
            self.unir_bolsa(asa, agarre[2])

        log.info(f'7. Levantar {self.p["subir"] * 100:.0f} cm y retraer el brazo')
        arriba = agarre + np.array([0.0, 0.0, self.p['subir']])
        # Sin revisar choques: al arrancar, la bolsa unida está apoyada en el suelo o la mesa.
        self.linea_recta(self.pose(arriba, giro), 'Levantar', evitar_choques=False)
        retraida = self.pose(np.array(self.p['retraer'], float), giro)
        if self.p['mantener_orientacion']:
            self.linea_recta(retraida, 'Retraer')
        else:
            self.mover(retraida, 'Retraer')

        self.esperar(1.0)
        final = apertura(*self.dedos)
        if not hay_objeto(final, CERRADA, self.p['minimo_frenado']):
            raise Fallo(f'al terminar los dedos quedaron a {final * 1000:.1f} mm: la bolsa se '
                        'cayó')
        log.info(f'Listo: bolsa levantada por el asa (dedos a {final * 1000:.1f} mm)')


def main():
    # SIGTERM (kill, timeout) como Ctrl+C, igual que agarre_node.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    rclpy.init()
    nodo = BolsaNode()
    try:
        nodo.correr()
    except Fallo as error:
        nodo.get_logger().error(f'No se pudo: {error}')
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
