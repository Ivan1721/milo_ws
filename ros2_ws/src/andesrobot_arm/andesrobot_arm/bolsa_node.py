"""Nodo bolsa_node: toma una bolsa por el asa (del suelo o una mesa) y la levanta. P17.

  ros2 run andesrobot_arm bolsa_node            # modelo en /ros2_ws/modelos/bolsa/best.pt

ESTADO (24-nov-2026): usa el modelo de PUNTOS CLAVE del equipo de visión (YOLO11n-pose, una clase
`bag_handle`, 3 puntos por asa: base izquierda, punta, base derecha), entrenado con 400 imágenes
de Gazebo de tres bolsas (tela cruda, tela negra y papel). La geometría (bolsa.py) está probada
con escenas sintéticas (test/test_bolsa.py). Guía en docs/robocup/README.md §8.1.

Es agarre_node (la misma clase, AgarreNode: mismos movimientos, misma pinza, mismos parámetros)
con otra visión: en vez de pedirle a objeto_node una caja y medidas, corre AQUÍ el modelo de puntos
y pasa los puntos del asa a 3D con la profundidad (ver bolsa.py):

  1. MIRAR: abre la pinza y prueba las `vistas` una por una hasta ver un asa que sirva.
  2. ENCONTRAR: en cada imagen nueva, el modelo da las asas (caja + 3 puntos con su confianza).
     De cada asa con la caja y los TRES puntos sobre `confianza`, cada punto se vuelve un punto
     3D (la capa de la nube de profundidad más cercana a la cámara alrededor del punto: el asa
     es delgada) y bolsa.asa_de_puntos la mide y la valida (que esté parada, que sobresalga de
     la boca...). De las que sirven, la más cercana a Milo. Con `n_mediciones` (3) seguidas
     que coinciden (vértice a menos de `tol_estable`), sigue. Si ninguna sirve, anota por qué.
  3. PREVIA: la pinza va `sobre` (10 cm) encima del vértice del asa, apuntando hacia abajo, con los
     dedos cruzados al asa (MoveIt, camino libre de choques).
  4. BAJAR en línea recta hasta que el asa quede `metido` (12 mm) adentro de los dedos.
  5. CERRAR del todo: el asa mide pocos mm, así que no se usa "ancho − apriete" como con el
     tenedor; los 20 N de la pinza la aprietan. Se comprueba que los dedos se frenaron (si no,
     abre, sube y termina con error). OJO: cerrada del todo la pinza deja 3 mm entre las caras;
     hay_objeto pide que se frenen `minimo_frenado` antes, o sea un asa de 3 mm + minimo_frenado.
     Las cintas de tela de las bolsas del equipo miden 4 mm: por eso minimo_frenado es 0.5 mm
     aquí (2 mm en agarre_node).
  6. UNIR (si `unir`): una caja del tamaño de la bolsa (`medidas_bolsa`) colgando bajo la pinza en
     la escena de MoveIt, para que el brazo no la lleve contra el robot al retraer.
  7. LEVANTAR `subir` (15 cm: la bolsa cuelga del asa y despega del suelo) en línea recta y
     RETRAER sin girar la pinza (mantener_orientacion, en línea recta).
  8. Revisar que los dedos sigan frenados.

Para probarlo en la simulación (bolsa de tela cruda con colisión, worlds/bolsa_prueba.sdf):
  ./sim.sh brazo bolsa:=true octomap:=false
  ros2 run andesrobot_arm bolsa_node
Parámetros propios (además de los de agarre_node): modelo, clase, confianza_punto, paso, radio,
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
    BASE_DER, BASE_IZQ, PUNTA, asa_de_puntos, elegir_asa, nube_de_profundidad, pose_de_agarre,
    promediar_asas, proyectar, punto_del_pixel)
from andesrobot_arm.capturar_camara import image_to_array
from andesrobot_arm.manija_node import profundidad_en_metros

# Valores por defecto distintos de los de agarre_node (con -p se cambian igual).
DEFECTOS = {
    # Vistas, de a 6 números: TCP (x, y, z) y punto al que apunta la pinza (x, y, z), en
    # base_footprint. Para la bolsa de papel de bolsa_prueba.sdf parada en el suelo frente a Milo
    # (centro en x = 0.685 m, asas en x = 0.60 y 0.77, punta a 0.42 m). Elegidas el 24-nov en
    # Gazebo probando 19:
    # - el asa de ATRÁS, recortada contra el suelo gris, la ve con confianza > 0.9 en sus 3
    #   puntos; la de adelante, vista desde arriba contra la boca café de la bolsa, casi nunca;
    # - la imagen de profundidad (640 x 400) cubre menos que la de color (640 x 480): una punta en
    #   la franja de arriba de la imagen queda sin profundidad. Estas vistas dejan el asa al medio.
    # Una vista para una bolsa en la MESA está pendiente.
    'vistas': [0.42, 0.0, 0.70, 0.62, 0.0, 0.40,
               0.45, 0.0, 0.75, 0.65, 0.0, 0.40,
               0.40, 0.0, 0.75, 0.60, 0.0, 0.42],
    # Confianza mínima de la caja del asa. En la validación del equipo (60 imágenes de Gazebo)
    # el F1 es máximo con 0.69, pero desde 0.3 casi no cambia.
    'confianza': 0.5,
    'subir': 0.15,                 # la bolsa cuelga del asa: con 15 cm despega del suelo
    'retraer': [0.55, 0.0, 1.05],  # más lejos del cuerpo que con el tenedor: la bolsa cuelga
    'tiempo_por_vista': 15.0,
}


class BolsaNode(AgarreNode):
    def __init__(self):
        super().__init__({'id': 'bolsa', 'clase': 'bag_handle'}, nombre='bolsa_node',
                         defectos=DEFECTOS)
        self.declare_parameter('modelo', '/ros2_ws/modelos/bolsa/best.pt')
        self.declare_parameter('clase', 'bag_handle')    # el nombre de la clase en el modelo
        # Confianza mínima de CADA uno de los 3 puntos: un punto tapado (la base detrás de la
        # bolsa, la punta fuera de la imagen) sale con confianza baja y no se usa.
        self.declare_parameter('confianza_punto', 0.5)
        self.declare_parameter('paso', 1)                # todos los píxeles de profundidad
        self.declare_parameter('radio', 6.0)             # [px] alrededor de cada punto clave
        self.declare_parameter('metido', 0.012)          # asa adentro de los dedos [m]
        self.declare_parameter('minimo_frenado', 0.0005)  # ver el paso 5 arriba
        self.declare_parameter('unir', True)
        # ancho, fondo, alto [m]: la bolsa de papel del equipo (asas.json)
        self.declare_parameter('medidas_bolsa', [0.32, 0.17, 0.33])
        for n in ('modelo', 'clase', 'confianza_punto', 'paso', 'radio', 'metido',
                  'minimo_frenado', 'unir', 'medidas_bolsa'):
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
        self.pub_asa = self.create_publisher(PoseStamped, '/arm/bolsa/asa', 1)

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
        if getattr(self.modelo, 'task', None) != 'pose':
            raise Fallo(f'el modelo es de tarea "{self.modelo.task}", no de puntos clave (pose)')

    def detectar(self):
        """Las asas que ve el modelo en la imagen a color: lista de (confianza de la caja,
        puntos 3 x 2 en píxeles, confianza de cada punto). Solo las de la clase pedida."""
        imagen = image_to_array(self.color)[:, :, ::-1]   # RGB -> BGR (ver objeto_node)
        r = self.modelo.predict(imagen, conf=self.p['confianza'], verbose=False)[0]
        if r.keypoints is None or len(r.boxes) == 0:
            return []
        nombres = self.modelo.names
        clases = r.boxes.cls.cpu().numpy()
        conf = r.boxes.conf.cpu().numpy()
        xy = r.keypoints.xy.cpu().numpy()
        conf_p = (r.keypoints.conf.cpu().numpy() if r.keypoints.conf is not None
                  else np.ones(xy.shape[:2]))
        return [(float(conf[i]), xy[i], conf_p[i]) for i in range(len(clases))
                if nombres[int(clases[i])] == self.p['clase']]

    def medir_asa(self):
        """Busca el asa en la imagen actual: (asa o None, problemas). Cada asa detectada con sus 3 puntos
        confiables se pasa a 3D y se valida (bolsa.asa_de_puntos); de las que sirven, la más
        cercana a Milo (bolsa.elegir_asa)."""
        detecciones = self.detectar()
        if not detecciones:
            return None, f'no veo {self.p["clase"]}'
        frame_color, frame_prof = self.color.header.frame_id, self.profundidad.header.frame_id
        c2p = self.transformacion(frame_prof, frame_color)
        r, t = self.transformacion(FRAME, frame_prof)
        nube = nube_de_profundidad(profundidad_en_metros(self.profundidad), self.k_prof,
                                   int(self.p['paso']))
        uvz = proyectar(nube, self.k_color, *c2p)
        validas, problemas = [], []
        for i, (_, puntos, conf_p) in enumerate(detecciones):
            if np.min(conf_p) < self.p['confianza_punto']:
                nombres = ('base izquierda', 'punta', 'base derecha')
                dudosos = [nombres[j] for j in range(3) if conf_p[j] < self.p['confianza_punto']]
                problemas.append(f'asa {i + 1}: no se ve bien ' + ', '.join(dudosos))
                continue
            en_3d = []
            for j in (BASE_IZQ, PUNTA, BASE_DER):
                p = punto_del_pixel(nube, uvz, puntos[j], self.p['radio'])
                if p is None:
                    break
                en_3d.append(r @ p + t)
            if len(en_3d) < 3:
                problemas.append(f'asa {i + 1}: sin profundidad en algún punto')
                continue
            asa, problema = asa_de_puntos(*en_3d)
            if asa is None:
                problemas.append(f'asa {i + 1}: {problema}')
            else:
                validas.append(asa)
        # Con un asa válida, `problema` igual lista las descartadas (sirve para depurar).
        return elegir_asa(validas), '; '.join(problemas)

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
            if sello < desde or (self.ultima is not None and sello == self.ultima):
                continue                                  # imagen vieja o ya procesada
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
                 f'{asa["alto"] * 100:.0f} cm sobre la boca (a {asa["boca_z"]:.2f} m); bases '
                 f'a {asa["largo"] * 100:.0f} cm; {asa["n_asas"]} asa(s) que sirven a la vista')

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
                        '(¿se movió la bolsa? ¿el asa mide menos de 3 mm + minimo_frenado?)')
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
