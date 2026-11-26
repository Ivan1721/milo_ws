"""Detección de objetos EN VIVO con la cámara de la pinza: cajas y distancia de cada objeto.

En palabras simples: unas `hz` veces por segundo toma la última imagen a color de la cámara, se la
pasa a un modelo YOLO y publica la misma imagen con una caja, el nombre, la confianza y la
distancia de cada objeto (la distancia sale de la imagen de profundidad, ver deteccion.py). Sirve
para ver qué reconoce el modelo con la cámara real, en tiempo real, y es la base para que el brazo
vaya a buscar lo que ve.

Modelos:
  - YOLOE (por defecto, /ros2_ws/modelos/yoloe-11s-seg.pt): "vocabulario abierto": no viene con
    una lista fija de objetos, se le dicen con palabras en inglés (`clases`). Con la cámara real
    vio la lata de Red Bull como "cup" (0.45) el 26-nov-2026, cuando el modelo del equipo no la
    veía. Para el texto usa mobileclip_blt.ts, que debe estar en la carpeta del modelo.
  - cualquier otro .pt de Ultralytics (por ejemplo el del equipo, modelos/objetos/best.pt, o
    weights/yolo26n.pt con las 80 clases de COCO): usa sus propias clases y `clases` se ignora.

Escucha:  /gripper_camera/color/image_raw, /gripper_camera/depth/image_raw,
          /gripper_camera/color/camera_info
Publica:  /arm/deteccion/imagen   (sensor_msgs/Image rgb8) la imagen con las cajas
          /arm/deteccion/objetos  (std_msgs/String, JSON) una lista con clase, confianza, caja
                                  [px], distancia [m] y punto (x, y, z) [m] en el frame óptico de
                                  color (sin distancia si no hay profundidad en la caja)

Parámetros: modelo, clases ("can, cup, bottle, ..."), confianza (0.25), hz (10).

Uso (cámara real, en el PC):  ./robot.sh camara alineada:=true deteccion:=true
     (alineada:=true: la profundidad en el punto de vista de la lente de color, así la distancia
     de cada caja es exacta; sin eso se aproxima escalando la caja)
     ros2 run andesrobot_arm deteccion_node --ros-args -p clases:="can, cup"
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from andesrobot_arm.capturar_camara import depth_to_mm, image_to_array
from andesrobot_arm.deteccion import distancia_en_caja, escalar_caja, leer_clases, punto_3d

# Objetos de la RoboCup@Home (docs/robocup/README.md) y parecidos, en inglés para YOLOE.
CLASES = ('can, cup, mug, bottle, bowl, plate, bag, box, card, ball, fork, spoon, knife, '
          'apple, banana, orange, sponge, toy')


class Deteccion(Node):
    def __init__(self):
        super().__init__('deteccion_node')
        self.declare_parameter('modelo', '/ros2_ws/modelos/yoloe-11s-seg.pt')
        self.declare_parameter('clases', CLASES)
        self.declare_parameter('confianza', 0.25)
        self.declare_parameter('hz', 10.0)
        p = self.get_parameter
        self.confianza = p('confianza').value
        self.color = None
        self.prof = None
        self.K = None
        self.cargar_modelo(p('modelo').value, leer_clases(p('clases').value))
        s = qos_profile_sensor_data
        self.create_subscription(Image, '/gripper_camera/color/image_raw', self.on_color, s)
        self.create_subscription(Image, '/gripper_camera/depth/image_raw', self.on_prof, s)
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info', self.on_k, s)
        self.pub_imagen = self.create_publisher(Image, '/arm/deteccion/imagen', 1)
        self.pub_objetos = self.create_publisher(String, '/arm/deteccion/objetos', 1)
        self.create_timer(1.0 / p('hz').value, self.detectar)
        self.cuadros, self.t_aviso = 0, time.monotonic()

    def cargar_modelo(self, ruta, clases):
        from ultralytics import YOLO, YOLOE
        if 'yoloe' in os.path.basename(ruta):
            # YOLOE busca mobileclip_blt.ts en la carpeta actual: se trabaja en la del modelo.
            os.chdir(os.path.dirname(ruta))
            self.modelo = YOLOE(ruta)
            self.modelo.set_classes(clases, self.modelo.get_text_pe(clases))
            self.get_logger().info(f'YOLOE {os.path.basename(ruta)} buscando: {", ".join(clases)}')
        else:
            self.modelo = YOLO(ruta)
            self.get_logger().info(f'{ruta}: {len(self.modelo.names)} clases propias')

    def on_color(self, msg):
        self.color = msg

    def on_prof(self, msg):
        self.prof = msg

    def on_k(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def detectar(self):
        if self.color is None:
            return
        msg = self.color
        rgb = image_to_array(msg)
        # image_to_array da RGB y Ultralytics supone BGR (como OpenCV): se dan vuelta los canales.
        r = self.modelo(rgb[:, :, ::-1], conf=self.confianza, verbose=False)[0]
        prof = depth_to_mm(self.prof) if self.prof is not None else None
        objetos, textos = [], []
        for b in r.boxes:
            caja = [float(v) for v in b.xyxy[0]]
            clase, conf = r.names[int(b.cls)], float(b.conf)
            obj = {'clase': clase, 'confianza': round(conf, 3),
                   'caja': [round(v) for v in caja]}
            z = None
            if prof is not None:
                caja_p = escalar_caja(caja, (msg.width, msg.height),
                                      (prof.shape[1], prof.shape[0]))
                z = distancia_en_caja(prof, caja_p)
            if z is not None:
                obj['distancia'] = round(z, 3)
                if self.K is not None:
                    u, v = (caja[0] + caja[2]) / 2, (caja[1] + caja[3]) / 2
                    obj['punto'] = [round(c, 3) for c in punto_3d(u, v, z, self.K)]
            objetos.append(obj)
            textos.append((caja, f'{z:.2f} m' if z is not None else 'sin prof.'))
        # Imagen con las cajas (plot() de Ultralytics da BGR) + la distancia bajo cada caja.
        import cv2
        dibujo = np.ascontiguousarray(r.plot(line_width=2))
        for caja, texto in textos:
            cv2.putText(dibujo, texto, (int(caja[0]), int(caja[3]) + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        salida = Image(header=msg.header, height=dibujo.shape[0], width=dibujo.shape[1],
                       encoding='rgb8', is_bigendian=0, step=dibujo.shape[1] * 3,
                       data=np.ascontiguousarray(dibujo[:, :, ::-1]).tobytes())
        self.pub_imagen.publish(salida)
        self.pub_objetos.publish(String(data=json.dumps(objetos, ensure_ascii=False)))
        self.cuadros += 1
        ahora = time.monotonic()
        if ahora - self.t_aviso >= 10.0:
            resumen = ', '.join(f'{o["clase"]} {o["confianza"]:.2f}' +
                                (f' a {o["distancia"]:.2f} m' if 'distancia' in o else '')
                                for o in objetos) or 'nada'
            self.get_logger().info(f'{self.cuadros / (ahora - self.t_aviso):.1f} imágenes/s; '
                                   f'ahora: {resumen}')
            self.cuadros, self.t_aviso = 0, ahora


def main():
    rclpy.init()
    nodo = Deteccion()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
