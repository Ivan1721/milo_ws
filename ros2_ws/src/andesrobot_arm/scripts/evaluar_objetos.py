#!/usr/bin/env python3
"""¿Qué clases reconoce el modelo de objetos en la simulación? Prueba objeto por objeto.

Pone cada objeto del catálogo (config/objetos.yaml) solo en la mesa, en (0.80, 0), lleva la pinza a
VISTAS (tres puntos de vista: de frente a 30° y 40°, y desde arriba), saca una foto con la cámara
de la pinza y le pasa el modelo YOLO (modelos/objetos/best.pt) con confianza mínima 0.1. Anota:

  - con qué confianza encontró la clase correcta en cada vista (vacío = no la encontró);
  - la detección más segura de la foto, aunque sea de otra clase (para ver con qué la confunde).

Escribe un CSV en /ros2_ws/resultados/objetos/ y las fotos con las cajas en
/ros2_ws/capturas/evaluacion_<fecha>/ (las dos carpetas, fuera de git), y al final una tabla.
Antes de empezar saca de la mesa los objetos del catálogo que haya (taza_blanca, bowl_blanco...).

Uso (dentro del contenedor, con la simulación de los objetos corriendo y nada más moviendo el
brazo; ~1 min por objeto):
  ros2 launch andesrobot_arm arm_sim.launch.py objetos:=true octomap:=false     # terminal 1
  python3 /ros2_ws/src/andesrobot_arm/scripts/evaluar_objetos.py                 # todos
  python3 /ros2_ws/src/andesrobot_arm/scripts/evaluar_objetos.py taza platano    # algunos
"""
import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent))      # andesrobot_arm/ sin tener que compilar
from andesrobot_arm.capturar_camara import image_to_array  # noqa: E402
from andesrobot_arm.catalogo_objetos import (  # noqa: E402
    altura_de_spawn, carpeta, elegir, leer_catalogo)

CATALOGO = AQUI.parent / 'config' / 'objetos.yaml'
MODELOS = Path('/ros2_ws/modelos/gazebo')
SITIO = (0.80, 0.0)          # donde va cada objeto: al centro de la primera fila
# Poses del TCP (x, y, z, qx, qy, qz, qw en base_footprint) con la cámara mirando a (0.80, 0,
# 0.77): calculadas con el URDF el 7-nov (la cámara va detrás de la muñeca, 16 cm detrás del TCP
# y 10 cm arriba, inclinada 20°). La evaluación del 7-nov (4 de 26) usó las de la cámara anterior.
VISTAS = {
    'frente30': (0.505, -0.009, 0.892, -0.0872, 0.0, -0.9962, 0.0),   # cámara a 50 cm, 30°
    'frente40': (0.531, -0.009, 0.941, -0.1736, 0.0, -0.9848, 0.0),   # cámara a 50 cm, 40°
    'arriba': (0.5, 0.0, 1.4, 0.3827, 0.0, 0.9239, 0.0),               # pinza 45° hacia abajo
}


class Evaluador(Node):
    def __init__(self, modelo):
        super().__init__('evaluar_objetos', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        from ultralytics import YOLO
        self.yolo = YOLO(modelo)
        self.foto = None
        self.create_subscription(Image, '/gripper_camera/color/image_raw',
                                 lambda m: setattr(self, 'foto', m), qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped, '/arm_target_pose', 1)
        self.poner = self.create_client(SpawnEntity, '/spawn_entity')
        self.sacar = self.create_client(DeleteEntity, '/delete_entity')
        self.lista = self.create_client(GetModelList, '/get_model_list')
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def llamar(self, cliente, req, seg=30.0):
        if not cliente.wait_for_service(timeout_sec=seg):
            raise RuntimeError(f'no está el servicio {cliente.srv_name} (¿corre la simulación?)')
        fut = cliente.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=seg)
        return fut.result()

    def esperar(self, seg):
        fin = time.time() + seg
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.05)

    def modelos_en_gazebo(self):
        return list(self.llamar(self.lista, GetModelList.Request()).model_names)

    def quitar(self, nombre):
        req = DeleteEntity.Request()
        req.name = nombre
        self.llamar(self.sacar, req)

    def colocar(self, objeto):
        d = MODELOS / carpeta(objeto)
        medida = json.loads((d / 'milo.json').read_text())['medidas']
        req = SpawnEntity.Request()
        req.name = objeto['id']
        req.xml = (d / 'model.sdf').read_text()
        req.initial_pose.position.x, req.initial_pose.position.y = SITIO
        req.initial_pose.position.z = altura_de_spawn(medida)
        req.reference_frame = 'world'
        r = self.llamar(self.poner, req)
        if not r.success:
            raise RuntimeError(r.status_message)

    def tcp(self):
        try:
            t = self.tf.lookup_transform('base_footprint', 'gripper_tcp', Time()).transform
        except TransformException:
            return None
        return np.array([t.translation.x, t.translation.y, t.translation.z])

    def ir(self, vista, limite=30.0):
        """Manda la pinza a una vista y espera que llegue (a 1.5 cm). True si llegó."""
        x, y, z, qx, qy, qz, qw = VISTAS[vista]
        msg = PoseStamped()
        msg.header.frame_id = 'base_footprint'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self.pub.publish(msg)
        fin = time.time() + limite
        while time.time() < fin:
            self.esperar(0.5)
            p = self.tcp()
            if p is not None and np.linalg.norm(p - [x, y, z]) < 0.015:
                return True
        return False

    def mirar(self, objeto, carpeta_fotos, vista):
        """Foto nueva + YOLO: (confianza de la clase correcta o None, (clase, conf) más segura)."""
        self.foto = None
        self.esperar(1.5)            # que la imagen sea de después de llegar
        while self.foto is None:
            self.esperar(0.1)
        # RGB -> BGR: Ultralytics supone que un arreglo viene en BGR (ver manija_node).
        imagen = image_to_array(self.foto)[:, :, ::-1]
        r = self.yolo.predict(imagen, conf=0.1, verbose=False)[0]
        r.save(filename=str(carpeta_fotos / f'{objeto["id"]}_{vista}.jpg'))
        nombres = [self.yolo.names[int(c)] for c in r.boxes.cls]
        confs = [float(c) for c in r.boxes.conf]
        correcta = max((c for n, c in zip(nombres, confs) if n == objeto['clase']), default=None)
        mejor = max(zip(confs, nombres), default=(None, None))
        return correcta, (mejor[1], mejor[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('ids', nargs='*', help='ids o clases del catálogo (vacío = todos)')
    ap.add_argument('--modelo', default='/ros2_ws/modelos/objetos/best.pt')
    a = ap.parse_args()
    catalogo = leer_catalogo(CATALOGO.read_text())
    objetos = elegir(catalogo, ','.join(a.ids)) if a.ids else catalogo
    faltan = [o['id'] for o in objetos if not (MODELOS / carpeta(o) / 'milo.json').exists()]
    if faltan:
        raise SystemExit('Faltan modelos: python3 /ros2_ws/src/andesrobot_arm/scripts/'
                         'descargar_objetos.py ' + ' '.join(faltan))
    fecha = time.strftime('%Y%m%d_%H%M')
    fotos = Path(f'/ros2_ws/capturas/evaluacion_{fecha}')
    fotos.mkdir(parents=True, exist_ok=True)
    salida = Path('/ros2_ws/resultados/objetos')
    salida.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    nodo = Evaluador(a.modelo)
    log = nodo.get_logger()
    ids = {o['id'] for o in catalogo}
    for nombre in nodo.modelos_en_gazebo():
        if nombre in ids:
            log.info(f'saco {nombre} de la mesa')
            nodo.quitar(nombre)
    filas = []
    for objeto in objetos:
        fila = {'id': objeto['id'], 'clase': objeto['clase']}
        try:
            nodo.colocar(objeto)
            nodo.esperar(2.0)        # que se asiente sobre la mesa
            for vista in VISTAS:
                if not nodo.ir(vista):
                    log.warn(f'{objeto["id"]}: la pinza no llegó a la vista {vista}')
                    fila[vista] = 'no llegó'
                    continue
                correcta, (otra, conf) = nodo.mirar(objeto, fotos, vista)
                fila[vista] = '' if correcta is None else f'{correcta:.2f}'
                fila[vista + '_mejor'] = '' if otra is None else f'{otra} {conf:.2f}'
            log.info(f'{objeto["id"]} ({objeto["clase"]}): '
                     + ', '.join(f'{v} {fila.get(v) or "-"} [{fila.get(v + "_mejor") or "nada"}]'
                                 for v in VISTAS))
        except Exception as error:  # noqa: BLE001 (se anota y se sigue con el siguiente)
            log.error(f'{objeto["id"]}: {error}')
            fila['error'] = str(error)
        finally:
            try:
                nodo.quitar(objeto['id'])
            except Exception:  # noqa: BLE001
                pass
        filas.append(fila)
    columnas = ['id', 'clase'] + [c for v in VISTAS for c in (v, v + '_mejor')] + ['error']
    archivo = salida / f'evaluacion_{fecha}.csv'
    with open(archivo, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=columnas)
        w.writeheader()
        w.writerows(filas)
    print(f'\n{"objeto":14s} {"clase":18s} ' + ' '.join(f'{v:>9s}' for v in VISTAS)
          + '  más segura (vista de frente 30°)')
    for fila in filas:
        print(f'{fila["id"]:14s} {fila["clase"]:18s} '
              + ' '.join(f'{(fila.get(v) or "-"):>9s}' for v in VISTAS)
              + f'  {fila.get("frente30_mejor") or fila.get("error", "")}')
    vistas_ok = sum(1 for f in filas if any(f.get(v, '')[:1].isdigit() for v in VISTAS))
    print(f'\nReconocidos en al menos una vista: {vistas_ok} de {len(filas)}')
    print(f'CSV: {archivo}   fotos: {fotos}')
    nodo.destroy_node()
    rclpy.try_shutdown()
    # os._exit: salir sin esperar a los hilos de YOLO/ROS (a veces dejaban el script colgado);
    # antes, vaciar lo impreso, que si no se pierde.
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
