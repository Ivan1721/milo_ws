#!/usr/bin/env python3
"""¿Qué reconoce el modelo de objetos cuando hay varios en la mesa? Prueba grupo por grupo.

Es la prueba del equipo de visión (prueba_completa.zip, 15-nov-2026: 5 grupos por tema, una foto
por grupo) pero con Milo y la cámara de la pinza, en la mesa de arm_sim y con los objetos de
nuestro catálogo. Los grupos están en andesrobot_arm/grupos.py (GRUPOS). Para cada grupo:

  1. saca de la mesa los objetos del catálogo que haya y pone los del grupo en dos filas, sin que
     se toquen (grupos.acomodar: los bajos adelante, algunos girados 90° si no caben derechos);
  2. lleva la pinza a cada una de VISTAS (las tres de evaluar_objetos.py) y saca una foto con la
     cámara de la pinza;
  3. pasa el modelo YOLO por la foto y, como sabemos dónde puso cada objeto, proyecta su centro en
     la imagen (con la TF odom -> cámara y la camera_info), proyecta también las 8 esquinas de su
     caja y le asigna cada caja de YOLO al objeto con el que más se superpone (grupos.asignar):
     "lo encontró" (caja de su clase), "lo confundió" (caja de otra clase) o "no salía en la foto".

En la simulación odom es el mundo de Gazebo (el plugin de las ruedas saca la odometría de la pose
real), así que aunque la base se corra un poco, la proyección sigue apuntando al objeto.

Escribe un CSV (una fila por objeto y vista) en /ros2_ws/resultados/objetos/grupos_<fecha>.csv,
las fotos con las cajas de YOLO y un punto en el centro de cada objeto en
/ros2_ws/capturas/grupos_<fecha>/ (las dos carpetas, fuera de git), y al final una tabla.

Uso (dentro del contenedor, con la simulación de los objetos corriendo y nada más moviendo el
brazo; ~1-2 min por grupo):
  ros2 launch andesrobot_arm arm_sim.launch.py objetos:=true octomap:=false     # terminal 1
  python3 /ros2_ws/src/andesrobot_arm/scripts/evaluar_grupos.py                  # los 5 grupos
  python3 /ros2_ws/src/andesrobot_arm/scripts/evaluar_grupos.py cocina varios    # algunos
  python3 .../evaluar_grupos.py --modelo /ros2_ws/modelos/entrega1_2026-11-15/best.pt
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
from gazebo_msgs.srv import SpawnEntity
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent))      # andesrobot_arm/ sin tener que compilar
sys.path.insert(0, str(AQUI))             # evaluar_objetos.py, al lado
from andesrobot_arm.capturar_camara import image_to_array  # noqa: E402
from andesrobot_arm.catalogo_objetos import altura_de_spawn, carpeta, leer_catalogo  # noqa: E402
from andesrobot_arm.grupos import (  # noqa: E402
    GRUPOS, acomodar, asignar, caja_de_sdf, centro_en_mesa, esquinas_en_mesa, matriz_de_tf,
    normalizar, proyectar, rectangulo, resumir)
from evaluar_objetos import CATALOGO, MODELOS, VISTAS as VISTAS_OBJETO, Evaluador  # noqa: E402

CAMARA = 'gripper_camera_color_optical_frame'
# Poses del TCP (x, y, z, qx, qy, qz, qw en base_footprint): las tres de evaluar_objetos.py
# (calculadas para mirar a (0.80, 0)). Con la mesa llena, frente40 y arriba ya muestran los dos
# extremos de la mesa. Se probaron también "arriba" corrida 20 cm a cada lado (15-nov): el brazo
# llega, pero la muñeca queda girada y la foto sale en diagonal, y el modelo encontraba menos.
VISTAS = dict(VISTAS_OBJETO)


class EvaluadorGrupos(Evaluador):
    """El de evaluar_objetos.py más: poner un objeto en cualquier lugar (y girado), la
    camera_info y la TF odom -> cámara de cada foto."""

    def __init__(self, modelo):
        super().__init__(modelo)
        self.info = None
        self.create_subscription(CameraInfo, '/gripper_camera/color/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)

    def colocar_en(self, objeto, caja, lugar):
        d = MODELOS / carpeta(objeto)
        req = SpawnEntity.Request()
        req.name = objeto['id']
        req.xml = (d / 'model.sdf').read_text()
        req.initial_pose.position.x, req.initial_pose.position.y = lugar[0], lugar[1]
        req.initial_pose.position.z = altura_de_spawn(caja)
        req.initial_pose.orientation.z = float(np.sin(lugar[2] / 2))
        req.initial_pose.orientation.w = float(np.cos(lugar[2] / 2))
        req.reference_frame = 'world'
        r = self.llamar(self.poner, req)
        if not r.success:
            raise RuntimeError(r.status_message)

    def foto_y_camara(self):
        """(imagen RGB, matriz 4x4 odom -> frame óptico, K 3x3) de una foto nueva."""
        self.foto = None
        self.esperar(1.5)            # que la imagen sea de después de llegar
        while self.foto is None or self.info is None:
            self.esperar(0.1)
        try:
            # lookup_transform(destino, origen) pasa puntos de odom al frame de la cámara
            t = self.tf.lookup_transform(CAMARA, 'odom', Time()).transform
        except TransformException as error:
            raise RuntimeError(f'sin TF odom -> {CAMARA}: {error}')
        m = matriz_de_tf((t.translation.x, t.translation.y, t.translation.z),
                         (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        return image_to_array(self.foto), m, np.array(self.info.k, float).reshape(3, 3)


def caja_del_objeto(objeto):
    """Caja {'min', 'max'} del modelo: la de milo.json, o la de su <box> si no tiene malla."""
    d = MODELOS / carpeta(objeto)
    medida = json.loads((d / 'milo.json').read_text()).get('medidas')
    return medida or caja_de_sdf((d / 'model.sdf').read_text())


def dibujar(imagen_bgr, uv, visibles, ids, resultados, archivo):
    """Guarda la foto con un punto en el centro proyectado de cada objeto: verde = lo encontró,
    rojo = no (o lo confundió)."""
    import cv2
    for (u, v), vis, ident, r in zip(uv, visibles, ids, resultados):
        if not vis:
            continue
        color = (0, 200, 0) if r['correcta'] is not None else (0, 0, 255)
        p = (int(u), int(v))
        cv2.circle(imagen_bgr, p, 6, color, -1)
        cv2.putText(imagen_bgr, ident, (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color, 2)
    cv2.imwrite(str(archivo), imagen_bgr)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('grupos', nargs='*', help=f'grupos (vacío = todos): {", ".join(GRUPOS)}')
    ap.add_argument('--modelo', default='/ros2_ws/modelos/objetos/best.pt')
    ap.add_argument('--conf', type=float, default=0.25, help='confianza mínima de YOLO')
    a = ap.parse_args()
    malos = [g for g in a.grupos if g not in GRUPOS]
    if malos:
        raise SystemExit(f'No hay grupo {malos} (hay: {", ".join(GRUPOS)})')
    grupos = {g: GRUPOS[g] for g in (a.grupos or GRUPOS)}
    catalogo = {o['id']: o for o in leer_catalogo(CATALOGO.read_text())}
    faltan = [i for ids in grupos.values() for i in ids
              if not (MODELOS / carpeta(catalogo[i]) / 'milo.json').exists()]
    if faltan:
        raise SystemExit('Faltan modelos: python3 /ros2_ws/src/andesrobot_arm/scripts/'
                         'descargar_objetos.py ' + ' '.join(faltan))
    fecha = time.strftime('%Y%m%d_%H%M')
    fotos = Path(f'/ros2_ws/capturas/grupos_{fecha}')
    fotos.mkdir(parents=True, exist_ok=True)
    salida = Path('/ros2_ws/resultados/objetos')
    salida.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    nodo = EvaluadorGrupos(a.modelo)
    log = nodo.get_logger()
    del_modelo = {normalizar(n) for n in nodo.yolo.names.values()}
    for nombre in nodo.modelos_en_gazebo():
        if nombre in catalogo:
            log.info(f'saco {nombre} de la mesa')
            nodo.quitar(nombre)
    filas, tabla = [], []
    for grupo, ids in grupos.items():
        objetos = [catalogo[i] for i in ids]
        cajas = [caja_del_objeto(o) for o in objetos]
        lugares = acomodar(cajas)
        centros = np.array([centro_en_mesa(lu, c) for lu, c in zip(lugares, cajas)])
        esquinas = [esquinas_en_mesa(lu, c) for lu, c in zip(lugares, cajas)]
        clases = [o['clase'] for o in objetos]
        por_objeto = {i: [] for i in ids}
        puestos = []
        try:
            for o, c, lu in zip(objetos, cajas, lugares):
                nodo.colocar_en(o, c, lu)
                puestos.append(o['id'])
            nodo.esperar(3.0)        # que se asienten sobre la mesa
            for vista in VISTAS:
                if not nodo.ir(vista, vistas=VISTAS):
                    log.warn(f'{grupo}: la pinza no llegó a la vista {vista}')
                    continue
                imagen, m, k = nodo.foto_y_camara()
                alto, ancho = imagen.shape[:2]
                uv, vis = proyectar(centros, m, k, ancho, alto)
                rects = [rectangulo(proyectar(e, m, k, ancho, alto)[0], ancho, alto)
                         for e in esquinas]
                # RGB -> BGR: Ultralytics supone que un arreglo viene en BGR (ver manija_node).
                r = nodo.yolo.predict(imagen[:, :, ::-1], conf=a.conf, verbose=False)[0]
                nombres = [nodo.yolo.names[int(c)] for c in r.boxes.cls]
                confs = [float(c) for c in r.boxes.conf]
                res = asignar(clases, rects, vis, r.boxes.xyxy.cpu().numpy(), nombres, confs)
                dibujar(r.plot(), uv, vis, ids, res,
                        fotos / f'{grupo}_{vista}.jpg')
                for o, x in zip(objetos, res):
                    por_objeto[o['id']].append(x)
                    filas.append({'grupo': grupo, 'id': o['id'], 'clase': o['clase'],
                                  'vista': vista, 'visible': int(x['visible']),
                                  'correcta': '' if x['correcta'] is None
                                  else f'{x["correcta"]:.2f}',
                                  'otra': '' if x['otra'] is None
                                  else f'{x["otra"][0]} {x["otra"][1]:.2f}'})
                log.info(f'{grupo} / {vista}: '
                         + ', '.join(f'{o["id"]} '
                                     + ('-' if not x['visible'] else
                                        f'{x["correcta"]:.2f}' if x['correcta'] is not None
                                        else f'[{x["otra"][0]}]' if x['otra'] else 'nada')
                                     for o, x in zip(objetos, res)))
        except Exception as error:  # noqa: BLE001 (se anota y se sigue con el siguiente)
            log.error(f'{grupo}: {error}')
        finally:
            for ident in puestos:
                try:
                    nodo.quitar(ident)
                except Exception:  # noqa: BLE001
                    pass
        for o in objetos:
            tabla.append((grupo, o, resumir(por_objeto[o['id']]),
                          normalizar(o['clase']) in del_modelo))
    archivo = salida / f'grupos_{fecha}.csv'
    with open(archivo, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['grupo', 'id', 'clase', 'vista', 'visible',
                                          'correcta', 'otra'])
        w.writeheader()
        w.writerows(filas)
    print(f'\nmodelo: {a.modelo}   conf >= {a.conf}   vistas: {", ".join(VISTAS)}')
    print(f'{"grupo":19s} {"objeto":16s} {"clase":18s} {"en foto":>7s} {"acierta":>7s} '
          f'{"mejor":>5s}  confundido con')
    for grupo, o, s, en_modelo in tabla:
        mejor = '-' if s['mejor'] is None else f'{s["mejor"]:.2f}'
        otras = ', '.join(f'{c} ({n})' for c, n in s['confusiones'].items())
        if not en_modelo:
            otras = '(el modelo no tiene esta clase) ' + otras
        print(f'{grupo:19s} {o["id"]:16s} {o["clase"]:18s} {s["vistas"]:7d} {s["aciertos"]:7d} '
              f'{mejor:>5s}  {otras}')
    posibles = [t for t in tabla if t[3]]
    bien = sum(1 for t in posibles if t[2]['aciertos'] > 0)
    print(f'\nEncontrados en al menos una vista: {bien} de {len(posibles)} '
          f'(sin contar {len(tabla) - len(posibles)} cuya clase el modelo no tiene)')
    print(f'CSV: {archivo}   fotos: {fotos}')
    nodo.destroy_node()
    rclpy.try_shutdown()
    # os._exit: salir sin esperar a los hilos de YOLO/ROS (ver evaluar_objetos.py).
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
