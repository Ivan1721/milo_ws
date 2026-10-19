"""Afina un modelo YOLO (Ultralytics) para encontrar la manija, con las imágenes simuladas.

Las imágenes y etiquetas salen de generar_dataset_manija.py, en dos corridas (con distinta
semilla, así las puertas de validación son variantes que el modelo nunca vio al entrenar):
  datasets/manija_sim/train/   images/ + labels_det/ + labels_seg/ + labels_pose/
  datasets/manija_sim/val/     igual

Tres tareas posibles (--tarea), según qué etiqueta se use:
  pose  caja + 3 keypoints (eje, agarre, punta) -> yolov8s-pose  (la recomendada: da el punto
        de agarre y la dirección de la palanca directamente)
  seg   máscara (polígono)                      -> yolov8s-seg   (para medir qué fracción de
        la manija se ve)
  det   solo la caja                            -> yolov8s
"Afinar" (fine-tuning): se parte de los pesos ya entrenados en COCO (personas, autos, tazas...),
que ya saben reconocer bordes y formas, y solo se les enseña la clase nueva "manija". Necesita
muchas menos imágenes y minutos que entrenar desde cero.

Uso (dentro del contenedor, sin simulación corriendo: la GPU queda libre):
  ./sim.sh shell
  python3 /ros2_ws/src/andesrobot_arm/scripts/entrenar_manija.py --tarea pose --epocas 100
Los pesos base se bajan solos a ros2_ws/modelos/ y el resultado queda en
ros2_ws/modelos/entrenamientos/manija_<tarea>/ (weights/best.pt, curvas, matriz de confusión,
fotos de validación con lo que predijo). Nada de esto va a git.
"""
import argparse
import os
import shutil

MODELOS = '/ros2_ws/modelos'
BASE = {'pose': 'yolov8s-pose.pt', 'seg': 'yolov8s-seg.pt', 'det': 'yolov8s.pt'}


def armar_dataset(datos, tarea):
    """Carpetas como las espera Ultralytics: images/{train,val} y labels/{train,val}.

    Ultralytics busca la etiqueta de cada imagen cambiando "images" por "labels" en la ruta,
    así que aquí se arma esa estructura con enlaces duros (no ocupan espacio extra) en
    <datos>/yolo_<tarea>/. Devuelve la ruta del data.yaml.
    """
    destino = os.path.join(datos, f'yolo_{tarea}')
    shutil.rmtree(destino, ignore_errors=True)
    for parte in ('train', 'val'):
        origen = os.path.join(datos, parte)
        if not os.path.isdir(os.path.join(origen, 'images')):
            raise SystemExit(f'Falta {origen}/images: corre antes generar_dataset_manija.py '
                             f'--salida {origen}')
        for sub, sub_origen in (('images', 'images'), ('labels', f'labels_{tarea}')):
            d = os.path.join(destino, sub, parte)
            os.makedirs(d)
            for f in os.listdir(os.path.join(origen, sub_origen)):
                os.link(os.path.join(origen, sub_origen, f), os.path.join(d, f))
    extra = ''
    if tarea == 'pose':
        # 3 keypoints con (x, y, visibilidad). flip_idx: al voltear la imagen de izquierda a
        # derecha (aumento de datos) cada keypoint sigue siendo el mismo (no hay "izquierdo" y
        # "derecho" que intercambiar). Voltear sirve: hay puertas con la bisagra al otro lado.
        extra = 'kpt_shape: [3, 3]\nflip_idx: [0, 1, 2]\n'
    yaml = os.path.join(destino, 'data.yaml')
    with open(yaml, 'w') as f:
        f.write(f'# Generado por entrenar_manija.py\npath: {destino}\n'
                f'train: images/train\nval: images/val\n{extra}names:\n  0: manija\n')
    n = {p: len(os.listdir(os.path.join(destino, 'images', p))) for p in ('train', 'val')}
    print(f'Dataset {tarea}: {n["train"]} imágenes de entrenamiento, {n["val"]} de validación')
    return yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--tarea', choices=sorted(BASE), default='pose')
    ap.add_argument('--datos', default='/ros2_ws/datasets/manija_sim')
    ap.add_argument('--epocas', type=int, default=100,
                    help='pasadas completas por las imágenes de entrenamiento')
    ap.add_argument('--img', type=int, default=640, help='tamaño de imagen para la red [px]')
    ap.add_argument('--lote', type=int, default=16,
                    help='imágenes por paso (bajar si se acaba la memoria de la GPU)')
    args = ap.parse_args()

    from ultralytics import YOLO   # tarda en importar: después de revisar los argumentos

    yaml = armar_dataset(args.datos, args.tarea)
    os.makedirs(MODELOS, exist_ok=True)
    os.chdir(MODELOS)   # Ultralytics baja los pesos base a la carpeta actual
    modelo = YOLO(BASE[args.tarea])
    modelo.train(data=yaml, epochs=args.epocas, imgsz=args.img, batch=args.lote,
                 project=os.path.join(MODELOS, 'entrenamientos'), name=f'manija_{args.tarea}',
                 exist_ok=True,
                 # Paciencia: si en 30 épocas no mejora en validación, para antes.
                 patience=30,
                 # Aumento de datos de color más fuerte que el de serie (variar tono,
                 # saturación y brillo): ayuda a pasar de la puerta simulada a una real.
                 hsv_h=0.05, hsv_s=0.8, hsv_v=0.5)
    # Evaluar el mejor modelo en validación y mostrar las métricas.
    metricas = modelo.val(data=yaml, imgsz=args.img, split='val')
    print('\n=== Validación (puertas que el modelo no vio) ===')
    print(f'mAP50-95 caja: {metricas.box.map:.3f}   mAP50 caja: {metricas.box.map50:.3f}')
    if args.tarea == 'pose':
        print(f'mAP50-95 keypoints: {metricas.pose.map:.3f}   '
              f'mAP50 keypoints: {metricas.pose.map50:.3f}')
    if args.tarea == 'seg':
        print(f'mAP50-95 máscara: {metricas.seg.map:.3f}   '
              f'mAP50 máscara: {metricas.seg.map50:.3f}')
    print(f'Pesos: {MODELOS}/entrenamientos/manija_{args.tarea}/weights/best.pt')


if __name__ == '__main__':
    main()
