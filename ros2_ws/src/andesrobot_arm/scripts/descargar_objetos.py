#!/usr/bin/env python3
"""Descarga los objetos reales (taza y bowl) para worlds/mesa_objetos.sdf.

Son modelos de Google Scanned Objects (escaneos 3D de objetos de verdad, con su textura; licencia
CC-BY 4.0) publicados en Gazebo Fuel. Quedan en /ros2_ws/modelos/gazebo/ (en tu PC
~/milo_ws/ros2_ws/modelos/gazebo/, no va a git): arm_sim.launch.py suma esa carpeta a
GAZEBO_MODEL_PATH, la lista de carpetas donde Gazebo busca los "model://...".

Se basa en crear_mundo.py del escenario sim_cup_bowl (mismos objetos, misma forma de descargar).

Uso (dentro del contenedor, una vez; necesita internet):
  python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py
"""
import argparse
import io
import re
import shutil
import urllib.request
import zipfile
from pathlib import Path

FUEL = 'https://fuel.gazebosim.org/1.0/GoogleResearch/models'
# Nombre en Fuel -> clase del modelo YOLO (modelos/objetos/best.pt) que debería reconocerlo.
OBJETOS = {
    'Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White': 'cup',
    'Threshold_Bead_Cereal_Bowl_White': 'bowl',
}


def poner_textura(d):
    """Dos arreglos para que Gazebo clásico dibuje el objeto con su color (medido el 6-nov: sin
    ellos la taza y el bowl blancos se veían negros):
      1. el material de la malla (meshes/model.mtl) pide "texture.png" en su misma carpeta, pero el
         zip la trae en materials/textures/: se copia al lado;
      2. el material no trae color ambiente (Ka) y queda en negro: se le pone blanco (1 1 1), como
         su color difuso (Kd)."""
    textura, destino = d / 'materials' / 'textures' / 'texture.png', d / 'meshes' / 'texture.png'
    if textura.exists() and not destino.exists():
        shutil.copyfile(textura, destino)
    mtl = d / 'meshes' / 'model.mtl'
    if mtl.exists():
        texto = mtl.read_text()
        if not re.search(r'^Ka ', texto, re.M):
            mtl.write_text(texto.rstrip('\n') + '\nKa 1.000000 1.000000 1.000000\n')


def descargar(nombre, carpeta):
    d = carpeta / nombre
    if (d / 'model.sdf').exists():
        poner_textura(d)
        print(f'{nombre}: ya estaba')
        return
    # Fuel guarda versiones: "tip" es la última; si no responde, la 1.
    for ver in ('tip', '1'):
        try:
            datos = urllib.request.urlopen(f'{FUEL}/{nombre}/{ver}/{nombre}.zip',
                                           timeout=120).read()
            zipfile.ZipFile(io.BytesIO(datos)).extractall(d)
            break
        except Exception as e:  # noqa: BLE001 (cualquier falla de red: probar la otra versión)
            print(f'{nombre} ({ver}): {e}')
    if not (d / 'model.sdf').exists():
        raise SystemExit(f'No se pudo descargar {nombre} (revisa internet)')
    # Las mallas vienen con rutas relativas ("meshes/model.obj"); Gazebo clásico las necesita
    # como "model://<nombre>/meshes/model.obj".
    sdf = (d / 'model.sdf').read_text()
    sdf = re.sub(r'<uri>(?!model://|https?://)\s*(.*?)\s*</uri>',
                 rf'<uri>model://{nombre}/\1</uri>', sdf)
    (d / 'model.sdf').write_text(sdf)
    # model.config: sin él Gazebo no reconoce la carpeta como modelo.
    if not (d / 'model.config').exists():
        (d / 'model.config').write_text(
            f'<?xml version="1.0"?><model><name>{nombre}</name><version>1.0</version>'
            f'<sdf version="1.6">model.sdf</sdf></model>\n')
    poner_textura(d)
    print(f'{nombre}: listo ({OBJETOS[nombre]})')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--carpeta', default='/ros2_ws/modelos/gazebo')
    a = ap.parse_args()
    carpeta = Path(a.carpeta)
    carpeta.mkdir(parents=True, exist_ok=True)
    for nombre in OBJETOS:
        descargar(nombre, carpeta)


if __name__ == '__main__':
    main()
