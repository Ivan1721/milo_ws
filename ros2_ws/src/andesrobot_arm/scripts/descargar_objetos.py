#!/usr/bin/env python3
"""Descarga los objetos del catálogo (config/objetos.yaml) para la mesa de la simulación.

Son escaneos 3D de objetos reales, con su textura (licencia CC-BY 4.0), de dos fuentes:
  fuel  Gazebo Fuel: ya vienen como modelo de Gazebo (los de GoogleResearch son Google Scanned
        Objects, la taza y el bowl del escenario original salen de ahí)
  ycb   el set YCB (ycb-benchmarks): vienen como malla .obj sola; aquí se les arma el model.sdf
Quedan en /ros2_ws/modelos/gazebo/<carpeta>/ (en tu PC ~/milo_ws/ros2_ws/modelos/gazebo/, no va a
git): arm_sim.launch.py suma esa carpeta a GAZEBO_MODEL_PATH, la lista de carpetas donde Gazebo
busca los "model://...".

A cada modelo, después de bajarlo:
  1. las rutas de sus mallas quedan como model://<carpeta>/... (catalogo_objetos.corregir_uris);
  2. su material se arregla para Gazebo clásico (si no, se ve negro): la textura junto a la malla
     y color ambiente blanco (Ka);
  3. si no trae masa (los de Google traen 1 kg por defecto), se le pone la del catálogo, con la
     inercia de una caja del tamaño de la malla;
  4. se anotan sus medidas en milo.json (la caja que la encierra): el launch las usa para dejar
     la base del objeto sobre la mesa.

Uso (dentro del contenedor; necesita internet; ~370 MB todos):
  python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py                 # todos
  python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py taza platano    # algunos
  python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py --lista      # ver el catálogo
"""
import argparse
import io
import json
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent))      # andesrobot_arm/ sin tener que compilar
from andesrobot_arm.catalogo_objetos import (  # noqa: E402
    carpeta, corregir_uris, leer_catalogo, medidas, vertices_obj, xml_inercial)

CATALOGO = AQUI.parent / 'config' / 'objetos.yaml'
FUEL = 'https://fuel.gazebosim.org/1.0'
YCB = 'http://ycb-benchmarks.s3.amazonaws.com/data'


def bajar(url):
    return urllib.request.urlopen(url, timeout=300).read()


def desde_fuel(objeto, d):
    """Gazebo Fuel: el zip ya es un modelo de Gazebo (model.config + model.sdf + mallas)."""
    nombre = urllib.parse.quote(objeto['nombre'])
    datos = bajar(f'{FUEL}/{urllib.parse.quote(objeto["dueno"])}/models/{nombre}/tip/{nombre}.zip')
    zipfile.ZipFile(io.BytesIO(datos)).extractall(d)


def desde_ycb(objeto, d):
    """YCB: la malla con textura de Google (google_16k); si no está (el tarro de Pringles), la de
    Berkeley (poisson). Se copian textured.obj/.mtl y su textura a meshes/ y se arma el modelo."""
    n = objeto['nombre']
    try:
        datos, sub = bajar(f'{YCB}/google/{n}_google_16k.tgz'), 'google_16k'
    except urllib.error.HTTPError:
        datos, sub = bajar(f'{YCB}/berkeley/{n}/{n}_berkeley_meshes.tgz'), 'poisson'
    with tempfile.TemporaryDirectory() as tmp:
        with tarfile.open(fileobj=io.BytesIO(datos)) as tar:
            # Solo lo de la carpeta que se usa (el tgz de Berkeley trae además nubes de 15 MB).
            miembros = [m for m in tar.getmembers() if f'/{sub}/' in m.name and m.isfile()]
            tar.extractall(tmp, members=miembros)
        origen = Path(tmp) / n / sub
        (d / 'meshes').mkdir(parents=True, exist_ok=True)
        for f in origen.iterdir():
            if f.suffix in ('.obj', '.mtl', '.png'):
                shutil.copy(f, d / 'meshes' / f.name)
    nombre = carpeta(objeto)
    malla = f'model://{nombre}/meshes/textured.obj'
    (d / 'model.sdf').write_text(
        f'<?xml version="1.0"?>\n<!-- {n} del set YCB (ycb-benchmarks.com, CC-BY 4.0), armado '
        'por scripts/descargar_objetos.py -->\n<sdf version="1.6">\n'
        f'  <model name="{nombre}">\n    <link name="link">\n'
        f'      <visual name="visual"><geometry><mesh><uri>{malla}</uri></mesh></geometry>'
        '</visual>\n'
        f'      <collision name="collision"><geometry><mesh><uri>{malla}</uri></mesh></geometry>'
        '</collision>\n    </link>\n  </model>\n</sdf>\n')


def arreglar_material(d):
    """Dos arreglos para que Gazebo clásico dibuje el objeto con su color (medido el 6-nov: sin
    ellos la taza y el bowl blancos se veían negros):
      1. el material de la malla (.mtl) pide su textura en la misma carpeta, pero algunos zip la
         traen en materials/textures/: se copia al lado;
      2. el material no trae color ambiente (Ka) y queda en negro: se le pone blanco (1 1 1)."""
    for mtl in d.rglob('*.mtl'):
        texto = mtl.read_text()
        for textura in re.findall(r'^map_Kd\s+(\S+)', texto, re.M):
            destino = mtl.parent / textura
            if not destino.exists():
                encontradas = list(d.rglob(Path(textura).name))
                if encontradas:
                    shutil.copyfile(encontradas[0], destino)
        # (el tarro de Pringles sí trae Ka, pero en negro "Ka 0 0 0": también se cambia)
        if re.search(r'^Ka\s+0(\.0*)?\s+0(\.0*)?\s+0(\.0*)?\s*$', texto, re.M):
            texto = re.sub(r'^Ka .*$', 'Ka 1.000000 1.000000 1.000000', texto, flags=re.M)
            mtl.write_text(texto)
        if not re.search(r'^Ka ', texto, re.M):
            mtl.write_text(texto.rstrip('\n') + '\nKa 1.000000 1.000000 1.000000\n')
        # 3. la malla .obj tiene que decir qué material usa ("usemtl"); la del tarro de Pringles
        #    (YCB Berkeley) no lo dice y se veía gris, sin textura.
        materiales = re.findall(r'^newmtl\s+(\S+)', texto, re.M)
        for obj in mtl.parent.glob('*.obj'):
            malla = obj.read_text()
            if materiales and 'usemtl' not in malla and 'mtllib' in malla:
                malla = re.sub(r'^(mtllib .*)$', r'\1\nusemtl ' + materiales[0], malla,
                               count=1, flags=re.M)
                obj.write_text(malla)


def archivo_sdf(d):
    """El .sdf del modelo: el que nombra model.config (casi siempre model.sdf)."""
    config = (d / 'model.config').read_text() if (d / 'model.config').exists() else ''
    m = re.search(r'<sdf[^>]*>\s*(\S+?)\s*</sdf>', config)
    return d / (m.group(1) if m else 'model.sdf')


def preparar(objeto, d):
    """Rutas, material, masa y medidas de un modelo recién bajado (ver el docstring)."""
    nombre = carpeta(objeto)
    if not (d / 'model.config').exists():
        (d / 'model.config').write_text(
            f'<?xml version="1.0"?><model><name>{nombre}</name><version>1.0</version>'
            f'<sdf version="1.6">model.sdf</sdf></model>\n')
    sdf_path = archivo_sdf(d)
    sdf = corregir_uris(sdf_path.read_text(), nombre)
    arreglar_material(d)
    # Medidas de la malla visual (.obj). Las .dae (la lata de Coca-Cola) no se miden: su origen
    # ya está en la base.
    medida = None
    uri = re.search(r'<visual.*?<uri>model://[^/]+/(.*?)</uri>', sdf, re.S)
    if uri and uri.group(1).endswith('.obj'):
        medida = medidas(vertices_obj((d / uri.group(1)).read_text()))
    if medida:
        centro = [(a + b) / 2 for a, b in zip(medida['min'], medida['max'])]
        if '<inertial>' not in sdf:
            bloque = xml_inercial(objeto['masa'], medida['tam'], centro)
            sdf = re.sub(r'(<link[^>]*>)', r'\1' + bloque, sdf, count=1)
        # Colisión = la caja que encierra la malla. Con la malla misma como colisión y la masa
        # real, el motor de física (ODE) no la mantiene parada: la taza se caía de lado sola
        # (medido el 6-nov). Se ve igual (la parte visual sigue siendo la malla); para agarrar
        # objetos de forma realista habría que volver a revisar esto.
        caja = (f'<pose>{centro[0]:.4f} {centro[1]:.4f} {centro[2]:.4f} 0 0 0</pose><geometry>'
                '<box><size>' + ' '.join(f'{v:.4f}' for v in medida['tam']) + '</size></box>'
                '</geometry>')
        sdf = re.sub(r'(<collision[^>]*>)\s*(?:<pose>[^<]*</pose>\s*)?<geometry>.*?</geometry>',
                     lambda m: m.group(1) + caja, sdf, flags=re.S)
    sdf_path.write_text(sdf)
    (d / 'milo.json').write_text(json.dumps(
        {'id': objeto['id'], 'clase': objeto['clase'], 'fuente': objeto['fuente'],
         'nombre': objeto['nombre'], 'medidas': medida}, indent=2))
    return medida


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('ids', nargs='*', help='ids o clases del catálogo (vacío = todos)')
    ap.add_argument('--carpeta', default='/ros2_ws/modelos/gazebo')
    ap.add_argument('--lista', action='store_true', help='solo mostrar el catálogo')
    ap.add_argument('--otra_vez', action='store_true', help='bajar aunque ya estén')
    a = ap.parse_args()
    catalogo = leer_catalogo(CATALOGO.read_text())
    if a.lista:
        for o in catalogo:
            print(f'{o["id"]:14s} {o["clase"]:18s} {o["fuente"]:5s} {o["nombre"]}')
        return
    pedidos = [o for o in catalogo
               if not a.ids or o['id'] in a.ids or o['clase'] in a.ids]
    base = Path(a.carpeta)
    base.mkdir(parents=True, exist_ok=True)
    fallas = []
    for objeto in pedidos:
        d = base / carpeta(objeto)
        try:
            if a.otra_vez and d.exists():
                shutil.rmtree(d)
            nuevo = not (d / 'model.config').exists() and not (d / 'model.sdf').exists()
            if nuevo:
                d.mkdir(parents=True, exist_ok=True)
                (desde_fuel if objeto['fuente'] == 'fuel' else desde_ycb)(objeto, d)
            medida = preparar(objeto, d)
            tam = ' x '.join(f'{v * 100:.0f}' for v in medida['tam']) + ' cm' if medida else '-'
            print(f'{objeto["id"]:14s} {"bajado" if nuevo else "ya estaba":10s} {tam}')
        except Exception as error:  # noqa: BLE001 (se informa y se sigue con los demás)
            fallas.append(objeto['id'])
            print(f'{objeto["id"]:14s} FALLÓ: {error}')
    if fallas:
        raise SystemExit(f'No se pudieron bajar: {", ".join(fallas)} (¿internet?)')


if __name__ == '__main__':
    main()
