"""Geometría para tomar una bolsa por el asa (sin ROS: se prueba sola). P17.

Lo usa bolsa_node. Parte de lo que da el modelo de PUNTOS CLAVE del equipo de visión (YOLO11n-pose,
entregado el 24-nov-2026, una clase `bag_handle`) y de la imagen de profundidad de la cámara de
la muñeca. Por cada asa que ve, el modelo da una caja y 3 puntos en la imagen a color, cada uno
con su confianza:

      punta (1)            <- lo más alto del arco del asa: ahí se agarra
      .-----.
     /       \\
    |         |
   (0)       (2)           <- base izquierda y base derecha: donde el asa sale de la bolsa
  ===============  boca de la bolsa

Un punto de la imagen es solo una DIRECCIÓN (un rayo que sale de la cámara); la profundidad dice a
qué distancia está. Pasos:

  1. Nube de puntos (`nube_de_profundidad`): cada píxel de la imagen de profundidad se vuelve un
     punto 3D en el frame de la cámara de profundidad. Es lo inverso de sacar una foto: con la
     distancia y la matriz de la cámara se sabe de dónde vino la luz.
  2. Dónde cae cada punto 3D en la imagen a COLOR (`proyectar`): los puntos clave son píxeles de
     esa imagen, y en la Gemini Plus las cámaras de color y de profundidad están a unos cm.
  3. El punto 3D de cada punto clave (`punto_del_pixel`): los puntos de la nube que caen a menos
     de `radio` píxeles del punto clave, y de esos, la CAPA DE ADELANTE (los más cercanos a la
     cámara, hasta `capa` más atrás). El asa es delgada (4-8 mm: unos pocos píxeles a 40 cm) y
     está delante de la bolsa o del suelo: si se tomara el píxel exacto, un error de 2 píxeles
     del modelo daría la distancia a lo que hay DETRÁS del asa. La capa de adelante es el asa.
  4. Medir y validar el asa (`asa_de_puntos`), en base_footprint (Z arriba):
       - el VÉRTICE es el punto 3D de la punta: ahí el asa va horizontal y ahí se agarra;
       - el EJE del asa: la dirección horizontal de una base a la otra. En la punta el asa va
         paralela a esa línea. Los dedos cierran CRUZADOS al eje: a través del grosor del asa;
       - la boca de la bolsa: la altura de la base MÁS BAJA. Medido en Gazebo el 24-nov: el
         modelo a veces pone una base 7-8 cm más arriba en el arco, no donde el asa sale de la
         bolsa (su posición horizontal sí sirve para el eje); la más baja es la más cercana a
         la boca;
       - cuánto sobresale el asa de la boca, la separación de las bases, el desnivel entre ellas
         y cuánto se sale la punta de la línea de las bases (asa doblada o caída hacia un lado).
         Desde arriba solo se puede tomar un asa PARADA.
  5. Elegir (`elegir_asa`): de las asas válidas, la más cercana a Milo (una bolsa parada muestra
     dos, una por pared).
  6. La pose para agarrarla (`pose_de_agarre`): pinza hacia abajo sobre el vértice, a una altura
     en que el asa queda `metido` (12 mm) adentro de las almohadillas de los dedos.

Probado con escenas sintéticas (test/test_bolsa.py: una imagen de profundidad dibujada punto por
punto con un asa delante del suelo). Convenciones como agarre.py: metros, base_footprint salvo
donde se dice (+X adelante, +Y izquierda, +Z arriba).

Historia: el primer borrador (13-nov) esperaba un modelo de SEGMENTACIÓN de la bolsa entera y
buscaba el asa por geometría en la nube (lo que sobresale de la boca); el equipo entregó en cambio
un modelo de puntos clave del asa, que dice directamente dónde está. Ese borrador está en el
historial de git (commit 6faa1a3).
"""
import numpy as np

from andesrobot_arm.agarre import PUNTA_DEDO, pose_desde_arriba

# Orden de los puntos clave del modelo del equipo (capturar_asas.py: "base izquierda, punto más
# alto, base derecha"; flip_idx [2, 1, 0]: al reflejar la imagen se cambian las bases).
BASE_IZQ, PUNTA, BASE_DER = 0, 1, 2


def nube_de_profundidad(profundidad, k, paso=1, rango=(0.25, 2.5)):
    """
    Puntos 3D (N x 3) de una imagen de profundidad, en el frame ÓPTICO de esa cámara (Z adelante,
    X a la derecha, Y abajo).

    profundidad  imagen alto x ancho en metros (0 o NaN = sin dato; ver
                 manija_node.profundidad_en_metros)
    k            matriz 3x3 de la cámara de profundidad (de su camera_info)
    paso         se usa un píxel de cada `paso` en cada dirección (2 = un cuarto de los píxeles).
                 Con el asa de pocos píxeles de ancho conviene 1 (todos).
    rango        distancias válidas [m] (la Gemini Plus mide de 0.25 a 2.5 m)
    """
    d = np.asarray(profundidad, float)[::paso, ::paso]
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u, v = u * paso, v * paso
    ok = np.isfinite(d) & (d >= rango[0]) & (d <= rango[1])
    z = d[ok]
    x = (u[ok] - k[0, 2]) / k[0, 0] * z
    y = (v[ok] - k[1, 2]) / k[1, 1] * z
    return np.column_stack([x, y, z])


def proyectar(puntos_prof, k_color, r, t):
    """
    Dónde cae cada punto (del frame de la cámara de profundidad) en la imagen a COLOR.
    Devuelve un arreglo N x 3: (u, v) en píxeles y la distancia z a lo largo del eje de la cámara
    a color (z <= 0: el punto está detrás de ella; su u, v no valen).

    r, t  pasan del frame de color al de profundidad (p_prof = r · p_color + t, lo mismo que en
          manija.punto_en_rayo); aquí se usa la vuelta: p_color = rᵀ · (p_prof − t).
    """
    p = np.asarray(puntos_prof, float).reshape(-1, 3)
    pc = (p - np.asarray(t, float)) @ np.asarray(r, float)      # = rᵀ · (p − t), punto por punto
    z = pc[:, 2]
    z_seguro = np.where(z > 1e-3, z, 1.0)
    u = k_color[0, 0] * pc[:, 0] / z_seguro + k_color[0, 2]
    v = k_color[1, 1] * pc[:, 1] / z_seguro + k_color[1, 2]
    return np.column_stack([u, v, z])


def punto_del_pixel(nube, uvz, pixel, radio=6.0, capa=0.02, minimo=3):
    """
    El punto 3D (del frame de la nube) que se ve en un píxel de la imagen a color, tomando la capa
    de adelante alrededor del píxel (ver el paso 3 arriba). None si no hay al menos `minimo`
    puntos con profundidad a menos de `radio` píxeles.

    nube    puntos N x 3 (de nube_de_profundidad)
    uvz     su proyección en la imagen a color (de proyectar), N x 3
    pixel   (u, v) del punto clave
    radio   [px] alrededor del punto clave. 6 px a 40 cm de distancia son ~5 mm.
    capa    [m] grosor de la capa de adelante. 2 cm: más que el grosor del asa vista de lado,
            menos que la distancia del asa a la pared de la bolsa detrás de ella.
    Ejemplo: asa a 0.40 m delante de un suelo a 0.80 m -> devuelve un punto a ~0.40 m.
    """
    uvz = np.asarray(uvz, float)
    cerca = (uvz[:, 2] > 1e-3) & (np.hypot(uvz[:, 0] - pixel[0], uvz[:, 1] - pixel[1]) <= radio)
    if np.count_nonzero(cerca) < minimo:
        return None
    z = uvz[cerca, 2]
    adelante = z <= z.min() + capa
    return np.median(np.asarray(nube, float)[cerca][adelante], axis=0)


def asa_de_puntos(izq, punta, der, alto_min=0.022, separacion=(0.05, 0.40), desnivel_max=0.12,
                  fuera_max=0.04):
    """
    Mide y valida un asa a partir del punto 3D de sus tres puntos clave (base_footprint).

    alto_min      cuánto tiene que sobresalir la punta sobre la boca: las puntas de los dedos
                  bajan `metido` (12 mm) por debajo de la punta del asa (ver pose_de_agarre), así
                  que con menos tocarían el borde de la bolsa (12 mm + 10 de holgura)
    separacion    distancia horizontal aceptable entre las bases [m] (las bolsas del equipo:
                  12 a 22 cm; menos de 5 cm es un asa vista de canto o mal detectada)
    desnivel_max  diferencia de altura máxima entre las bases [m]. Holgada (12 cm) porque el
                  modelo a veces sube una base 8 cm por el arco; más que eso es una bolsa
                  tumbada o un punto que cayó en otra cosa
    fuera_max     cuánto se puede salir la punta, en horizontal, de la línea entre las bases [m]
                  (más: el asa está doblada o caída hacia un lado y desde arriba no se toma)

    Devuelve (asa, problema): asa es un dict con centro (= el vértice; se llama `centro` para usar
    agarre.estable), eje (unitario, horizontal), boca_z, alto (sobre la boca), largo (separación
    de las bases) y fuera; si el asa no sirve, (None, motivo en palabras).
    """
    izq, punta, der = (np.asarray(p, float) for p in (izq, punta, der))
    entre = der[:2] - izq[:2]
    largo = float(np.linalg.norm(entre))
    if largo < 1e-6:
        return None, 'las dos bases del asa caen en el mismo punto'
    eje_xy = entre / largo
    if eje_xy[0] < 0 or (abs(eje_xy[0]) < 1e-9 and eje_xy[1] < 0):
        eje_xy = -eje_xy                          # mismo criterio que bolsa/agarre: hacia +X
    cruz = np.array([-eje_xy[1], eje_xy[0]])
    boca_z = float(min(izq[2], der[2]))
    r = {'centro': punta, 'eje': np.array([eje_xy[0], eje_xy[1], 0.0]), 'boca_z': boca_z,
         'alto': float(punta[2] - boca_z), 'largo': largo,
         'fuera': float(abs((punta[:2] - izq[:2]) @ cruz))}

    problemas = []
    if r['alto'] < alto_min:
        problemas.append(f'el asa sobresale solo {r["alto"] * 1000:.0f} mm de la boca')
    if not separacion[0] <= largo <= separacion[1]:
        problemas.append(f'las bases del asa están a {largo * 100:.0f} cm')
    if abs(izq[2] - der[2]) > desnivel_max:
        problemas.append(f'las bases tienen {abs(izq[2] - der[2]) * 100:.0f} cm de desnivel '
                         '(¿bolsa tumbada?)')
    if r['fuera'] > fuera_max:
        problemas.append(f'la punta se sale {r["fuera"] * 100:.0f} cm de la línea de las bases '
                         '(¿asa doblada?)')
    if problemas:
        return None, '; '.join(problemas)
    return r, ''


def elegir_asa(asas):
    """De las asas válidas (dicts de asa_de_puntos), la más cercana a Milo (base_footprint en el
    origen): suele ser la de la pared que da a la cámara, la que se ve entera. Le agrega
    `n_asas` (cuántas válidas había). None si no hay ninguna."""
    if not asas:
        return None
    elegida = min(asas, key=lambda a: float(np.linalg.norm(a['centro'][:2])))
    return dict(elegida, n_asas=len(asas))


def pose_de_agarre(asa, metido=0.012):
    """
    (posición, giro 3x3) del TCP para agarrar el asa desde arriba: pinza hacia abajo sobre el
    vértice y dedos cruzados al eje del asa (agarre.pose_desde_arriba).

    Altura: las puntas de los dedos bajan PUNTA_DEDO (20.4 mm) por debajo del TCP y la almohadilla
    de cada dedo mide 21 mm. Se pone el asa `metido` (12 mm) por encima de las puntas, o sea a la
    mitad de la almohadilla: z_TCP = z_vértice + 20.4 mm − 12 mm.
    Las puntas quedan `metido` por debajo del vértice: por eso asa_de_puntos pide que el asa
    sobresalga al menos 22 mm de la boca (12 mm + 10 de holgura).
    Ejemplo: vértice a 0.50 m -> TCP a 0.5084 m, puntas a 0.488 m.
    """
    z = float(asa['centro'][2]) + PUNTA_DEDO - metido
    return pose_desde_arriba(asa['centro'], asa['eje'], z)


def promediar_asas(asas):
    """Combina varias mediciones del asa (de asa_de_puntos/elegir_asa): mediana de cada dato; el
    eje se promedia con el signo alineado al primero (el asa girada 180° es la misma)."""
    base = asas[0]['eje']
    ejes = np.array([a['eje'] if a['eje'] @ base >= 0 else -a['eje'] for a in asas])
    eje = np.median(ejes, axis=0)
    r = {k: float(np.median([a[k] for a in asas])) for k in ('boca_z', 'alto', 'largo', 'fuera')}
    r['centro'] = np.median([a['centro'] for a in asas], axis=0)
    r['eje'] = eje / np.linalg.norm(eje)
    r['n_asas'] = int(max(a.get('n_asas', 1) for a in asas))
    return r
