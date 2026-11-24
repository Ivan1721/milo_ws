"""Grupos de objetos para probar el detector con varios objetos a la vez, sin ROS: se prueba solo.

Lo usa scripts/evaluar_grupos.py. La idea viene de la prueba del equipo de visión
(prueba_completa.zip, 15-nov-2026): una mesa con TODOS los objetos en 5 grupos por tema (cocina,
cubiertos y comida, bebidas y aseo, objetos pequeños, varios), una foto por grupo y, por cada
clase, si el modelo la encontró. Su prueba usa una cámara suelta, sin Milo, y modelos de
Sketchfab que no vinieron en el zip; esta usa a Milo con la cámara de la pinza, la mesa de
arm_sim y los objetos de nuestro catálogo (config/objetos.yaml).

Diferencia con su prueba: además de "¿apareció la clase en la foto?", aquí se sabe DÓNDE está cada
objeto (lo pusimos nosotros), así que se proyecta su centro en la imagen y se mira qué caja de
YOLO lo contiene. Así se distingue "lo encontró", "lo vio como otra cosa" y "ni siquiera estaba
en la foto", y una detección de la clase correcta sobre otro objeto no cuenta como acierto.

Medidas de la mesa (mesa_objetos.sdf, en base_footprint): tablero a 0.72 m, de x = 0.70 a 1.30 y
de y = −0.40 a 0.40 (ver catalogo_objetos.py).
"""
import numpy as np

from andesrobot_arm.catalogo_objetos import ALTO_MESA

# Los 5 grupos de la prueba del equipo (cocina, cubiertos y comida, bebidas y aseo, objetos
# pequeños, varios), con nombres de una palabra para pedirlos en la terminal, armados con nuestro
# catálogo. Entre paréntesis, lo que
# tiene el equipo en ese lugar cuando no es el mismo modelo.
GRUPOS = {
    # Los cuatro primeros son los mismos modelos de Google que usa el equipo; su jarra
    # (Pitcher_Base) no existe en Fuel: va la nuestra.
    'cocina': ['taza_blanca', 'bowl_turquesa', 'plato', 'bandeja', 'jarro'],
    # (cubiertos_sketchfab trae cuchara, tenedor y cuchillo en un modelo; acá van los tres de YCB
    # sueltos. fruit_sketchfab y food_sketchfab -> plátano y sopa de YCB.)
    'cubiertos': ['tenedor', 'cuchara', 'cuchillo', 'platano', 'sopa', 'cereal'],
    # (bottle y cleaning de Sketchfab -> mostaza y cloro de YCB; drink no tiene modelo.)
    'bebidas': ['redbull', 'pringles_equipo', 'botella', 'cloro'],
    # (key y colgate_box no tienen modelo; rubik y pen de Sketchfab -> rubik y plumón de YCB.)
    'pequenos': ['tarjeta_equipo', 'rubik', 'plumon'],
    # (libro_google y clothing_sketchfab -> libro y sombrero de Fuel. Su bag era la bolsa negra de
    # la entrega del 9-nov (bolsa_equipo); desde el 15-nov va la de tela de entorno_objetos.zip,
    # la que corresponde al modelo de 9 clases.)
    'varios': ['consola', 'martillo', 'libro', 'sombrero', 'bolsa_tela'],
    # El mundo de prueba del modelo "para simulación" del equipo (entorno_objetos.zip, 15-nov):
    # un objeto por cada una de sus 9 clases, los mismos modelos con que parece entrenado.
    'equipo': ['bowl_turquesa', 'redbull', 'tarjeta_equipo', 'cubiertos_equipo', 'cuchara_equipo',
               'pringles_equipo', 'rubik_equipo', 'botella_equipo', 'bolsa_tela'],
}

# Nombres de clase del modelo -> nombre de clase del catálogo. El modelo de 27 clases usa los
# mismos nombres que el catálogo; el de 9 clases del 15-nov acorta dos.
ALIAS = {'redbull': 'redbull_can', 'rubik': 'rubik_cube'}

# Filas de la mesa: la de adelante con el borde delantero de los objetos a 2 cm del borde de la
# mesa (x = 0.72) y la de atrás con el borde trasero a 2 cm del fondo (x = 1.28).
BORDE_DELANTERO = 0.72
BORDE_TRASERO = 1.28
# Los objetos más de ESBELTO veces más altos que su lado más corto (lata, botella, caja de cereal,
# libro parado) van siempre atrás: el 15-nov la lata de Red Bull, adelante, se cayó entre dos
# vistas (quizás la tocó la pinza, que pasa cerca del borde delantero al cambiar de vista).
ESBELTO = 2.0
MEDIO_ANCHO = 0.38          # de y = −0.38 a 0.38: 2 cm a cada lado de la mesa
HUECO = 0.03                # mínimo entre dos objetos
ANCHO_IDEAL = 0.64          # se reparten de y = −0.32 a 0.32 si caben


def normalizar(clase):
    """Nombre de una clase del modelo como la llama el catálogo."""
    return ALIAS.get(clase, clase)


def _fila(indices, tam, x_borde, signo):
    """Centros (x, y) de las cajas de una fila, o None si no cabe. Se reparten a lo ancho de
    ANCHO_IDEAL (que en las fotos no queden pegados: YOLO separa mejor las cajas) y, si no
    alcanza, se aprietan hasta HUECO entre ellos, sin pasar el ancho de la mesa."""
    anchos = tam[indices, 1]
    if len(indices) == 1:
        return {indices[0]: (x_borde + signo * tam[indices[0], 0] / 2, 0.0)}
    hueco = max(HUECO, (ANCHO_IDEAL - anchos.sum()) / (len(indices) - 1))
    total = anchos.sum() + hueco * (len(indices) - 1)
    if total > 2 * MEDIO_ANCHO:
        hueco = (2 * MEDIO_ANCHO - anchos.sum()) / (len(indices) - 1)
        if hueco < HUECO:
            return None
        total = 2 * MEDIO_ANCHO
    y, centros = -total / 2, {}
    for i, ancho in zip(indices, anchos):
        centros[i] = (x_borde + signo * tam[i, 0] / 2, y + ancho / 2)
        y += ancho + hueco
    return centros


def acomodar(cajas):
    """
    Dónde poner cada objeto de un grupo para que no se toquen.

    cajas   lista de {'min': [x, y, z], 'max': [...]} (la caja de la malla en el frame del
            modelo, como en milo.json)
    Devuelve una lista de (x, y, giro) para el ORIGEN de cada modelo (en el orden de entrada):
    giro es 0 o π/2 [rad] en Z, y (x, y) deja el centro de su caja en su lugar. Hay dos filas: la
    de adelante pegada al borde delantero de la mesa y la de atrás al fondo. Se prueban todas las
    formas de repartir los objetos entre las dos filas, cada uno derecho o girado 90° (con 6
    objetos, 64 x 64), y entre las que caben sin que dos cajas se pisen se queda con la de menor
    costo (los esbeltos, ver ESBELTO, nunca adelante):
      1 por cada objeto de diferencia entre las filas (que queden parejas),
      + 10 x la "cara" que los de adelante le muestran a la cámara (alto x ancho en y, en m²):
        los chicos adelante no tapan a los de atrás desde la cámara, que mira desde arriba y
        adelante. Con solo el alto, el 15-nov la bolsa (35 x 26 cm) y el sombrero (17 x 40 cm)
        quedaron adelante y taparon todo lo demás,
      + 0.01 por objeto girado (derecho, si da lo mismo).
    ValueError si ninguna forma cabe.
    """
    lo = np.array([c['min'] for c in cajas], float)
    hi = np.array([c['max'] for c in cajas], float)
    tam0, centro = hi - lo, (hi + lo) / 2
    n = len(cajas)
    esbelto = tam0[:, 2] > ESBELTO * tam0[:, :2].min(axis=1)
    mejor, costo_mejor = None, np.inf
    for giros in range(2 ** n):
        girado = np.array([giros >> i & 1 for i in range(n)], bool)
        tam = tam0.copy()
        tam[girado, 0], tam[girado, 1] = tam0[girado, 1], tam0[girado, 0]
        for mascara in range(2 ** n):
            adelante = [i for i in range(n) if not mascara >> i & 1]
            atras = [i for i in range(n) if mascara >> i & 1]
            if any(esbelto[i] for i in adelante):
                continue
            costo = (abs(len(adelante) - len(atras))
                     + 10 * (tam[adelante, 2] * tam[adelante, 1]).sum() + 0.01 * girado.sum())
            if costo >= costo_mejor - 1e-9:
                continue
            centros = {}
            for fila, x_borde, signo in ((adelante, BORDE_DELANTERO, 1),
                                         (atras, BORDE_TRASERO, -1)):
                c = _fila(fila, tam, x_borde, signo) if fila else {}
                if c is None:
                    break
                centros.update(c)
            else:
                if not _se_pisan(centros, tam):
                    mejor, costo_mejor = (centros, girado), costo
    if mejor is None:
        raise ValueError(f'los {n} objetos no caben en la mesa sin pisarse')
    centros, girado = mejor
    salida = []
    for i in range(n):
        # el centro de la caja, girado con el modelo, tiene que caer en centros[i]
        cx, cy = (-centro[i, 1], centro[i, 0]) if girado[i] else (centro[i, 0], centro[i, 1])
        salida.append((float(centros[i][0] - cx), float(centros[i][1] - cy),
                       float(np.pi / 2 if girado[i] else 0.0)))
    return salida


def _se_pisan(centros, tam):
    """True si dos cajas (vistas desde arriba, con HUECO de margen) se pisan."""
    ids = list(centros)
    for a in range(len(ids)):
        for b in range(a + 1, len(ids)):
            i, j = ids[a], ids[b]
            dx = abs(centros[i][0] - centros[j][0]) - (tam[i, 0] + tam[j, 0]) / 2
            dy = abs(centros[i][1] - centros[j][1]) - (tam[i, 1] + tam[j, 1]) / 2
            if dx < HUECO - 1e-6 and dy < HUECO - 1e-6:     # 1e-6: el redondeo de _fila
                return True
    return False


def centro_en_mesa(lugar, caja):
    """Centro 3D (base_footprint) de la caja de un objeto puesto en lugar = (x, y, giro) (lo que
    da acomodar) con su base 3 mm sobre el tablero (como lo pone altura_de_spawn)."""
    lo, hi = np.array(caja['min'], float), np.array(caja['max'], float)
    c = (hi + lo) / 2
    cg, sg = np.cos(lugar[2]), np.sin(lugar[2])
    return np.array([lugar[0] + cg * c[0] - sg * c[1], lugar[1] + sg * c[0] + cg * c[1],
                     ALTO_MESA + 0.003 + (hi[2] - lo[2]) / 2])


def matriz_de_tf(traslacion, cuaternion):
    """Matriz 4x4 de una transformación de TF: traslación (x, y, z) y cuaternión (x, y, z, w)."""
    x, y, z, w = cuaternion
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, traslacion
    return m


def proyectar(puntos, camara_desde_mundo, k, ancho, alto):
    """
    Píxeles (u, v) de puntos 3D y si caen dentro de la imagen.

    puntos              N x 3 en el frame "mundo" (odom en la simulación = el mundo de Gazebo)
    camara_desde_mundo  4x4 que pasa del mundo al frame ÓPTICO de la cámara (Z adelante, X a la
                        derecha, Y hacia abajo), o sea la TF mundo -> cámara invertida
    k                   matriz 3x3 de la cámara (camera_info.k)
    Devuelve (uv N x 2, visible N): visible = delante de la cámara y dentro de la imagen.
    """
    p = np.c_[np.asarray(puntos, float), np.ones(len(puntos))] @ camara_desde_mundo.T
    z = p[:, 2]
    delante = z > 0.05
    uv = np.full((len(p), 2), np.nan)
    uv[delante] = (p[delante, :3] @ np.asarray(k, float).T)[:, :2] / z[delante, None]
    dentro = (delante & (uv[:, 0] >= 0) & (uv[:, 0] < ancho)
              & (uv[:, 1] >= 0) & (uv[:, 1] < alto))
    return uv, dentro


def esquinas_en_mesa(lugar, caja):
    """Las 8 esquinas (8 x 3, base_footprint) de la caja de un objeto puesto en lugar = (x, y,
    giro), con su base 3 mm sobre el tablero."""
    lo, hi = np.array(caja['min'], float), np.array(caja['max'], float)
    cg, sg = np.cos(lugar[2]), np.sin(lugar[2])
    salida = []
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (0.0, hi[2] - lo[2]):
                salida.append([lugar[0] + cg * x - sg * y, lugar[1] + sg * x + cg * y,
                               ALTO_MESA + 0.003 + z])
    return np.array(salida)


def rectangulo(uv_esquinas, ancho, alto):
    """Rectángulo (x1, y1, x2, y2) que ocupa en la imagen un objeto, a partir de sus 8 esquinas
    proyectadas (las que quedan detrás de la cámara vienen como NaN), recortado a la imagen."""
    p = uv_esquinas[~np.isnan(uv_esquinas[:, 0])]
    if not len(p):
        return np.array([0.0, 0.0, 0.0, 0.0])
    return np.array([np.clip(p[:, 0].min(), 0, ancho), np.clip(p[:, 1].min(), 0, alto),
                     np.clip(p[:, 0].max(), 0, ancho), np.clip(p[:, 1].max(), 0, alto)])


def iou(a, b):
    """Intersección sobre unión de dos rectángulos (x1, y1, x2, y2): 1 = iguales, 0 = no se
    tocan."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def asignar(clases, rects, visibles, cajas, nombres, confianzas, iou_min=0.25):
    """
    Qué vio YOLO en el lugar de cada objeto.

    clases      clase del catálogo de cada objeto
    rects       rectángulo de cada objeto en la imagen (rectangulo()), visibles: si su centro
                cayó en la imagen (proyectar())
    cajas       M x 4 cajas de YOLO (x1, y1, x2, y2 en píxeles), nombres y confianzas de cada una
    Cada caja de YOLO se le asigna al objeto visible con el que más se superpone (IoU), si es al
    menos iou_min; si no, a ninguno. Así una caja sobre media mesa (el modelo a veces ve la mesa
    entera como `clothing` o `card`) no se le cuenta a nadie, y un objeto grande con otro delante
    (la bolsa con el Rubik adelante) recibe su caja aunque el del frente quede dentro de ella.
    (Primera versión, 15-nov: por el centro del objeto dentro de la caja; con la bolsa fallaba.)
    Devuelve, por objeto, un diccionario:
      visible   el centro del objeto cayó en la imagen
      correcta  la mayor confianza de las cajas de SU clase que se le asignaron (o None)
      otra      (clase, confianza) de la caja de otra clase que más se le superpone, si no tuvo
                ninguna de su clase (con qué lo confundió), o None
    """
    cajas = np.asarray(cajas, float).reshape(-1, 4)
    nombres = [normalizar(n) for n in nombres]
    visibles = np.asarray(visibles, bool)
    asignadas = {i: [] for i in range(len(clases))}       # objeto -> [(iou, caja)]
    for j, caja in enumerate(cajas):
        superpone = [iou(caja, r) if v else 0.0 for r, v in zip(rects, visibles)]
        if superpone and max(superpone) >= iou_min:
            i = int(np.argmax(superpone))
            asignadas[i].append((superpone[i], j))
    salida = []
    for i, (clase, vis) in enumerate(zip(clases, visibles)):
        r = {'visible': bool(vis), 'correcta': None, 'otra': None}
        buenas = [confianzas[j] for _, j in asignadas[i] if nombres[j] == clase]
        if buenas:
            r['correcta'] = float(max(buenas))
        elif asignadas[i]:
            _, j = max(asignadas[i])
            r['otra'] = (nombres[j], float(confianzas[j]))
        salida.append(r)
    return salida


def resumir(por_vista):
    """
    Junta lo de todas las vistas de un objeto.

    por_vista   lista de resultados de asignar() de un mismo objeto (uno por vista)
    Devuelve {'vistas': en cuántas cayó en la imagen, 'aciertos': en cuántas lo encontró,
    'mejor': la mayor confianza correcta (o None), 'confusiones': {clase: veces}}.
    """
    confusiones = {}
    for r in por_vista:
        if r['otra']:
            confusiones[r['otra'][0]] = confusiones.get(r['otra'][0], 0) + 1
    correctas = [r['correcta'] for r in por_vista if r['correcta'] is not None]
    return {'vistas': sum(r['visible'] for r in por_vista), 'aciertos': len(correctas),
            'mejor': max(correctas) if correctas else None, 'confusiones': confusiones}


def caja_de_sdf(texto):
    """Caja {'min', 'max'} de un modelo hecho de UNA caja (la tarjeta y la bolsa del equipo no
    traen malla, así que milo.json no tiene medidas): <box><size> y la <pose> de su colisión."""
    import re
    m = re.search(r'<collision.*?</collision>', texto, re.S)
    bloque = m.group(0) if m else texto
    tam = np.array(re.search(r'<size>\s*([^<]+)</size>', bloque).group(1).split(), float)
    p = re.search(r'<pose>\s*([^<]+)</pose>', bloque)
    c = np.array(p.group(1).split()[:3], float) if p else np.zeros(3)
    return {'min': (c - tam / 2).round(4).tolist(), 'max': (c + tam / 2).round(4).tolist()}
