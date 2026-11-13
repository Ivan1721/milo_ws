"""Geometría para tomar una bolsa por el asa (sin ROS: se prueba sola). BORRADOR, P17.

Lo usa bolsa_node. Parte de la MÁSCARA de la bolsa que da un modelo de segmentación de instancias
(una sola clase, `bag`) y de la imagen de profundidad de la cámara de la muñeca. No hace falta que
el modelo marque el asa aparte: el asa se encuentra por geometría, porque es lo que sobresale por
encima de la boca de la bolsa (el reglamento habla de "asas rígidas verticales").

  1. Nube de puntos (`nube_de_profundidad`): cada píxel de la imagen de profundidad (uno de cada
     `paso`) se vuelve un punto 3D en el frame de la cámara de profundidad. Es lo inverso de sacar
     una foto: con la distancia y la matriz de la cámara se sabe de dónde vino la luz.
  2. Qué puntos son de la bolsa (`en_la_mascara`): cada punto se proyecta en la imagen a COLOR
     (la máscara es de esa imagen; en la Gemini Plus las dos cámaras están a unos cm) y se mira si
     cae dentro de la máscara.
  3. La boca de la bolsa (`altura_de_la_boca`): los puntos de la bolsa se reparten en rebanadas
     horizontales de 1 cm. Las paredes llenan muchas rebanadas seguidas con muchos puntos; el asa,
     que es delgada, deja pocos. La boca es el techo de esas rebanadas "llenas" seguidas.
  4. El asa (`buscar_asa`): los puntos (de la bolsa, o de toda la imagen si la máscara no incluyó
     el asa) que están sobre la boca y dentro de la huella de la bolsa vista desde arriba. Si hay
     dos asas separadas (bolsa parada: quedan a ~16 cm, el fondo de la bolsa) se separan y se
     elige la más cercana a Milo. De esa asa:
       - el VÉRTICE: su punto más alto, donde el asa va horizontal (ahí se agarra);
       - el EJE del asa: hacia dónde va el asa en ese tramo horizontal (PCA, como el eje largo de
         agarre.analizar_objeto). Los dedos cierran CRUZADOS a ese eje: a través del
         grosor del asa;
       - cuánto sobresale de la boca, su largo y su grosor, para validar.
  5. La pose para agarrarla (`pose_de_agarre`): pinza hacia abajo sobre el vértice, a una altura
     en que el asa queda `metido` (12 mm) adentro de las almohadillas de los dedos.

Nada de esto se probó todavía con el modelo ni en Gazebo (no hay bolsa con asa en la simulación):
solo con las nubes sintéticas de test/test_bolsa.py. Convenciones como agarre.py: metros, todo en
base_footprint salvo donde se dice (+X adelante, +Y izquierda, +Z arriba).
"""
import numpy as np

from andesrobot_arm.agarre import PUNTA_DEDO, extension, pose_desde_arriba


def nube_de_profundidad(profundidad, k, paso=2, rango=(0.25, 2.5)):
    """
    Puntos 3D (N x 3) de una imagen de profundidad, en el frame ÓPTICO de esa cámara (Z adelante,
    X a la derecha, Y abajo).

    profundidad  imagen alto x ancho en metros (0 o NaN = sin dato; ver
                 manija_node.profundidad_en_metros)
    k            matriz 3x3 de la cámara de profundidad (de su camera_info)
    paso         se usa un píxel de cada `paso` en cada dirección (2 = un cuarto de los píxeles)
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


def en_la_mascara(puntos_prof, mascara, k_color, r, t):
    """
    ¿Qué puntos (del frame de la cámara de profundidad) caen dentro de la máscara de la imagen a
    color? Devuelve un arreglo de True/False, uno por punto.

    r, t  pasan del frame de color al de profundidad (p_prof = r · p_color + t, lo mismo que en
          manija.punto_en_rayo); aquí se usa la vuelta: p_color = rᵀ · (p_prof − t).
    """
    p = np.asarray(puntos_prof, float).reshape(-1, 3)
    pc = (p - np.asarray(t, float)) @ np.asarray(r, float)      # = rᵀ · (p − t), punto por punto
    m = np.asarray(mascara, bool)
    dentro = np.zeros(len(p), bool)
    delante = pc[:, 2] > 1e-3
    u = np.round(k_color[0, 0] * pc[delante, 0] / pc[delante, 2] + k_color[0, 2]).astype(int)
    v = np.round(k_color[1, 1] * pc[delante, 1] / pc[delante, 2] + k_color[1, 2]).astype(int)
    en_imagen = (u >= 0) & (u < m.shape[1]) & (v >= 0) & (v < m.shape[0])
    idx = np.flatnonzero(delante)[en_imagen]
    dentro[idx] = m[v[en_imagen], u[en_imagen]]
    return dentro


def altura_de_la_boca(z, rebanada=0.01, fraccion=0.5):
    """
    Altura [m] de la boca de la bolsa: el techo de las rebanadas "llenas" seguidas, contando desde
    abajo. Una rebanada está llena si tiene al menos `fraccion` de la mediana de puntos de las
    rebanadas con algún punto (las paredes tienen muchos; el asa, delgada, pocos).
    Se cuenta desde abajo y se corta en la primera rebanada que no está llena: así el tramo
    horizontal del asa (que en su rebanada puede juntar tantos puntos como una pared) no cuenta,
    porque entre él y la boca hay rebanadas casi vacías (las patas del asa).
    Ejemplo: bolsa de 41 cm de alto en el suelo -> ~0.41.
    """
    z = np.asarray(z, float)
    bordes = np.arange(z.min(), z.max() + 2 * rebanada, rebanada)
    cuenta, bordes = np.histogram(z, bordes)
    referencia = np.median(cuenta[cuenta > 0])
    llena = cuenta >= fraccion * referencia
    if not llena.any():
        return float(z.max())
    i = int(np.argmax(llena))           # la primera llena desde abajo
    while i + 1 < len(llena) and llena[i + 1]:
        i += 1
    return float(bordes[i + 1])


def ejes_horizontales(xy):
    """(principal, cruzado): direcciones horizontales unitarias en que unos puntos están más y
    menos repartidos (PCA). El principal apunta hacia +X (o +Y si es perpendicular a X)."""
    xy = np.asarray(xy, float)
    valores, vectores = np.linalg.eigh(np.cov((xy - xy.mean(axis=0)).T))
    eje = vectores[:, int(np.argmax(valores))]
    if eje[0] < 0 or (abs(eje[0]) < 1e-9 and eje[1] < 0):
        eje = -eje
    return eje, np.array([-eje[1], eje[0]])


def separar_asas(puntos, separacion=0.03):
    """
    Si los puntos del asa son en verdad DOS asas (una en cada pared), los separa. Las dos asas van
    paralelas, así que hay una dirección horizontal (cruzada a ellas) en que los puntos dejan un
    hueco: se ordenan por esa coordenada y se corta en el hueco más grande, si mide más de
    `separacion`. Se prueban las dos direcciones del PCA, porque con dos asas juntas la dirección
    "más repartida" puede ser justamente la que las separa (12 cm de asa contra 16 cm entre asas).
    Devuelve una lista con uno o dos grupos de puntos.
    """
    p = np.asarray(puntos, float)
    if len(p) < 2:
        return [p]
    mejor = None                                   # (hueco, orden, corte)
    for direccion in ejes_horizontales(p[:, :2]):
        s = p[:, :2] @ direccion
        orden = np.argsort(s)
        huecos = np.diff(s[orden])
        i = int(np.argmax(huecos))
        if mejor is None or huecos[i] > mejor[0]:
            mejor = (huecos[i], orden, i + 1)
    hueco, orden, corte = mejor
    if hueco < separacion:
        return [p]
    return [p[orden[:corte]], p[orden[corte:]]]


def buscar_asa(bolsa, todos=None, margen_boca=0.01, alto_max=0.25, huella=0.02, minimo=15,
               separacion=0.03, alto_min=0.022, largo=(0.04, 0.40), grosor_max=0.045):
    """
    Encuentra el asa de una bolsa en una nube de puntos (base_footprint, Z arriba).

    bolsa        puntos dentro de la máscara de la bolsa (N x 3)
    todos        puntos de toda la imagen, para buscar el asa si la máscara no la incluyó
                 (None = solo los de la máscara)
    margen_boca  el asa empieza este tanto sobre la boca [m] (el borde del papel no cuenta)
    alto_max     hasta dónde buscar sobre la boca [m]
    huella       margen alrededor de la bolsa vista desde arriba [m]
    alto_min     cuánto tiene que sobresalir el asa de la boca: si menos, las puntas de los dedos
                 tocarían el borde de la bolsa (ver pose_de_agarre)
    largo        largo [m] aceptable del asa a lo largo de su eje
    grosor_max   más grueso no cabe entre los dedos (la pinza abre 49 mm)

    Devuelve (asa, problema): asa es un dict con boca_z, centro (= el vértice, x y z; se llama
    `centro` para usar agarre.estable), eje (unitario, horizontal), alto (sobre la boca), largo,
    grosor, n (puntos del asa) y n_asas (1 o 2 vistas); si no hay asa, (None, motivo en palabras).
    """
    b = np.asarray(bolsa, float).reshape(-1, 3)
    if len(b) < 3 * minimo:
        return None, f'pocos puntos de la bolsa con profundidad ({len(b)})'
    boca = altura_de_la_boca(b[:, 2])
    # Huella de la bolsa vista desde arriba: el rectángulo (girado con la bolsa) que ocupan sus
    # puntos de debajo de la boca, más `huella` de margen.
    cuerpo = b[b[:, 2] <= boca]
    principal, cruzado = ejes_horizontales(cuerpo[:, :2])
    centro_xy = cuerpo[:, :2].mean(axis=0)
    lim_p = extension((cuerpo[:, :2] - centro_xy) @ principal, paso=0.005)
    lim_c = extension((cuerpo[:, :2] - centro_xy) @ cruzado, paso=0.005)

    c = b if todos is None else np.asarray(todos, float).reshape(-1, 3)
    rel = c[:, :2] - centro_xy
    a, s = rel @ principal, rel @ cruzado
    en_huella = ((a >= lim_p[0] - huella) & (a <= lim_p[1] + huella)
                 & (s >= lim_c[0] - huella) & (s <= lim_c[1] + huella))
    sobre = (c[:, 2] > boca + margen_boca) & (c[:, 2] < boca + alto_max)
    asa = c[en_huella & sobre]
    if len(asa) < minimo:
        return None, (f'nada sobresale de la boca (a {boca:.2f} m): ¿asas dobladas hacia adentro, '
                      'bolsa tumbada o el asa fuera de la imagen?')

    grupos = [g for g in separar_asas(asa, separacion) if len(g) >= minimo]
    if not grupos:
        return None, 'las asas tienen muy pocos puntos'
    # La más cercana a Milo (base_footprint en el origen): suele ser la que la cámara ve entera.
    elegida = min(grupos, key=lambda g: np.linalg.norm(g[:, :2].mean(axis=0)))
    eje_xy, cruz = ejes_horizontales(elegida[:, :2])
    tope = elegida[elegida[:, 2] >= elegida[:, 2].max() - 0.01]   # el tramo de arriba (1 cm)
    vertice = np.array([*tope[:, :2].mean(axis=0), np.median(tope[:, 2])])
    l_min, l_max = extension((elegida[:, :2] - vertice[:2]) @ eje_xy)
    g_min, g_max = extension((tope[:, :2] - vertice[:2]) @ cruz, minimo=1)
    r = {'boca_z': boca, 'centro': vertice, 'eje': np.array([eje_xy[0], eje_xy[1], 0.0]),
         'alto': float(vertice[2] - boca), 'largo': float(l_max - l_min),
         'grosor': float(g_max - g_min), 'n': len(elegida), 'n_asas': len(grupos)}

    problemas = []
    if r['alto'] < alto_min:
        problemas.append(f'el asa sobresale solo {r["alto"] * 1000:.0f} mm de la boca')
    if not largo[0] <= r['largo'] <= largo[1]:
        problemas.append(f'el asa mide {r["largo"] * 100:.0f} cm de largo')
    if r['grosor'] > grosor_max:
        problemas.append(f'el asa mide {r["grosor"] * 1000:.0f} mm de grosor (no cabe)')
    if problemas:
        return None, '; '.join(problemas)
    return r, ''


def pose_de_agarre(asa, metido=0.012):
    """
    (posición, giro 3x3) del TCP para agarrar el asa desde arriba: pinza hacia abajo sobre el
    vértice y dedos cruzados al eje del asa (agarre.pose_desde_arriba).

    Altura: las puntas de los dedos bajan PUNTA_DEDO (20.4 mm) por debajo del TCP y la almohadilla
    de cada dedo mide 21 mm. Se pone el asa `metido` (12 mm) por encima de las puntas, o sea a la
    mitad de la almohadilla: z_TCP = z_vértice + 20.4 mm − 12 mm.
    Las puntas quedan `metido` por debajo del vértice: por eso buscar_asa pide que el asa
    sobresalga al menos 22 mm de la boca (12 mm + 10 de holgura).
    Ejemplo: vértice a 0.50 m -> TCP a 0.5084 m, puntas a 0.488 m.
    """
    z = float(asa['centro'][2]) + PUNTA_DEDO - metido
    return pose_desde_arriba(asa['centro'], asa['eje'], z)


def promediar_asas(asas):
    """Combina varias mediciones del asa (de buscar_asa): mediana de cada dato; el eje se promedia
    con el signo alineado al primero (el asa girada 180° es la misma)."""
    base = asas[0]['eje']
    ejes = np.array([a['eje'] if a['eje'] @ base >= 0 else -a['eje'] for a in asas])
    eje = np.median(ejes, axis=0)
    r = {k: float(np.median([a[k] for a in asas]))
         for k in ('boca_z', 'alto', 'largo', 'grosor')}
    r['centro'] = np.median([a['centro'] for a in asas], axis=0)
    r['eje'] = eje / np.linalg.norm(eje)
    r['n'] = int(sum(a['n'] for a in asas))
    r['n_asas'] = int(max(a['n_asas'] for a in asas))
    return r
