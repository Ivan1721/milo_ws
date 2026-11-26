"""Cuentas de la detección en vivo (deteccion_node), sin ROS ni modelo, para poder probarlas.

En palabras simples: YOLO dibuja una caja alrededor de cada objeto en la imagen a COLOR. Para saber
a qué distancia está, se miran los píxeles de la imagen de PROFUNDIDAD que caen dentro de esa caja
(solo el centro, para no tomar el piso de los bordes) y se toma la mediana: así un píxel sin dato o
un borde raro no cambian el resultado. Con esa distancia y la calibración de la cámara (K) se
calcula el punto 3D del objeto, en el frame óptico de la cámara (+Z hacia adelante, +X a la
derecha, +Y hacia abajo de la imagen).

  - escalar_caja: la caja viene en píxeles de la imagen a color (640x480); la profundidad de la
    Gemini sin alinear es de 640x400. Se pasa la caja a la otra resolución (aproximado: con
    alineada:=true las dos imágenes son el mismo punto de vista y la cuenta es exacta).
  - distancia_en_caja: mediana de la profundidad dentro del centro de la caja [m].
  - punto_3d: píxel + distancia -> (x, y, z) [m] con la matriz K de camera_info.
  - leer_clases: "lata, taza" -> ['lata', 'taza'] (las clases que se le piden a YOLOE).
"""
import numpy as np


def escalar_caja(caja, de, a):
    """Caja (x1, y1, x2, y2) en una imagen de tamaño `de` (ancho, alto) -> la misma caja en una
    imagen de tamaño `a`."""
    sx, sy = a[0] / de[0], a[1] / de[1]
    x1, y1, x2, y2 = caja
    return (x1 * sx, y1 * sy, x2 * sx, y2 * sy)


def distancia_en_caja(prof_mm, caja, centro=0.5):
    """Mediana de la profundidad [m] en el `centro` (fracción del ancho y alto) de la caja.

    prof_mm: imagen de profundidad en mm (0 = sin dato). None si no hay ningún píxel con dato."""
    alto, ancho = prof_mm.shape[:2]
    x1, y1, x2, y2 = caja
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    mx, my = (x2 - x1) * centro / 2, (y2 - y1) * centro / 2
    c1, c2 = int(max(0, np.floor(cx - mx))), int(min(ancho, np.ceil(cx + mx)))
    f1, f2 = int(max(0, np.floor(cy - my))), int(min(alto, np.ceil(cy + my)))
    trozo = prof_mm[f1:f2, c1:c2]
    validos = trozo[trozo > 0]
    if validos.size == 0:
        return None
    return float(np.median(validos)) / 1000.0


def punto_3d(u, v, z, K):
    """Píxel (u, v) a distancia z [m] -> (x, y, z) [m] en el frame óptico de la cámara.

    K: matriz 3x3 de la cámara (fx, fy en la diagonal, cx, cy en la última columna)."""
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    return ((u - cx) * z / fx, (v - cy) * z / fy, z)


def leer_clases(texto):
    """'lata, taza ,botella' -> ['lata', 'taza', 'botella'] (vacías fuera)."""
    return [c.strip() for c in texto.split(',') if c.strip()]
