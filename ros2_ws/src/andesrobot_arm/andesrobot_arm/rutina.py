"""Cuentas de "grabar y reproducir" (rutina_node), sin ROS ni ventana, para poder probarlas.

En palabras simples: en vez de que la visión diga dónde está el objeto, una persona lleva el brazo
a mano (con los sliders o con el marcador de MoveIt) por las poses que hacen falta (arriba del
objeto, bajar, cerrar la garra, subir, dejarlo...) y en cada una aprieta "Guardar punto". Cada
punto guarda los 6 ángulos del brazo y la apertura de la garra. Después "Reproducir" hace lo mismo
solo:

  1. Agrupa los puntos en TRAMOS (`tramos`): una serie de puntos seguidos con la garra igual es una
     sola trayectoria del brazo (pasa por todos sin parar); cuando en un punto la garra cambia, el
     brazo primero llega a ese punto y recién ahí se abre o cierra la garra.
  2. A cada trayectoria le pone tiempos (`tiempos`): cada tramo entre dos puntos dura lo que tarde
     la articulación que más se mueve a `velocidad` rad/s (como los sliders), nunca menos de
     `minimo` segundos.

La rutina se guarda en un archivo YAML (`guardar`/`cargar`), para repetirla otro día. Ojo: los
ángulos son los del brazo real tal como los cuenta el Arduino; si se enciende en otra pose de
arranque, la rutina queda corrida.
"""
import yaml

from andesrobot_arm.sliders import duracion

ARTICULACIONES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# Diferencia de apertura (por dedo, en m) a partir de la cual se considera que la garra cambió.
UMBRAL_GARRA = 0.001


def nuevo_punto(angulos, garra, nombre=''):
    """Un punto de la rutina: 6 ángulos [rad], apertura de la garra [m por dedo] y un nombre."""
    if len(angulos) != len(ARTICULACIONES):
        raise ValueError(f'hacen falta {len(ARTICULACIONES)} ángulos, llegaron {len(angulos)}')
    return {'nombre': nombre, 'angulos': [float(a) for a in angulos], 'garra': float(garra)}


def tramos(puntos, garra_inicial):
    """Lista de pasos para reproducir: ('brazo', [ángulos, ...]) o ('garra', apertura).

    `garra_inicial` es la apertura actual (medida), para saber si el primer punto ya la cambia."""
    pasos, seguidos, garra = [], [], garra_inicial
    for p in puntos:
        seguidos.append(p['angulos'])
        if abs(p['garra'] - garra) > UMBRAL_GARRA:
            pasos.append(('brazo', seguidos))
            pasos.append(('garra', p['garra']))
            seguidos, garra = [], p['garra']
    if seguidos:
        pasos.append(('brazo', seguidos))
    return pasos


def tiempos(inicio, poses, velocidad, minimo=1.0):
    """Tiempo [s, desde el comienzo] en que la trayectoria llega a cada una de `poses`, partiendo
    de `inicio` (los ángulos actuales)."""
    t, antes, salida = 0.0, inicio, []
    for q in poses:
        t += duracion(antes, q, velocidad, minimo)
        salida.append(t)
        antes = q
    return salida


def guardar(ruta, puntos, nota=''):
    with open(ruta, 'w') as archivo:
        yaml.safe_dump({'nota': nota, 'articulaciones': ARTICULACIONES, 'puntos': puntos},
                       archivo, allow_unicode=True, sort_keys=False)


def cargar(ruta):
    with open(ruta) as archivo:
        datos = yaml.safe_load(archivo)
    if datos.get('articulaciones', ARTICULACIONES) != ARTICULACIONES:
        raise ValueError(f'{ruta}: las articulaciones no son {ARTICULACIONES}')
    return [nuevo_punto(p['angulos'], p['garra'], p.get('nombre', ''))
            for p in datos.get('puntos', [])]
