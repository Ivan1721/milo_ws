"""Carga las ubicaciones con nombre (config/ubicaciones_*.yaml). Sin ROS: se prueba solo."""

import math

import yaml


# Archivo de lugares que anotas tú después de mapear (con tf2_echo), al lado del mapa.
# Ruta dentro del contenedor (ros2_ws/ de tu PC = /ros2_ws).
DEFAULT_PATH = '/ros2_ws/src/andesrobot_slam/maps/casa_lugares.yaml'


def default_path():
    """El archivo de lugares de la casa (ver DEFAULT_PATH)."""
    return DEFAULT_PATH


def load(path=None):
    """Devuelve (frame_id, {nombre: dict(x, y, yaw, ...)}).

    Acepta dos formatos:
      - simple, una línea por lugar:   sofa: {x: -3.15, y: -4.03, yaw: -1.55}
      - completo: frame_id: map  +  ubicaciones: {sofa: {...}, ...}  (room, id, place opcionales)
    """
    with open(path or default_path()) as fh:
        data = yaml.safe_load(fh) or {}
    if 'ubicaciones' in data:
        locs = data['ubicaciones']
    else:
        locs = {k: v for k, v in data.items() if isinstance(v, dict)}
        data = {}
    for name, d in locs.items():
        d.setdefault('yaw', 0.0)
        d.setdefault('room', '-')
        d['name'] = name
    return data.get('frame_id', 'map'), locs


def resolve(name, locs):
    """Busca por nombre exacto, id del rulebook o prefijo único ("din" -> dinner_table)."""
    key = str(name).strip().lower().replace(' ', '_')
    if key in locs:
        return locs[key]
    for d in locs.values():
        if d.get('id') is not None and str(d['id']) == key:
            return d
    matches = [d for n, d in locs.items() if n.startswith(key)]
    if len(matches) == 1:
        return matches[0]
    raise KeyError(
        f'Ubicación "{name}" no existe o es ambigua. Opciones: {", ".join(sorted(locs))}'
    )


def yaw_to_quat(yaw):
    """(z, w) del cuaternión para un giro yaw en el plano."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
