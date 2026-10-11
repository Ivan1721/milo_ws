"""Carga las ubicaciones con nombre (config/ubicaciones_*.yaml). Sin ROS: se prueba solo."""

import math
import os

import yaml


def default_path():
    """El .yaml de la casa RoboCup instalado en el paquete."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory('andesrobot_navigation'),
        'config',
        'ubicaciones_robocup_casa.yaml',
    )


def load(path=None):
    """Devuelve (frame_id, {nombre: dict(x, y, yaw, room, id, place)}).

    También acepta el número del rulebook como nombre ("16" -> dinner_table).
    """
    with open(path or default_path()) as fh:
        data = yaml.safe_load(fh)
    locs = data['ubicaciones']
    for name, d in locs.items():
        d.setdefault('yaw', 0.0)
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
