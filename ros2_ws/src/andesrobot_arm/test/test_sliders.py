"""Pruebas de sliders.py (las cuentas de la ventana sliders_node), sin ROS ni ventana.

python3 -m pytest -q test/test_sliders.py
"""
import math

import pytest

from andesrobot_arm import brazo_real
from andesrobot_arm.sliders import (
    DEDO_ABIERTA, DEDO_CERRADA, a_segundos_nanos, acotar, dedos, duracion, limites_de_urdf)

URDF = '''<robot name="prueba">
  <joint name="joint_1" type="revolute"><limit lower="-1.5" upper="2.0" effort="20" velocity="1"/>
  </joint>
  <joint name="vertical_lift_joint" type="prismatic">
    <limit lower="-0.4" upper="0" effort="100" velocity="0.1"/></joint>
  <joint name="joint_6" type="continuous"/>
  <joint name="fija" type="fixed"/>
  <ros2_control name="GazeboSystem" type="system">
    <joint name="vertical_lift_joint"><command_interface name="velocity"/></joint>
  </ros2_control>
</robot>'''


def test_limites_de_urdf():
    """Lee lower/upper de cada articulación pedida; continuous queda en ±π; las demás no; el
    <joint> del bloque ros2_control (sin <limit>) no pisa el de verdad."""
    lim = limites_de_urdf(URDF, ['joint_1', 'vertical_lift_joint', 'joint_6', 'joint_2'])
    assert lim['joint_1'] == (-1.5, 2.0)
    assert lim['vertical_lift_joint'] == (-0.4, 0.0)
    assert lim['joint_6'] == (-math.pi, math.pi)
    assert 'joint_2' not in lim and 'fija' not in lim


def test_duracion_la_manda_la_mas_lenta():
    """joint_2 0 -> 30° a 0.1 rad/s = 5.24 s; la que se mueve menos no importa."""
    t = duracion([0.0, 0.0], [math.radians(30), math.radians(5)], 0.1)
    assert t == pytest.approx(math.radians(30) / 0.1)
    # Velocidad por articulación: el lift a 0.02 m/s, 10 cm = 5 s.
    assert duracion([0.0, 0.0], [-0.1, 0.1], [0.02, 0.1]) == pytest.approx(5.0)


def test_duracion_minima():
    """Un movimiento chico (o ninguno) dura al menos `minimo`."""
    assert duracion([0.0], [0.001], 0.1, minimo=1.0) == 1.0
    assert duracion([0.3], [0.3], 0.1, minimo=2.0) == 2.0


def test_segundos_nanos():
    assert a_segundos_nanos(5.25) == (5, 250000000)
    assert a_segundos_nanos(1.0) == (1, 0)


def test_dedos_en_espejo_y_acotados():
    assert dedos(0.010) == [0.010, -0.010]
    assert dedos(0.05) == [DEDO_ABIERTA, -DEDO_ABIERTA]
    assert dedos(-0.05) == [DEDO_CERRADA, -DEDO_CERRADA]
    assert acotar(5, 0, 3) == 3 and acotar(-1, 0, 3) == 0


def test_misma_garra_que_brazo_real():
    """Las aperturas de la garra coinciden con las de brazo_real.py (y la simulación)."""
    assert (DEDO_ABIERTA, DEDO_CERRADA) == (brazo_real.DEDO_ABIERTA, brazo_real.DEDO_CERRADA)
