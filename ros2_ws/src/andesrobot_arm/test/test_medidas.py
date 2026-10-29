# Pruebas de medidas.py: comparación de medidas y líneas del xacro, sin ROS ni xacro.
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_medidas.py
import pytest

from andesrobot_arm.medidas import lineas_topes, mas_cercano


def test_mas_cercano_elige_el_urdf_o_el_oficial():
    assert mas_cercano(90.0, 89.6, 122.0) == 'URDF'
    assert mas_cercano(121.0, 89.6, 122.0) == 'EB300'


def test_mas_cercano_dice_ninguno_si_no_se_parece_a_ninguno():
    assert mas_cercano(105.0, 89.6, 122.0) == 'ninguno'


def test_lineas_topes_convierte_grados_a_radianes():
    lineas = lineas_topes({'joint_2': (-90.0, 90.0)})
    assert lineas[0].startswith('<xacro:property name="joint_2_lower" value="-1.5708"/>')
    assert lineas[1].startswith('<xacro:property name="joint_2_upper" value="1.5708"/>')


def test_lineas_topes_rechaza_un_rango_al_reves():
    with pytest.raises(ValueError):
        lineas_topes({'joint_3': (120.0, -120.0)})
