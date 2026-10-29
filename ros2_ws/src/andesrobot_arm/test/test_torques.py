# Pruebas de torques.py: físicas simples con respuesta conocida, sin ROS ni xacro.
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_torques.py
import numpy as np

from andesrobot_arm.torques import ModeloMasas, torque_maximo

# Un brazo de juguete: una articulación horizontal (eje Y) en el origen y un eslabón de 1 kg
# cuyo centro de masa está a 1 m en X. Con el eslabón horizontal, g hace 1 kg · 9.81 m/s² · 1 m.
URDF = """
<robot name="juguete">
  <link name="base"/>
  <link name="brazo">
    <inertial><origin xyz="1 0 0" rpy="0 0 0"/><mass value="1.0"/></inertial>
  </link>
  <link name="motor">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2.0"/></inertial>
  </link>
  <joint name="hombro" type="revolute">
    <parent link="base"/><child link="brazo"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="motor_fijo" type="fixed">
    <parent link="brazo"/><child link="motor"/><origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_brazo_horizontal_pide_m_g_d():
    # 1 kg a 1 m + motor de 2 kg a 0.5 m: 9.81 · (1·1 + 2·0.5) = 19.62 N·m
    modelo = ModeloMasas(URDF)
    assert abs(abs(modelo.torque('hombro')) - 19.62) < 1e-9


def test_brazo_vertical_no_pide_torque():
    modelo = ModeloMasas(URDF)
    assert abs(modelo.torque('hombro', {'hombro': np.pi / 2})) < 1e-9


def test_carga_extra_en_la_punta_suma():
    modelo = ModeloMasas(URDF)
    sin = abs(modelo.torque('hombro'))
    con = abs(modelo.torque('hombro', carga=(0.5, 'brazo', np.array([2.0, 0.0, 0.0]))))
    assert abs((con - sin) - 0.5 * 9.81 * 2.0) < 1e-9


def test_masa_colgante_cuenta_los_links_fijos():
    assert ModeloMasas(URDF).masa_colgante('hombro') == 3.0


def test_barrido_encuentra_el_horizontal():
    modelo = ModeloMasas(URDF)
    mejor = torque_maximo(modelo, ['hombro'], {'hombro': (-3.14, 3.14)}, n=2000)
    assert 19.4 < mejor['hombro'][0] <= 19.62 + 1e-9
