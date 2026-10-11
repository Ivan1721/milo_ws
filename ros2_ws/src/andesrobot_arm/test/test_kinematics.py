# Tests de la cinemática del brazo de Milo (kinematics.py), sin ROS corriendo.
# Necesitan andesrobot_description instalado (lee su xacro):
#   source install/setup.bash && cd src/andesrobot_arm && python3 -m pytest -q test
import numpy as np
import pytest

from andesrobot_arm.arm_ik_node import trajectory_duration
from andesrobot_arm.kinematics import (
    ArmKinematics, LIFT_BASE, LIFT_JOINT, matrix_to_quaternion, quaternion_to_matrix,
    rpy_to_matrix)


@pytest.fixture(scope='module')
def arm():
    return ArmKinematics.from_xacro()


@pytest.fixture(scope='module')
def arm_lift():
    return ArmKinematics.from_xacro(base=LIFT_BASE)


# Todo en cero: el TCP queda en la suma de los offsets del URDF (link_6_1 en (-0.130577, 0,
# 0.856077) desde arm_base_link_1) más 0.102 m hacia -X (gripper_tcp), sin rotación.
def test_pose_cero_coincide_con_el_urdf(arm):
    tcp = arm.fk(np.zeros(arm.dof))
    np.testing.assert_allclose(tcp[:3, 3], [-0.232577, 0.0, 0.856077], atol=1e-9)
    np.testing.assert_allclose(tcp[:3, :3], np.eye(3), atol=1e-12)


# La parte lineal del Jacobiano tiene que coincidir con derivar la FK numéricamente.
def test_jacobiano_igual_a_diferencias_finitas(arm):
    q = np.random.default_rng(0).uniform(-np.pi, np.pi, arm.dof)
    h = 1e-6
    numeric = np.zeros((3, arm.dof))
    for i in range(arm.dof):
        dq = np.zeros(arm.dof)
        dq[i] = h
        numeric[:, i] = (arm.fk(q + dq)[:3, 3] - arm.fk(q - dq)[:3, 3]) / (2 * h)
    np.testing.assert_allclose(arm.jacobian(q)[:3], numeric, atol=1e-8)


# Poses alcanzables (FK de ángulos al azar): la IK tiene que volver a llegar a ellas.
def test_ik_llega_a_poses_alcanzables(arm):
    rng = np.random.default_rng(1)
    for k in range(20):
        target = arm.fk(rng.uniform(-np.pi, np.pi, arm.dof))
        q, ok, _ = arm.ik(target, seed=k)
        assert ok
        np.testing.assert_allclose(arm.fk(q), target, atol=1e-3)


# Con el lift (7 articulaciones), poses alcanzables y un punto cerca del suelo, frente a
# Milo, que el brazo solo no alcanza: hay que bajar el lift.
def test_ik_con_lift(arm_lift):
    li = arm_lift.joint_names.index(LIFT_JOINT)
    weights = np.ones(arm_lift.dof)
    weights[li] = 10.0
    rng = np.random.default_rng(3)
    for k in range(10):
        target = arm_lift.fk(arm_lift._random_q(rng))
        q, ok, _ = arm_lift.ik(target, weights=weights, seed=k)
        assert ok
        assert -0.4 <= q[li] <= 0.6
        np.testing.assert_allclose(arm_lift.fk(q), target, atol=1e-3)
    q0 = np.zeros(arm_lift.dof)
    reach_lift0 = arm_lift.fk(q0)[2, 3]
    target = np.eye(4)
    target[:3, 3] = [0.55, 0.0, 0.05]
    q, ok, _ = arm_lift.ik(target, position_only=True, weights=weights, seed=0)
    assert ok and q[li] < 0.0 and reach_lift0 > 1.0


def test_cuaternion_ida_y_vuelta():
    rng = np.random.default_rng(2)
    for _ in range(50):
        rot = rpy_to_matrix(*rng.uniform(-np.pi, np.pi, 3))
        np.testing.assert_allclose(quaternion_to_matrix(*matrix_to_quaternion(rot)), rot,
                                   atol=1e-9)


# La duración la pone la articulación más lenta, y nunca baja del mínimo.
def test_duracion_de_trayectoria():
    assert trajectory_duration([0, 0], [0.1, -1.0], 0.5, 1.0) == 2.0
    assert trajectory_duration([0, 0], [0.1, 0.1], 0.5, 1.0) == 1.0
    assert trajectory_duration([0, 0], [0.3, 1.0], [0.1, 0.5], 1.0) == pytest.approx(3.0)
