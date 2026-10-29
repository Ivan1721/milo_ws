# Pruebas de moveit_opciones.py: las decisiones de arm_sim que no necesitan ROS.
#   cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_moveit_opciones.py
import pytest

from andesrobot_arm.moveit_opciones import (
    RUCKIG, TOTG, aplicar_ruckig, archivo_cinematica, con_ruckig)

ADAPTADORES = (f'{TOTG} default_planner_request_adapters/ResolveConstraintFrames '
               'default_planner_request_adapters/FixWorkspaceBounds')


def test_ruckig_va_antes_que_totg():
    lista = con_ruckig(ADAPTADORES).split()
    assert lista[0] == RUCKIG and lista[1] == TOTG and len(lista) == 4


def test_ruckig_no_se_repite():
    una_vez = con_ruckig(ADAPTADORES)
    assert con_ruckig(una_vez) == una_vez


def test_ruckig_sin_totg_es_un_error():
    with pytest.raises(ValueError):
        con_ruckig('default_planner_request_adapters/FixWorkspaceBounds')


def test_archivo_del_solver():
    assert archivo_cinematica('kdl') == 'config/kinematics.yaml'
    assert archivo_cinematica('pick_ik') == 'config/kinematics_pick_ik.yaml'
    with pytest.raises(ValueError):
        archivo_cinematica('trac_ik')


def test_aplicar_ruckig_encuentra_los_adaptadores_anidados():
    parametros = {'planning_pipelines': ['ompl'],
                  'ompl': {'planning_plugin': 'x', 'request_adapters': ADAPTADORES},
                  'otro': {'hondo': {'request_adapters': ADAPTADORES}}}
    assert aplicar_ruckig(parametros) == 2
    assert parametros['ompl']['request_adapters'].startswith(RUCKIG)
    assert parametros['otro']['hondo']['request_adapters'].startswith(RUCKIG)


def test_aplicar_ruckig_sin_adaptadores_no_cambia_nada():
    assert aplicar_ruckig({'ompl': {'planning_plugin': 'x'}}) == 0
