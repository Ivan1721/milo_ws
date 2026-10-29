"""Opciones de MoveIt que se eligen al lanzar arm_sim (sin ROS: se prueba sola).

arm_sim.launch.py acepta dos argumentos para probar mejoras sin tocar lo que ya funciona (con los
valores por defecto el comportamiento es el de siempre):

  ik:=kdl | pick_ik        solver de cinemática inversa de MoveIt. KDL es el de siempre.
                           pick_ik (PickNik) resuelve más casos cerca de los límites que KDL y es
                           el reemplazo que sí hay para ROS 2 Humble: TRAC-IK no tiene paquete
                           para Humble (ver docs/tecnologias/README.md).
  suavizado:=totg | ruckig Cómo se le pone tiempo a la trayectoria. TOTG es el de siempre
                           (velocidad y aceleración acotadas). ruckig agrega encima un suavizado
                           que acota también el tirón (jerk): importa para los motores paso a
                           paso, que pierden pasos con cambios bruscos de aceleración.

Aquí viven las dos decisiones que no dependen de ROS, para poder probarlas con pytest.
"""

RUCKIG = 'default_planner_request_adapters/AddRuckigTrajectorySmoothing'
TOTG = 'default_planner_request_adapters/AddTimeOptimalParameterization'

SOLVERS = {
    'kdl': 'config/kinematics.yaml',
    'pick_ik': 'config/kinematics_pick_ik.yaml',
}


def archivo_cinematica(ik):
    """Archivo de andesrobot_moveit_config con los parámetros del solver elegido."""
    if ik not in SOLVERS:
        raise ValueError(f"ik:={ik!r} no existe; opciones: {', '.join(SOLVERS)}")
    return SOLVERS[ik]


def con_ruckig(adaptadores):
    """
    La lista de adaptadores de OMPL (un texto separado por espacios) con el suavizado Ruckig.

    Los adaptadores se aplican de atrás hacia adelante (el primero de la lista es el último):
    Ruckig tiene que ir antes que TOTG para correr DESPUÉS de que la trayectoria ya tiene
    tiempo. Si ya está, no lo repite.
    """
    lista = adaptadores.split()
    if RUCKIG in lista:
        return ' '.join(lista)
    if TOTG not in lista:
        raise ValueError('ompl_planning.yaml no tiene AddTimeOptimalParameterization: '
                         'Ruckig necesita una trayectoria que ya tenga tiempo')
    lista.insert(lista.index(TOTG), RUCKIG)
    return ' '.join(lista)


def aplicar_ruckig(parametros):
    """
    Pone Ruckig en todos los `request_adapters` que haya (a cualquier profundidad) de un
    diccionario de parámetros de MoveIt, y devuelve cuántos cambió. Se busca por nombre porque
    cómo anida MoveItConfigsBuilder los pipelines cambia entre versiones de MoveIt.
    """
    cambiados = 0
    for clave, valor in parametros.items():
        if clave == 'request_adapters' and isinstance(valor, str):
            parametros[clave] = con_ruckig(valor)
            cambiados += 1
        elif isinstance(valor, dict):
            cambiados += aplicar_ruckig(valor)
    return cambiados
