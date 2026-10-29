"""Medidas del brazo real contra el URDF y el DH oficial del EB300 (sin ROS: se prueba sola).

Para qué sirve (pendiente P04, docs/PENDIENTES.md): cuando el brazo esté armado, se miden a mano
las distancias entre ejes y los topes de cada articulación. Aquí se comparan esas medidas con lo
que dice el URDF (del CAD) y con lo que publica el fabricante, y se escriben las líneas del xacro
que hay que cambiar. La hoja con el procedimiento paso a paso es docs/medicion_brazo.md.

Las seis distancias son las de la tabla del informe (sección 5), medidas con el brazo en la pose
cero (todas las articulaciones en 0):
  d1  base del brazo → eje del hombro (joint_2), en altura
  a2  hombro → codo (joint_2 → joint_3), en altura
  a3  codo → muñeca 1 (joint_3 → joint_4), en altura
  d4  corrimiento lateral del eje de joint_1 al eje de la muñeca 2 (joint_5)
  d5  eje de la muñeca 1 → eje de la muñeca 3 (joint_4 → joint_6), en altura
  d6  eje de la muñeca 2 → brida (joint_5 → joint_6), en horizontal
"""
import numpy as np

# Qué mide cada parámetro y su valor en el DH oficial del EB300 [mm] (informe, sección 5).
PARAMETROS = {
    'd1': ('base → eje del hombro', 95.0),
    'a2': ('hombro → codo', 348.0),
    'a3': ('codo → muñeca 1', 318.0),
    'd4': ('corrimiento lateral joint_1 → muñeca 2', 122.0),
    'd5': ('muñeca 1 → muñeca 3', 99.0),
    'd6': ('muñeca 2 → brida', 57.0),
}


def distancias_urdf(kin):
    """Las seis distancias [mm] del URDF, de un ArmKinematics con la cadena de 6 ejes."""
    frames, _ = kin.frames([0.0] * kin.dof)
    p = [f[:3, 3] for f in frames]    # posición de joint_1 … joint_6 en el frame de la base
    mm = 1000.0
    return {
        'd1': abs(p[1][2]) * mm,
        'a2': abs(p[2][2] - p[1][2]) * mm,
        'a3': abs(p[3][2] - p[2][2]) * mm,
        'd4': abs(p[4][0] - p[0][0]) * mm,
        'd5': abs(p[5][2] - p[3][2]) * mm,
        'd6': abs(p[5][0] - p[4][0]) * mm,
    }


def mas_cercano(medido, urdf, oficial, tolerancia=2.0):
    """'URDF', 'EB300' o 'ninguno': a cuál de los dos se parece más la medida [mm]."""
    d_urdf, d_ofic = abs(medido - urdf), abs(medido - oficial)
    if min(d_urdf, d_ofic) > tolerancia:
        return 'ninguno'
    return 'URDF' if d_urdf <= d_ofic else 'EB300'


def lineas_topes(topes):
    """
    Líneas del xacro para los topes medidos. topes = {'joint_2': (min_grados, max_grados), ...}.
    Los ángulos se miden desde la pose cero, con el sentido positivo del eje de esa articulación.
    """
    lineas = []
    for joint in sorted(topes):
        bajo, alto = topes[joint]
        if not bajo < alto:
            raise ValueError(f'{joint}: el tope mínimo ({bajo}) debe ser menor que el '
                             f'máximo ({alto})')
        for lado, grados in (('lower', bajo), ('upper', alto)):
            valor = f'{np.radians(grados):.4f}'
            lineas.append(f'<xacro:property name="{joint}_{lado}" value="{valor}"/>'
                          f'   <!-- {grados:g}° -->')
    return lineas
