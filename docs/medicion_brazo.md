# Hoja de medición del brazo real

Para cerrar el pendiente **P04** de [`PENDIENTES.md`](PENDIENTES.md). Se hace **con el brazo armado**
(P01). Sirve para tres cosas: poner los topes reales de cada articulación, decidir si la muñeca del
CAD o la del fabricante es la que vale, y confirmar los desplazamientos a lo largo de los ejes.

**Qué necesitas:** calibrador o regla de acero, un inclinómetro (sirve una app de celular) o un
transportador, un nivel y lápiz. Anota todo en las tablas de abajo y pásalo a la sesión que siga.

## 1. La pose cero

Todas las medidas se toman desde la **pose cero** del URDF: todas las articulaciones en 0 y el lift
en su posición media. En esa pose **el brazo apunta hacia arriba y la pinza hacia +X** (hacia
adelante del robot). Marca con lápiz, en cada articulación, la línea de cero cuando el brazo esté
así; son las marcas de las que salen los ángulos.

Sentido positivo de cada articulación (regla de la mano derecha, con el eje tal como queda en la
pose cero): `joint_1` +Z · `joint_2` +X · `joint_3` −X · `joint_4` +X · `joint_5` +Z · `joint_6` +X.
Un ángulo medido en el sentido contrario es **negativo**.

## 2. Topes de cada articulación

Mueve la articulación despacio hasta que tope en cada extremo y lee el ángulo desde la marca de cero
(repite 3 veces y anota el promedio). Si una articulación no tiene tope y da vueltas completas,
escríbelo.

| Articulación | Tope mínimo [°] | Tope máximo [°] | Observaciones |
| --- | ---: | ---: | --- |
| `joint_1` |  |  |  |
| `joint_2` |  |  |  |
| `joint_3` |  |  |  |
| `joint_4` |  |  |  |
| `joint_5` |  |  |  |
| `joint_6` |  |  |  |
| lift (`vertical_lift_joint`), carrera [m] |  |  | hoy −0.4 … +0.6 m |

## 3. Distancias entre ejes

Con el brazo en la pose cero, mide entre los **ejes de giro** (el centro del eje, no el borde de la
pieza). Los valores del URDF y del fabricante ya están calculados:

| Parámetro | Qué mide | URDF [mm] | EB300 [mm] | Medido [mm] |
| --- | --- | ---: | ---: | ---: |
| `d1` | base del brazo → eje del hombro, en altura | 93.0 | 95 |  |
| `a2` | hombro → codo, en altura | 350.1 | 348 |  |
| `a3` | codo → muñeca 1, en altura | 323.5 | 318 |  |
| `d4` | corrimiento lateral del eje de `joint_1` al eje de `joint_5` | 89.6 | 122 |  |
| `d5` | eje de `joint_4` → eje de `joint_6`, en altura | 89.5 | 99 |  |
| `d6` | eje de `joint_5` → brida, en horizontal | 41.0 | 57 |  |

## 4. Desplazamientos a lo largo de los ejes

Son los cinco desplazamientos de la sección 5 del informe. Cada uno corre el plano en que trabaja el brazo. Mide la distancia **a lo largo del eje** entre los
dos planos de las piezas que une:

| Tramo | URDF [mm] | Medido [mm] |
| --- | ---: | ---: |
| `joint_1` → `joint_2` | 65.0 |  |
| `joint_2` → `joint_3` | 1.1 |  |
| `joint_3` → `joint_4` | −26.0 |  |
| `joint_4` → `joint_5` (a lo largo de `joint_4`) | 49.5 |  |
| `joint_5` → `joint_6` (a lo largo de `joint_6`; es `d6`) | 41.0 |  |

## 5. Pasar las medidas al repo

1. **Comparar** (dentro del contenedor, `./sim.sh shell`; fuera, con `--urdf`):
   `python3 /ros2_ws/src/andesrobot_arm/scripts/comparar_medidas.py --d4 121 --d5 98 --d6 56`
   imprime la tabla con la diferencia contra el URDF y a cuál (URDF o EB300) se parece cada medida.
2. **Topes:** el mismo script escribe las líneas del xacro:
   `python3 …/comparar_medidas.py --tope joint_2=-10,170 --tope joint_3=-120,120`
   Pégalas en la sección ARTICULACIONES de `andesrobot.urdf.xacro`, sobre las propiedades
   `joint_N_lower` / `joint_N_upper` de esa articulación (hoy valen ±π).
3. **Muñeca:** si la medida se parece al fabricante (EB300), hay que corregir el `origin xyz` de
   `joint_4`, `joint_5` y `joint_6` en el xacro. Eso cambia la cinemática: después de tocarlo, corre
   `scripts/figuras.py`, actualiza las tablas de la sección 5 del informe y el valor de
   `pose_cero_coincide_con_el_urdf` en `test/test_kinematics.py`.
4. **Comprobar:** `./sim.sh build`, las pruebas (`pytest`) y `prueba_gazebo.sh`. Con topes reales,
   el giro casi completo de `joint_1` cerca de ±π desaparece.
5. **Documentar:** informe §5 (límites y comparación con el DH) y §23, y tachar P04 en `PENDIENTES.md`.

**Antes de mover el brazo real** con MoveIt o la IK, los topes tienen que estar puestos: los ±π de hoy
no protegen nada.
