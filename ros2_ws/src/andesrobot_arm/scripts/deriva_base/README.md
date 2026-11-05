# Deriva de la base en Gazebo (P13)

En `./sim.sh brazo` la base de Milo se va corriendo hacia atrás mientras el brazo se mueve, aunque
el brazo no toque nada: en una corrida de 300 fotos de `generar_dataset_manija.py` terminó 0.41 m
atrás y girada 24°. Esta carpeta reproduce el efecto **sin ROS**, mide qué lo causa y prueba el
arreglo. Medido el 5 de noviembre de 2026.

## Qué se simula

`deriva.cc` es un plugin de Gazebo que hace con el brazo y las ruedas lo mismo que hacen en
`arm_sim` los plugins de ROS (leído en su código fuente de Humble):

- **Brazo** (como `gazebo_ros2_control` con mando por posición): en cada paso de física pone cada
  articulación en la posición pedida con `SetPosition(q, true)` y velocidad 0; la posición pedida
  cambia a 100 Hz (el `update_rate` de `arm_controllers.yaml`) siguiendo una curva cúbica, como el
  `joint_trajectory_controller` con un solo punto.
- **Ruedas** (como `gazebo_ros_diff_drive` con `/cmd_vel` = 0): un "motor" de ODE en cada eje que
  pide velocidad 0 con un torque máximo de 5 N·m (`fmax`).
- Se manda el brazo a N posturas al azar (semilla fija, lift entre −0.2 y 0.4 m, articulaciones
  dentro de ±0.8 a ±1.5 rad) y se anota cuánto se movió la base y cuánto giraron las ruedas.

El robot es el mismo xacro de `arm_sim` (34.0 kg, mallas como colisión), sin la cámara de la pinza
(0.122 kg; sus sensores necesitan render). Mundo: suelo plano, ODE con paso de 1 ms, como la arena.

## Resultado

30 movimientos por variante:

| Variante | La base se corrió | Por movimiento |
|---|---:|---:|
| Como el repo (50 iteraciones de ODE) | 10.2 mm | 0.39 mm |
| Torque de rueda 20 N·m / 100 N·m | 10.2 / 10.3 mm | 0.39 mm |
| Orden del brazo a 1000 Hz en vez de 100 Hz | 10.1 mm | 0.39 mm |
| Fricción en el eje de las ruedas 1 N·m | 10.2 mm | 0.39 mm |
| Movimientos 3 veces más lentos | 25.9 mm | 0.94 mm |
| Brazo quieto 20 s después de cada movimiento (6 movimientos) | 18.5 mm | 3.1 mm |
| Fricción de las ruedas 2.0 en vez de 1.2 | 21.2 mm | 0.71 mm |
| Fricción de la rueda loca 0.5 en vez de 0 | 107.1 mm | 3.6 mm |
| **200 iteraciones de ODE** | **0.3 mm** | **0.01 mm** |
| 1000 iteraciones de ODE | 0.2 mm | 0.01 mm |

Cómo se lee:

- **Las ruedas no patinan: ruedan.** En todas las variantes, lo que giraron las ruedas (× el radio)
  es lo mismo que se movió la base. No es falta de fricción con el suelo.
- **No es falta de torque ni son los golpes del brazo**: con 20 o 100 N·m, o con la orden diez veces
  más fina, da lo mismo.
- **Depende del tiempo, no del movimiento**: más lento corre más, y con el brazo **quieto** sigue
  corriéndose. Es un arrastre lento.
- **La causa es el solver de ODE.** Gazebo resuelve las restricciones con un método iterativo
  (`quick`) que por defecto hace 50 iteraciones. Con el peso del brazo sobre el eje, el "motor" de
  velocidad 0 de las ruedas no alcanza a cumplirse del todo en cada paso: queda una velocidad
  residual muy chica que, sumada en miles de pasos, hace rodar la rueda. Un motor de velocidad no
  corrige la posición, así que lo que se corrió no vuelve. Con 200 iteraciones el error residual
  desaparece.

## Arreglo propuesto (para el equipo de la base)

En el bloque `<physics>` del mundo (`andesrobot_gazebo/worlds/andesrobot_arena.world`, y en
`tools/gen_casa_robocup.py` para `robocup_casa.world`):

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode><solver><type>quick</type><iters>200</iters></solver></ode>
</physics>
```

Costo: con 10 movimientos (55 s simulados), la física tardó 14 s con 50 iteraciones, 18 s con 100
y 26 s con 200 (incluye ~5 s de arranque del contenedor). Con 100 iteraciones la deriva baja a la
mitad (0.21 mm por movimiento); con 200, a cero. Sin cámaras sigue más rápido que el tiempo real;
**falta medir** el factor de tiempo real con `./sim.sh brazo` completo (cámara, MoveIt, RViz) y
comprobar que la base sigue manejando igual con `./sim.sh` y `./sim.sh nav`.

## Cómo correrlo

Desde el PC, con Docker (no hace falta el contenedor `milo`):

```bash
cd ~/milo_ws/ros2_ws/src/andesrobot_arm/scripts/deriva_base
./correr.sh base N=30                  # como está el repo
./correr.sh iters200 N=30 ITERS=200    # con el arreglo
```

La primera vez construye la imagen `milo-deriva` (Gazebo 11 de Ubuntu sobre `ros:humble-ros-base`,
unos 700 MB) y clona `xacro`. Variables: `N`, `ESCALA`, `ESPERA`, `TASA`, `TORQUE`, `SEMILLA` (en
`deriva.cc`) y `CASTER_MU`, `MU_RUEDA`, `FRICCION_EJE`, `ITERS`, `SOLVER` (en `preparar.py`).
Cada fila de `ros2_ws/resultados/deriva_base/NOMBRE.txt` (no se versiona) es un movimiento:
duración, desplazamiento de la base en x e y (mm), giro (°) y giro de cada rueda (rad). El clon
de `xacro` y el plugin compilado quedan en `~/.cache/milo_deriva/`, fuera de `ros2_ws/src`
(colcon tomaría el clon como un paquete). Los archivos que escribe el contenedor son de root.
