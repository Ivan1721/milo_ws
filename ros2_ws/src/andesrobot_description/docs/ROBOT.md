# AndesRobot — modelo

Convención REP-103/105: +X adelante, +Y izquierda, +Z arriba.

Rev. 1 (2026-11-08): fusión del modelo de este repo, del modelo de estructura y masas de `milo_teleop`
y del brazo/pinza/cámara del equipo garra (`milo_ws`, rama `brazo`).

## Frames
| Frame | Dónde |
|---|---|
| `base_footprint` | suelo, bajo el centro del eje de las ruedas motrices |
| `base_link` | centro de los perfiles a la altura del eje; z = 0.08935 m (radio de rueda 0.08255 + 6.8 mm: con los soportes SK16 el eje queda bajo el origen) |
| `laser` | RPLIDAR C1, x = 0.408, z = 0.370 (desde base_footprint), 0° hacia +X |
| `imu_link_1` | IMU HFI-B9 sobre el acrílico: x = 0.08, z = 0.143 |
| `cabeza_link` | cabeza (carcasa + RealSense D435 + pantalla) en la cara frontal de la columna, z 1.36–1.51 m |
| `gripper_tcp` | punto de agarre, 0.102 m hacia −X de `link_6_1` |

## Estructura (rev. 1)
- Base 500×500 mm de 23 perfiles V-slot 3030 (anillos inferior y superior, 4 verticales, travesaños)
  con un **collar** de 3030 que abraza la columna 6060 por sus 4 caras en el nivel inferior y superior.
  74 escuadras 3030 + 2 soportes de eje SK16 (`herrajes_link`).
- Columna V-slot 6060 de 1.5 m con tapa; caja trasera de 3030 de 50 cm de alto (`rear_box_link`).
- Interior de la base: acrílico 3 mm, 2 baterías 10S2P, mainboard del hoverboard, regulador 36→12 V.
- Electrónica: 6 TB6600 en la cara delantera de la caja trasera, Jetson Orin Nano y Arduino Mega en la cara trasera de la columna.
- Carcasas de PAI 2 mm (base y caja trasera), monitor 12" en la columna (1.135 m, 20°), cabeza en la cara frontal.
- El brazo va atornillado **directo a la placa del carro** (sin la base redonda impresa): `arm_base_link_1` es solo un frame.

## Base diferencial
- Ruedas de hoverboard 6.5": radio 0.08255 m, **separación 0.3604 m** (centro a centro de la banda).
- Rueda loca de 3" con freno en x = 0.349 m.

## Masa y estabilidad
- Total **36.8 kg** (con la cámara de la pinza; 36.6 kg sin ella). Sin lastre: las masas salen de las piezas reales:
  perfiles a 0.835 kg/m, piezas impresas por vóxeles (PLA, 4 paredes, 7 capas techo/piso, 60 % de relleno),
  electrónica y baterías de sus fichas, motores del brazo instalados.
- **Estimadas (pesar):** ruedas 3.0 kg c/u, rueda loca 0.55, carro del elevador 0.8, monitor 0.55,
  pantalla de la cabeza 0.15, IMU 0.02. **Faltan:** motor del elevador, cables y tornillería, partes metálicas de los reductores EBA.
- Con el brazo plegado y el carro abajo: centro de masa a x = 0.082 m, z = 0.445 m. Vuelca hacia atrás
  sobre el eje acelerando a ~1.8 m/s², hacia adelante frenando a ~5.9 m/s², de lado a ~3.0 m/s².
  Con el carro en +0.43 m (lo más alto antes de tocar la cabeza): z = 0.62 m → 1.3 / 4.3 / 2.2 m/s².
- Diff drive: aceleración 0.8 m/s² y torque máx. 5 N·m por rueda.

## RPLIDAR C1 (simulado)
360°, 500 muestras/vuelta (0.72°), 10 Hz, 0.05–12 m, ruido σ = 12 mm.
La columna tapa ±4.5° detrás del sensor: se filtra en `andesrobot_slam/config/rplidar_c1_filter.yaml`.
El plano de escaneo (+30 mm sobre la base del sensor) es estimado: confirmar con el plano mecánico del C1.

## Argumentos del xacro
| Arg | Default | Uso |
|---|---|---|
| `lock_arm` | false | true = brazo, lift y dedos fijos en la **pose plegada** (simulación de base) |
| `simple_collision` | false | true = colisiones con cajas/cilindros en vez de mallas CAD (brazo plegado = una caja) |
| `sim_lidar` / `sim_imu` | true | apagar sensores simulados (depuración) |
| `use_hardware` | false | true = ros2_control del hoverboard (Milo físico) |
| `hoverboard_port` | /dev/hoverboard | puerto serie del hoverboard |
| `arm_control` | false | true = brazo, lift y pinza con ros2_control en Gazebo (`andesrobot.arm_control.xacro`; lo usa `andesrobot_arm`) |
| `arm_controllers_file` | (vacío) | YAML de controladores para el plugin `gazebo_ros2_control` (con `arm_control`) |
| `gripper_camera` | false | true = cámara Orbbec Gemini Plus detrás de la muñeca con sus sensores de Gazebo (`andesrobot.gripper_camera.xacro`) |

## Brazo
- 6 articulaciones `revolute` (joint_1..6) + lift prismático (`vertical_lift_joint`, −0.4…0.6 m) + pinza.
- **Pose plegada** (lo más bajo y recogido dentro de la huella de la base, sin choques; buscada por optimización):
  lift −0.40 m, J1 0.2775, J2 0.5965, J3 2.4000, J4 −1.0642, J5 1.8334, J6 0.8381 rad. Punto más alto del brazo
  a 0.928 m. Con `lock_arm:=true` queda fija ahí; `display.launch.py` arranca los sliders en ella.
- Límites del brazo **provisorios**: ±π, 20 N·m, 1 rad/s (propiedades `arm_lower`/`arm_upper`/`arm_effort`/`arm_velocity`
  y topes por articulación `joint_N_lower`/`joint_N_upper`; hoja de medición: `docs/medicion_brazo.md`).
- Pinza: `gripper_force` (20 N) es el tope de fuerza de cada dedo y `gripper_damping` (400 N·s/m) su amortiguamiento.
  El dedo izquierdo es `mimic` del derecho salvo con `arm_control:=true`.
- Motores **instalados** como links fijos `motor_joint_N` en el eslabón anterior a su articulación:
  J1/J3 NEMA23 1.26 N·m (0.70 kg), J2 NEMA23 3 N·m (1.60 kg), J4–J6 NEMA17 (0.28 kg); servo de la pinza 55 g.
  (La guía del EB300 pide otros motores: ver comentario en el xacro.)
- Reductores EBA-23/EBA-17 de 38.4:1, drivers a 800 pasos/rev.

## Pendiente antes del robot real
- Medir los topes físicos del brazo y cambiar `arm_lower`/`arm_upper`/`arm_effort`/`arm_velocity`.
- La cabeza ocupa el final del recorrido del carro: con el carro sobre ~+0.43 m o el brazo estirado hacia arriba, chocan.
- Verificar el yaw de `laser_joint` según cómo quede montado el conector del C1.
- Calibrar `wheel_radius` y `wheel_separation` en `milo_controllers.yaml`.
- Pesar las piezas marcadas como estimadas.
- Componentes del plano eléctrico que todavía no están en el URDF (falta su ubicación):
  el 7.º TB6600 y el **motor del elevador** (NEMA 23 1,26 N·m, ~0,70 kg); los conversores 42→24 V, 24→12 V y 24→6,6 V
  (el URDF tiene un solo regulador 36→12 V); hub USB, hub DisplayPort MST, USB-TTL, MPU-9250, WonderMV y la seta de emergencia.
- La pantalla de la cabeza es la Victron GX Touch 50 del plano; el monitor es el MUCAI N116A de 11,6" (el URDF usa uno genérico de 12").
