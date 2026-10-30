# Pendientes del brazo de Milo

Lista corta de lo que falta, con lo mínimo para empezar cada tarea. **Está hecha para no tener que leer
todo el informe** (~24 000 palabras): lee la tabla, elige un pendiente y abre solo los archivos que
nombra su campo «Dónde mirar».

Cómo usarla:
- Un pendiente cerrado se tacha en la tabla y se borra su sección; el informe (sección 23) tiene el contexto largo.
- Al agregar uno nuevo, usa el mismo formato: contexto, qué hacer, listo cuando, dónde mirar.
- Las etiquetas (`brazo`, `hardware`, `simulación`, `documentación`, `ci`, `otro-equipo`) sirven para filtrar con `grep`.

| # | Pendiente | Etiquetas | Depende de |
|---|---|---|---|
| P01 | Armar el brazo físico | hardware | — |
| P02 | IMU MPU-9250 en el URDF, la simulación y su soporte | brazo, simulación, hardware | — |
| P03 | Plano electrónico | hardware, documentación | — |
| P04 | Medir el brazo real: topes, muñeca y ejes | brazo, hardware | P01 |
| P05 | Pesar reductores y piezas impresas; recalcular masas y torques | brazo, hardware | P01 |
| P06 | Driver del brazo real (firmware Arduino + ros2_control) | brazo, hardware | P01, P03, P04 |
| P07 | Cámara: profundidad realista y Gemini Plus real | brazo, simulación | — |
| P08 | MoveIt 2: probar pick_ik y Ruckig, objetos conocidos y afinar el OctoMap | brazo, simulación | — |
| P09 | Desafío de la puerta: del detector a empujar | brazo, simulación | — |
| P10 | Simulación con mando por esfuerzo, PID y gravedad | brazo, simulación | — |
| P11 | CI etapa 2: pruebas del brazo con ROS en GitHub Actions (la etapa 1 ya está) | ci | — |
| P12 | Documentación: artifact, `docs/tecnologias` y referencias | documentación | — |
| P13 | La base resbala en Gazebo (coordinar con el equipo de la base) | otro-equipo, simulación | — |

---

## P01 · Armar el brazo físico
**Contexto:** las piezas impresas están listas; falta armarlo.
**Qué hacer:** armar el brazo con los motores paso a paso (NEMA 23 en las tres primeras articulaciones, NEMA 17 en las demás), sus reductores y la pinza EBG-20; montarlo sobre el lift y la columna.
**Listo cuando:** cada eje gira sin atascos a mano y hay fotos y medidas del montaje en `docs/hardware/`.
**Dónde mirar:** informe §6; `docs/hardware/README.md`.

## P02 · IMU MPU-9250 en el URDF, la simulación y su soporte
**Contexto:** la IMU va bajo la cámara, en la pinza. Hoy no está en el modelo ni en la simulación (solo existe la IMU de la base).
**Qué hacer:** (1) diseñar el soporte en Fusion 360; (2) agregar un `imu_link` al xacro con una articulación fija bajo la cámara (el CAD está girado 180° en Z; un hijo nuevo probablemente necesite `rpy="0 0 π"`); (3) publicar su dato en simulación con el plugin de IMU de Gazebo (plantilla: la IMU de la base en `andesrobot.sim.xacro`); (4) agregar una comprobación a `scripts/prueba_gazebo.sh`.
**Listo cuando:** `./sim.sh brazo` publica el topic con datos, el frame sale en TF y queda documentado (informe §14 o §4).
**Dónde mirar:** `docs/tecnologias` §5; `docs/literatura` §7; `andesrobot_description/urdf/`.

## P03 · Plano electrónico
**Contexto:** el brazo real se controla con un Arduino Mega 2560 y 6 drivers TB6600 que mueven los motores paso a paso.
**Qué hacer:** dibujar el esquema (KiCad, recomendado en `docs/tecnologias`): STEP / DIR / EN de cada driver, alimentación, finales de carrera (homing) e IMU por I²C. Revisar la corriente: los TB6600 (hasta 4 A) quedan justos para los NEMA 23 de 4.0 A. Lista de materiales y tabla de pines.
**Listo cuando:** el esquema y la tabla de pines están versionados (por ejemplo en `docs/electronica/`).
**Dónde mirar:** `docs/tecnologias` §5–6; `docs/literatura` §6.

## P04 · Medir el brazo real: topes, muñeca y ejes
**Contexto:** los límites son provisorios (±π, 20 N·m, 1 rad/s) y la IK puede devolver posturas que el brazo real no hace. La muñeca del CAD (d₄, d₅, d₆) difiere 10–32 mm del DH oficial del EB300, y los desplazamientos a lo largo de los ejes no están confirmados.
**Ya hecho:** cada articulación tiene ahora sus propios topes en el xacro (`joint_N_lower` / `joint_N_upper`, hoy ±π; el URDF procesado quedó idéntico). `docs/medicion_brazo.md` es la hoja para medir: pose cero, sentido positivo de cada eje, tablas de topes, de las 6 distancias del DH y de los 5 desplazamientos. `scripts/comparar_medidas.py` compara lo medido con el URDF y el EB300 y escribe las líneas del xacro (`andesrobot_arm/medidas.py`, 4 pruebas).
**Falta (tú, con el brazo armado):** llenar las tablas de la hoja y pasarlas.
**Falta (después de medir):** pegar los topes en el xacro; si la muñeca resulta ser la del fabricante, corregir los `origin` de `joint_4..6`, regenerar figuras, actualizar las tablas de la sección 5 y `pose_cero_coincide_con_el_urdf`.
**Listo cuando:** el xacro tiene los valores medidos, la tabla de la sección 5 queda actualizada y `pytest` y `prueba_gazebo.sh` siguen pasando.
**Dónde mirar:** `docs/medicion_brazo.md`; informe §5; `andesrobot_arm/docs/BRAZO.md`.
**Depende de:** armar el brazo físico (P01).

## P05 · Pesar reductores y piezas impresas; recalcular masas y torques
**Contexto:** las masas ya incluyen motores y pinza en ABS (8.05 kg el brazo completo). Faltan los reductores y las piezas impresas reales.
**Ya hecho:** `andesrobot_arm/scripts/calcular_torques.py` calcula el torque estático de cada articulación con las masas del URDF (`andesrobot_arm/torques.py`, 5 pruebas en `test/test_torques.py`) y `--pesaje` imprime la tabla de piezas para pesar. Resultado preliminar, sin carga en la pinza (20 000 posturas al azar): `joint_2` ≈ 23 N·m, `joint_3` ≈ 8 N·m, `joint_4` ≈ 0.9 N·m, `joint_5` ≈ 0.3 N·m. `joint_2` ya supera el límite provisorio de 20 N·m. Con 0.5 kg en la pinza sube a ≈ 27 y 10 N·m. Ante la retención de los motores (2.4 / 1.85 / 0.65 N·m) pide reductores de al menos ≈ 19:1 en `joint_2` y ≈ 9:1 en `joint_3` con margen 2.
**Falta (tú):** (1) pesar las piezas con `calcular_torques.py --pesaje` y anotar el peso real; (2) pesar los reductores EBA-23 y EBA-17 y averiguar su relación de reducción (no está en los documentos del repo).
**Falta (después de pesar):** poner las masas medidas en `andesrobot.urdf.xacro` (los reductores van como masa extra en el link del motor), volver a correr el script y reemplazar `arm_effort` por el torque calculado.
**Listo cuando:** la tabla de masas de la sección 6 tiene los valores medidos, el URDF los usa, `arm_effort` sale del cálculo y las pruebas pasan.
**Dónde mirar:** informe §6 («Masas corregidas en el URDF»); `docs/hardware/README.md`; `andesrobot_arm/scripts/calcular_torques.py`.

## P06 · Driver del brazo real (firmware Arduino + ros2_control)
**Contexto:** el control del brazo existe solo en simulación. El camino natural es un plugin de ros2_control que hable por serie con el Arduino, como el del hoverboard.
**Qué hacer:** (1) protocolo serie PC ↔ Arduino; (2) firmware con AccelStepper y homing con los finales de carrera; (3) interface de hardware en C++ en `ros2_ws/src/drivers/` (plantilla: `hoverboard_hardware_interface`); (4) ajustar `arm_controllers.yaml` y `robot.sh`.
**Listo cuando:** se mueve un eje a una posición, se lee su `joint_states` y el homing funciona.
**Dónde mirar:** informe §18; `docs/tecnologias` §2 y §5; `docs/literatura` §5–6.

## P07 · Cámara: profundidad realista y Gemini Plus real
**Contexto:** la profundidad simulada es perfecta (sin ruido ni huecos) y la cámara real no está conectada. La simulada publica `32FC1` en metros y la real `16UC1` en milímetros.
**Qué hacer:** nodo `gemini_realista` (ruido que crece con Z², sombras del proyector, huecos por material y ángulo, infrarrojo con patrón); conectar la Gemini Plus real con `OrbbecSDK_ROS2` con los mismos nombres de topics; un único lugar que convierta metros a milímetros; calibración mano–ojo (por ejemplo `easy_handeye2`).
**Listo cuando:** `capturar_camara` y los topics funcionan igual con la cámara simulada y la real; queda en el informe §14.
**Dónde mirar:** informe §14 («Cómo hacer realista la profundidad»); `BRAZO.md`; `andesrobot.gripper_camera.xacro`.

## P08 · MoveIt 2: probar pick_ik y Ruckig, objetos conocidos y afinar el OctoMap
**Contexto:** MoveIt 2 ya planifica sin choques (KDL + OMPL + TOTG + OctoMap). **TRAC-IK no tiene paquete para ROS 2 Humble** (comprobado en el índice oficial de rosdistro, 29-oct-2026), así que el reemplazo de KDL es **pick_ik**, que sí lo tiene.
**Ya hecho (sin probar en la simulación):** dos argumentos de `arm_sim.launch.py` que no cambian nada por defecto. `ik:=pick_ik` usa `config/kinematics_pick_ik.yaml` (`ros-humble-pick-ik` agregado al Dockerfile: hay que reconstruir la imagen). `suavizado:=ruckig` agrega el adaptador `AddRuckigTrajectorySmoothing` después de TOTG. La lógica sin ROS está en `moveit_opciones.py` (6 pruebas). Detalle y cómo lanzarlo: `BRAZO.md`, sección MoveIt 2.
**Falta (tú, en el contenedor):** (1) reconstruir la imagen; (2) `ros2 launch andesrobot_arm arm_sim.launch.py ik:=pick_ik` y `prueba_gazebo.sh`: ¿resuelve igual o más poses que KDL, sin tardar mucho más? Si `move_group` se queja de un parámetro, ajustar `kinematics_pick_ik.yaml` con el README de la versión instalada; (3) lo mismo con `suavizado:=ruckig`: ¿los movimientos arrancan y frenan sin golpes?
**Falta (diseño, requiere simulación):** objetos de la mesa como objetos conocidos (hoy el OctoMap los vuelve obstáculos y no se pueden agarrar: habría que publicarlos como `CollisionObject` con permiso de colisión con la pinza y sacarlos del OctoMap); afinar el OctoMap (resolución de `sensors_3d.yaml`; el filtrado del propio robot ya está con `padding_offset`); si la base se llegara a mover, un frame de escena distinto de `base_footprint`.
**Listo cuando:** pick_ik y Ruckig probados (por defecto si mejoran, descartados si no) y documentados en el informe §13 y la Figura C, y los objetos conocidos resueltos o descartados con una razón.
**Dónde mirar:** `BRAZO.md` (MoveIt 2); `andesrobot_moveit_config/config/`; `arm_sim.launch.py`; `arm_moveit_node.py`.

## P09 · Desafío de la puerta: del detector a empujar
**Contexto:** hay una puerta de prueba que se abre empujando y un detector YOLO de la manija entrenado (60 de 60 en validación, simulación contra simulación). Todavía no cierra el lazo con el brazo.
**Ya hecho (sin probar en la simulación):** `andesrobot_arm/manija.py` (geometría sin ROS, 12 pruebas con una puerta sintética): píxel a color → punto 3D con la cámara de profundidad (rayo), plano de la puerta como respaldo cuando la manija brillante hace fallar la profundidad, validación de la detección, pose previa de la pinza y arco que recorre la palanca al bajarla. `manija_node.py` los usa para detectar en vivo y publicar `/manija/agarre`, `/manija/pose_previa` y `/manija/marcadores` (`BRAZO.md`, «Detectar la manija»).
**Falta (tú, con la simulación):** (1) `./sim.sh build` y lanzar `puerta:=true`; (2) `manija_node`: ¿los marcadores caen sobre la manija en RViz?, ¿el error de posición es de pocos mm?; (3) `enviar_previa:=true`: ¿MoveIt llega a la pose previa?
**Falta (código, después de probar 2 y 3):** (a) guiar los últimos centímetros con la imagen a color y los keypoints cuando la profundidad se pierde (< 0.25 m); (b) un nodo que ejecute el movimiento: tocar la palanca, bajarla siguiendo `arco_palanca` y empujar a lo largo de la normal; (c) después, tirar de la puerta y el pestillo.
**Falta (físico):** probar el detector con 30–50 fotos de una puerta real y sumarlas al entrenamiento si hace falta.
**Listo cuando:** el brazo abre la puerta de prueba en simulación sin intervención manual, y queda en el informe §14.
**Dónde mirar:** informe §14 («Detectar la manija» y «Cómo sigue»); `BRAZO.md`; `andesrobot_arm/manija.py` y `manija_node.py`; `scripts/generar_dataset_manija.py`; `scripts/entrenar_manija.py`.

## P10 · Simulación con mando por esfuerzo, PID y gravedad
**Contexto:** con el mando por posición, Gazebo coloca cada articulación en el ángulo pedido como si fuera infinitamente rígida. Agarrar objetos no es realista y un choque mueve la base entera (medido: 3° de inclinación).
**Qué hacer:** interfaz de comando por esfuerzo opcional con un PID por articulación; compensar la gravedad con la postura *actual* (con un par constante la gravedad suma una rigidez negativa, y la pose solo es estable si `kp·I + ∂g/∂q` es definida positiva). Solo aplica a la simulación; el EB300 real usa motores paso a paso en lazo abierto.
**Listo cuando:** una pose se mantiene estable y un choque con la mesa frena el brazo en vez de empujar la base.
**Dónde mirar:** informe §13 («Control en Gazebo»), §21 (prueba de choque) y §15.

## P11 · CI etapa 2: pruebas del brazo con ROS en GitHub Actions
**Ya hecho (etapa 1):** `.github/workflows/ci.yml` corre en cada push a `brazo`/`main` y en cada PR, sin ROS ni Docker: `pytest` de `andesrobot_safety` y de las pruebas del brazo sin ROS (`test_torques`, `test_medidas`, `test_revisar_docs`), `flake8` a 99 columnas en el brazo, y `scripts/revisar_docs.py` (enlaces y anclas rotos, figuras y secciones en orden, `\operatorname` y conteo de pruebas del informe contra el código). Probado: pasa en verde en el PR #3.
**Falta (opcional):** `test_kinematics.py` (11 pruebas) necesita ROS: importa `geometry_msgs` y lee el xacro instalado de `andesrobot_description`. Un job con contenedor `ros:humble` que haga `colcon build` de `andesrobot_description` y `andesrobot_arm` y corra `pytest`. La prueba de humo de Gazebo no entra (necesita la imagen completa de Docker).
**Listo cuando:** el job queda verde en un PR y `CLAUDE.md` lo menciona (hoy dice que `test_kinematics.py` se corre a mano).
**Dónde mirar:** `.github/workflows/ci.yml`; `CLAUDE.md` («Tests» y «CI»).

## P12 · Documentación: artifact, `docs/tecnologias` y referencias
**Qué hacer:**
1. Actualizar el artifact de claude.ai para que coincida con el informe (figuras 8/9/10, sección 15, pick_ik y Ruckig como opciones sin probar, pruebas, fórmulas con `\mathrm`).
2. `docs/tecnologias/README.md` (~línea 409) dice que llegar a cada vista sin chocar «requiere MoveIt 2», que ya está integrado.
3. Verificar las 56 referencias de `docs/literatura` y `docs/tecnologias`: 16 ya verificadas por búsqueda web; faltan 37 y confirmar L12 (Chan–Dubey), L49 (TIAGo) y L50 (Fetch & Freight). Requiere habilitar `api.openalex.org`, `export.arxiv.org`, `arxiv.org` y `doi.org` en Network access del entorno.
4. Skill propio `.claude/skills/actualizar-informe` que, después de cada avance, revise qué documentos quedaron desactualizados (cifras, figuras, pendientes), con las reglas del `CLAUDE.md`.
**Listo cuando:** el artifact coincide con el informe, el punto 2 está corregido y las 56 referencias tienen veredicto.

## P13 · La base resbala en Gazebo (coordinar con el equipo de la base)
**Contexto:** en Gazebo la base retrocede cerca de 1 mm por cada movimiento del brazo, aunque no toque nada (el brazo es pesado y la base liviana). En una corrida de 300 fotos terminó 41 cm atrás y girada 24°, y MoveIt falló en 1 de cada 3 vistas. La física de la base es del otro equipo.
**Qué hacer:** avisarles y pedirles revisar la fricción de las ruedas y el lastre (`ballast_mass`) en la simulación. Mientras tanto, el generador de imágenes baja la aceleración, salta vistas muy cercanas y se detiene a los 15 cm de deriva.
**Listo cuando:** la deriva por movimiento del brazo es despreciable, o el equipo de la base confirma que no se va a cambiar.
**Dónde mirar:** informe §14 («Un hallazgo de la simulación: la base resbala») y §17.
