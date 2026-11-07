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
| P12 | Documentación: terminar de verificar referencias | documentación | — |
| P13 | La base resbala en Gazebo: arreglo aplicado, falta medirlo en la simulación completa | simulación | — |
| P14 | Detector de objetos: reconoce 4 de 26 objetos simulados | brazo, simulación | — |
| P15 | Agarrar objetos de la mesa (plan en 6 fases; fase 1 hecha, fase 2 con la caja) | brazo, simulación | P08 (fase 3) |

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
**Ya hecho:** `andesrobot_arm/manija.py` (geometría sin ROS, 16 pruebas con una puerta sintética): píxel a color → punto 3D con la cámara de profundidad (rayo), plano de la puerta como respaldo cuando la manija brillante hace fallar la profundidad, validación de la detección, pose previa de la pinza y arco que recorre la palanca al bajarla. `manija_node.py` los usa para detectar en vivo y publicar `/manija/agarre`, `/manija/pose_previa` y `/manija/marcadores` (`BRAZO.md`, «Detectar la manija»). Desde el 5-nov también publica la distancia del efector final a la manija (`/manija/distancia`, `/manija/agarre_tcp`) y, con `acercar:=true`, lleva la pinza en dos tramos (pose previa a 30 cm, nueva detección, 5 cm a ciegas); probado sin Gazebo con `scripts/probar_manija_sin_gazebo.py` (queda a 0.050 m, manija a 0.3 mm), que además encontró y arregló el rayo que pasa al lado de la palanca (`corregir_saliente`).
**Probado en Gazebo (7-nov), sin ventanas:** con la cámara ya detrás de la muñeca y los colores corregidos, `acercar:=true` llega a la pose previa y termina diciendo 0.050 m; medido con TF, la pinza quedó a 0.063 m del punto de agarre verdadero (la manija detectada estaba 1.3 cm más cerca). `BRAZO.md`, «Detectar la manija».
**Falta (tú, con la simulación):** mirar en RViz que los marcadores de `/manija/marcadores` caen sobre la manija y averiguar de dónde salen esos 1.3 cm (¿la profundidad en la palanca, el keypoint `agarre`?).
**Falta (código):** (a) guiar los últimos centímetros con la imagen a color y los keypoints cuando la profundidad se pierde (< 0.25 m; hoy el último tramo es a ciegas); (b) ejecutar el movimiento desde donde termina `acercar`: tocar la palanca, bajarla siguiendo `arco_palanca` y empujar a lo largo de la normal; (c) después, tirar de la puerta y el pestillo.
**Falta (físico):** probar el detector con 30–50 fotos de una puerta real y sumarlas al entrenamiento si hace falta.
**Listo cuando:** el brazo abre la puerta de prueba en simulación sin intervención manual, y queda en el informe §14.
**Dónde mirar:** [`prueba_desde_cero.md`](prueba_desde_cero.md) §9 (pose de partida frente a la manija y comandos); informe §14 («Detectar la manija» y «Cómo sigue»); `BRAZO.md`; `andesrobot_arm/manija.py` y `manija_node.py`; `scripts/generar_dataset_manija.py`; `scripts/entrenar_manija.py`.

## P10 · Simulación con mando por esfuerzo, PID y gravedad
**Contexto:** hasta el 7-nov el brazo iba por posición: Gazebo colocaba cada articulación en el ángulo pedido como si fuera infinitamente rígida, un objeto agarrado no subía con la pinza y un choque movía la base entera (medido: 3° de inclinación). Ahora va por velocidad (P15, fase 2), que levanta objetos, pero Gazebo impone la velocidad sin límite de fuerza: un choque probablemente siga sin frenar el brazo (no medido).
**Qué hacer:** interfaz de comando por esfuerzo opcional con un PID por articulación; compensar la gravedad con la postura *actual* (con un par constante la gravedad suma una rigidez negativa, y la pose solo es estable si `kp·I + ∂g/∂q` es definida positiva). Solo aplica a la simulación; el EB300 real usa motores paso a paso en lazo abierto.
**Ya hecho (7-nov-2026):** la pinza ya va por esfuerzo con un PID (P15, fase 1); sirve de plantilla: `gains` en `arm_controllers.yaml`, tope de fuerza = `effort` del `<limit>` y amortiguamiento implícito en el URDF. El brazo pasó de posición a velocidad con un P (`ff_velocity_scale` 1; P15, fase 2): el paso a esfuerzo solo cambia la interfaz y las ganancias.
**Listo cuando:** una pose se mantiene estable y un choque con la mesa frena el brazo en vez de empujar la base.
**Dónde mirar:** informe §13 («Control en Gazebo»), §21 (prueba de choque) y §15.

## P11 · CI etapa 2: pruebas del brazo con ROS en GitHub Actions
**Ya hecho (etapa 1):** `.github/workflows/ci.yml` corre en cada push a `brazo`/`main` y en cada PR, sin ROS ni Docker: `pytest` de `andesrobot_safety` y de las pruebas del brazo sin ROS (`test_torques`, `test_medidas`, `test_revisar_docs`), `flake8` a 99 columnas en el brazo, y `scripts/revisar_docs.py` (enlaces y anclas rotos, figuras y secciones en orden, `\operatorname` y conteo de pruebas del informe contra el código). Probado: pasa en verde en el PR #3.
**Falta (opcional):** `test_kinematics.py` (11 pruebas) necesita ROS: importa `geometry_msgs` y lee el xacro instalado de `andesrobot_description`. Un job con contenedor `ros:humble` que haga `colcon build` de `andesrobot_description` y `andesrobot_arm` y corra `pytest`. La prueba de humo de Gazebo no entra (necesita la imagen completa de Docker).
**Listo cuando:** el job queda verde en un PR y `CLAUDE.md` lo menciona (hoy dice que `test_kinematics.py` se corre a mano).
**Dónde mirar:** `.github/workflows/ci.yml`; `CLAUDE.md` («Tests» y «CI»).

## P12 · Documentación: terminar de verificar referencias
**Ya hecho (5-nov-2026):** `docs/tecnologias` (§11) ya no dice que MoveIt está pendiente. Referencias: 27 de 56 confirmadas por búsqueda web (el detalle está al inicio de `docs/literatura` §13). Skill del proyecto `.claude/skills/actualizar-informe`: qué documento toca cada cambio y los mismos chequeos del CI. Artifact «Informe técnico de Milo» (claude.ai): decidido no sincronizarlo; queda congelado en el 19-oct con un aviso arriba que lleva a `docs/informe/README.md` en GitHub.
**Falta:** **Referencias:** comprobar los clásicos y las páginas web, y encontrar el registro de 49 y 50. Para hacerlo con la herramienta automática hay que habilitar `api.openalex.org`, `export.arxiv.org`, `arxiv.org` y `doi.org` en Network access del entorno.
**Listo cuando:** las 56 referencias tienen veredicto.
**Dónde mirar:** `docs/literatura/README.md` §13; `docs/tecnologias/README.md` («Referencias de esta sección»); `.claude/skills/actualizar-informe/SKILL.md`.

## P13 · La base resbala en Gazebo: arreglo aplicado, falta medirlo en la simulación completa
**Contexto:** en Gazebo la base retrocedía cerca de 1 mm por cada movimiento del brazo, aunque no tocara nada (300 fotos: 41 cm y 24°, MoveIt fallaba 1 de cada 3 vistas).
**Ya hecho (5-nov-2026):** causa medida con `andesrobot_arm/scripts/deriva_base/`: las ruedas **ruedan**, no patinan; el solver de ODE (quick, 50 iteraciones) no mantiene en 0 el motor de velocidad de las ruedas bajo el peso del brazo. Con 200 iteraciones la deriva pasa de 10.2 mm a 0.3 mm en 30 movimientos (física ~2 veces más lenta). Aplicado en `andesrobot_arena.world` y en `tools/gen_casa_robocup.py` (+ `robocup_casa.world` regenerado); los dos pasan `gz sdf -k`.
**Falta:** en el contenedor, medir el factor de tiempo real de `./sim.sh brazo` (cámara, MoveIt) y comprobar que la base maneja igual en `./sim.sh` y `./sim.sh nav` (avisar al equipo de la base). Correr `generar_dataset_manija.py` largo y, si ya no deriva, quitar las defensas que sobren (`--deriva_max`, aceleración 0.2).
**Listo cuando:** una corrida de 300 fotos no se detiene por deriva y la simulación sigue cerca del tiempo real.
**Dónde mirar:** `andesrobot_arm/scripts/deriva_base/README.md`; informe §14 («Un hallazgo de la simulación: la base resbala»).

## P14 · Detector de objetos: reconoce 4 de 26 objetos simulados
**Contexto:** el modelo YOLO de objetos del reglamento (`ros2_ws/modelos/objetos/best.pt`, YOLO11n, 27 clases, entrenado con fotos reales) y `objeto_node` llevan la pinza frente a un objeto (probado con la taza: queda a 0.103 m). La mesa simulada tiene 26 objetos escaneados de 23 clases (`config/objetos.yaml`; sin modelo: colgate_box, key, redbull_can, drink).
**Ya hecho (7-nov-2026):** `scripts/evaluar_objetos.py` midió cada objeto solo, desde 3 vistas: el modelo reconoce las dos tazas, el tenedor y (apenas, 0.13) el juego de cartas; a los otros 22 no los ve, o los confunde (mochila → clothing, sopa → drink), y ve la mesa desde arriba como `card` (0.4–0.9). Las fotos se ven bien: el límite es del modelo con imágenes de simulación.
**Qué hacer:** como con la manija, generar fotos etiquetadas de los objetos simulados (variando vista, distancia, luz y objetos juntos; la caja sale de la posición conocida del objeto + TF, como en `generar_dataset_manija.py`) y afinar el modelo sumándolas a `dataset_reglamento2`, sin perder lo que ya reconoce en fotos reales. Agregar fotos de mesa vacía como fondo, para que deje de verla como `card`. Opcional: modelos para las 4 clases que faltan (caja y lata con foto real como textura).
**Listo cuando:** `evaluar_objetos.py` reconoce la mayoría de las 23 clases, sin que baje el resultado con fotos reales, y `objeto_node` lleva la pinza frente a objetos de varias clases.
**Dónde mirar:** `BRAZO.md` («Objetos sobre la mesa»); `scripts/evaluar_objetos.py`; `scripts/generar_dataset_manija.py` (de plantilla); el CSV y las fotos en `ros2_ws/resultados/objetos/` y `ros2_ws/capturas/evaluacion_*`.

## P15 · Agarrar objetos de la mesa (plan en 6 fases)
**Contexto:** `objeto_node` ya deja la pinza a 0.10 m de un objeto, pero no lo agarra. La pinza EBG-20 abre de 3 a 49 mm, así que casi ningún objeto del catálogo cabe entero: se agarran cubiertos, plumón y plátano enteros; tazas, jarro, bowls, plato, bandeja y martillo por el asa, el borde o el mango; botella, tarro, sopa, manzana, Rubik, cereal, cloro, libro y caja de azúcar no caben (medidas en `milo.json` de cada modelo). En la pinza real el servo empuja con su fuerza máxima y se frena contra el objeto, que es más rígido; en Gazebo los dedos van por **posición**, sin límite de fuerza, así que atraviesan el objeto o lo expulsan.
**Ya hecho (7-nov-2026), fase 1:** dedos mandados por fuerza con tope de 20 N (`gripper_force`, `gripper_damping` en el URDF; piñón medido en el CAD: radio de paso ≈ 7 mm, así que el MG99x daría ≈ 65 N por dedo como máximo), PID en `gripper_controller` y contacto blando en los dedos (`andesrobot.arm_control.xacro`). Cada dedo tiene su PID y se mandan los dos en espejo (`[q, −q]`): con el izquierdo como `mimic` no volvía a abrir del todo. `scripts/prueba_pinza.py`: contra cajas fijas de 10 a 45 mm los dedos quedan a 0.05 mm del ancho sin temblar (20 de 20 cierres); prueba de humo completa en verde.
**Qué hacer, en orden** (cada fase tiene su «listo cuando»):
1. ~~**Pinza como la real:** dedos mandados por esfuerzo, con un PID cuya salida no pasa la fuerza de la pinza (F ≈ τ / (2·r) por dedo; τ del servo MG99x ≈ 1 N·m, r = radio del piñón, a medir en el CAD; en simulación 10–20 N) y fricción alta en las caras de los dedos. Revisar si el `mimic` del dedo izquierdo respeta la fuerza. *Listo cuando* al cerrar sobre una caja de 3 cm fija a la mesa los dedos se frenan a 3 cm ± 1 mm sin atravesarla ni vibrar.~~ Hecho.
2. **Objeto agarrable:** ~~una caja suelta de 3 cm~~ (hecho, ver abajo) y después el plumón o el tenedor, con colisión de piezas convexas (V-HACD / CoACD, ninguno está en la imagen) en vez de la caja que lo encierra. *Listo cuando* se cierra, levanta y sostiene 10 s sin resbalar (`scripts/prueba_levantar.py` sirve de base: cambiar la caja por el modelo).
   **Ya hecho (7-nov-2026), la caja:** `scripts/prueba_levantar.py` (caja suelta de 30 mm sobre un pedestal, cerrar, subir el lift 5 cm, sostener 10 s). Con el brazo por posición la caja no subía (0 mm de 50: `gazebo_ros2_control` pone las articulaciones de un salto, sin velocidad, y el roce no arrastra): el brazo pasó a **velocidad** con un P (`arm_controllers.yaml`), prueba de humo en verde con y sin MoveIt. Además: mandar cada dedo 2 mm más adentro del ancho, no cerrar del todo (si no, nada centra la caja y se va de lado), y contacto de los dedos kp 1e6 / kd 300. De 0.1 a 0.5 kg se corre 0.05–0.4 mm en 10 s (8 de 8); con 1 kg resbala 2–3 mm.
3. **MoveIt sabe del objeto** (P08): `CollisionObject` en vez de OctoMap, choque permitido con los dedos y objeto unido a la pinza al levantar. *Listo cuando* MoveIt planifica el acercamiento y, ya con el objeto, lo esquiva al moverse.
4. **Agarre con la pose verdadera de Gazebo** (sin cámara): pose previa, bajada en línea recta (`computeCartesianPath`), cerrar, comprobar que el dedo se frenó antes de cerrar del todo, unir y levantar. *Listo cuando* se anota cuántos de 10 intentos (posiciones distintas) terminan con el objeto levantado.
5. **Agarre con la cámara:** caja de YOLO + profundidad → puntos del objeto sin la mesa → PCA (eje largo y ancho) → agarre desde arriba si el ancho < 45 mm. *Listo cuando* se mide la tasa de éxito en varias posiciones y se compara con la fase 4.
6. **Asas y bordes** con keypoints, como la manija (generador de `generar_dataset_manija.py`); después comparar con un detector de agarres aprendido (Contact-GraspNet, GraspGen).
**Robot real:** el MG99x es un servo hobby sin lectura de posición ni de fuerza, así que «el dedo se frenó» no se puede leer; habría que verlo con la cámara o un sensor. Trabado contra el objeto el servo gasta corriente y se calienta: mandarlo a un poco menos que el ancho del objeto, no a cerrar del todo.
**Listo cuando:** el brazo levanta en simulación un objeto detectado con la cámara, sin intervención manual, y queda en el informe §14.
**Dónde mirar:** `docs/literatura` §9; `docs/tecnologias` («Detección y agarre»); informe §13 («Control en Gazebo»); `scripts/prueba_pinza.py`; `andesrobot.arm_control.xacro`; `config/arm_controllers.yaml`; `andesrobot_arm/objetos.py` y `objeto_node.py`; P08 y P10.
