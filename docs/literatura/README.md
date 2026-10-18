# Repaso de literatura: metodología para el brazo de Milo

Qué dice la literatura de robótica sobre cada paso que falta para llevar el brazo de Milo de la
simulación al robot real, qué de eso ya se usa en este repositorio y qué conviene aplicar después.
Se limita al **brazo, el lift y la pinza** (la base móvil la desarrolla otro equipo).

> **En palabras simples:** el brazo ya funciona en el simulador: sabe calcular cómo mover cada
> motor para llevar la pinza a un punto. Para que funcione en el robot real faltan varias cosas:
> que no choque consigo mismo ni con la base, que los motores reales se muevan suave, que la
> cámara encuentre el objeto, medir el brazo real para corregir el modelo, y armar la electrónica.
> Para cada una de esas tareas hay métodos conocidos y probados; este documento los resume, dice
> cuáles se usan en robots parecidos a Milo y recomienda cuál aplicar.

**Cómo leerlo.** Cada sección tiene la misma forma: qué problema resuelve (en palabras simples),
qué métodos existen, qué hace Milo hoy y qué se recomienda. Las citas entre corchetes, como [12],
están en la lista de referencias del final. Los términos técnicos están explicados en el
[glosario del informe](../informe/README.md#glosario).

## Contenido

1. [Dónde está Milo hoy](#1-dónde-está-milo-hoy)
2. [Modelo cinemático: DH, URDF y productos de exponenciales](#2-modelo-cinemático-dh-urdf-y-productos-de-exponenciales)
3. [Cinemática inversa y redundancia](#3-cinemática-inversa-y-redundancia)
4. [Planificación de movimiento y choques](#4-planificación-de-movimiento-y-choques)
5. [Trayectorias para motores paso a paso](#5-trayectorias-para-motores-paso-a-paso)
6. [Control del brazo real: Arduino Mega, TB6600 y ros2_control](#6-control-del-brazo-real-arduino-mega-tb6600-y-ros2_control)
7. [IMU en la pinza (MPU-9250)](#7-imu-en-la-pinza-mpu-9250)
8. [Calibración: modelo cinemático y cámara–pinza](#8-calibración-modelo-cinemático-y-cámarapinza)
9. [Percepción y agarre con la cámara de profundidad](#9-percepción-y-agarre-con-la-cámara-de-profundidad)
10. [Simulación y paso a lo real](#10-simulación-y-paso-a-lo-real)
11. [Robots parecidos a Milo](#11-robots-parecidos-a-milo)
12. [Metodología propuesta, por fases](#12-metodología-propuesta-por-fases)
13. [Referencias](#13-referencias)
14. [Artículos descargados](#14-artículos-descargados)

---

## 1. Dónde está Milo hoy

| Tarea | Estado en `milo_ws` | Sección |
| --- | --- | --- |
| Modelo del brazo (URDF) con masas corregidas | hecho; límites de las articulaciones provisorios (±180°) | 2, 8 |
| Cinemática directa, jacobiano e inversa con el lift (7 ejes) | hecho y probado, solo en simulación | 3 |
| Evitar choques del brazo con la base, la columna o la mesa | hecho con MoveIt 2 (18-10-2026): escena con el robot, la mesa y el OctoMap de la cámara | 4 |
| Trayectorias suaves (aceleración limitada) | hecho (TOTG en MoveIt); falta limitar el tirón (Ruckig) | 5 |
| Electrónica y driver del brazo real | **no hay**: piezas impresas sin montar, falta el plano eléctrico | 6 |
| IMU MPU-9250 debajo de la cámara | por agregar al modelo y al montaje | 7 |
| Medir el brazo real y calibrar | pendiente (la muñeca del modelo no coincide del todo con el DH oficial) | 8 |
| Cámara de profundidad en la pinza | simulada, ideal (sin ruido); la real no está conectada | 9 |
| Detectar el objeto y decidir cómo agarrarlo | no hay | 9 |

---

## 2. Modelo cinemático: DH, URDF y productos de exponenciales

> **En palabras simples:** para hacer cuentas con el brazo hay que describir con números dónde
> está cada motor y hacia dónde gira. Hay varias "recetas" para escribir esa descripción.

**Qué dice la literatura.**
- **Denavit–Hartenberg (DH)** [1] describe cada articulación con 4 números. Es la convención
  clásica de los libros [2, 3] y la que usa el fabricante del EB300. Obliga a poner los ejes de
  cada pieza de una forma particular, y hay dos variantes (la original y la "modificada" de
  Craig [3]) que se confunden con facilidad.
- **Productos de exponenciales (PoE)** [4, 5] describe cada articulación por su eje y un punto,
  medidos en una sola postura de referencia. No exige elegir ejes por pieza y es la base de los
  libros modernos [5].
- **URDF** es el formato de ROS: cada unión tiene un desplazamiento fijo (`origin`) y un eje
  (`axis`). Matemáticamente es una cadena de transformaciones como PoE, y es lo que leen el
  simulador, RViz y los planificadores de ROS 2 [15, 16].

**Qué hace Milo.** Lee el URDF directo (`kinematics.py`) y no mantiene una tabla DH aparte: así,
si se corrige una medida en el modelo, todo se actualiza solo. Es lo recomendado cuando el resto
del sistema ya usa ROS.

**Recomendación.** Mantener el URDF como única fuente. Usar la tabla DH del fabricante **solo para
comparar** (ya está en la sección 5 del informe) y para la calibración (sección 8).

---

## 3. Cinemática inversa y redundancia

> **En palabras simples:** la cinemática inversa responde "¿qué ángulo pongo en cada motor para
> que la pinza llegue a este punto?". Para algunos brazos hay una fórmula directa; para otros hay
> que buscar la respuesta de a poco.

**Qué dice la literatura.**
- **Analítica (fórmula cerrada).** Existe cuando la muñeca es "esférica" (los tres últimos ejes se
  cruzan en un punto) [2]. IKFast la genera de forma automática para muchos brazos [6]. En el EB300
  **no** se cumple (joint_4 y joint_6 son paralelos), así que no hay fórmula cerrada general.
- **Numérica con el jacobiano.** Se itera con la inversa del jacobiano: pseudoinversa [7], y su
  versión amortiguada (*damped least squares*, DLS) [8, 9], que no se descontrola cerca de las
  singularidades. Buss [10] compara estos métodos de forma muy didáctica.
- **Solvers generales.** KDL (el clásico de ROS) falla cerca de los límites de las articulaciones;
  **TRAC-IK** [11] corre en paralelo un Newton mejorado y una optimización (SQP) y resuelve
  bastante más casos en el mismo tiempo. Es el que se suele usar con MoveIt.
- **Redundancia (más motores que lo necesario).** Con 7 ejes para 6 números de una pose hay
  infinitas soluciones. Se elige una con pesos por articulación (*weighted least norm*) [12], o con
  objetivos secundarios en el "espacio nulo" (alejarse de los límites, de las singularidades) [13].
  La **manipulabilidad** de Yoshikawa [14] mide qué tan lejos de una singularidad está una postura.

**Qué hace Milo.** DLS con pesos (el lift pesa 10, los demás 1) y reinicios al azar, en
`kinematics.py` (`moveit:=false`), y KDL dentro de MoveIt 2 (el default desde el 18-10-2026).
Coincide con lo recomendado en [8–10, 12] para una muñeca no esférica.

**Recomendación.**
- Seguir con la IK propia para pruebas y para la página interactiva.
- Para el robot real, usar **TRAC-IK** dentro de MoveIt 2 (sección 4), que ya resuelve el mismo
  problema y además respeta los choques.
- Agregar un objetivo secundario que aleje las articulaciones de sus límites cuando se conozcan
  los topes reales; hoy los límites son ±180° provisorios.

---

## 4. Planificación de movimiento y choques

> **En palabras simples:** saber la postura final no basta: hay que llegar ahí sin chocar con la
> base, la columna, la mesa o el propio brazo. Planificar es buscar un camino libre de choques.

**Qué dice la literatura.**
- **Métodos por muestreo:** prueban posturas al azar y las conectan si el tramo entre ellas no
  choca. PRM [17], RRT [18] y RRT-Connect [19] son los más usados; la biblioteca **OMPL** [20] los
  reúne.
- **Métodos por optimización:** parten de un camino y lo deforman para alejarlo de los obstáculos
  y suavizarlo: CHOMP [21], STOMP [22] y TrajOpt [23].
- **MoveIt 2** [16] junta todo: lee el URDF, revisa choques con las mallas, usa OMPL o los
  optimizadores, llama al solver de IK (KDL o TRAC-IK) y manda la trayectoria al controlador por
  la misma acción `FollowJointTrajectory` que ya usa Milo.

**Qué hace Milo.** Desde el 18-10-2026 planifica con MoveIt 2 (`andesrobot_moveit_config`):
OMPL/RRT-Connect sobre una escena con el robot, la mesa y el OctoMap de la cámara; una pose
dentro de la mesa se rechaza. La IK propia (`moveit:=false`) sigue sin revisar choques.

**Recomendación.** Hecho: MoveIt 2 con el mismo URDF, el grupo de 7 ejes (lift + joint_1…6) y la
pinza como herramienta. Siguientes pasos: afinar el OctoMap (resolución, filtrado del propio
robot), agregar los objetos de la mesa como objetos conocidos para poder agarrarlos, y TRAC-IK.
El nodo `arm_ik_node` queda como alternativa simple (`moveit:=false`).

---

## 5. Trayectorias para motores paso a paso

> **En palabras simples:** un motor paso a paso que arranca o frena de golpe se salta pasos y
> queda en una posición distinta a la que cree el programa. Hay que arrancar y frenar con rampas.

**Qué dice la literatura.**
- **Parametrización en el tiempo:** dado un camino, calcular la velocidad en cada punto respetando
  límites de velocidad y aceleración. TOTG [24] es el que usa MoveIt.
- **Límite de tirón (*jerk*):** además de la aceleración, limita qué tan rápido cambia. **Ruckig**
  [25] lo hace en tiempo real (microsegundos) y está integrado en MoveIt 2 y ros2_control.

**Qué hace Milo.** Con MoveIt, TOTG pone rampas de aceleración (`joint_limits.yaml`, al 50 %).
`arm_ik_node` (`moveit:=false`) usa una duración común con velocidad constante, que en motores
reales provocaría pasos perdidos.

**Recomendación.** Usar la parametrización con aceleración limitada de MoveIt (TOTG o Ruckig) y,
en el microcontrolador, rampas de aceleración por motor (sección 6). Medir la aceleración máxima
de cada eje con carga antes de fijar los límites.

---

## 6. Control del brazo real: Arduino Mega, TB6600 y ros2_control

> **En palabras simples:** el computador decide los ángulos; un Arduino genera los pulsos para
> los 6 drivers TB6600, y cada driver mueve su motor. Hace falta un "traductor" entre ROS y el
> Arduino, y un plano eléctrico de todas las conexiones.

**Qué dice la literatura y la práctica.**
- **ros2_control** [26] separa el controlador (que ya usa Milo en simulación) del hardware. Para el
  brazo real basta escribir un **hardware interface** que mande posiciones y lea el estado: el
  mismo esquema que ya tiene el driver del hoverboard de la base.
- **micro-ROS** [27] permite que un microcontrolador hable ROS 2 directamente, pero necesita
  microcontroladores de 32 bits con decenas de KB de RAM. El **Arduino Mega 2560 (8 bits, 8 KB de
  RAM) no está soportado** [27, 28]. Con el Mega lo práctico es un protocolo serie simple propio.
- **Lazo abierto vs. cerrado:** los motores paso a paso con TB6600 funcionan en lazo abierto (no
  miden su posición). Si se salta un paso, nadie se entera. Las soluciones habituales son
  encoders magnéticos por eje (o motores paso a paso con encoder integrado), o al menos un
  **homing** con finales de carrera al encender, para saber dónde está cada eje.

**Qué hace Milo.** Nada todavía en el robot real: el control existe solo en Gazebo.

**Recomendación (en orden).**
1. **Plano eléctrico** antes de cablear: fuente de 24 V 15 A, 6 TB6600 (STEP/DIR/ENABLE a pines
   del Mega), motores, finales de carrera, la IMU (sección 7) y masas comunes. Revisar la corriente
   de cada driver contra la del motor (4.0 A los NEMA23 de 82 mm, 2.1 A los NEMA17).
2. **Firmware del Mega** con rampas por motor (por ejemplo, la biblioteca AccelStepper), homing al
   encender y un protocolo serie: "ir a estas 6 posiciones" y "¿dónde estás?".
3. **Hardware interface de ros2_control** que hable ese protocolo, como el del hoverboard. Así el
   mismo `arm_controller` y el mismo `arm_ik_node` (o MoveIt) sirven sin cambios.
4. Si se quiere ROS 2 en el microcontrolador, cambiar a una placa de 32 bits soportada por
   micro-ROS (ESP32, Teensy, STM32) en vez del Mega.

---

## 7. IMU en la pinza (MPU-9250)

> **En palabras simples:** una IMU mide aceleración (incluida la gravedad), giro y, si tiene
> magnetómetro, el campo magnético. Puesta en la pinza, dice hacia dónde apunta la pinza "de
> verdad", independiente de lo que crean los motores.

**Qué dice la literatura.**
- **Fusión de sensores:** el giróscopo es preciso a corto plazo pero se desvía con el tiempo; el
  acelerómetro da la vertical (gravedad) pero es ruidoso. Los filtros de **Madgwick** [29] y
  **Mahony** [30] los combinan y dan la orientación; ROS tiene ambos (`imu_filter_madgwick`,
  `imu_complementary_filter`).
- **Ángulos de articulaciones con IMUs:** comparando la orientación medida por la IMU con la que
  predice el modelo se estiman ángulos y se detectan diferencias, como se hace en análisis de la
  marcha [31].
- **Magnetómetro cerca de motores:** los imanes de los motores paso a paso y las corrientes de los
  cables distorsionan el campo magnético; en esa posición conviene usar la IMU en modo de 6 ejes
  (sin magnetómetro).

**Para qué serviría en Milo.**
- **Detectar pasos perdidos:** si la inclinación de la pinza que mide la IMU no coincide con la de
  la cinemática directa, algún motor se saltó pasos (sección 6).
- **Corregir la orientación de la cámara** que está justo encima (calibración, sección 8).
- **Ver vibraciones** del brazo al frenar, para ajustar las rampas (sección 5).

**Precauciones.** El MPU-9250 está discontinuado por el fabricante desde 2018; su reemplazo es el
**ICM-20948** [32], parecido pero no compatible en registros. Funciona a 3.3 V: con el Mega (5 V)
hace falta un adaptador de nivel en la conexión I²C. Hay que agregarlo al URDF (un link fijo bajo
la cámara con su masa) y, en Gazebo, un sensor IMU con el plugin `libgazebo_ros_imu_sensor`, como
el de la base.

---

## 8. Calibración: modelo cinemático y cámara–pinza

> **En palabras simples:** el modelo dice que la pinza está en un punto; el brazo real, por
> tolerancias de impresión y montaje, queda unos milímetros (o centímetros) al lado. Calibrar es
> medir el robot real y corregir el modelo.

**Qué dice la literatura.**
- **Calibración cinemática:** se mide la posición real de la pinza en muchas posturas y se
  ajustan las medidas del modelo para que coincidan (mínimos cuadrados sobre los parámetros) [33].
- **Calibración mano–ojo (*hand-eye*):** encontrar dónde está exactamente la cámara respecto de la
  pinza, moviendo el brazo y mirando un patrón fijo. Los métodos clásicos son Tsai–Lenz [34] y
  Park–Martin [35]; ROS 2 tiene paquetes que los implementan (por ejemplo, `easy_handeye2`).
- **Cómo medir el desempeño:** la norma **ISO 9283** [36] define exactitud (qué tan cerca llega
  del punto pedido) y repetibilidad (qué tan cerca llega de sí mismo al repetir), con un
  procedimiento estándar de poses de prueba.

**Qué hace Milo.** Las medidas salen del CAD; la muñeca difiere 10–32 mm del DH oficial (informe,
sección 5). No hay calibración.

**Recomendación.**
1. Al montar, **medir** las distancias entre ejes con calibre y los topes de cada articulación;
   decidir entre el CAD y el DH oficial y fijar los límites reales en el xacro.
2. Medir **repetibilidad y exactitud** con un procedimiento simplificado de ISO 9283 (5 poses, 30
   repeticiones), por ejemplo con la cámara de la pinza y un patrón ArUco.
3. Hacer la **calibración mano–ojo** de la Gemini Plus y guardar la transformación en el xacro de
   la cámara.

---

## 9. Percepción y agarre con la cámara de profundidad

> **En palabras simples:** antes de agarrar algo hay que encontrarlo en la imagen, saber dónde
> está en 3D y decidir por dónde tomarlo con la pinza.

**Qué dice la literatura.**
- **Error de las cámaras de profundidad:** crece con el cuadrado de la distancia y hay huecos en
  bordes y superficies oscuras o brillantes [37, 38]. Es lo que modela la sección 14 del informe.
- **Encontrar el objeto:** detectores 2D como YOLO [39] sobre la imagen a color, y la profundidad
  para pasar a 3D. Para objetos conocidos, estimadores de pose 6D como FoundationPose [40].
- **Decidir el agarre:** redes que proponen agarres para pinzas de dos dedos a partir de la nube
  de puntos: GPD [41], Dex-Net 2.0 [42], Contact-GraspNet [43] y AnyGrasp [44], entrenadas en
  bases de datos como GraspNet-1Billion [45].
- **Mirar y luego moverse vs. servo visual:** "mirar y luego moverse" (*look-then-move*) mide una
  vez y se acerca sin ver; el servo visual [46] corrige el movimiento con la imagen en cada
  instante.

**Qué hace Milo.** La cámara está simulada y entrega color, profundidad, IR y nube. No hay
detección. Su profundidad no mide a menos de 0.25 m, así que no ve el agarre final.

**Recomendación.** Empezar por **mirar y luego moverse**, que encaja con el rango mínimo de la
cámara: medir desde 0.3–0.6 m, detectar con YOLO (o por color para la mesa de prueba), sacar la
posición 3D de la nube y mandarla a `/arm_target_pose`. Los métodos de agarre aprendidos [43, 44]
son el paso siguiente, cuando la detección simple funcione.

---

## 10. Simulación y paso a lo real

> **En palabras simples:** lo que funciona en el simulador no siempre funciona en el robot,
> porque el simulador es "demasiado perfecto". Para reducir esa diferencia se hace el simulador
> más realista o más variado.

**Qué dice la literatura.** La **aleatorización del dominio** [47] varía en cada prueba los
colores, la luz, las posiciones y el ruido de los sensores, para que lo aprendido no dependa de
una escena en particular. Simuladores más nuevos (Isaac Sim, Gazebo moderno) traen modelos de
ruido de cámaras.

**Qué hace Milo.** Gazebo Classic con mando por posición ideal y cámara sin ruido.

**Recomendación.** Implementar el nodo de profundidad realista propuesto en la sección 14 del
informe y aleatorizar la mesa de prueba (posición, color y tamaño de los objetos) en cada
lanzamiento.

---

## 11. Robots parecidos a Milo

| Robot | En qué se parece | Qué se puede tomar |
| --- | --- | --- |
| **Hello Robot Stretch** [48] | base diferencial chica + **lift en un mástil** + brazo; pensado para casas | su análisis de estabilidad estática y la idea de usar el lift como eje principal de altura |
| **PAL TIAGo** [49] | base móvil + **torso que sube y baja** + brazo de 7 ejes | su configuración de MoveIt con el torso como un eje más del grupo |
| **Fetch** [50] | base móvil + torso prismático + brazo de 7 ejes | igual que TIAGo; documentado y con soporte en ROS |

La revisión de Thakar et al. [51] clasifica los manipuladores móviles con ruedas y cómo coordinan
la base, el torso y el brazo; sirve de mapa general.

---

## 12. Metodología propuesta, por fases

Cada fase termina con una prueba que se puede medir.

| Fase | Qué hacer | Cómo se comprueba | Secciones |
| --- | --- | --- | --- |
| 1. Montaje y medición | armar el brazo con las piezas impresas, medir distancias entre ejes y topes, pesar las piezas | URDF actualizado; diferencia CAD/real documentada | 8 |
| 2. Electrónica | plano eléctrico (Mega, 6 TB6600, motores, finales de carrera, IMU), cableado | cada motor gira y hace homing desde el Mega | 6, 7 |
| 3. Firmware y driver | rampas por motor, protocolo serie, hardware interface de ros2_control | `arm_controller` mueve el brazo real con la misma acción que en Gazebo | 5, 6 |
| 4. IMU | agregarla al URDF y a Gazebo, filtro de Madgwick, comparar con la cinemática directa | detecta un paso perdido provocado a propósito | 7 |
| 5. Choques | ✅ hecho (18-10-2026): MoveIt 2 con el grupo de 7 ejes y KDL; falta TRAC-IK | una pose dentro de la mesa se rechaza (prueba de humo) | 3, 4 |
| 6. Calibración | cinemática y mano–ojo | exactitud y repetibilidad según ISO 9283 simplificada | 8 |
| 7. Percepción | mirar y luego moverse con detección simple | agarra los objetos de la mesa de prueba desde distintas posiciones | 9 |
| 8. Realismo | profundidad realista y escena aleatoria en Gazebo | el mismo detector funciona en simulación y en el robot | 10 |

---

## 13. Referencias

**Libros y modelado**

1. J. Denavit, R. S. Hartenberg, "A kinematic notation for lower-pair mechanisms based on matrices", *ASME J. Applied Mechanics*, 1955.
2. B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009.
3. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3.ª ed., Pearson, 2005.
4. R. W. Brockett, "Robotic manipulators and the product of exponentials formula", *Mathematical Theory of Networks and Systems*, 1984.
5. K. M. Lynch, F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017.

**Cinemática inversa**

6. R. Diankov, *Automated Construction of Robotic Manipulation Programs* (IKFast), tesis doctoral, Carnegie Mellon University, 2010.
7. D. E. Whitney, "Resolved motion rate control of manipulators and human prostheses", *IEEE Trans. Man-Machine Systems*, 1969.
8. Y. Nakamura, H. Hanafusa, "Inverse kinematic solutions with singularity robustness for robot manipulator control", *ASME J. Dynamic Systems, Measurement, and Control*, 1986.
9. C. W. Wampler, "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods", *IEEE Trans. Systems, Man, and Cybernetics*, 1986.
10. S. R. Buss, "Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods", reporte técnico, UC San Diego, 2004.
11. P. Beeson, B. Ames, "TRAC-IK: An open-source library for improved solving of generic inverse kinematics", *IEEE-RAS Humanoids*, 2015. [ROS](https://index.ros.org/p/trac_ik/)
12. T. F. Chan, R. V. Dubey, "A weighted least-norm solution based scheme for avoiding joint limits for redundant joint manipulators", *IEEE Trans. Robotics and Automation*, 1995.
13. B. Siciliano, J.-J. Slotine, "A general framework for managing multiple tasks in highly redundant robotic systems", *ICAR*, 1991.
14. T. Yoshikawa, "Manipulability of robotic mechanisms", *Int. J. Robotics Research*, 1985.

**Software de ROS**

15. S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild", *Science Robotics*, 2022.
16. D. Coleman, I. Sucan, S. Chitta, N. Correll, "Reducing the barrier to entry of complex robotic software: a MoveIt! case study", *J. Software Engineering for Robotics*, 2014. MoveIt 2: [moveit.ai](https://moveit.ai)

**Planificación y trayectorias**

17. L. E. Kavraki, P. Švestka, J.-C. Latombe, M. H. Overmars, "Probabilistic roadmaps for path planning in high-dimensional configuration spaces", *IEEE Trans. Robotics and Automation*, 1996.
18. S. M. LaValle, "Rapidly-exploring random trees: A new tool for path planning", reporte técnico, Iowa State University, 1998.
19. J. J. Kuffner, S. M. LaValle, "RRT-Connect: An efficient approach to single-query path planning", *ICRA*, 2000.
20. I. A. Sucan, M. Moll, L. E. Kavraki, "The Open Motion Planning Library", *IEEE Robotics & Automation Magazine*, 2012.
21. N. Ratliff, M. Zucker, J. A. Bagnell, S. Srinivasa, "CHOMP: Gradient optimization techniques for efficient motion planning", *ICRA*, 2009.
22. M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor, S. Schaal, "STOMP: Stochastic trajectory optimization for motion planning", *ICRA*, 2011.
23. J. Schulman et al., "Motion planning with sequential convex optimization and convex collision checking" (TrajOpt), *Int. J. Robotics Research*, 2014.
24. T. Kunz, M. Stilman, "Time-optimal trajectory generation for path following with bounded acceleration and velocity", *RSS*, 2012.
25. L. Berscheid, T. Kröger, "Jerk-limited real-time trajectory generation with arbitrary target states" (Ruckig), *RSS*, 2021. [arXiv:2105.04830](https://arxiv.org/abs/2105.04830)

**Control y hardware**

26. S. Chitta et al., "ros_control: A generic and simple control framework for ROS", *J. Open Source Software*, 2017.
27. micro-ROS, "Supported hardware". [micro.ros.org](https://micro.ros.org/docs/overview/hardware/)
28. Foro de ROS, "micro-ROS on Arduino?" (sin soporte para microcontroladores de 8 bits). [discourse.ros.org](https://discourse.ros.org/t/micro-ros-on-arduino/17009/3)

**IMU**

29. S. O. H. Madgwick, A. J. L. Harrison, R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm", *IEEE ICORR*, 2011.
30. R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear complementary filters on the special orthogonal group", *IEEE Trans. Automatic Control*, 2008.
31. T. Seel, J. Raisch, T. Schauer, "IMU-based joint angle measurement for gait analysis", *Sensors*, 2014.
32. TDK InvenSense, nota de aplicación AN-000146, "Migration from MPU-9250 to ICM-20948". [Compatibilidad](https://adm.invensense.tdk.com/icm-20948-and-mpu-9250-compatibility)

**Calibración y desempeño**

33. B. W. Mooring, Z. S. Roth, M. R. Driels, *Fundamentals of Manipulator Calibration*, Wiley, 1991.
34. R. Y. Tsai, R. K. Lenz, "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration", *IEEE Trans. Robotics and Automation*, 1989.
35. F. C. Park, B. J. Martin, "Robot sensor calibration: solving AX = XB on the Euclidean group", *IEEE Trans. Robotics and Automation*, 1994.
36. ISO 9283:1998, *Manipulating industrial robots — Performance criteria and related test methods*.

**Percepción y agarre**

37. K. Khoshelham, S. O. Elberink, "Accuracy and resolution of Kinect depth data for indoor mapping applications", *Sensors*, 2012.
38. C. V. Nguyen, S. Izadi, D. Lovell, "Modeling Kinect sensor noise for improved 3D reconstruction and tracking", *3DIMPVT*, 2012.
39. J. Redmon, S. Divvala, R. Girshick, A. Farhadi, "You Only Look Once: Unified, real-time object detection", *CVPR*, 2016.
40. B. Wen, W. Yang, J. Kautz, S. Birchfield, "FoundationPose: Unified 6D pose estimation and tracking of novel objects", *CVPR*, 2024.
41. A. ten Pas, M. Gualtieri, K. Saenko, R. Platt, "Grasp pose detection in point clouds", *Int. J. Robotics Research*, 2017.
42. J. Mahler et al., "Dex-Net 2.0: Deep learning to plan robust grasps with synthetic point clouds and analytic grasp metrics", *RSS*, 2017.
43. M. Sundermeyer, A. Mousavian, R. Triebel, D. Fox, "Contact-GraspNet: Efficient 6-DoF grasp generation in cluttered scenes", *ICRA*, 2021.
44. H.-S. Fang et al., "AnyGrasp: Robust and efficient grasp perception in spatial and temporal domains", *IEEE Trans. Robotics*, 2023.
45. H.-S. Fang, C. Wang, M. Gou, C. Lu, "GraspNet-1Billion: A large-scale benchmark for general object grasping", *CVPR*, 2020.
46. F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches", *IEEE Robotics & Automation Magazine*, 2006.

**Simulación**

47. J. Tobin et al., "Domain randomization for transferring deep neural networks from simulation to the real world", *IROS*, 2017.

**Manipuladores móviles**

48. C. C. Kemp, A. Edsinger, H. M. Clever, B. Matulevich, "The design of Stretch: A compact, lightweight mobile manipulator for indoor human environments", *ICRA*, 2022. [arXiv:2109.10892](https://arxiv.org/abs/2109.10892)
49. J. Pages, L. Marchionni, F. Ferro, "TIAGo: the modular robot that adapts to different research needs", *IROS Workshop on Robot Modularity*, 2016.
50. M. Wise, M. Ferguson, D. King, E. Diehr, D. Dymesich, "Fetch & Freight: Standard platforms for service robot applications", *IJCAI Workshop on Autonomous Mobile Service Robots*, 2016.
51. S. Thakar et al., "A survey of wheeled mobile manipulation: A decision-making perspective", *ASME J. Mechanisms and Robotics*, 2023.

---

## 14. Artículos descargados

Los PDF de acceso abierto están en `docs/literatura/pdf/`, **solo en el PC donde se descargaron**:
son de terceros, así que esa carpeta está en `.gitignore`, como `docs/hardware/`. En otro PC se
vuelven a bajar desde los enlaces de esta tabla. Cada archivo empieza con el número de la
referencia.

**Descargados (26):**

| Ref. | Archivo | Nota | Fuente |
| --- | --- | --- | --- |
| [05] | `05_lynch_park_modern_robotics.pdf` | libro completo, versión preliminar de los autores | [enlace](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) |
| [06] | `06_diankov_ikfast_tesis.pdf` | tesis (IKFast) | [enlace](http://www.programmingvision.com/rosen_diankov_thesis.pdf) |
| [10] | `10_buss_ik_survey.pdf` |  | [enlace](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf) |
| [15] | `15_macenski_ros2.pdf` | arXiv | [enlace](https://arxiv.org/abs/2211.07752) |
| [16] | `16_coleman_moveit.pdf` | arXiv | [enlace](https://arxiv.org/abs/1404.3785) |
| [18] | `18_lavalle_rrt.pdf` |  | [enlace](http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf) |
| [19] | `19_kuffner_rrt_connect.pdf` |  | [enlace](http://www.kuffner.org/james/papers/kuffner_icra2000.pdf) |
| [20] | `20_sucan_ompl.pdf` | versión de los autores | [enlace](https://www.kavrakilab.org/publications/sucan-moll2012open-motion-planning.pdf) |
| [21] | `21_ratliff_chomp.pdf` | versión de los autores | [enlace](https://www.ri.cmu.edu/pub_files/2009/5/icra09-chomp.pdf) |
| [22] | `22_kalakrishnan_stomp.pdf` | copia en la wiki de ROS | [enlace](https://mirror.umd.edu/roswiki/attachments/Papers(2f)ICRA2011_Kalakrishnan/kalakrishnan_icra2011.pdf) |
| [23] | `23_schulman_trajopt.pdf` | versión de los autores | [enlace](http://rll.berkeley.edu/~sachin/papers/Schulman-IJRR2014.pdf) |
| [24] | `24_kunz_totg.pdf` | actas abiertas de RSS | [enlace](http://www.roboticsproceedings.org/rss08/p27.pdf) |
| [25] | `25_berscheid_ruckig.pdf` | arXiv | [enlace](https://arxiv.org/abs/2105.04830) |
| [26] | `26_chitta_ros_control.pdf` | revista abierta (JOSS) | [enlace](https://joss.theoj.org/papers/10.21105/joss.00456) |
| [29] | `29_madgwick_reporte.pdf` | reporte interno de 2010, la versión larga del artículo de 2011 | [enlace](https://x-io.co.uk/downloads/madgwick_internal_report.pdf) |
| [32] | `32_tdk_mpu9250_a_icm20948.pdf` | nota de aplicación de TDK (copia en un espejo) | [enlace](https://docs.ampnuts.ru/eevblog.docs/TDK/ICM20948/AN-000146-v2.0-TDK_Migration_MPU_9250toICM-20948.pdf) |
| [39] | `39_redmon_yolo.pdf` | arXiv | [enlace](https://arxiv.org/abs/1506.02640) |
| [40] | `40_wen_foundationpose.pdf` | arXiv | [enlace](https://arxiv.org/abs/2312.08344) |
| [41] | `41_tenpas_gpd.pdf` | arXiv | [enlace](https://arxiv.org/abs/1706.09911) |
| [42] | `42_mahler_dexnet2.pdf` | arXiv | [enlace](https://arxiv.org/abs/1703.09312) |
| [43] | `43_sundermeyer_contact_graspnet.pdf` | arXiv | [enlace](https://arxiv.org/abs/2103.14127) |
| [44] | `44_fang_anygrasp.pdf` | arXiv | [enlace](https://arxiv.org/abs/2212.08333) |
| [45] | `45_fang_graspnet1b.pdf` | acceso abierto de CVF | [enlace](https://openaccess.thecvf.com/content_CVPR_2020/papers/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.pdf) |
| [47] | `47_tobin_domain_randomization.pdf` | arXiv | [enlace](https://arxiv.org/abs/1703.06907) |
| [48] | `48_kemp_stretch.pdf` | arXiv | [enlace](https://arxiv.org/abs/2109.10892) |
| [50] | `50_wise_fetch.pdf` | copia en archive.org (la empresa cerró) | [enlace](https://web.archive.org/web/2019/http://docs.fetchrobotics.com/FetchAndFreight2016.pdf) |

**Abiertos, pero la página bloquea las descargas automáticas (5).** Se bajan a mano desde el
navegador y se guardan en `docs/literatura/pdf/` con el mismo formato de nombre:

| Ref. | Guardar como | Fuente |
| --- | --- | --- |
| [30] | `30_mahony_filtro.pdf` | [HAL](https://hal.science/hal-00488376) |
| [31] | `31_seel_imu_angulos.pdf` | [Sensors (MDPI)](https://www.mdpi.com/1424-8220/14/4/6891) |
| [37] | `37_khoshelham_kinect.pdf` | [Sensors (MDPI)](https://www.mdpi.com/1424-8220/12/2/1437) |
| [46] | `46_chaumette_servo_visual.pdf` | [HAL Inria](https://inria.hal.science/inria-00350283) |
| [51] | `51_thakar_survey_manip_movil.pdf` | [NSF PAR](https://par.nsf.gov/biblio/10393722) |

**De pago o en papel (18):** libros [1–3], artículos de revistas y congresos sin versión abierta
[4, 7–9, 11–14, 17, 34, 35, 38, 49], la norma ISO 9283 [36] y el libro de calibración [33]. Se
consiguen por la biblioteca de la universidad (IEEE Xplore, ScienceDirect, SAGE) o con el DOI.
Las páginas web [27, 28] se leen en línea. Para TRAC-IK [11], el
[repositorio](https://bitbucket.org/traclabs/trac_ik) y la [página de ROS](https://index.ros.org/p/trac_ik/)
explican el método.
