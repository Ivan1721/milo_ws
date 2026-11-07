# Probar el brazo desde cero: de un PC vacío a la pinza frente a la manija

Orden para probar todo lo del brazo en un computador que no tiene nada instalado: Docker, la
imagen, la simulación, MoveIt, el detector de la manija y el acercamiento de la pinza. Cada paso
depende del anterior: **si uno falla, arréglalo antes de seguir**. Escrito el 5 de noviembre de 2026.

> **En palabras simples:** primero se instala Docker y se construye la "caja" con todo el software
> (la imagen). Después se abre la simulación del brazo y se corren las pruebas automáticas. Al final
> se pone una puerta frente a Milo, se manda la pinza a un punto desde donde la cámara ve la manija
> y un programa la detecta y acerca la pinza solo, hasta quedar a 5 cm. Lo mismo con una taza
> sobre una mesa.

Cada paso dice qué escribir y qué **debe pasar**. Si no pasa, anótalo con el mensaje de error.
Documentos relacionados: [`README.md`](../README.md) (instalación),
[`BRAZO.md`](../ros2_ws/src/andesrobot_arm/docs/BRAZO.md) (detalle del brazo, la cámara y la manija),
[`PENDIENTES.md`](PENDIENTES.md) (qué cierra cada prueba) y
[dependencias](dependencias/README.md) (qué trae la imagen).

## Contenido

1. [Instalar Docker y el repositorio](#1-instalar-docker-y-el-repositorio)
2. [Construir la imagen](#2-construir-la-imagen)
3. [Simulación del brazo](#3-simulación-del-brazo)
4. [Velocidad de la simulación](#4-velocidad-de-la-simulación)
5. [Pruebas sin simulación](#5-pruebas-sin-simulación)
6. [Prueba de humo de Gazebo](#6-prueba-de-humo-de-gazebo)
7. [pick_ik y Ruckig](#7-pick_ik-y-ruckig)
8. [Modelo de la manija](#8-modelo-de-la-manija)
9. [Pinza frente a la manija y acercamiento](#9-pinza-frente-a-la-manija-y-acercamiento)
10. [Pinza frente a un objeto de la mesa](#10-pinza-frente-a-un-objeto-de-la-mesa)
11. [Corrida larga sin deriva](#11-corrida-larga-sin-deriva)
12. [Cámara real](#12-cámara-real)
13. [Sin internet](#13-sin-internet)
14. [Glosario](#14-glosario)

---

## 1. Instalar Docker y el repositorio

Sigue la sección 1 del [`README.md`](../README.md#1--instalar-una-vez-por-computador): Docker,
reiniciar, `docker run --rm hello-world` y, si hay tarjeta NVIDIA, el NVIDIA Container Toolkit.

**Ojo con el repositorio:** el README clona `alejapinzonf/Navegation_ws`. El trabajo del brazo está
en `Ivan1721/milo_ws`, rama `brazo`:

```bash
git clone -b brazo https://github.com/Ivan1721/milo_ws.git ~/milo_ws
```

**Debe pasar:** existe `~/milo_ws` y `git -C ~/milo_ws branch` muestra `* brazo`.

## 2. Construir la imagen

```bash
cd ~/milo_ws && ./sim.sh imagen
```

**Debe pasar:** termina sin error y `docker images milo` muestra `milo:humble`. Tarda bastante
(más de 30 minutos la primera vez: descarga ROS, MoveIt, PyTorch y YOLO). Las versiones quedan
fijas (`docker/requisitos_vision.txt`), salvo los paquetes apt de ROS. Arrancar Milo **no**
reconstruye la imagen: si después un `git pull` cambia algo en `docker/`, aparece un aviso y hay que
volver a ejecutar `./sim.sh imagen`, y luego `./sim.sh stop` para que el contenedor use la nueva.

## 3. Simulación del brazo

```bash
./sim.sh brazo
```

La primera vez compila el workspace antes de abrir.

**Debe pasar:**
- Se abren Gazebo y RViz, con Milo frente a una mesa.
- En RViz, con la herramienta Interact (tecla `i`), arrastras la esfera naranja y el brazo va ahí.
- Si mandas la pinza dentro de la mesa, MoveIt lo rechaza (el log lo dice en español).

Déjalo abierto para el paso 4.

## 4. Velocidad de la simulación

El arreglo de la deriva de la base (P13) hace que la física calcule más, así que la simulación
puede ir más lenta. Con `./sim.sh brazo` corriendo, en otra terminal:

```bash
./sim.sh shell
gz stats
```

**Debe pasar:** el factor de tiempo real (`Factor`) queda cerca de 1.0. Si baja mucho de 1, anótalo
en P13. Después cierra la simulación con `Ctrl+C` (el contenedor sigue arriba).

## 5. Pruebas sin simulación

En `./sim.sh shell`:

```bash
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/andesrobot_arm && python3 -m pytest -q test
cd /ros2_ws/src/andesrobot_safety && python3 -m pytest -q test
python3 /ros2_ws/src/andesrobot_arm/scripts/probar_manija_sin_gazebo.py
```

**Debe pasar:**
- El brazo da `57 passed` y el filtro de seguridad `11 passed`.
- La prueba de la manija termina en `BIEN`, con la pinza a unos 0.05 m del punto de agarre.

## 6. Prueba de humo de Gazebo

Sin otra simulación abierta, una a la vez:

```bash
bash /ros2_ws/src/andesrobot_arm/scripts/prueba_gazebo.sh
MOVEIT=false bash /ros2_ws/src/andesrobot_arm/scripts/prueba_gazebo.sh
```

**Debe pasar:** las dos terminan en `TODO OK`. Si un arranque falla después de otro, ejecuta
`./sim.sh stop` en el PC: Gazebo a veces deja procesos colgados.

## 7. pick_ik y Ruckig

`prueba_gazebo.sh` no tiene opción para estos dos, así que se prueban a mano. En `./sim.sh shell`,
una opción a la vez:

```bash
ros2 launch andesrobot_arm arm_sim.launch.py ik:=pick_ik
ros2 launch andesrobot_arm arm_sim.launch.py suavizado:=ruckig
```

**Debe pasar:**
- **pick_ik:** con las mismas poses de la esfera que en el paso 3, llega a las mismas o a más que
  KDL, sin tardar mucho más.
- **Ruckig:** los movimientos arrancan y frenan sin golpes.
- Si `move_group` reclama por un parámetro, anota el mensaje (ver P08 en
  [`PENDIENTES.md`](PENDIENTES.md)).

## 8. Modelo de la manija

El modelo YOLO entrenado no va en git, así que en un PC nuevo no existe. Tiene que quedar en
`ros2_ws/modelos/entrenamientos/manija_pose/weights/best.pt`. Dos opciones:

- **Rápida:** copiar `best.pt` desde un PC que ya lo tenga, a esa misma ruta.
- **Desde cero:** generar las fotos y entrenar con los comandos de
  [`BRAZO.md`](../ros2_ws/src/andesrobot_arm/docs/BRAZO.md), sección «Detectar la manija». Las
  360 fotos tardan de 9 a 11 s cada una (más de una hora), y después hay que entrenar.

**Debe pasar:** `ls ~/milo_ws/ros2_ws/modelos/entrenamientos/manija_pose/weights/` muestra `best.pt`.

## 9. Pinza frente a la manija y acercamiento

**Terminal 1** (`./sim.sh shell`): la puerta en vez de la mesa, y sin OctoMap.

```bash
ros2 launch andesrobot_arm arm_sim.launch.py puerta:=true octomap:=false
```

Sin `octomap:=false`, MoveIt toma la puerta que ve la cámara como obstáculo y no deja acercarse.

**Terminal 2** (`./sim.sh shell`): mandar la pinza a la pose de partida, desde donde la cámara ve la
manija.

```bash
ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_footprint}, pose: {position: {x: 0.373, y: -0.319, z: 1.062}, orientation: {x: 0.0441, y: 0.0, z: -0.999, w: 0.0}}}"
```

**Debe pasar:** el brazo llega y en RViz la pinza queda frente a la puerta apuntando a la manija.
En la imagen de `/gripper_camera/color/image_raw` se ve la manija.

**De dónde sale la pose** (calculada con el URDF y la cinemática del repositorio):
- **La manija:** su punto de agarre está en x 0.745, y −0.31, z 1.0 m respecto de
  `base_footprint`, mientras la base no se haya movido desde que partió la simulación.
- **La cámara:** va 6 cm por encima de la punta de la pinza (`gripper_tcp`) e inclinada 20° hacia
  abajo. Por eso la pose deja la cámara a 45 cm de la manija y 15° más arriba, mirándola de frente.
  45 cm está dentro de lo que la profundidad mide bien (desde 0.25 m).
- **La orientación** (`z: -0.999`) es la pinza girada 180° en Z, apuntando hacia la puerta (+X).

Si MoveIt no llega, la misma orientación sirve más lejos:

| Distancia de la cámara a la manija | `x` | `y` | `z` |
|---|---:|---:|---:|
| 45 cm (la de arriba) | 0.373 | −0.319 | 1.062 |
| 55 cm | 0.276 | −0.319 | 1.088 |
| 65 cm | 0.179 | −0.319 | 1.114 |

También se puede arrastrar la esfera naranja en RViz hasta que la manija quede al centro de la
imagen de la cámara.

**Terminal 2:** detectar la manija y acercar la pinza.

```bash
ros2 run andesrobot_arm manija_node --ros-args -p use_sim_time:=true -p acercar:=true
```

Cuando detecta la manija 3 veces seguidas en el mismo lugar (a menos de 2 cm entre una y otra),
lleva la pinza a 30 cm del punto de agarre. Ahí la detecta de nuevo, ya más cerca y de frente, y
avanza a ciegas hasta quedar a 5 cm.

**Debe pasar:**
- En RViz los marcadores de `/manija/marcadores` caen sobre la manija, con un error de pocos mm.
- MoveIt llega a la pose previa (30 cm) y después a 5 cm.
- El log termina con «Listo: la pinza quedó a … m del punto de agarre», y esa distancia coincide
  con la que se ve en RViz.

**Si no funciona:**
- **El log dice que las detecciones saltan:** súbele la tolerancia con `-p tol_estable:=0.04`.
- **Se ve la manija pero no la detecta:** baja la confianza con `-p confianza:=0.3`.
- **El log dice que `acercar` está apagado:** falta `-p acercar:=true`.

## 10. Pinza frente a un objeto de la mesa

Lo mismo que con la manija, pero con el modelo de objetos del reglamento (27 clases: cup, bowl,
bottle...) y una taza y un bowl sobre la mesa. Detalle en
[`BRAZO.md`](../ros2_ws/src/andesrobot_arm/docs/BRAZO.md), «Objetos sobre la mesa».

**Antes, una vez por computador:** el modelo de objetos tampoco va en git. Copia su `best.pt` a
`ros2_ws/modelos/objetos/best.pt`, y baja la taza y el bowl (necesita internet; quedan en
`ros2_ws/modelos/gazebo/`):

```bash
./sim.sh shell
python3 /ros2_ws/src/andesrobot_arm/scripts/descargar_objetos.py
```

**Terminal 1** (`./sim.sh shell`):

```bash
ros2 launch andesrobot_arm arm_sim.launch.py objetos:=true octomap:=false
```

**Debe pasar:** en Gazebo hay una taza y un bowl **blancos** sobre la mesa (si se ven negros,
vuelve a correr `descargar_objetos.py`: arregla el material).

**Terminal 2** (`./sim.sh shell`): mandar la pinza a la pose de partida, con la cámara a 50 cm de los
objetos mirando 40° hacia abajo.

```bash
ros2 topic pub --once /arm_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_footprint}, pose: {position: {x: 0.451, y: -0.009, z: 1.016}, orientation: {x: -0.1732, y: 0.0, z: -0.9849, w: 0.0}}}"
```

**Terminal 2:** detectar la taza y acercar la pinza.

```bash
ros2 run andesrobot_arm objeto_node --ros-args -p use_sim_time:=true -p clase:=cup -p acercar:=true
```

**Debe pasar:**
- El log dice `veo: cup` y la posición de la taza, cerca de x 0.77, y 0.10, z 0.80.
- La pinza va a 30 cm de la taza, la detecta de nuevo y termina con «Listo: la pinza quedó a … m
  del objeto», cerca de 0.10 m. Así salió en la prueba del 6-nov-2026 (0.103 m).

**Ojo:** el modelo no reconoce el bowl de este escenario (de frente no lo ve; desde arriba lo llama
`cup`). Con `clase:=bowl` el nodo dirá «no veo bowl».

## 11. Corrida larga sin deriva

Antes, la base se corría 41 cm en una corrida de 300 fotos. Con la puerta lanzada como en el paso 9:

```bash
cd /ros2_ws/src/andesrobot_arm/scripts
python3 generar_dataset_manija.py --n 300 --variar 5 --semilla 1 --salida /ros2_ws/datasets/prueba_deriva
```

**Debe pasar:** llega a 300 fotos sin detenerse por deriva (cierra P13).

## 12. Cámara real

Solo si la Gemini Plus está conectada. La regla USB se instala una vez por computador:

```bash
sudo cp docker/udev/99-gemini.rules /etc/udev/rules.d/
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Después:

```bash
./robot.sh camara rviz:=true
```

**Debe pasar:** se ven color, profundidad e infrarrojo a unos 30 Hz.

## 13. Sin internet

Desconecta el computador de la red y repite los pasos 3 y 9.

**Debe pasar:** todo funciona igual. Así se comprueba que YOLO no intenta descargar nada
(`YOLO_AUTOINSTALL=false`).

## 14. Glosario

| Término | Qué es |
|---|---|
| Imagen de Docker | La "caja" con Ubuntu, ROS y todas las bibliotecas; se construye una vez con `./sim.sh imagen` |
| Contenedor | La caja en funcionamiento; `./sim.sh stop` lo apaga y el próximo arranque lo crea de nuevo |
| MoveIt | El programa que calcula un camino del brazo sin chocar |
| KDL / pick_ik | Dos formas de resolver la cinemática inversa (qué ángulos llevan la pinza a una pose) |
| Ruckig | Suaviza la trayectoria para que el brazo no arranque ni frene de golpe |
| OctoMap | Mapa 3D de obstáculos que arma MoveIt con la cámara |
| YOLO | La red neuronal que detecta la manija en la imagen |
| Pose | Posición (x, y, z) más orientación (un cuaternión x, y, z, w) |
| `base_footprint` | El punto del piso bajo el eje de las ruedas; las poses del brazo se miden desde ahí |
| Deriva de la base | La base que se corre en Gazebo cuando el brazo se mueve (P13) |
| Factor de tiempo real | Cuántos segundos simulados pasan por cada segundo real (1.0 = igual de rápido) |
