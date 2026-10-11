##Cambiar el nombre a milo_ws

 1. INSTALAR DOCKER (una sola vez por computador)

sudo apt update
sudo apt install -y docker.io docker-compose-v2 docker-buildx unzip x11-xserver-utils
sudo usermod -aG docker $USER

Despues reinicia el computador (o cierra sesion y vuelve a entrar).

Para ver si quedo bien:

docker run --rm hello-world

Tiene que decir "Hello from Docker!".
Si dice "permission denied", es que falta reiniciar.


** Solo si tu computador tiene tarjeta NVIDIA **

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker


 2. PONER EL WORKSPACE EN TU CARPETA PERSONAL (una vez)


Descarga milo_ws.zip y despues:

unzip ~/Downloads/milo_ws.zip -d ~
cd ~/milo_ws

(si tu Ubuntu esta en español cambia Downloads por Descargas)


--------------------------------------------------------------
 3. SIMULACION (cualquiera, no necesitas el robot)
--------------------------------------------------------------

Terminal 1 - abrir todo (Gazebo + Milo + mapa en RViz):

cd ~/milo_ws
./sim.sh

La primera vez se demora harto (descarga y compila). Despues es rapido.

Terminal 2 - manejar a Milo con el teclado:

cd ~/milo_ws
./sim.sh teleop

  i = adelante      , = atras
  j = girar izq     l = girar der
  k = frenar
  q / z = mas rapido / mas lento

  OJO: la terminal del teleop tiene que estar seleccionada
  (haz clic en ella) para que lea las teclas.

Terminal 3 - guardar el mapa cuando termines de recorrer:

cd ~/milo_ws
./sim.sh mapa nombre_del_mapa

Queda guardado en: ~/milo_ws/ros2_ws/src/andesrobot_slam/maps/

Apagar todo:   Ctrl+C en la terminal 1 y despues   ./sim.sh stop

Si Gazebo o RViz se ven NEGROS:
  ./sim.sh stop
  ./sim.sh software


--------------------------------------------------------------
 3b. CASA ROBOCUP@HOME + NAVEGACION AUTONOMA (simulacion)
--------------------------------------------------------------

La casa del rulebook (Laundry, Bedroom, Living room, Kitchen, 9 x 9 m,
puertas de 0.90 m, con los 18 muebles/ubicaciones) ya esta en Gazebo.
Milo parte AFUERA de la puerta del living mirando hacia adentro.
Vista de arriba: ros2_ws/src/andesrobot_gazebo/worlds/robocup_casa.png

PASO 1 - mapear la casa (una vez):

Terminal 1:   ./sim.sh casa
Terminal 2:   ./sim.sh teleop        (recorre TODAS las piezas, despacio)
Terminal 3:   ./sim.sh mapa casa

  OJO: empieza a mapear desde donde aparece Milo (no lo muevas antes de
  lanzar). Asi el mapa queda alineado con las ubicaciones con nombre.
  Cuando termines: Ctrl+C en la terminal 1.

PASO 2 - navegar solo:

Terminal 1:   ./sim.sh nav casa      (Gazebo + AMCL + Nav2 + RViz)
Terminal 2:   ./sim.sh ir --lista    ver las ubicaciones
              ./sim.sh ir kitchen    ir a la cocina
              ./sim.sh ir 16         ir al dinner table (numero del rulebook)
              ./sim.sh ir sofa bed entrada     varias seguidas

  Tambien sirve "2D Goal Pose" en RViz (clic y arrastrar).
  Si tocas el teleop (./sim.sh teleop) le quitas el control a Nav2
  mientras aprietes teclas; al soltar, Nav2 sigue.

PASO 3 - experimentos (para el paper):

./sim.sh tour        visita las 18 ubicaciones en orden aleatorio
./sim.sh tour 3      lo mismo 3 veces
  Guarda un CSV en ros2_ws/resultados/ con: exito, tiempo, metros
  recorridos, largo del plan, error final (m y grados), recuperaciones.

Cambiar la casa (mover muebles, puertas, ubicaciones):
  editar  ros2_ws/src/andesrobot_gazebo/tools/gen_casa_robocup.py
  correr  ./sim.sh shell  y adentro:
          python3 /ros2_ws/src/andesrobot_gazebo/tools/gen_casa_robocup.py
  (revisa solo que Milo quepa, llegue y pueda girar en cada ubicacion)
  y despues rehacer el mapa (paso 1).

Como fluyen las velocidades en modo navegacion:
  Nav2 -> /nav/cmd_vel --+
  teclado -> /cmd_vel_teleop --> twist_mux -> filtro de seguridad -> Milo
  El filtro sigue activo (config: andesrobot_safety/config/safety_nav.yaml).


--------------------------------------------------------------
 4. MILO FISICO (en el portatil que va arriba de Milo)
--------------------------------------------------------------

Conecta al portatil el lidar y el cable USB del hoverboard.

Ver en que puerto quedo cada uno:

cd ~/milo_ws
./robot.sh puertos

  El que dice idVendor "10c4" es el LIDAR.
  El otro es el HOVERBOARD.
  Anota los nombres (por ejemplo /dev/ttyUSB0 y /dev/ttyUSB1).

Terminal 1 - prender a Milo (cambia los numeros por los tuyos):

LIDAR=/dev/ttyUSB0 HOVER=/dev/ttyUSB1 ./robot.sh

Terminal 2 - manejar:

cd ~/milo_ws
./robot.sh teleop

  La PRIMERA vez hazlo con Milo levantado, ruedas en el aire:
  - con "i" las dos ruedas tienen que ir hacia adelante
  - con "j" tiene que girar a la izquierda
  Si eso anda bien, recien ahi al suelo.

Terminal 3 - guardar el mapa:

cd ~/milo_ws
./robot.sh mapa nombre_del_mapa

Apagar todo:   Ctrl+C en la terminal 1 y despues   ./robot.sh stop

Navegacion autonoma con Milo real (SIN PROBAR todavia, ir con cuidado):
  ./robot.sh nav <mapa>         con un mapa hecho con ./robot.sh mapa
  ./robot.sh ir <lugar>         las ubicaciones hay que medirlas en el lugar real
                                (ver robot_navigation.launch.py)


--------------------------------------------------------------
 5. OTRAS COSAS UTILES
--------------------------------------------------------------

./sim.sh shell       abre una terminal dentro de Docker
./sim.sh build       vuelve a compilar si cambiaste codigo
./sim.sh gpu         dice con que tarjeta grafica esta dibujando

(con robot.sh funcionan igual: ./robot.sh shell, ./robot.sh build)

Milo no se acerca a menos de 30 cm de las cosas (filtro de seguridad).
Si quieres cambiar esa distancia:
  ros2_ws/src/andesrobot_safety/config/safety.yaml



--------------------------------------------------------------
 6. BRAZO (solo simulacion)
--------------------------------------------------------------

Terminal 1 - Gazebo + Milo con el brazo + RViz:

cd ~/milo_ws
./sim.sh brazo

En RViz elige la herramienta "Interact" (tecla i), arrastra la esfera
naranja que esta en la pinza y suelta: el brazo (y el lift) van ahi.
Si no se mueve, esa pose no se alcanza: clic derecho en la esfera,
"Volver a la pinza".

Apagar:   Ctrl+C en la terminal 1 y despues   ./sim.sh stop

Sobre la pinza va la camara Orbbec, y frente a Milo una mesa con 3 objetos.
En RViz se ven sus imagenes (color, profundidad, infrarrojo) y la nube de puntos.
Para guardar una imagen de cada canal, en otra terminal:

cd ~/milo_ws
./sim.sh shell
ros2 run andesrobot_arm capturar_camara

Quedan en ~/milo_ws/ros2_ws/capturas/

Mandar poses a mano, abrir/cerrar la pinza, la camara, medidas y ecuaciones:
  ros2_ws/src/andesrobot_arm/docs/BRAZO.md

OJO: los limites de las articulaciones del brazo son provisorios.
No usar con el brazo real hasta medirlos.
