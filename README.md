
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

