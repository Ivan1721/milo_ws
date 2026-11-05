# Puertos fijos de Milo (udev)

El RPLIDAR C1 y el hoverboard aparecen los dos como `/dev/ttyUSBx` y el número cambia según el orden
en que se conectan. Con esta regla quedan siempre como `/dev/rplidar` y `/dev/hoverboard`.

1. En el computador de Milo, con los dos conectados: `./robot.sh puertos`
   Muestra cada `/dev/ttyUSBx` con su `idVendor`, `idProduct` y `serial`.
2. Copia esos valores en `99-milo.rules` (reemplaza los `CAMBIAR_...`).
   Si el adaptador no tiene `serial`, borra esa condición de la línea.
3. Instala la regla:
   ```bash
   sudo cp docker/udev/99-milo.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ls -l /dev/rplidar /dev/hoverboard
   ```
Mientras tanto se puede usar el puerto directo:
`ros2 launch andesrobot_bringup robot_mapping.launch.py lidar_port:=/dev/ttyUSB0 hoverboard_port:=/dev/ttyUSB1`

## Cámara Orbbec Gemini Plus

La regla `99-gemini.rules` da permiso de usar la cámara sin ser root. No hay que completar nada; se
instala una vez en cada PC donde se conecte la cámara:

```bash
sudo cp docker/udev/99-gemini.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Después se revisa que los dos dispositivos digan `crw-rw-rw-` (si no, desconectar y volver a conectar
la cámara):

```bash
lsusb -d 2bc5: | sed -E 's#Bus ([0-9]+) Device ([0-9]+).*#/dev/bus/usb/\1/\2#' | xargs ls -l
```
