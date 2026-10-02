# hoverboard_hardware_interface (Milo)

Plugin de `ros2_control` para las ruedas de Milo: un hoverboard con firmware
[hoverboard-firmware-hack-FOC](https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC)
(`VARIANT_USART`, 115200 baudios). Basado en
[DataBot-Labs/hoverboard_ros2_control](https://github.com/DataBot-Labs/hoverboard_ros2_control)
(Robert Gruberski, Apache-2.0).

## Calibración de Milo (ya probada en el robot, no tocar sin probar)
En `src/hoverboard_hardware_interface.cpp`, `write()`:

| Qué | Valor | Por qué |
|---|---|---|
| Sentido | `targetLeft = -rightCmd`, `targetRight = -leftCmd` | en Milo el avance salía invertido y el giro bien: invierte solo la traslación |
| Encoders | izquierda ← motor derecho y viceversa, posición negada (`read()` / callback) | consistente con lo anterior |
| Velocidad máx. por rueda | 6 km/h (1.67 m/s) | tope duro en el driver |
| Rampa | 0.5 m/s² | evita arranques y frenadas bruscas |
| Encoder | 90 ticks/vuelta | 15 pares de polos × 6 estados Hall |

Encima de esto, el `diff_drive_controller` (`andesrobot_bringup/config/milo_controllers.yaml`)
y el filtro de seguridad limitan a 0.4 m/s.

## Parámetros (en `andesrobot_description/urdf/andesrobot.ros2_control.xacro`)
`device` (`/dev/hoverboard` por defecto, ver `docker/udev`), `baud_rate`, `loop_rate`,
`encoder_ticks_per_revolution`, `left_wheel_joint_name`, `right_wheel_joint_name`.
