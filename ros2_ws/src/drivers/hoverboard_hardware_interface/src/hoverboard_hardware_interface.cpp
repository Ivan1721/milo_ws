// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Implementación del hardware interface del hoverboard (ver el .hpp para el ciclo de vida).
// Las partes marcadas "MILO" son la calibración hecha en el robot real: no cambiar sin probar.

#include "hoverboard_hardware_interface/hoverboard_hardware_interface.hpp"
#include <algorithm>
#include <cmath>

namespace hoverboard_hardware_interface
{
    // on_init: se llama una vez al cargar el plugin. Lee los <param> del bloque <ros2_control>
    // del URDF y revisa que cada rueda tenga las interfaces que este driver sabe manejar.
    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Primero dejar que la clase base guarde la info del URDF (info_).
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Leer cada parámetro (.at() falla si falta alguno en el URDF). stof/stoi = texto a número.
        hardwareConfig.leftWheelJointName = info.hardware_parameters.at("left_wheel_joint_name");
        hardwareConfig.rightWheelJointName = info.hardware_parameters.at("right_wheel_joint_name");
        hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));
        // hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

        serialPortConfig.device = info.hardware_parameters.at("device");
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        // Crear el estado de cada rueda con su nombre y los pasos por vuelta del encoder (90).
        leftWheel = MotorWheel(info.hardware_parameters.at("left_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));
        rightWheel = MotorWheel(info.hardware_parameters.at("right_wheel_joint_name"), 
                                std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution")));

        // Revisar cada <joint> del bloque ros2_control: debe tener exactamente 1 comando (velocity)
        // y 2 estados en este orden: position y velocity. Si no, error fatal (el URDF está mal).
        for (const hardware_interface::ComponentInfo & joint : info.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // Entrega a ros2_control punteros a las variables que se pueden LEER: posición y velocidad
    // de cada rueda. El joint_state_broadcaster y el diff_drive_controller leen de aquí.
    std::vector<hardware_interface::StateInterface> HoverboardHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheel.name, hardware_interface::HW_IF_POSITION, &leftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheel.name, hardware_interface::HW_IF_POSITION, &rightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.velocity));

        return state_interfaces;
    }

    // Entrega punteros a las variables que se pueden ESCRIBIR: el comando de velocidad de cada
    // rueda. El diff_drive_controller escribe aquí y write() lo manda al hoverboard.
    std::vector<hardware_interface::CommandInterface> HoverboardHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.command));

        return command_interfaces;
    }

    // on_configure: abrir el puerto serie y registrar qué hacer con cada mensaje que llegue.
    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Configuring... please wait a moment...");

        if (!serialPortService.connect(serialPortConfig.device, serialPortConfig.baudRate, serialPortConfig.timeout))
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // std::bind arma una función que, al recibir un MotorWheelFeedback, llama a
        // this->motorWheelFeedbackCallback(mensaje).
        serialPortService.BindMotorWheelFeedbackCallback(
            std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1)
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // on_cleanup: cerrar el puerto serie al apagar.
    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Cleaning up... please wait a moment...");

        if (!serialPortService.disconnect())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // on_activate / on_deactivate: no hacen nada especial, solo avisan en la terminal.
    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Activating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Deactivating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // read: 50 veces por segundo. Lee el puerto (lo que actualiza los contadores de pasos vía el
    // callback) y calcula posición y velocidad de cada rueda.
    hardware_interface::return_type HoverboardHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
    {
        serialPortService.read();

        // Guardar la posición anterior, calcular la nueva (ángulo en rad, con el signo invertido:
        // parte de la calibración de Milo) y la velocidad = (nueva - anterior) / tiempo transcurrido.
        double lastPosition = leftWheel.position;
        leftWheel.position = -leftWheel.calculateEncoderAngle();
        leftWheel.velocity = (leftWheel.position - lastPosition) / period.seconds();

        lastPosition = rightWheel.position;
        rightWheel.position = -rightWheel.calculateEncoderAngle();
        rightWheel.velocity = (rightWheel.position - lastPosition) / period.seconds();

        return hardware_interface::return_type::OK;
    }

    // write: 50 veces por segundo. Toma la velocidad que pidió el diff_drive_controller para cada
    // rueda (rad/s), aplica la calibración y los límites de Milo, y la manda al hoverboard.
    hardware_interface::return_type HoverboardHardwareInterface::write(
        const rclcpp::Time &,
        const rclcpp::Duration & period)
    {
        // ============================================================
        // MILO - CAPA DE SEGURIDAD DEL DRIVER
        //
        // Convencion ROS:
        //   +X = adelante
        //   +Z = giro antihorario
        //
        // En el hardware de Milo el componente lineal estaba invertido
        // mientras que el giro era correcto.
        //
        // Para invertir SOLO la traslacion y conservar el giro:
        //
        //   targetLeft  = -rightCommand
        //   targetRight = -leftCommand
        //
        // Forward:
        //   (+,+) -> (-,-)
        //
        // Giro:
        //   (-,+) -> (-,+)
        //
        // ============================================================

        // constexpr = constante que se calcula al compilar.
        constexpr double WHEEL_RADIUS_M = 0.08255;

        // 6 km/h
        constexpr double MAX_LINEAR_SPEED_MS =
            6.0 / 3.6;

        // Velocidad angular maxima equivalente de cada rueda
        constexpr double MAX_WHEEL_SPEED_RAD_S =
            MAX_LINEAR_SPEED_MS / WHEEL_RADIUS_M;

        // Rampa suave.
        //
        // 0.5 m/s^2:
        // de 6 km/h a cero tarda aproximadamente 3.3 s.
        //
        // Evita frenadas y aceleraciones bruscas.
        constexpr double MAX_LINEAR_ACCEL_MS2 =
            0.50;

        constexpr double MAX_WHEEL_ACCEL_RAD_S2 =
            MAX_LINEAR_ACCEL_MS2 / WHEEL_RADIUS_M;

        // Conversión usada originalmente por este driver:
        // 1 RPM = 0.10472 rad/s
        constexpr double RAD_S_PER_RPM = 0.10472;


        // ============================================================
        // 1. LEER COMANDOS ROS
        // ============================================================

        const double rosLeft =
            std::isfinite(leftWheel.command)
            ? leftWheel.command
            : 0.0;

        const double rosRight =
            std::isfinite(rightWheel.command)
            ? rightWheel.command
            : 0.0;


        // ============================================================
        // 2. CORREGIR SOLO ADELANTE / ATRAS
        //
        // Swap + negate:
        //
        // mantiene el sentido de giro actual,
        // invierte solamente el avance.
        // ============================================================

        double targetLeft =
            -rosRight;

        double targetRight =
            -rosLeft;


        // ============================================================
        // 3. LIMITADOR DURO DE 6 km/h
        //
        // Ninguna rueda puede superar la velocidad equivalente
        // a 6 km/h.
        // ============================================================

        targetLeft = std::clamp(
            targetLeft,
            -MAX_WHEEL_SPEED_RAD_S,
             MAX_WHEEL_SPEED_RAD_S
        );

        targetRight = std::clamp(
            targetRight,
            -MAX_WHEEL_SPEED_RAD_S,
             MAX_WHEEL_SPEED_RAD_S
        );


        // ============================================================
        // 4. LIMITADOR DE ACELERACION / FRENADO
        //
        // Importante:
        // NO hacemos:
        //
        //     comando = 0 instantaneamente
        //
        // El valor converge progresivamente al target.
        // ============================================================

        // "static" = estas variables conservan su valor entre una llamada a write() y la siguiente
        // (es la velocidad que se mandó la vez anterior, para la rampa).
        static double limitedLeft = 0.0;
        static double limitedRight = 0.0;

        double dt = period.seconds();

        // El controller_manager trabaja a 50 Hz.
        // Si por cualquier motivo recibimos un periodo absurdo,
        // usamos 20 ms.
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.2)
        {
            dt = 0.02;
        }

        // Cuánto puede cambiar la velocidad en este ciclo: aceleración máx x tiempo del ciclo.
        // clamp(valor, -max, +max) recorta la diferencia a ese rango.
        const double maxDelta =
            MAX_WHEEL_ACCEL_RAD_S2 * dt;

        const double deltaLeft =
            std::clamp(
                targetLeft - limitedLeft,
                -maxDelta,
                 maxDelta
            );

        const double deltaRight =
            std::clamp(
                targetRight - limitedRight,
                -maxDelta,
                 maxDelta
            );

        limitedLeft += deltaLeft;
        limitedRight += deltaRight;


        // ============================================================
        // 5. CONVERTIR A PROTOCOLO HOVERBOARD
        //
        // Conservamos exactamente la conversion speed/steer
        // utilizada originalmente por el driver.
        // ============================================================

        const double leftRPM =
            limitedLeft / RAD_S_PER_RPM;

        const double rightRPM =
            limitedRight / RAD_S_PER_RPM;

        const double speed =
            (leftRPM + rightRPM) / 2.0;

        const double steer =
            (leftRPM - speed) * 2.0;


        // Armar el mensaje para el hoverboard: speed, steer y checksum (XOR de los campos).
        // static_cast<int16_t> = convertir el decimal a entero de 16 bits (se descartan decimales).
        MotorWheelDriveControl motorWheelDriveControl;

        motorWheelDriveControl.speed =
            static_cast<int16_t>(speed);

        motorWheelDriveControl.steer =
            static_cast<int16_t>(steer);

        motorWheelDriveControl.checksum =
            static_cast<uint16_t>(
                motorWheelDriveControl.head ^
                motorWheelDriveControl.steer ^
                motorWheelDriveControl.speed
            );


        // Log reducido: el original imprimia a 50 Hz.
        RCLCPP_DEBUG(
            rclcpp::get_logger("HoverboardHardwareInterface"),
            "ROS L/R %.2f %.2f | OUT L/R %.2f %.2f | speed/steer %d %d",
            rosLeft,
            rosRight,
            limitedLeft,
            limitedRight,
            motorWheelDriveControl.speed,
            motorWheelDriveControl.steer
        );


        // Mandar el struct tal cual (sus bytes en memoria) por el puerto serie.
        serialPortService.write(
            reinterpret_cast<const char *>(&motorWheelDriveControl),
            sizeof(MotorWheelDriveControl)
        );

        return hardware_interface::return_type::OK;
    }


    // Se llama con cada mensaje completo del hoverboard: actualizar el contador de pasos de cada
    // rueda. Ojo: la rueda IZQUIERDA de ROS toma el encoder del motor DERECHO del hoverboard y
    // viceversa (cableado de Milo, parte de su calibración).
    void HoverboardHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback motorWheelFeedback) 
    {
        leftWheel.updateEncoderTicks(motorWheelFeedback.rightMotorEncoderCumulativeCount);
        rightWheel.updateEncoderTicks(motorWheelFeedback.leftMotorEncoderCumulativeCount);
    }
}

// Registrar esta clase como plugin de ros2_control (hardware_interface::SystemInterface).
// Así el controller_manager la puede cargar por su nombre, el que aparece en el URDF.
PLUGINLIB_EXPORT_CLASS(hoverboard_hardware_interface::HoverboardHardwareInterface, hardware_interface::SystemInterface)
