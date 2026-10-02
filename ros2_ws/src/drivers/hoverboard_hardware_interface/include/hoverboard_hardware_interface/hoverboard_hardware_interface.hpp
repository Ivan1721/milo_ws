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

// HoverboardHardwareInterface: el "hardware interface" de ros2_control para las ruedas de Milo.
//
// ros2_control carga esta clase como plugin (ver hoverboard_hardware_interface.xml) y la usa así:
//   on_init()      leer los parámetros del URDF (<ros2_control> en andesrobot.ros2_control.xacro)
//   on_configure() abrir el puerto serie
//   on_activate()  empezar a funcionar
//   y después, 50 veces por segundo:
//     read()   -> leer encoders del hoverboard y actualizar posición/velocidad de cada rueda
//     (el diff_drive_controller calcula qué velocidad quiere en cada rueda)
//     write()  -> mandar esa velocidad al hoverboard
//   on_deactivate() / on_cleanup() al apagar.
// Las "interfaces" son punteros a variables de esta clase que el controller lee/escribe directo.

#pragma once

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "serial_port_service.hpp"
#include "motor_wheel.hpp"

namespace hoverboard_hardware_interface
{
    // SystemInterface = tipo de hardware de ros2_control que maneja varias articulaciones.
    class HoverboardHardwareInterface : public hardware_interface::SystemInterface
    {
        // Configuración general (valores por defecto; se reemplazan con los del URDF).
        struct HardwareConfig
        {
            std::string leftWheelJointName = "left_wheel_joint";
            std::string rightWheelJointName = "right_wheel_joint";

            float loopRate = 30.0;
            int encoderTicksPerRevolution = 1024;
        };

        // Configuración del puerto serie.
        struct SerialPortConfig
        {
            std::string device = "/dev/ttyUSB0";
            int baudRate = 115200;
            int timeout = 1000;
        };

    public:
        // Macro de ROS que define los tipos de puntero compartido de la clase.
        RCLCPP_SHARED_PTR_DEFINITIONS(HoverboardHardwareInterface)

        // Ciclo de vida (ver explicación arriba). "override" = reemplaza la función de la clase base.
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Lista de variables que se pueden LEER (posición, velocidad) y ESCRIBIR (comando).
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Se llaman en cada ciclo del controller_manager.
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

        // La llama SerialPortService cuando llega un mensaje completo del hoverboard.
        void motorWheelFeedbackCallback(MotorWheelFeedback);

    private:

        // El puerto serie, la configuración y el estado de cada rueda.
        SerialPortService serialPortService;

        HardwareConfig hardwareConfig;
        SerialPortConfig serialPortConfig;

        MotorWheel leftWheel;
        MotorWheel rightWheel;

        bool connect();
        bool disconnect();
    };
}
