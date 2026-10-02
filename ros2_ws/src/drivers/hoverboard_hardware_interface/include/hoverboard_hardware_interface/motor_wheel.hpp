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

// MotorWheel: guarda el estado de UNA rueda (comando, posición, velocidad) y convierte el
// contador de pasos del hoverboard en un ángulo acumulado en radianes.

#pragma once

#include <string>
#include <cmath>

// El contador de pasos que manda el hoverboard va de 0 a 9000 y después vuelve a 0
// ("da la vuelta", como un cuentakilómetros). Estos valores sirven para detectar esa vuelta.
#define ENCODER_MIN_VALUE 0
#define ENCODER_MAX_VALUE 9000
// Si pasa de >70% (6300) a <30% (2700) de un mensaje al siguiente, dio la vuelta hacia arriba;
// si pasa de <30% a >70%, dio la vuelta hacia abajo (la rueda va para atrás).
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

namespace hoverboard_hardware_interface
{
    class MotorWheel {
    public:
        // Nombre del joint en el URDF (left_wheel_joint / right_wheel_joint).
        std::string name = "";

        // Pasos totales acumulados (puede crecer sin límite, por eso 64 bits).
        int64_t encoderTicks = 0;
        // Último valor crudo recibido (0..9000) y cuántas vueltas del contador llevamos.
        int16_t encoderTicksPrevious = 0;
        int16_t encoderOverflowCount = 0;

        // Umbrales de "vuelta" calculados en el constructor (2700 y 6300).
        int16_t encoderLowWrap;
        int16_t encoderHighWrap;

        // Lo que comparte con ros2_control (las "interfaces"):
        double command = 0.0;    // velocidad pedida (rad/s) -> la escribe el diff_drive_controller
        double position = 0.0;   // ángulo de la rueda (rad)  -> la lee el controller
        double velocity = 0.0;   // velocidad medida (rad/s)  -> la lee el controller

        // Cuántos radianes avanza la rueda por cada paso del encoder.
        double radiansPerRevolution = 0.0;

        MotorWheel() = default;

        // Constructor: nombre del joint y pasos por vuelta (90 en Milo).
        MotorWheel(const std::string &wheelJointName, int encoderTicksPerRevolution)
        {
            name = wheelJointName;
            // Una vuelta = 2*pi rad; dividida en 90 pasos = 0.0698 rad (4°) por paso.
            // (El nombre dice "PerRevolution" pero es "por paso".)
            radiansPerRevolution = ((2 * M_PI) / encoderTicksPerRevolution);

            encoderLowWrap = ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
            encoderHighWrap = ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
        }

        // Ángulo total de la rueda en radianes = pasos acumulados x radianes por paso.
        double calculateEncoderAngle()
        {
            return encoderTicks * radiansPerRevolution;
        }

        // Se llama con cada mensaje nuevo del hoverboard (newTicks = contador crudo 0..9000).
        void updateEncoderTicks(int16_t newTicks)
        {
            // Pasó de "casi 9000" a "casi 0": el contador dio la vuelta hacia adelante.
            if(newTicks < encoderLowWrap && encoderTicksPrevious > encoderHighWrap)
            {
                encoderOverflowCount++;
            }

            // Pasó de "casi 0" a "casi 9000": dio la vuelta hacia atrás.
            if(newTicks > encoderHighWrap && encoderTicksPrevious < encoderLowWrap)
            {
                encoderOverflowCount--;
            }

            // Total = vueltas completas del contador x 9000 + valor actual.
            encoderTicks = (encoderOverflowCount * ENCODER_MAX_VALUE) + newTicks;
            encoderTicksPrevious = newTicks;
        }

    private:

    };
}
