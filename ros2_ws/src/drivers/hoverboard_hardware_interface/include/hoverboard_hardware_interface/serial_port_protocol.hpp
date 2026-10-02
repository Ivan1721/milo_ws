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

// PROTOCOLO SERIE DEL HOVERBOARD (firmware hack-FOC, VARIANT_USART).
// Define cómo son, byte a byte, los mensajes que se mandan y reciben por el puerto serie.
// Estos structs se envían/reciben tal cual en memoria, así que el orden y tamaño de cada
// campo tienen que coincidir EXACTO con el firmware.

// "#pragma once" = incluir este archivo una sola vez aunque lo pidan varios .cpp.
#pragma once

// Tipos de tamaño fijo: uint16_t = entero sin signo de 16 bits, int16_t = con signo.
#include <cstdint>

// Marca de inicio de cada mensaje (0xABCD). Sirve para encontrar dónde empieza un mensaje
// dentro del flujo continuo de bytes.
#define HEAD_FRAME 0xABCD

// enum class MOTOR_STATES {
//     UNOCUPPIED = 0b00,
//     RUN = 0b01,
//     BRAKE = 0b11,
//     LOCK_SHAFT = 0b10,
// };

// Mensaje que ENVÍA el hoverboard (feedback), periódicamente.
typedef struct {
    uint16_t head;                              // 0xABCD
    int16_t  command1;                          // eco del comando recibido (steer)
    int16_t  command2;                          // eco del comando recibido (speed)
    int16_t  rightMotorSpeed;                   // velocidad medida motor derecho (rpm)
    int16_t  leftMotorSpeed;                    // velocidad medida motor izquierdo (rpm)
    int16_t  rightMotorEncoderCumulativeCount;  // contador de pasos Hall del motor derecho
    int16_t  leftMotorEncoderCumulativeCount;   // contador de pasos Hall del motor izquierdo
    int16_t  batteryVoltage;                    // voltaje de la batería (x100)
    int16_t  boardTemperature;                  // temperatura de la placa (x10 °C)
    uint16_t commandLed;                        // estado de los LEDs
    uint16_t checksum;                          // XOR de todos los campos (detectar errores)
} MotorWheelFeedback;

// Mensaje que se le ENVÍA al hoverboard (comando).
//   speed = velocidad promedio de las dos ruedas; steer = diferencia entre ellas (giro).
//   El firmware mezcla los dos valores para obtener la velocidad de cada rueda
//   (ver write() en hoverboard_hardware_interface.cpp para cómo se calculan).
typedef struct {
    uint16_t head = HEAD_FRAME;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;                          // head ^ steer ^ speed
} MotorWheelDriveControl;
