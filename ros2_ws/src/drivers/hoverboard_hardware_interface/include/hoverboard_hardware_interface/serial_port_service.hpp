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

// SerialPortService: abre el puerto serie (/dev/hoverboard), lee los bytes que manda el
// hoverboard, arma los mensajes MotorWheelFeedback y escribe los comandos.
// Usa Boost.Asio, una librería de C++ para comunicación (red y puertos serie).

#pragma once

#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"

#include "serial_port_protocol.hpp"

// Cuántos bytes leer como máximo en cada lectura.
#define SERIAL_PORT_READ_BUF_SIZE 256

// Puntero compartido a un puerto serie (se libera solo cuando nadie lo usa).
typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace hoverboard_hardware_interface
{
    class SerialPortService
    {
        public:

        SerialPortService() = default;

        // Abrir / cerrar el puerto. Devuelven true si salió bien.
        bool connect(const std::string &serial_device, int baud_rate, int timeout);
        bool disconnect();

        // read(): leer lo que haya llegado y procesarlo (la que se usa).
        // asyncRead(): versión asíncrona, no se usa (quedó a medio hacer en el original).
        void read();
        void asyncRead();

        // Escribir bytes al puerto.
        int write(const char *, const int &);

        // Registrar la función a llamar cada vez que llega un mensaje completo del hoverboard.
        void BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)>);

        private:

        // Objetos de Boost.Asio: el servicio de E/S, el puerto, y un candado (mutex) para que
        // dos hilos no usen el puerto al mismo tiempo.
        boost::asio::io_service io_service;
        serial_port_ptr port;
        boost::mutex mutex;

        // Estado del "armador" de mensajes: los últimos 2 bytes (para detectar 0xABCD),
        // cuántos bytes del mensaje actual llevamos, y el byte anterior.
        uint16_t head_frame = 0;
        uint16_t msg_counter = 0;
        uint8_t msg_command = 0;

        char prev_byte = 0;
        // Puntero a la posición donde escribir el siguiente byte dentro de motorWheelFeedback.
        char* p{};

        // Buffer donde llegan los bytes crudos.
        char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE]{};

        void onReceive(const boost::system::error_code&, size_t);

        // La función registrada con BindMotorWheelFeedbackCallback.
        std::function<void(MotorWheelFeedback)> motorWheelFeedbackCallback;

        // Mensaje que se va armando byte a byte.
        MotorWheelFeedback motorWheelFeedback {};
    };
}
