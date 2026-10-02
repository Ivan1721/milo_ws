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

// Implementación de SerialPortService (ver el .hpp para la idea general).

#include "hoverboard_hardware_interface/serial_port_service.hpp"

using namespace hoverboard_hardware_interface;

// Abre el puerto serie y lo configura como lo espera el hoverboard: 115200 baudios, 8 bits
// de datos, 1 bit de parada, sin paridad y sin control de flujo ("8N1").
bool SerialPortService::connect(const std::string &serial_device, int baud_rate, int timeout)
{
    boost::system::error_code ec;

    // Si ya estaba abierto, error (no abrir dos veces).
    if (port) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already opened...");
        return false;
    }

    // Crear el objeto puerto y abrir el dispositivo (ej. /dev/hoverboard). ec guarda el error si falla.
    port = serial_port_ptr(new boost::asio::serial_port(io_service));
    port->open(serial_device, ec);

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Connection to the %s failed..., error: %s",
            serial_device.c_str(), ec.message().c_str());
        return false;
    }

    // Configuración 8N1 a la velocidad pedida.
    port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port_base::character_size(8));
    port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // TODO: try to run it asynchronously
    // boost::thread t([ObjectPtr = &io_service] { return ObjectPtr->run(); });
    // t.detach();

    return true;
}

// Cierra el puerto. El "lock" toma el candado para que nadie esté leyendo mientras se cierra.
bool SerialPortService::disconnect()
{
    boost::mutex::scoped_lock look(mutex);

    if (port) {
        port->cancel();
        port->close();
        port.reset();
    }

    io_service.stop();
    io_service.reset();

    return true;
}

// Lee los bytes disponibles y arma mensajes MotorWheelFeedback completos.
// Los bytes llegan como un chorro continuo; para saber dónde empieza un mensaje se busca la
// marca 0xABCD. Después se copian bytes hasta completar el tamaño del struct, y ahí se avisa
// (callback) al hardware interface.
void SerialPortService::read()
{
    boost::mutex::scoped_lock look(mutex);

    // Leer lo que haya en el puerto (hasta 256 bytes). Espera si todavía no llegó nada.
    size_t bytes_transferred = port->read_some(boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE));
    
    // Revisar byte por byte.
    for (unsigned int i = 0; i < bytes_transferred; ++i) {

        // Juntar este byte con el anterior en un número de 16 bits (el byte nuevo va en la parte alta).
        head_frame = ((uint16_t)(read_buf_raw[i]) << 8) | (uint8_t) prev_byte;

        // ¿Es la marca de inicio 0xABCD? Empezar un mensaje nuevo: copiar esos 2 bytes al struct.
        if (head_frame == HEAD_FRAME)
        {
            p = (char*) &motorWheelFeedback;
            *p++ = prev_byte;
            *p++ = read_buf_raw[i];
            msg_counter = 2;
        }
        // Si estamos dentro de un mensaje: copiar el byte en la siguiente posición del struct.
        else if (msg_counter >= 2 && msg_counter < sizeof(MotorWheelFeedback)) {
            *p++ = read_buf_raw[i];
            msg_counter++;
        }

        // Mensaje completo: entregarlo (actualiza los encoders) y esperar el próximo inicio.
        if (msg_counter == sizeof(MotorWheelFeedback))
        {
            motorWheelFeedbackCallback(motorWheelFeedback);

            msg_counter = 0;
        }

        // Recordar este byte para juntarlo con el siguiente.
        prev_byte = read_buf_raw[i];
    }
}

// Lectura asíncrona: quedó a medio implementar en el driver original y NO se usa.
void SerialPortService::asyncRead()
{
    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return;
    }

    port->async_read_some(
            boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
            boost::bind(&SerialPortService::onReceive,
                    this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

// Callback de la lectura asíncrona (no se usa).
void SerialPortService::onReceive(const boost::system::error_code& ec, size_t bytes_transferred)
{
    RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "onReceive async event...");

    // boost::mutex::scoped_lock look(mutex);

    // if (port.get() == nullptr || !port->is_open()) return;

    // if (ec) {
    //     asyncRead();
    //     return;
    // }
}

// Escribe "size" bytes al puerto (el comando MotorWheelDriveControl). Devuelve cuántos escribió.
int SerialPortService::write(const char * message, const int & size)
{
    boost::system::error_code ec;

    if (port.get() == nullptr || !port->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortService"), "Port is already closed...");
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    return port->write_some(boost::asio::buffer(message, size), ec);
}

// Guarda la función a llamar con cada mensaje completo (la registra on_configure()).
void SerialPortService::BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> fn) {
    motorWheelFeedbackCallback = fn;
}