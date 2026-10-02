# CMakeLists.txt: instrucciones para COMPILAR el driver en C++ (colcon llama a CMake).
# Versión mínima de CMake.
cmake_minimum_required(VERSION 3.8)

# Nombre del proyecto (= nombre del paquete).
project(hoverboard_hardware_interface)

# Usar C++17 si no se indicó otra versión.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Activar advertencias del compilador (ayudan a encontrar errores).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar las librerías que necesita el driver (si falta alguna, la compilación se detiene):
# ament = sistema de compilación de ROS 2; hardware_interface = ros2_control;
# pluginlib = cargar plugins; rclcpp = ROS 2 en C++; Boost.System = para el puerto serie.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

# Encontrar automáticamente las dependencias declaradas en package.xml.
ament_auto_find_build_dependencies()

# Librería 1: el manejo del puerto serie (serial_port_service.cpp).
# SHARED = librería dinámica (.so) que se carga al ejecutar.
ament_auto_add_library(
  serial_port_service
  SHARED
  src/serial_port_service.cpp
)

# Dónde están sus headers (.hpp): carpeta include/ al compilar, include/ al instalar.
target_include_directories(
  serial_port_service
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enlazarla con Boost.System.
target_link_libraries(
  serial_port_service
  Boost::system
)

# Librería 2: el plugin de ros2_control (hoverboard_hardware_interface.cpp).
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/hoverboard_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# El plugin usa la librería del puerto serie.
target_link_libraries(
  ${PROJECT_NAME}
  serial_port_service
  Boost::system
)

# Y depende de estos paquetes de ROS 2.
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Registrar el plugin: le dice a ROS que el archivo hoverboard_hardware_interface.xml
# describe un plugin de tipo hardware_interface (así el controller_manager lo encuentra).
pluginlib_export_plugin_description_file(
  hardware_interface
  hoverboard_hardware_interface.xml
)

# Tests de estilo de código (solo si se compila con tests).
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Instalar todo (librerías, headers, package.xml) en install/.
ament_auto_package()
