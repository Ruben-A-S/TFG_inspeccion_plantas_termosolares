# 1. Requisitos iniciales
cmake_minimum_required(VERSION 3.8)
project(faker_procesado)

# 2. Herramientas de construcción de ROS 2
find_package(ament_cmake REQUIRED)

# 3. Reglas de Instalación (Lo más importante para este paquete)
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

# 4. Cierre obligatorio
ament_package()
