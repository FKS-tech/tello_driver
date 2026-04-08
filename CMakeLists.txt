cmake_minimum_required(VERSION 3.8)
project(tello_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY scr/tello_driver/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Instalar scripts Python como executáveis
install(
  PROGRAMS
  src/tello_driver/control_node.py
  src/tello_driver/joy_node.py
  src/tello_driver/stream_node.py
  src/tello_driver/telemetry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
