cmake_minimum_required(VERSION 3.8)
project(industrial_joystick_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(siemens_et200sp_modbus_driver_msgs REQUIRED)

add_executable(industrial_joystick_controller src/industrial_joystick_controller.cpp)

ament_target_dependencies(industrial_joystick_controller rclcpp sensor_msgs geometry_msgs siemens_et200sp_modbus_driver_msgs)

install(TARGETS
  industrial_joystick_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
