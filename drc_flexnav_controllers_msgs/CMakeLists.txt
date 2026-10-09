cmake_minimum_required(VERSION 3.8)
project(drc_flexnav_controllers_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamicsControllerStatus.msg"
  "msg/MRACControllerStatus.msg"
  "msg/PlcRequest.msg"
  "msg/PlcMissionResponse.msg"
  "msg/PlcCommand.msg"
  # "srv/AddThreeInts.srv"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs std_srvs
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()