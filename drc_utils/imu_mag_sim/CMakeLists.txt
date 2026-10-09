cmake_minimum_required(VERSION 3.5)
project(imu_mag_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

add_library(magnetometer_plugin SHARED plugins/magnetometer_plugin.cpp)
ament_target_dependencies(magnetometer_plugin rclcpp sensor_msgs std_msgs gazebo_ros)
target_link_libraries(magnetometer_plugin ${GAZEBO_LIBRARIES})

install(TARGETS magnetometer_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/ament-env-hooks/magnetometer_plugin.dsv.in")

ament_package()
