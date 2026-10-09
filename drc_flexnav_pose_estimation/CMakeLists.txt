cmake_minimum_required(VERSION 3.8)
project(drc_flexnav_pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-value -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drc_flexnav_controllers_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/extended_kalman_filter.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  "Eigen3"
  )


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(dynamics_controller_status_to_accel_stamped src/dynamics_controller_status_to_accel_stamped.cpp)
target_link_libraries(dynamics_controller_status_to_accel_stamped ${PROJECT_NAME})
target_include_directories(
  dynamics_controller_status_to_accel_stamped PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynamics_controller_status_to_accel_stamped rclcpp geometry_msgs drc_flexnav_controllers_msgs)

add_executable(extended_kalman_filter_node src/extended_kalman_filter_node.cpp)
target_link_libraries(extended_kalman_filter_node ${PROJECT_NAME})
target_include_directories(
  extended_kalman_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  extended_kalman_filter_node
  "rclcpp"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "Eigen3"
  "message_filters"
)

install(TARGETS extended_kalman_filter_node dynamics_controller_status_to_accel_stamped
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
