cmake_minimum_required(VERSION 3.8)
project(drc_flexnav_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(${PROJECT_NAME}_msgs REQUIRED)
find_package(odometry_arc REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pid_controllers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(siemens_et200sp_modbus_driver_msgs REQUIRED)
find_package(curtis_acf2a_can_driver REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/state_feedback_linearization.cpp
  src/dynamics_controller.cpp
  src/mrac_controller.cpp
  src/utils.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(siemens_et200sp_broadcaster SHARED
  src/siemens_et200sp_broadcaster.cpp)
target_compile_features(siemens_et200sp_broadcaster PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(siemens_et200sp_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(odometry_broadcaster SHARED
  src/odometry_broadcaster.cpp)
target_compile_features(odometry_broadcaster PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(odometry_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(robot_signaling_controller SHARED
  src/robot_signaling_controller.cpp)
target_compile_features(robot_signaling_controller PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(robot_signaling_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(robot_load_controller SHARED
  src/robot_load_controller.cpp)
target_compile_features(robot_load_controller PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(robot_load_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(laser_scanner_controller SHARED
  src/laser_scanner_controller.cpp)
target_compile_features(laser_scanner_controller PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(laser_scanner_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "odometry_arc"
  "controller_interface"
  "controller_manager"
  "geometry_msgs"
  "${PROJECT_NAME}_msgs"
  "nav_msgs"
  "realtime_tools"
  "pid_controllers"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "Eigen3"
  "curtis_acf2a_can_driver"
)

ament_target_dependencies(siemens_et200sp_broadcaster
  "rclcpp"
  "controller_interface"
  "controller_manager"
  "std_msgs"
  "siemens_et200sp_modbus_driver_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(odometry_broadcaster
  "rclcpp"
  "controller_interface"
  "controller_manager"
  "std_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
  "nav_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(robot_signaling_controller
  "rclcpp"
  "controller_interface"
  "controller_manager"
  "realtime_tools"
  "std_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(robot_load_controller
  "rclcpp"
  "controller_interface"
  "controller_manager"
  "realtime_tools"
  "std_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(laser_scanner_controller
  "rclcpp"
  "controller_interface"
  "controller_manager"
  "realtime_tools"
  "std_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME} PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

target_compile_definitions(siemens_et200sp_broadcaster PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(siemens_et200sp_broadcaster PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

target_compile_definitions(odometry_broadcaster PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(odometry_broadcaster PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

target_compile_definitions(robot_signaling_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(robot_signaling_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

target_compile_definitions(robot_load_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(robot_load_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

target_compile_definitions(laser_scanner_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_LIBRARY")
target_compile_definitions(laser_scanner_controller PRIVATE "DRC_FLEXNAV_CONTROLLERS_BUILDING_DLL")

# Pluginlib export
pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface siemens_et200sp_broadcaster.xml)
pluginlib_export_plugin_description_file(controller_interface odometry_broadcaster.xml)
pluginlib_export_plugin_description_file(controller_interface robot_signaling_controller.xml)
pluginlib_export_plugin_description_file(controller_interface robot_load_controller.xml)
pluginlib_export_plugin_description_file(controller_interface laser_scanner_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} siemens_et200sp_broadcaster odometry_broadcaster robot_signaling_controller robot_load_controller laser_scanner_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(PROGRAMS scripts/accel_step.py scripts/accel_step.sh scripts/twist_step.py scripts/twist_step.sh
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_libraries(
  siemens_et200sp_broadcaster
)

ament_export_libraries(
  odometry_broadcaster
)
ament_export_libraries(
  robot_signaling_controller
)

ament_export_libraries(
  robot_load_controller
)

ament_export_libraries(
  laser_scanner_controller
)

ament_export_targets(
  export_${PROJECT_NAME}
)
# Install the plugin XML file
install(
  FILES ${PROJECT_NAME}_plugins.xml siemens_et200sp_broadcaster.xml odometry_broadcaster.xml robot_signaling_controller.xml robot_load_controller.xml laser_scanner_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(siemens_et200sp_broadcaster)
ament_export_libraries(odometry_broadcaster)
ament_export_libraries(robot_signaling_controller)
ament_export_libraries(robot_load_controller)
ament_export_libraries(laser_scanner_controller)

ament_export_include_directories(include)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
