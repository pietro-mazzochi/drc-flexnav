cmake_minimum_required(VERSION 3.8)
project(drc_flexnav_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(curtis_acf2a_can_driver REQUIRED)
find_package(siemens_et200sp_modbus_driver REQUIRED)
find_package(siemens_et200sp_modbus_driver_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(drc_flexnav_controllers_msgs REQUIRED)
find_package(odometry_arc REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)
find_package(imu_lpms_cu3 REQUIRED)
find_package(imu_lpms_cu3_msgs REQUIRED)

# A ros2_control's controller is a plugin and a plugin should be a shared library
add_library(siemens_et200sp_modbus_hardware_component SHARED
  src/siemens_et200sp_modbus_hardware_component.cpp)
target_link_libraries(siemens_et200sp_modbus_hardware_component
  PUBLIC ${LIBMODBUS_LIBRARIES}
)
target_compile_features(siemens_et200sp_modbus_hardware_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(siemens_et200sp_modbus_hardware_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  ${MODBUS_INCLUDE_DIRS}
)

add_library(curtis_acf2a_can_hardware_component SHARED
  src/curtis_acf2a_can_hardware_component.cpp)
target_compile_features(curtis_acf2a_can_hardware_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(curtis_acf2a_can_hardware_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(imu_lpms_cu3_hardware_component SHARED
  src/imu_lpms_cu3_hardware_component.cpp)
target_compile_features(imu_lpms_cu3_hardware_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(imu_lpms_cu3_hardware_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


ament_target_dependencies(
  siemens_et200sp_modbus_hardware_component PUBLIC
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
  "hardware_interface"
  "siemens_et200sp_modbus_driver"
  "siemens_et200sp_modbus_driver_msgs"
  "drc_flexnav_controllers_msgs"
  "realtime_tools"
  "odometry_arc"
  "nav_msgs"
  "sensor_msgs"
  "std_srvs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "Eigen3"
)

ament_target_dependencies(
  curtis_acf2a_can_hardware_component PUBLIC
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
  "hardware_interface"
  "curtis_acf2a_can_driver"
)

ament_target_dependencies(
  imu_lpms_cu3_hardware_component PUBLIC
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
  "hardware_interface"
  "imu_lpms_cu3"
) 

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(siemens_et200sp_modbus_hardware_component PRIVATE "DRC_FLEXNAV_HARDWARE_BUILDING_LIBRARY")

target_compile_definitions(curtis_acf2a_can_hardware_component PRIVATE "DRC_FLEXNAV_HARDWARE_BUILDING_LIBRARY")

target_compile_definitions(imu_lpms_cu3_hardware_component PRIVATE "DRC_FLEXNAV_HARDWARE_BUILDING_LIBRARY")


pluginlib_export_plugin_description_file(
	 hardware_interface siemens_et200sp_modbus_hardware_component.xml)

pluginlib_export_plugin_description_file(
  hardware_interface curtis_acf2a_can_hardware_component.xml)

pluginlib_export_plugin_description_file(
  hardware_interface imu_lpms_cu3_hardware_component.xml)

install(
  DIRECTORY include/
  DESTINATION include/drc_flexnav_hardware
)

install(
  TARGETS siemens_et200sp_modbus_hardware_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS imu_lpms_cu3_hardware_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(  
  TARGETS curtis_acf2a_can_hardware_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  siemens_et200sp_modbus_hardware_component
)

ament_export_libraries(
  curtis_acf2a_can_hardware_component
)

ament_export_libraries(
  imu_lpms_cu3_hardware_component
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
  "hardware_interface"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
