
cmake_minimum_required(VERSION 3.10)
project(siemens_et200sp_modbus_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(${PROJECT_NAME}_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

include_directories(include ${MODBUS_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/siemens_et200sp_modbus_driver.cpp)
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${MODBUS_LIBRARIES}
)

add_executable(siemens_et200sp_modbus_driver_node src/siemens_et200sp_modbus_driver_node.cpp)
target_link_libraries(siemens_et200sp_modbus_driver_node ${PROJECT_NAME})
ament_target_dependencies(siemens_et200sp_modbus_driver_node ${PROJECT_NAME}_msgs rclcpp std_msgs)
ament_target_dependencies(siemens_et200sp_modbus_driver_node rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME} siemens_et200sp_modbus_driver_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()




