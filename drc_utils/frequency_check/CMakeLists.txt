cmake_minimum_required(VERSION 3.5)
project(frequency_check)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(frequency_check_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include
  ${frequency_check_msg_INCLUDE_DIRS}
)

add_executable(frequency_check src/sensor_subscriber.cpp)

ament_target_dependencies(frequency_check
  rclcpp
  sensor_msgs
  frequency_check_msg
)

install(TARGETS frequency_check
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
