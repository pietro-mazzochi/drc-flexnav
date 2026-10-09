cmake_minimum_required(VERSION 3.10)
project(imu_lpms_cu3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(${PROJECT_NAME}_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
            src/${PROJECT_NAME}_sensor.cpp
            src/${PROJECT_NAME}_aux.cpp)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_msgs rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs ${PROJECT_NAME}_msgs)
ament_export_include_directories(include)

ament_package()
