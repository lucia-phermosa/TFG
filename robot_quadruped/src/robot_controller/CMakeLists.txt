cmake_minimum_required(VERSION 3.8)
project(robot_controller)

## Compiler warnings (opcional pero recomendable)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

 add_executable(robot_controller 
  src/main.cpp 
##  src/InverseKinematics/InverseKinematics.cpp 
##  src/RobotController.cpp
##  src/RobotController/RestController.cpp
)

ament_target_dependencies(robot_controller
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

