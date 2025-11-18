cmake_minimum_required(VERSION 3.5.0)
project(demo)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Instalar recursos del paquete (launch, config, urdf, worlds, etc.)
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/control/src)
set(INC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/control/include)

set(QUAD_SOURCES
  # ${CORE_DIR}/Kinematics.cpp
  # ${CORE_DIR}/Planner.cpp
  # ${CORE_DIR}/Leg.cpp
  # ${CORE_DIR}/Body.cpp
  # ${CORE_DIR}/Gaits.cpp
  # ${CORE_DIR}/Set.cpp
  # ${CORE_DIR}/MamdaniFuzzy.cpp
  # ${CORE_DIR}/Horizontal_controller.cpp
  ${CORE_DIR}/keyboard_cmd_parser.cpp
)

add_library(quadruped_core ${QUAD_SOURCES})
target_include_directories(quadruped_core PUBLIC
  $<BUILD_INTERFACE:${INC_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(quadruped_core
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  tf2_msgs
  nav_msgs
  std_msgs
)

add_executable(main ${CORE_DIR}/main.cpp)
target_link_libraries(main quadruped_core)
ament_target_dependencies(main
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  tf2_msgs
  nav_msgs
  std_msgs
)

install(TARGETS quadruped_core main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${INC_DIR}/ DESTINATION include)

# add_executable(gait_duty_autoadapt ${CORE_DIR}/main.cpp)  # o el .cpp que uses
# ament_target_dependencies(gait_duty_autoadapt
#   rclcpp trajectory_msgs nav_msgs sensor_msgs builtin_interfaces std_msgs)

# install(TARGETS gait_duty_autoadapt
#   DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/walk.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_all_joints.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/entitypose_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
