cmake_minimum_required(VERSION 3.8)
project(panda_moveit_assignment)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(cartesian_planner src/cartesian_planner.cpp)

ament_target_dependencies(cartesian_planner
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
)

install(TARGETS cartesian_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
