cmake_minimum_required(VERSION 3.8)
project(so101_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(moveit_pick_sequence src/moveit_pick_sequence.cpp)
add_executable(moveit_pick_from_cube src/moveit_pick_from_cube.cpp)
add_executable(moveit_reach_cube_position src/moveit_reach_cube_position.cpp)
add_executable(v3_moveit_ik_target_test src/v3_moveit_ik_target_test.cpp)
add_executable(v3_moveit_cube_joint_pick src/v3_moveit_cube_joint_pick.cpp)

ament_target_dependencies(
  moveit_pick_sequence
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  moveit_pick_from_cube
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  moveit_reach_cube_position
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  v3_moveit_ik_target_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  v3_moveit_cube_joint_pick
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
)

install(TARGETS
  moveit_pick_sequence
  moveit_pick_from_cube
  moveit_reach_cube_position
  v3_moveit_ik_target_test
  v3_moveit_cube_joint_pick
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
