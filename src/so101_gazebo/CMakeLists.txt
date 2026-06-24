cmake_minimum_required(VERSION 3.8)
project(so101_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/gazebo_cube_attach.py
  scripts/simple_gripper_tf.py
  scripts/gazebo_pick_place_sequence.py
  scripts/wrist_color_detector.py
  scripts/wrist_vision_test_setup.py
  scripts/visual_servo_error.py
  scripts/visual_servo_pan_controller.py
  scripts/visual_servo_wrist_controller.py
  scripts/visual_servo_centering_sequence.py
  scripts/center_then_pick_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
