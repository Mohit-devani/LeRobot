#include <memory>
#include <string>
#include <thread>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


void publish_state(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
  const std::string& state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  publisher->publish(msg);
}


bool move_to_joint_target(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::map<std::string, double>& target,
  const std::string& name)
{
  auto logger = rclcpp::get_logger("moveit_reach_cube_position");

  RCLCPP_INFO(logger, "Planning joint target: %s", name.c_str());

  move_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  if (!success) {
    RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
    return false;
  }

  auto result = move_group.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Reached: %s", name.c_str());
  return true;
}


bool move_to_position_target(
  moveit::planning_interface::MoveGroupInterface& arm,
  double x,
  double y,
  double z,
  const std::string& name)
{
  auto logger = rclcpp::get_logger("moveit_reach_cube_position");

  RCLCPP_INFO(
    logger,
    "Planning position target %s: x=%.3f y=%.3f z=%.3f",
    name.c_str(),
    x,
    y,
    z
  );

  arm.setStartStateToCurrentState();

  bool target_ok = arm.setPositionTarget(
    x,
    y,
    z,
    "gripper_frame_link"
  );

  if (!target_ok) {
    RCLCPP_ERROR(logger, "setPositionTarget failed: %s", name.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm.plan(plan));

  if (!success) {
    RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
    return false;
  }

  auto result = arm.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Reached position target: %s", name.c_str());
  return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "moveit_reach_cube_position",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  auto state_pub = node->create_publisher<std_msgs::msg::String>(
    "/pick_place_state",
    10
  );

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");

  arm.setEndEffectorLink("gripper_frame_link");
  arm.setPlanningTime(8.0);
  arm.setNumPlanningAttempts(20);
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setGoalPositionTolerance(0.04);

  gripper.setPlanningTime(3.0);
  gripper.setMaxVelocityScalingFactor(0.3);
  gripper.setMaxAccelerationScalingFactor(0.3);

  RCLCPP_INFO(node->get_logger(), "Waiting for cube_link TF...");

  geometry_msgs::msg::TransformStamped cube_tf;

  try {
    cube_tf = tf_buffer.lookupTransform(
      "base_link",
      "cube_link",
      tf2::TimePointZero,
      tf2::durationFromSec(3.0)
    );
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger(), "Could not read cube_link TF: %s", ex.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  double cube_x = cube_tf.transform.translation.x;
  double cube_y = cube_tf.transform.translation.y;
  double cube_z = cube_tf.transform.translation.z;

  RCLCPP_INFO(
    node->get_logger(),
    "Cube position: x=%.3f y=%.3f z=%.3f",
    cube_x,
    cube_y,
    cube_z
  );

  std::map<std::string, double> home = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> place = {
    {"shoulder_pan", 0.80},
    {"shoulder_lift", 0.35},
    {"elbow_flex", 0.50},
    {"wrist_flex", 0.20},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> gripper_open = {
    {"gripper", 0.25}
  };

  std::map<std::string, double> gripper_closed = {
    {"gripper", -0.15}
  };

  double pre_pick_z = cube_z + 0.16;
  double pick_z = cube_z + 0.07;
  double lift_z = cube_z + 0.20;

  publish_state(state_pub, "START");

  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  move_to_joint_target(arm, home, "HOME");

  bool pre_pick_ok = move_to_position_target(
    arm,
    cube_x,
    cube_y,
    pre_pick_z,
    "PRE_PICK_ABOVE_CUBE"
  );

  if (!pre_pick_ok) {
    RCLCPP_ERROR(node->get_logger(), "Pre-pick failed. Stopping.");
    publish_state(state_pub, "IDLE");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool pick_ok = move_to_position_target(
    arm,
    cube_x,
    cube_y,
    pick_z,
    "PICK_AT_CUBE"
  );

  if (!pick_ok) {
    RCLCPP_ERROR(node->get_logger(), "Pick pose failed. Stopping.");
    publish_state(state_pub, "IDLE");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  move_to_joint_target(gripper, gripper_closed, "GRIPPER_CLOSE");
  publish_state(state_pub, "GRIP_CLOSE");

  move_to_position_target(
    arm,
    cube_x,
    cube_y,
    lift_z,
    "LIFT_CUBE"
  );

  move_to_joint_target(arm, place, "PLACE");
  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  publish_state(state_pub, "GRIP_OPEN");

  move_to_joint_target(arm, home, "HOME");

  publish_state(state_pub, "IDLE");

  RCLCPP_INFO(node->get_logger(), "IK-style cube position pick-place complete.");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
