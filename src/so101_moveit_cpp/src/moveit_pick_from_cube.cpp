#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}


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
  auto logger = rclcpp::get_logger("moveit_pick_from_cube");

  RCLCPP_INFO(logger, "Planning to: %s", name.c_str());

  move_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (!success) {
    RCLCPP_ERROR(logger, "Planning failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Executing: %s", name.c_str());

  auto result = move_group.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Reached: %s", name.c_str());
  return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "moveit_pick_from_cube",
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

  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

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
    "Cube coordinates: x=%.3f, y=%.3f, z=%.3f",
    cube_x,
    cube_y,
    cube_z
  );

  double distance = std::sqrt(cube_x * cube_x + cube_y * cube_y);

  // SO101 visual direction correction:
  // y negative should rotate shoulder positive.
  double pick_angle = -std::atan2(cube_y, cube_x);
  pick_angle = clamp(pick_angle, -0.90, 0.90);

  double reach = (distance - 0.22) / 0.12;
  reach = clamp(reach, 0.0, 1.0);

  double shoulder_pick = 0.32 + 0.12 * reach;
  double elbow_pick = 0.55 + 0.20 * reach;
  double wrist_pick = 0.25 + 0.15 * reach;

  double shoulder_pre = shoulder_pick * 0.70;
  double elbow_pre = elbow_pick * 0.70;
  double wrist_pre = wrist_pick * 0.60;

  RCLCPP_INFO(
    node->get_logger(),
    "Computed pick_angle=%.3f, distance=%.3f, reach=%.3f",
    pick_angle,
    distance,
    reach
  );

  std::map<std::string, double> home = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pre_pick = {
    {"shoulder_pan", pick_angle},
    {"shoulder_lift", shoulder_pre},
    {"elbow_flex", elbow_pre},
    {"wrist_flex", wrist_pre},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pick = {
    {"shoulder_pan", pick_angle},
    {"shoulder_lift", shoulder_pick},
    {"elbow_flex", elbow_pick},
    {"wrist_flex", wrist_pick},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> lift = {
    {"shoulder_pan", pick_angle},
    {"shoulder_lift", 0.20},
    {"elbow_flex", 0.35},
    {"wrist_flex", 0.10},
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

  publish_state(state_pub, "START");

  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  move_to_joint_target(arm, home, "HOME");
  move_to_joint_target(arm, pre_pick, "PRE_PICK_FROM_CUBE");
  move_to_joint_target(arm, pick, "PICK_FROM_CUBE");

  move_to_joint_target(gripper, gripper_closed, "GRIPPER_CLOSE");
  publish_state(state_pub, "GRIP_CLOSE");

  move_to_joint_target(arm, lift, "LIFT");
  move_to_joint_target(arm, place, "PLACE");

  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  publish_state(state_pub, "GRIP_OPEN");

  move_to_joint_target(arm, home, "HOME");

  publish_state(state_pub, "IDLE");

  RCLCPP_INFO(node->get_logger(), "MoveIt cube-coordinate pick-place complete.");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
