#include <memory>
#include <thread>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


bool move_to_joint_target(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::map<std::string, double>& target,
  const std::string& name)
{
  auto logger = rclcpp::get_logger("so101_moveit_sequence");

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
    "so101_moveit_sequence",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");

  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

  gripper.setPlanningTime(3.0);
  gripper.setMaxVelocityScalingFactor(0.3);
  gripper.setMaxAccelerationScalingFactor(0.3);

  std::map<std::string, double> home = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pre_pick = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.35},
    {"elbow_flex", 0.45},
    {"wrist_flex", 0.20},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pick = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.45},
    {"elbow_flex", 0.65},
    {"wrist_flex", 0.30},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> lift = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.20},
    {"elbow_flex", 0.35},
    {"wrist_flex", 0.10},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> place = {
    {"shoulder_pan", 0.8},
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

  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  move_to_joint_target(arm, home, "HOME");
  move_to_joint_target(arm, pre_pick, "PRE_PICK");
  move_to_joint_target(arm, pick, "PICK");
  move_to_joint_target(gripper, gripper_closed, "GRIPPER_CLOSE");
  move_to_joint_target(arm, lift, "LIFT");
  move_to_joint_target(arm, place, "PLACE");
  move_to_joint_target(gripper, gripper_open, "GRIPPER_OPEN");
  move_to_joint_target(arm, home, "HOME");

  RCLCPP_INFO(rclcpp::get_logger("so101_moveit_sequence"), "MoveIt pick-place sequence complete.");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
