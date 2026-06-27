#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


struct PositionTarget
{
  double dx;
  double dy;
  double dz;
  std::string name;
};


bool move_to_joint_target(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& arm,
  const std::map<std::string, double>& target,
  const std::string& name)
{
  RCLCPP_INFO(node->get_logger(), "Planning joint target: %s", name.c_str());

  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_ok = static_cast<bool>(arm.plan(plan));

  if (!plan_ok) {
    RCLCPP_ERROR(node->get_logger(), "Joint planning failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Executing joint target: %s", name.c_str());

  auto result = arm.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Joint execution failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Reached joint target: %s", name.c_str());
  return true;
}


bool move_position_only_ik_after_pan(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& arm)
{
  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();

  auto current_pose_stamped = arm.getCurrentPose("gripper_frame_link");
  auto current_pose = current_pose_stamped.pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose after pan: x=%.3f y=%.3f z=%.3f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z
  );

  std::vector<PositionTarget> targets = {
    {-0.01, 0.00, 0.00, "POSITION_ONLY_INWARD_1CM"},
    {-0.02, 0.00, 0.00, "POSITION_ONLY_INWARD_2CM"},
    {-0.03, 0.00, 0.00, "POSITION_ONLY_INWARD_3CM"},
    {-0.01, 0.00, -0.01, "POSITION_ONLY_INWARD_1CM_DOWN_1CM"},
    {-0.02, 0.00, -0.01, "POSITION_ONLY_INWARD_2CM_DOWN_1CM"},
    {0.00, 0.00, -0.01, "POSITION_ONLY_DOWN_1CM"}
  };

  for (const auto& target : targets) {
    double x = current_pose.position.x + target.dx;
    double y = current_pose.position.y + target.dy;
    double z = current_pose.position.z + target.dz;

    RCLCPP_INFO(
      node->get_logger(),
      "Trying position-only IK target %s: x=%.3f y=%.3f z=%.3f",
      target.name.c_str(),
      x,
      y,
      z
    );

    arm.clearPoseTargets();
    arm.setStartStateToCurrentState();

    bool target_ok = arm.setPositionTarget(
      x,
      y,
      z,
      "gripper_frame_link"
    );

    if (!target_ok) {
      RCLCPP_WARN(node->get_logger(), "setPositionTarget failed: %s", target.name.c_str());
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = static_cast<bool>(arm.plan(plan));

    if (!plan_ok) {
      RCLCPP_WARN(node->get_logger(), "Position-only IK planning failed: %s", target.name.c_str());
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "Position-only IK planning succeeded: %s", target.name.c_str());
    RCLCPP_INFO(node->get_logger(), "Executing position-only IK target: %s", target.name.c_str());

    auto result = arm.execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Position-only IK execution failed: %s", target.name.c_str());
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "Position-only IK reached: %s", target.name.c_str());
    return true;
  }

  RCLCPP_ERROR(node->get_logger(), "All position-only IK targets failed");
  return false;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "v3_moveit_ik_target_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  arm.setEndEffectorLink("gripper_frame_link");
  arm.setPoseReferenceFrame("base_link");
  arm.setPlanningTime(8.0);
  arm.setNumPlanningAttempts(20);
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setGoalPositionTolerance(0.04);
  arm.setGoalOrientationTolerance(1.57);

  RCLCPP_INFO(node->get_logger(), "V3 PAN PLUS POSITION-ONLY IK TEST STARTED");

  double cube_x = 0.32;
  double cube_y = 0.10;

  double pan = -std::atan2(cube_y, cube_x);

  RCLCPP_INFO(
    node->get_logger(),
    "Cube estimate x=%.3f y=%.3f -> shoulder_pan=%.3f",
    cube_x,
    cube_y,
    pan
  );

  std::map<std::string, double> pan_to_cube = {
    {"shoulder_pan", pan},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  bool pan_ok = move_to_joint_target(
    node,
    arm,
    pan_to_cube,
    "PAN_TOWARD_CUBE"
  );

  if (!pan_ok) {
    RCLCPP_ERROR(node->get_logger(), "PAN_TOWARD_CUBE failed");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  bool position_ik_ok = move_position_only_ik_after_pan(node, arm);

  if (!position_ik_ok) {
    RCLCPP_ERROR(node->get_logger(), "PAN_PLUS_POSITION_ONLY_IK failed");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "V3 PAN PLUS POSITION-ONLY IK TEST COMPLETE");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
