#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


void publish_state(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
  const std::string& state)
{
  std_msgs::msg::String msg;
  msg.data = state;
  publisher->publish(msg);
}


bool move_to_joint_target(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& group,
  const std::map<std::string, double>& target,
  const std::string& name)
{
  RCLCPP_INFO(node->get_logger(), "Planning joint target: %s", name.c_str());

  group.clearPoseTargets();
  group.setStartStateToCurrentState();
  group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_ok = static_cast<bool>(group.plan(plan));

  if (!plan_ok) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Executing joint target: %s", name.c_str());

  auto result = group.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed: %s", name.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Reached joint target: %s", name.c_str());
  return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "v3_moveit_cube_joint_pick",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  bool got_cube_pose = false;
  double cube_x = 0.0;
  double cube_y = 0.0;
  double cube_z = 0.0;

  auto cube_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pick_cube_pose",
    10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      cube_x = msg->pose.position.x;
      cube_y = msg->pose.position.y;
      cube_z = msg->pose.position.z;
      got_cube_pose = true;
    }
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

  RCLCPP_INFO(node->get_logger(), "V3 MOVEIT CUBE JOINT PICK STARTED");
  RCLCPP_INFO(node->get_logger(), "Waiting for /pick_cube_pose...");

  auto wait_start = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !got_cube_pose) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - wait_start).count();

    if (elapsed > 8) {
      RCLCPP_ERROR(node->get_logger(), "Timeout waiting for /pick_cube_pose");
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Received /pick_cube_pose: x=%.3f y=%.3f z=%.3f",
    cube_x,
    cube_y,
    cube_z
  );

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");

  arm.setPlanningTime(8.0);
  arm.setNumPlanningAttempts(20);
  arm.setMaxVelocityScalingFactor(0.60);
  arm.setMaxAccelerationScalingFactor(0.60);

  gripper.setPlanningTime(3.0);
  gripper.setMaxVelocityScalingFactor(0.80);
  gripper.setMaxAccelerationScalingFactor(0.80);

  double pan = -std::atan2(cube_y, cube_x);

  RCLCPP_INFO(
    node->get_logger(),
    "Live cube pose x=%.3f y=%.3f -> shoulder_pan=%.3f",
    cube_x,
    cube_y,
    pan
  );

  std::map<std::string, double> home = {
    {"shoulder_pan", 0.0},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pan_to_cube = {
    {"shoulder_pan", pan},
    {"shoulder_lift", 0.0},
    {"elbow_flex", 0.0},
    {"wrist_flex", 0.0},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> pre_grasp = {
    {"shoulder_pan", pan},
    {"shoulder_lift", -0.30},
    {"elbow_flex", 0.65},
    {"wrist_flex", 0.55},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> grasp = {
    {"shoulder_pan", pan},
    {"shoulder_lift", -0.12},
    {"elbow_flex", 0.42},
    {"wrist_flex", 0.35},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> lift = {
    {"shoulder_pan", pan},
    {"shoulder_lift", -0.25},
    {"elbow_flex", 0.60},
    {"wrist_flex", 0.45},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> place = {
    {"shoulder_pan", 0.75},
    {"shoulder_lift", -0.45},
    {"elbow_flex", 0.45},
    {"wrist_flex", -0.35},
    {"wrist_roll", 0.0}
  };

  std::map<std::string, double> gripper_open = {
    {"gripper", 0.25}
  };

  std::map<std::string, double> gripper_closed = {
    {"gripper", -0.15}
  };

  publish_state(state_pub, "START");

  if (!move_to_joint_target(node, gripper, gripper_open, "GRIPPER_OPEN")) return 1;
  if (!move_to_joint_target(node, arm, home, "HOME")) return 1;
  if (!move_to_joint_target(node, arm, pan_to_cube, "PAN_TOWARD_LIVE_CUBE")) return 1;
  if (!move_to_joint_target(node, arm, pre_grasp, "PRE_GRASP")) return 1;
  if (!move_to_joint_target(node, arm, grasp, "GRASP")) return 1;

  if (!move_to_joint_target(node, gripper, gripper_closed, "GRIPPER_CLOSE")) return 1;
  publish_state(state_pub, "GRIP_CLOSE");
  RCLCPP_INFO(node->get_logger(), "Published GRIP_CLOSE. Waiting for cube attach...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_joint_target(node, arm, lift, "LIFT")) return 1;
  if (!move_to_joint_target(node, arm, place, "PLACE")) return 1;

  if (!move_to_joint_target(node, gripper, gripper_open, "GRIPPER_OPEN_RELEASE")) return 1;
  publish_state(state_pub, "GRIP_OPEN");
  RCLCPP_INFO(node->get_logger(), "Published GRIP_OPEN. Waiting for cube release...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (!move_to_joint_target(node, arm, home, "HOME_END")) return 1;

  publish_state(state_pub, "IDLE");

  RCLCPP_INFO(node->get_logger(), "V3 MOVEIT LIVE CUBE POSE PICK COMPLETE");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
