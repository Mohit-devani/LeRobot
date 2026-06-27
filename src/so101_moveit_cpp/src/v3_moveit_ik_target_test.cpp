#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


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
  arm.setGoalPositionTolerance(0.05);
  arm.setGoalOrientationTolerance(0.50);

  RCLCPP_INFO(node->get_logger(), "V3 IK DIAGNOSTIC STARTED");
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "Pose reference frame: %s", arm.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", arm.getEndEffectorLink().c_str());

  arm.setStartStateToCurrentState();

  auto current_pose_stamped = arm.getCurrentPose("gripper_frame_link");
  auto current_pose = current_pose_stamped.pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose frame=%s x=%.3f y=%.3f z=%.3f",
    current_pose_stamped.header.frame_id.c_str(),
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z
  );

  RCLCPP_INFO(node->get_logger(), "Trying exact current pose IK");

  bool ik_ok = arm.setJointValueTarget(
    current_pose,
    "gripper_frame_link"
  );

  if (!ik_ok) {
    RCLCPP_ERROR(node->get_logger(), "IK failed even for exact current pose");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "IK solved for exact current pose");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_ok = static_cast<bool>(arm.plan(plan));

  if (!plan_ok) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed after exact current pose IK");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded for exact current pose");
  RCLCPP_INFO(node->get_logger(), "Executing exact current pose plan");

  auto result = arm.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed for exact current pose");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "V3 IK DIAGNOSTIC COMPLETE");

  rclcpp::shutdown();
  spinner.join();

  return 0;
}
