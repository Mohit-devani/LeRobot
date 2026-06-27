#include <memory>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


struct Target
{
  double dx;
  double dy;
  double dz;
  std::string name;
};


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
  arm.setMaxVelocityScalingFactor(0.10);
  arm.setMaxAccelerationScalingFactor(0.10);
  arm.setGoalPositionTolerance(0.03);
  arm.setGoalOrientationTolerance(0.30);

  RCLCPP_INFO(node->get_logger(), "V3 SMALL IK MOVEMENT TEST STARTED");
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "Pose reference frame: %s", arm.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", arm.getEndEffectorLink().c_str());

  arm.setStartStateToCurrentState();

  auto current_pose_stamped = arm.getCurrentPose("gripper_frame_link");
  auto target_pose = current_pose_stamped.pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose frame=%s x=%.3f y=%.3f z=%.3f",
    current_pose_stamped.header.frame_id.c_str(),
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z
  );

  std::vector<Target> targets = {
    {-0.01, 0.00, 0.00, "INWARD_1CM"},
    {-0.02, 0.00, 0.00, "INWARD_2CM"},
    {-0.01, 0.00, -0.01, "INWARD_1CM_DOWN_1CM"},
    {-0.02, 0.00, -0.01, "INWARD_2CM_DOWN_1CM"},
    {0.00, 0.00, -0.01, "DOWN_1CM"}
  };

  for (const auto& target : targets) {
    auto pose = target_pose;

    pose.position.x += target.dx;
    pose.position.y += target.dy;
    pose.position.z += target.dz;

    RCLCPP_INFO(
      node->get_logger(),
      "Trying IK target %s: x=%.3f y=%.3f z=%.3f",
      target.name.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z
    );

    arm.clearPoseTargets();
    arm.setStartStateToCurrentState();

    bool ik_ok = arm.setJointValueTarget(
      pose,
      "gripper_frame_link"
    );

    if (!ik_ok) {
      RCLCPP_WARN(node->get_logger(), "IK failed: %s", target.name.c_str());
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "IK solved: %s", target.name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_ok = static_cast<bool>(arm.plan(plan));

    if (!plan_ok) {
      RCLCPP_WARN(node->get_logger(), "Planning failed: %s", target.name.c_str());
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "Planning succeeded: %s", target.name.c_str());
    RCLCPP_INFO(node->get_logger(), "Executing small IK movement: %s", target.name.c_str());

    auto result = arm.execute(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Execution failed: %s", target.name.c_str());
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "V3 SMALL IK MOVEMENT TEST COMPLETE: %s", target.name.c_str());

    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  RCLCPP_ERROR(node->get_logger(), "All small IK movement targets failed");

  rclcpp::shutdown();
  spinner.join();
  return 1;
}
