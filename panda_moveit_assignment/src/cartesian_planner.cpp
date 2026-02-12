#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using moveit::planning_interface::MoveGroupInterface;

// Add collision object
void addCollisionObject(
    const rclcpp::Logger &logger,
    moveit::planning_interface::PlanningSceneInterface &psi)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "panda_link0";
  obj.id = "Box_0";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.70, 0.70, 0.70};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.75;
  pose.position.y = 0.10;
  pose.position.z = 0.35;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  psi.applyCollisionObject(obj);
  RCLCPP_INFO(logger, "Collision object added.");
}

// Compute and execute one Cartesian segment
bool executeCartesianSegment(
    const rclcpp::Logger &logger,
    MoveGroupInterface &move_group,
    const geometry_msgs::msg::Pose &target_pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      0.01,
      trajectory,
      true);

  RCLCPP_INFO(logger, "Segment fraction: %.2f", fraction);

  if (fraction < 0.95)
  {
    RCLCPP_ERROR(logger, "Collision detected — planning failed.");
    return false;
  }

  MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  auto result = move_group.execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execution failed.");
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "panda_cartesian_node",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroupInterface move_group(node, "panda_arm");

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  moveit::planning_interface::PlanningSceneInterface psi;
  addCollisionObject(node->get_logger(), psi);

  auto current_state = move_group.getCurrentState(5.0);

  if (!current_state)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to fetch robot state.");
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  move_group.setNamedTarget("ready");
  MoveGroupInterface::Plan ready_plan;

  if (move_group.plan(ready_plan) ==
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.execute(ready_plan);
  }

  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose start_pose =
      move_group.getCurrentPose().pose;

  geometry_msgs::msg::Pose waypoint2 = start_pose;
  waypoint2.position.y += 0.15;

  geometry_msgs::msg::Pose waypoint3 = waypoint2;
  waypoint3.position.x += 0.50;

  RCLCPP_INFO(node->get_logger(), "Executing segment 1→2");

  if (!executeCartesianSegment(node->get_logger(),
                               move_group,
                               waypoint2))
  {
    rclcpp::shutdown();
    spinner.join();
    return 0;
  }

  move_group.setStartStateToCurrentState();

  RCLCPP_INFO(node->get_logger(), "Planning segment 2→3");

  bool success = executeCartesianSegment(node->get_logger(),
                                         move_group,
                                         waypoint3);

  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Collision with Box_0 detected — planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
