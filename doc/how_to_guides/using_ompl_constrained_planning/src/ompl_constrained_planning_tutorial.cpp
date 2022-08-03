#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/color_rgba.hpp>

static const auto LOGGER = rclcpp::get_logger("ompl_constrained_planning_demo");
int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ompl_constrained_planning_demo_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // For the move_group_interface to plan and spin separately
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "panda_arm");

  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };

  rclcpp::sleep_for(1s);

  // Create some helpful lambdas
  auto current_pose = move_group_interface.getCurrentPose();

  // Creates a pose at a given positional offset from the current pose
  auto get_relative_pose = [current_pose, &moveit_visual_tools](double x, double y, double z) {
    auto target_pose = current_pose;
    target_pose.pose.position.x += x;
    target_pose.pose.position.y += y;
    target_pose.pose.position.z += z;
    moveit_visual_tools.publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
    moveit_visual_tools.publishSphere(target_pose.pose, rviz_visual_tools::GREEN, 0.05);
    moveit_visual_tools.trigger();
    return target_pose;
  };

  // Resets the demo by cleaning up any constraints and markers
  auto reset_demo = [&move_group_interface, &moveit_visual_tools]() {
    move_group_interface.clearPathConstraints();
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();
  };

  reset_demo();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to start with the box constraint example");

  // Create the first planning problem
  auto target_pose = get_relative_pose(0.0, 0.3, -0.3);

  // Let's try the simple box constraints first!
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  box_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.1, 0.4, 0.4 };
  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = current_pose.pose.position.x;
  box_pose.position.y = 0.15;
  box_pose.position.z = 0.45;
  box_pose.orientation.w = 1.0;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                              box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                              box_pose.position.z + box.dimensions[2] / 2);
  moveit_visual_tools.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools.trigger();

  // We need to wrap the constraints in a generic `Constraints` message.
  moveit_msgs::msg::Constraints box_constraints;
  //  box_constraints.position_constraints.emplace_back(box_constraint);

  // Don't forget the path constraints! That's the whole point of this tutorial.
  move_group_interface.setPathConstraints(box_constraints);

  // Now we have everything we need to configure and solve a planning problem - plan to the target pose
  move_group_interface.setPoseTarget(target_pose);
  move_group_interface.setPlanningTime(10.0);

  // And let the planner find a solution.
  // The move_group node should automatically visualize the solution in Rviz if a path is found.
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Plan multiple times, printing the duration of each plan
  typedef std::chrono::high_resolution_clock Clock;
  for (size_t attempt = 0; attempt < 100; ++attempt)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Attempt: " << attempt);
    auto start_time = Clock::now();
    auto success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    auto end_time = Clock::now();
    RCLCPP_INFO_STREAM(
        LOGGER,
        "Planning time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());
  }

  // Done!
  moveit_visual_tools.prompt("Press 'Next' to finish");
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();
  move_group_interface.clearPathConstraints();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
