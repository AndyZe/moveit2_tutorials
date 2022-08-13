#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string LOGNAME = "moveit_cpp_tutorial";

  /* Otherwise robot with zeros joint_states */
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "moveit_cpp_tutorial",
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Start the next plan
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan around an obstacle");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // First we create the collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.4, 0.01, 0.01 };

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.15;
  box_pose.position.z = 0.6;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
  }  // Unlock PlanningScene

  // Planning start and goal
  planning_components->setStartStateToCurrentState();
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "panda_link0";
  target_pose1.pose.orientation.x = 0.924;
  target_pose1.pose.orientation.y = -0.382;
  target_pose1.pose.orientation.z = 0;
  target_pose1.pose.orientation.w = 0;
  target_pose1.pose.position.x = 0.28;
  target_pose1.pose.position.y = -0.4;
  target_pose1.pose.position.z = 0.5;
  planning_components->setGoal(target_pose1, "panda_link8");

  typedef std::chrono::high_resolution_clock Clock;
  auto t1 = Clock::now();
  auto plan_solution5 = planning_components->plan();
  auto t2 = Clock::now();
  RCLCPP_ERROR_STREAM(LOGGER, std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());

  if (plan_solution5)
  {
    visual_tools.publishText(text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    /* Uncomment if you want to execute the plan */
    /* planning_components->execute(); // Execute the plan */
  }

  // END_TUTORIAL
  visual_tools.prompt("Press 'next' to end the demo");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
