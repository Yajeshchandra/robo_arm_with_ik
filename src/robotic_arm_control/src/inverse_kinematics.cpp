#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "inverse_kinematics");
  ros::NodeHandle nh;

  // Create a MoveIt! MoveGroupInterface object
  moveit::planning_interface::MoveGroupInterface move_group("arm");

  // Define the end effector link name
  std::string end_effector_link = "end_effector_link";

  // Create a PoseStamped message
  geometry_msgs::PoseStamped current_pose;
  current_pose.header.frame_id = "base_link";

  // Read parameters from the ROS parameter server
  double x, y, z;
  nh.getParam("x", x);
  nh.getParam("y", y);
  nh.getParam("z", z);

  current_pose.pose.position.x = x;
  current_pose.pose.position.y = y;
  current_pose.pose.position.z = z;

  // Set the pose target
  move_group.setPoseTarget(current_pose, end_effector_link);

  // Create a Plan object
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Plan the motion
  move_group.plan(plan);

  // Execute the planned motion
  move_group.execute(plan);

  return 0;
}