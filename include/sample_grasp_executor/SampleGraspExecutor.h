#ifndef SAMPLE_GRASP_EXECUTOR_H_
#define SAMPLE_GRASP_EXECUTOR_H_

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class SampleGraspExecutor
{

public:

  /**
   * \brief Constructor
   */
  SampleGraspExecutor();

  ~SampleGraspExecutor();

private:
  void executeGraspCallback(const rail_manipulation_msgs::PickupGoalConstPtr &goal);

  ros::NodeHandle n;
  ros::NodeHandle pnh;

  // topics
  ros::Publisher planning_scene_publisher;

  ros::Publisher approach_debug_pub;
  ros::Publisher grasp_debug_pub;

  // services
  ros::ServiceClient compute_cartesian_path_client;
  ros::ServiceClient planning_scene_client;

  // actionlib
  actionlib::SimpleActionServer<rail_manipulation_msgs::PickupAction> pickup_server;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> *gripper_client;

  // MoveIt! interfaces
  moveit::planning_interface::MoveGroupInterface *arm_group;
  moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
  std::string move_group_name;

  // tf
  tf2_ros::TransformBroadcaster tf_broadcaster;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // collision tracking
  std::vector<std::string> gripper_names;
  std::vector<std::string> attached_objects;
};

#endif  // SAMPLE_GRASP_EXECUTOR_H_
