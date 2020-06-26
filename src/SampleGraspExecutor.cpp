#include <sample_grasp_executor/SampleGraspExecutor.h>

using namespace std;

SampleGraspExecutor::SampleGraspExecutor() :
    pnh("~"), tf_listener(tf_buffer),
    pickup_server(pnh, "execute_grasp", boost::bind(&SampleGraspExecutor::executeGraspCallback, this, _1), false)
{
  //read in parameters
  string gripper_client_name;
  pnh.param<string>("move_group_name", move_group_name, "arm");
  pnh.param<string>("gripper_client", gripper_client_name, "gripper_actions/gripper_manipulation");
  pnh.param<bool>("z_forward", z_forward, true);

  // set up link names
  // TODO: This may need to be changed for your specific gripper hardware.  It's used to allow the gripper to make contact
  // TODO: with objects in the environment for grasping.
  gripper_names.push_back("robotiq_85_base_link");
  gripper_names.push_back("robotiq_85_left_finger_link");
  gripper_names.push_back("robotiq_85_left_finger_tip_link");
  gripper_names.push_back("robotiq_85_left_inner_knuckle_link");
  gripper_names.push_back("robotiq_85_left_knuckle_link");
  gripper_names.push_back("robotiq_85_right_finger_link");
  gripper_names.push_back("robotiq_85_right_finger_tip_link");
  gripper_names.push_back("robotiq_85_right_inner_knuckle_link");
  gripper_names.push_back("robotiq_85_right_knuckle_link");

  // topics
  planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

  approach_debug_pub = pnh.advertise<geometry_msgs::PoseStamped>("approach_pose", 1);
  grasp_debug_pub = pnh.advertise<geometry_msgs::PoseStamped>("grasp_pose", 1);

  // services
  compute_cartesian_path_client = n.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");
  planning_scene_client = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  //actionlib
  // TODO: this may change for your gripper; we use the interface provided by GT-RAIL/robotiq_85_gripper_actions
  // TODO: and GT-RAIL/robotiq_85_gripper
  gripper_client = new actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction>(gripper_client_name);

  arm_group = new moveit::planning_interface::MoveGroupInterface(move_group_name);
  arm_group->startStateMonitor();

  planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

  pickup_server.start();
}

SampleGraspExecutor::~SampleGraspExecutor()
{
  delete gripper_client;
  delete arm_group;
  delete planning_scene_interface;
}

void SampleGraspExecutor::executeGraspCallback(const rail_manipulation_msgs::PickupGoalConstPtr &goal)
{
  rail_manipulation_msgs::PickupResult result;

  // STEP 1: set all poses in appropriate reference frames that we'll need

  string group_reference_frame = arm_group->getPoseReferenceFrame();
//  string group_reference_frame = "world_frame";  // Note: code for debugging without hardware/simulated execution

  //transform pose to reference group coordinate frame (fixes an annoying bug that spams warnings to the terminal...)
  geometry_msgs::PoseStamped grasp_pose;
  if (goal->pose.header.frame_id != group_reference_frame)
  {
    grasp_pose.header.stamp = ros::Time(0);
    grasp_pose.header.frame_id = group_reference_frame;

    geometry_msgs::TransformStamped group_to_grasp_transform = tf_buffer.lookupTransform(group_reference_frame,
                                                                                         goal->pose.header.frame_id,
                                                                                         ros::Time(0),
                                                                                         ros::Duration(1.0));
    tf2::doTransform(goal->pose, grasp_pose, group_to_grasp_transform);
  } else
  {
    grasp_pose = goal->pose;
  }

  ros::Time current_time = ros::Time::now();
  geometry_msgs::TransformStamped grasp_transform;
  grasp_transform.child_frame_id = "grasp_execution_frame";
  grasp_transform.header.frame_id = grasp_pose.header.frame_id;
  grasp_transform.header.stamp = current_time;
  grasp_transform.transform.translation.x = grasp_pose.pose.position.x;
  grasp_transform.transform.translation.y = grasp_pose.pose.position.y;
  grasp_transform.transform.translation.z = grasp_pose.pose.position.z;
  grasp_transform.transform.rotation = grasp_pose.pose.orientation;
  tf_broadcaster.sendTransform(grasp_transform);

  //calculate grasp pose in frame from goal grasp pose
  geometry_msgs::PoseStamped grasp_execution_pose;
  grasp_execution_pose.header.frame_id = "grasp_execution_frame";
  if (z_forward)  // necessary for the eef coordinate frame for the GEN3 JACO
  {
    grasp_execution_pose.pose.orientation.w = 0.7071;
    grasp_execution_pose.pose.orientation.y = 0.7071;
  }
  else
  {
    grasp_execution_pose.pose.orientation.w = 1.0;
  }

  geometry_msgs::TransformStamped from_grasp_transform =
      tf_buffer.lookupTransform(group_reference_frame, "grasp_execution_frame", current_time, ros::Duration(3.0));
  geometry_msgs::PoseStamped transformed_grasp_pose;
  transformed_grasp_pose.header.frame_id = group_reference_frame;
  tf2::doTransform(grasp_execution_pose, transformed_grasp_pose, from_grasp_transform);

  //calculate approach pose from goal grasp pose
  grasp_execution_pose.pose.position.x -= 0.12;
  geometry_msgs::PoseStamped transformed_approach_pose;
  transformed_approach_pose.header.frame_id = group_reference_frame;
  tf2::doTransform(grasp_execution_pose, transformed_approach_pose, from_grasp_transform);

  approach_debug_pub.publish(transformed_approach_pose);
  grasp_debug_pub.publish(transformed_grasp_pose);

  // STEP 2: Plan and move to the grasp approach pose

  //plan and move to approach pose
  stringstream ss("");
  ss << move_group_name << "[RRTConnectkConfigDefault]";
  arm_group->setPlannerId(ss.str());
  arm_group->setPlanningTime(3.0);
  arm_group->setStartStateToCurrentState();
  arm_group->setPoseTarget(transformed_approach_pose);

  //send the goal, and also check for preempts
  int error_code = arm_group->move().val;
  if (error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while moving to approach pose.");
    result.success = false;
    result.executionSuccess = false;
    pickup_server.setPreempted(result);
    return;
  }
  else if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to approach pose.");
    result.success = false;
    result.executionSuccess = false;
    pickup_server.setAborted(result);
    return;
  }


  // STEP 3: Open gripper
  // TODO: change to appropriate gripper command for your hardware
  rail_manipulation_msgs::GripperGoal gripper_goal;
  gripper_goal.close = false;
  gripper_client->sendGoal(gripper_goal);
  gripper_client->waitForResult(ros::Duration(10.0));
  if (!gripper_client->getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.success = false;
    result.executionSuccess = false;
    pickup_server.setAborted(result, "Unable to open gripper.");
    return;
  }

  // STEP 4: Disable collision between gripper links and object
  moveit_msgs::GetPlanningScene planning_scene_srv;
  vector<string> collision_objects;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!planning_scene_client.call(planning_scene_srv))
  {
    ROS_INFO("Could not get the current planning scene!");
  }
  else
  {
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);
    // disable collisions between gripper links and octomap
    acm.setEntry("<octomap>", gripper_names, true);
    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher.publish(planning_scene_update);

    ros::Duration(0.5).sleep(); //delay for publish to go through
  }

  //STEP 5: plan and move to grasp pose
  moveit_msgs::GetCartesianPath grasp_path;
  grasp_path.request.waypoints.push_back(transformed_grasp_pose.pose);
  grasp_path.request.max_step = 0.01;
  grasp_path.request.jump_threshold = 1.5;  // From nimbus
  grasp_path.request.avoid_collisions = false;
  grasp_path.request.group_name = move_group_name;
  moveit::core::robotStateToRobotStateMsg(*(arm_group->getCurrentState()), grasp_path.request.start_state);

  int max_planning_attempts = 5;
  for (int num_attempts=0; num_attempts < max_planning_attempts; num_attempts++)
  {
    ROS_INFO("Attempting to plan path to grasp. Attempt: %d/%d",
             num_attempts + 1, max_planning_attempts);
    if (grasp_path.response.fraction >= 0.5)
    {
      ROS_INFO("Succeeded in computing %f of the path to grasp",
               grasp_path.response.fraction);
      break;
    }
    else if (!compute_cartesian_path_client.call(grasp_path)
             || grasp_path.response.fraction < 0
             || num_attempts >= max_planning_attempts - 1)
    {
      ROS_INFO("Could not calculate a Cartesian path for grasp!");
      result.executionSuccess = false;
      result.success = false;
      pickup_server.setAborted(result);
      return;
    }
  }

  //execute the grasp plan
  moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
  grasp_plan.trajectory_ = grasp_path.response.solution;
  moveit::core::robotStateToRobotStateMsg(*(arm_group->getCurrentState()), grasp_plan.start_state_);
  error_code = arm_group->execute(grasp_plan).val;
  if (error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    ROS_INFO("Preempted while moving to executing grasp.");
    result.executionSuccess = false;
    result.success = false;
    pickup_server.setAborted(result);
    return;
  }
  else if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Failed to move to execute grasp.");
    result.executionSuccess = false;
    result.success = false;
    pickup_server.setAborted(result);
    return;
  }

  //STEP 6: Close gripper
  // TODO: change to appropriate gripper command for your hardware
  gripper_goal.close = true;
  gripper_client->sendGoal(gripper_goal);
  gripper_client->waitForResult(ros::Duration(10.0));
  if (!gripper_client->getResult()->success)
  {
    ROS_INFO("Closing gripper failed.");
    result.success = false;
    result.executionSuccess = false;
    pickup_server.setAborted(result, "Unable to close gripper.");
    return;
  }

  //DONE
  result.success = true;
  result.executionSuccess = true;
  pickup_server.setSucceeded(result);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_grasp_executor");

  SampleGraspExecutor sge;

  ros::spin();
}
