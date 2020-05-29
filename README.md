# sample_grasp_executor
An example node for executing a grasp using the remote_manipulation_markers package.

The code is designed to work with the GT-RAIL fork of the kinova-ros package for the 7-DOF Jaco2 arm with a robotiq-85 2-finger gripper, but the code has comments throughout denoting each module that would need to be replaced for different hardware.

To run the code on your own platform, you can optionally set the name of the MoveIt! move group for your robot arm, and the name of the gripper control action server, as follows:
```
rosrun SampleGraspExecutor sample_grasp_executor _move_group_name:=your_move_group_name _gripper_client:=your_gripper_client_topic_name
```
