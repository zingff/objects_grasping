# Pickup experiment with GPD
# Description
ROS package for implementing the grasping algorithms [GPD (Grasp Pose Detection)](https://github.com/atenpas/gpd) on Kinova Gen3 7DOF.

All works were done on Ubuntu 20.04 with ROS Noetic.

## Required ROS packages for the Gen3:


## Software and version
* [ros_kortex](https://github.com/Kinovarobotics/ros_kortex): any
* [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision): any
* [gpd_ros](https://github.com/atenpas/gpd_ros/): 2.0.0
* [opencv](https://opencv.org/): 4.2.0
* [PCL](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#): 1.10

## Actions for grasping task
* motionPlanning
  * getCurrentState
  * moveToJointConfig
    * rotateSingleJoint
    * moveToInitial
  * moveToCartesianPose
    * moveToPreGraspPose
    * moveToGraspPose
    * moveToPostGraspPose
  * gripperCommand
    * openGripper
    * closeGripper
    * openToGripperPosition
* perception
  * camera
    * takeSnapshot
  * pcl
    * passThroughFilter
    * planarSegmentation
    * voxelGrid
    * concatenatePointCloud
* GraspPose
  * generateGraspPose

Note: 
* When ticking `moveToJointConfig` and `moveToCartesianPose`, both actions callback `getCurrentState` to obtain current joint configuration (joint angles for each joint, i.e., $q_1$,...,$q_7$) and current Cartesian pose (position and orientation, i.e., x, y, z, $\theta_x$, $\theta_y$, $\theta_z$, of the end-effector).

TODO
* determine parameters ports, both in and out, for each action.

## Task sequence (workflow) for a single grasping task
* openGripper
* moveToInitial
* takeSnapShot
* passThroughFilter
* planarSegmentation
* voxelGrid
* generateGraspPose
* moveToPreGraspPose
* moveToGraspPose
* closeGripper
* moveToPreGraspPose
* moveToPostGraspPose