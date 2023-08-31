/*****************************************************************
  Manipulation Class Definition
*****************************************************************/

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PickupAction.h>


#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/GripperCommandActionGoal.h>

#include <fstream>

#include <visualization_msgs/Marker.h>

// Dynamic pointers for manipulation
typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
private:
  ros::Subscriber grasp_config; // GPD msg subscriber

  const double pi = std::acos(-1);  // Pi constant

  const std::string BASE_LINK_NAME  = "base_link";
  const std::string GRASP_LINK_NAME = "anygrasp/grasp_0";

  // Move arm functions:
  void setJointGroup(double j0, double j1, double j2,
                     double j3, double j4, double j5, double j6);
  void getCurrentState();
  void move(std::vector<double>);

  tf::TransformListener grasp_listener;


public:
  // data collection vars, for testing
  ros::Time begin;
  ros::Time end;

  // Gripper
  ros::Publisher gripper_command;
  ros::Publisher grasps_visualization_pub_;

  // Constructor
  Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);

  // Moveit planning variables
  moveit::core::RobotStatePtr current_state;
  std::vector<double> joint_group_positions;
  std::string PLANNING_GROUP;
  PlanningScenePtr planning_scene_ptr;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  MoveGroupPtr move_group_ptr;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Hard coded arm goTo functions
  void goTop();
  void goRight();
  void goLeft();
  void goVertical();
  void goWait();
  void goPlace();
  void goPostGrasp();
  void lastJointRotation(double rotate_angle);

  // Collision Objects
  void set_objects();
  void remove_objects();

  // Gripper commands, pick pipeline
  void closedGripper(trajectory_msgs::JointTrajectory &);
  void openGripper(trajectory_msgs::JointTrajectory &);

  // Gripper Action commands
  void close_gripper();
  void open_gripper();
  control_msgs::GripperCommandActionGoal gripper_cmd;

  /* GPD Functions */
  // void callback(const gpd_ros::GraspConfigList msg);
  void path_planning();
  void set_target_pose();
  void plan_pose_goal();
  void pick_and_place();
  void pickup();
  // void place(float);

  // For AnyGrasp
  void reach_anygrasp();
  tf::StampedTransform getTransform(tf::TransformListener & listener, 
                                    std::string target_frame, 
                                    std::string source_frame);
  void goSnapshotPostion();
  void ompl_plan(double x, double y, double z);
  void get_utensil();
  void trajectory_saving();
  void trajectory_replay(std::string waypoints_path);
  void goTemp(); // after get bowl handle
  void godown();

  // GPD pre/post grasp transform function  
  // std::vector<geometry_msgs::Pose> gpd_grasp_to_pose(gpd_ros::GraspConfig &);

  // GPD grasp atributes
  bool getting_grasps = true; // Flag, used for multiple runs
  bool pose_success;
  // gpd_ros::GraspConfigList grasp_candidates;
  // gpd_ros::GraspConfig grasp;
  std_msgs::Float32 score;

  bool grabbed_object;  // Flags a good grasping plan
  
  // Planning variables
  geometry_msgs::Pose target_pose;
  geometry_msgs::Vector3 orientation;
  geometry_msgs::Vector3 axis;
  geometry_msgs::Point pose;
  geometry_msgs::Point position;
  geometry_msgs::Point pose_sample;
  geometry_msgs::Vector3 grasp_orientation;
  moveit_msgs::CollisionObject collision_object;
  tf2::Quaternion q;
};

#endif