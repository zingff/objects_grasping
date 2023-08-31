#include "manipulation_class.hpp"

void Manipulation::reach_anygrasp()
{

  tf::StampedTransform T_base_grasp;
  T_base_grasp = getTransform(grasp_listener, BASE_LINK_NAME, GRASP_LINK_NAME);
  geometry_msgs::Vector3 grasp_pose;
  tf::Matrix3x3 rotation_y;
  rotation_y.setRPY(0, M_PI / 2.0, 0); // Rotate 90 degrees about the y-axis
  tf::Matrix3x3 rotation_z;
  rotation_z.setRPY(0, 0, - M_PI / 2.0); // Rotate -90 degrees about the z-axis
  tf::Matrix3x3 grasp_orientation = T_base_grasp.getBasis();
  grasp_orientation = grasp_orientation * rotation_y * rotation_z;
  grasp_orientation.getRPY(grasp_pose.x, grasp_pose.y, grasp_pose.z);



  tf::Vector3 translation_offset(0, 0, 0.02);  // offset about z-axis
  tf::Vector3 grasp_translation(T_base_grasp.getOrigin().x(),
                                 T_base_grasp.getOrigin().y(),
                                 T_base_grasp.getOrigin().z());
  grasp_translation = grasp_orientation * grasp_translation + translation_offset;

  this->orientation = grasp_pose;
  this->position.x = T_base_grasp.getOrigin().x();
  this->position.y = T_base_grasp.getOrigin().y();  
  this->position.z = T_base_grasp.getOrigin().z() + 0.02; 
  // this->position.x = grasp_translation.x();
  // this->position.y = grasp_translation.y();  
  // this->position.z = grasp_translation.z();

  std::cout << grasp_translation.x() << std::endl;
  std::cout << grasp_translation.y() << std::endl;
  std::cout << grasp_translation.z() << std::endl;
  
  std::cout << grasp_pose.x*57.3 << std::endl;
  std::cout << grasp_pose.y*57.3 << std::endl;
  std::cout << grasp_pose.z*57.3 << std::endl;


  set_target_pose();
  plan_pose_goal();
  this->move_group_ptr->move();
  ros::Duration(2.0).sleep();

  close_gripper();

}