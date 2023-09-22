#include "manipulation_class.hpp"

/**
 * @brief this function will rotate the last joint of the manipulator for a given angle.
 * 
 * @param rotate_angle in radian
 */
void Manipulation::lastJointRotation(double rotate_angle)
{
  getCurrentState();
  ROS_INFO("Rotate the last joint");
  setJointGroup(joint_group_positions[0] + rotate_angle, 
                joint_group_positions[1], 
                joint_group_positions[2],
                joint_group_positions[3],
                joint_group_positions[4],
                joint_group_positions[5],
                joint_group_positions[6]);
  move(joint_group_positions);
}