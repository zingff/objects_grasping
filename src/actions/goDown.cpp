#include "manipulation_class.hpp"

/**
 * @brief This function will move the manipulator to a place the grasped object. Note that this function is similar to Manipulation::move().
 * 
 */
void Manipulation::goDown()
{
  getCurrentState();
  ROS_INFO("Moving to snapshot Position");
  setJointGroup(1.15159, 0.618337, 0.593246, 2.20513, 5.16716, 0.744263, 0.126573);
  move(joint_group_positions);
}