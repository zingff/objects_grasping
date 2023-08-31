#include "manipulation_class.hpp"

void Manipulation::godown()
{
  getCurrentState();
  ROS_INFO("Moving to snapshot Position");
  setJointGroup(1.15159, 0.618337, 0.593246, 2.20513, 5.16716, 0.744263, 0.126573);
  move(joint_group_positions);
}