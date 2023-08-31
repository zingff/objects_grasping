#include "manipulation_class.hpp"

void Manipulation::goTemp()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(1.38351, 0.484841, 3.13812, 4.04086-6.28, 5.92295, 1.08355, 0.26627);
    move(joint_group_positions);
}
