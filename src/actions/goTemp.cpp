#include "manipulation_class.hpp"

/**
 * @brief This function will move the manipulator to a temp position, the 7 parameter represent the 7 joint positions of the arm. This function is for testing, i.e., move to a temp position. Note that this function is similar to Manipulation::move().
 * 
 */
void Manipulation::goTemp()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(1.38351, 0.484841, 3.13812, 4.04086-6.28, 5.92295, 1.08355, 0.26627);
    move(joint_group_positions);
}
