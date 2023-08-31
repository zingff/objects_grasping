#include "manipulation_class.hpp"

void Manipulation::goTop()
{
    getCurrentState();
    ROS_INFO("Moving to Top Position");
    // original 
    setJointGroup(0.043, -0.1824, -0.0133, 2.208, -0.0188, 0.7828, -1.524);
    // this config is closer to Home
    // setJointGroup(0, 6.12427-6.28, 3.15113, 3.9903-6.28, 0.0248641, 5.68259-6.28, 1.58098);
    // setJointGroup(0.319602, 6.05876-6.28, 3.1419, 4.05812-6.28, 0.0315863, 5.53112-6.28, 1.56028);
    
    move(joint_group_positions);
}