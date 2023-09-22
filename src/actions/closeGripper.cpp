#include "manipulation_class.hpp"

/**
 * @brief This function will close the gripper to 80% position. Note that this scale of gripper position is for grasping objects with width less than 0.8*0.85 cm
 * 
 */
void Manipulation::close_gripper()
{
    this->gripper_cmd.goal.command.position = 0.8;
    gripper_command.publish(gripper_cmd);
    ROS_WARN("Closing gripper...");
}