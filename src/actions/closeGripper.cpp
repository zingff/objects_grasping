#include "manipulation_class.hpp"

void Manipulation::close_gripper()
{
    this->gripper_cmd.goal.command.position = 0.8;
    gripper_command.publish(gripper_cmd);
    ROS_WARN("Closing gripper...");
}