#include "manipulation_class.hpp"

void Manipulation::open_gripper()
{
    this->gripper_cmd.goal.command.position = 0;
    gripper_command.publish(gripper_cmd);
    ROS_WARN("Opening gripper...");
}