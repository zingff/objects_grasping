#include "manipulation_class.hpp"

/**
 * @brief This function will set the target Cartesian pose for the manipulator to move, which is used in function Manipulator::plan_to_goal().
 * 
 */
void Manipulation::set_target_pose()
{
    this->q.setRPY(this->orientation.x, this->orientation.y, this->orientation.z); // x-pi

    this->q.normalize();
    this->target_pose.orientation = tf2::toMsg(this->q);

    this->target_pose.position.x = this->position.x;
    this->target_pose.position.y = this->position.y;
    this->target_pose.position.z = this->position.z;
}