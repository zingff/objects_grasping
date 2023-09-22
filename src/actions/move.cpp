#include "manipulation_class.hpp"

/**
 * @brief This function will move the manipulator to a specific position which is given by joint_group_positions
 * 
 * @param joint_group_positions desired joint positions, in radian
 */
void Manipulation::move(std::vector<double> joint_group_positions)
{
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->move();
}