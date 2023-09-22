#include "manipulation_class.hpp"

/**
 * @brief This function will get current states such as joint position and Cartesian pose of the manipulator
 * 
 */
void Manipulation::getCurrentState()
{

    const robot_state::JointModelGroup *joint_model_group =
        move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    current_state = move_group_ptr->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}