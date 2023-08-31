#include "manipulation_class.hpp"

void Manipulation::plan_pose_goal()
{
    this->move_group_ptr->setPoseTarget(this->target_pose);
    this->move_group_ptr->setGoalPositionTolerance(0.01);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);
    this->move_group_ptr->setPlanningTime(5.0);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}