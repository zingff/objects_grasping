/*****************************************************************
  GPD (Grasp Pose Detection) Specific Function Definitions
*****************************************************************/

#include <manipulation_class.hpp>
#include <tf/transform_listener.h>

// Get grasp msgs from gpd_ros package 
// void Manipulation::callback(const gpd_ros::GraspConfigList msg)
// {
//     ROS_WARN("Received Grasp Candidates");
//     grasp_candidates = msg;
//     this->getting_grasps = false;
// }

// //  Plan to each grasp candidate in order of score until a valid one is found
// //  Assign grasp msg values to class variables
// void Manipulation::path_planning()
// {
//     int n = sizeof(this->grasp_candidates);
//     if (n == 0)
//     {
//         ROS_ERROR("Grasp list is empty");
//     }

//     this->score = this->grasp_candidates.grasps[0].score;

//     int i = 0;
//     this->grabbed_object = 0;

//     // Plan to the top grasps to see if they are possible
//     for (i = 0; ((this->score.data > -150) && i < 10); i++)
//     {
//         this->grasp = this->grasp_candidates.grasps[i];
//         this->position = this->grasp.position;
//         this->pose_sample = this->grasp.sample;
//         this->orientation = this->grasp.approach;
//         this->axis = this->grasp.axis;
//         this->score = this->grasp.score;
//         ROS_INFO("Grasp score: %f", this->score.data);

//         set_target_pose();
//         plan_pose_goal();

//         if (this->pose_success)
//         {
//             this->grabbed_object = true;
//             ROS_WARN("Plan success");
//             break;
//         }
//     }
// }

void Manipulation::set_target_pose()
{
    this->q.setRPY(this->orientation.x, this->orientation.y, this->orientation.z); // x-pi
    // std::cout << this->orientation.x << std::endl;
    // std::cout << this->orientation.y << std::endl;
    // std::cout << this->orientation.z << std::endl;
    // std::cout << this->q.getZ() << std::endl;
    // std::cout << "target_orientation: " << this->orientation.x*57.3 << ", "<< this->orientation.y*57.3 << ", " << this->orientation.z*57.3 <<std::endl;
    // std::cout << "target_position: " << this->pose_sample.x << ", " << this->pose_sample.y << ", "  << this->pose_sample.z << std::endl;
    this->q.normalize();
    this->target_pose.orientation = tf2::toMsg(this->q);
    // std::cout << "target_orientation_again" << std::endl;
    // std::cout << this->target_pose.orientation.w << std::endl;
    // std::cout << this->target_pose.orientation.x << std::endl;
    // std::cout << this->target_pose.orientation.y << std::endl;
    // std::cout << this->target_pose.orientation.z << std::endl;

    // this->target_pose.position.x = this->pose_sample.x;
    // this->target_pose.position.y = this->pose_sample.y;
    // this->target_pose.position.z = this->pose_sample.z;

    this->target_pose.position.x = this->position.x;
    this->target_pose.position.y = this->position.y;
    this->target_pose.position.z = this->position.z;
}

void Manipulation::plan_pose_goal()
{
    this->move_group_ptr->setPoseTarget(this->target_pose);
    this->move_group_ptr->setGoalPositionTolerance(0.01);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);
    this->move_group_ptr->setPlanningTime(5.0);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// TODO: change the ompl name 
// TODO: change to upright way
// TODO: if planning not successful, shutdown
void Manipulation::ompl_plan(double x, double y, double z)
{
  auto current_pose = this->move_group_ptr->getCurrentPose();
  std::cout << "current quat:" << std::endl;
  std::cout << current_pose.pose.orientation << std::endl;
  this->target_pose.orientation = current_pose.pose.orientation;
  // this->target_pose.position = current_pose.pose.position;

  // this->target_pose.position.x = 0;
  // this->target_pose.position.y = -0.5;
  // this->target_pose.position.z = 0.3;

  // this->target_pose.orientation.x = 0.481745;
  // this->target_pose.orientation.y = -0.587967;
  // this->target_pose.orientation.z = 0.439422;
  // this->target_pose.orientation.w = 0.439422;

  // this->target_pose.position.x = 0.360;
  // this->target_pose.position.y = -0.146 + 0.0;
  // this->target_pose.position.z = 0.13;
  
  // this->target_pose.position.x = 0.357675;
  // this->target_pose.position.y = -0.0315239;
  // this->target_pose.position.z = 0.13;

  this->target_pose.position.x = x;
  this->target_pose.position.y = y;
  this->target_pose.position.z = z;

  // this->target_pose.position.x = 0.247481;
  // this->target_pose.position.y = -0.441109;
  // this->target_pose.position.z = 0.279088;

  this->move_group_ptr->setPoseTarget(this->target_pose);
  this->move_group_ptr->setPlanningTime(20.0);
  this->move_group_ptr->setMaxVelocityScalingFactor(0.5);
  this->move_group_ptr->setMaxAccelerationScalingFactor(0.5);
  // this->move_group_ptr->setPlannerId("RRTConnect");

  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = this->move_group_ptr->getPoseReferenceFrame();
  orientation_constraint.link_name = this->move_group_ptr->getEndEffectorLink();
  // orientation_constraint.link_name = "end_effector_link";

  // current_pose = this->move_group_ptr->getCurrentPose();
  orientation_constraint.orientation = this->target_pose.orientation;
  // orientation_constraint.orientation.x = -0.566237;
  // orientation_constraint.orientation.y = -0.568729;
  // orientation_constraint.orientation.z = -0.454364;
  // orientation_constraint.orientation.w = 0.386622;
  orientation_constraint.absolute_x_axis_tolerance = 0.5;
  orientation_constraint.absolute_y_axis_tolerance = 0.5;
  orientation_constraint.absolute_z_axis_tolerance = 0.5;
  orientation_constraint.weight = 1.0;
  // moveit_msgs::Constraints orientation_constraints;
  // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
  // this->move_group_ptr->setPathConstraints(orientation_constraints);

  this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "Planning state: " << this->pose_success << std::endl;
  if (this->pose_success == false)
  {
    ros::shutdown();
  }
  this->move_group_ptr->move();

  

}
                           
