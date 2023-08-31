#include "manipulation_class.hpp"

tf::StampedTransform Manipulation::getTransform(
                          tf::TransformListener & listener,std::string target_frame,
                          std::string source_frame)
{
  tf::StampedTransform T_target_source;
  // ros::Time now = ros::Time::now();
  ros::Time now = ros::Time(0);
  listener.waitForTransform(target_frame, source_frame, now, ros::Duration(18.0));
  listener.lookupTransform(target_frame, source_frame, now, T_target_source);
  std::cout << "Transform from '" << source_frame << "' to '" << target_frame << "':" << std::endl;
  std::cout << "Translation: (x=" << T_target_source.getOrigin().x()
            << ", y=" << T_target_source.getOrigin().y()
            << ", z=" << T_target_source.getOrigin().z() << ")" << std::endl;
  std::cout << "Rotation: (x=" << T_target_source.getRotation().x()
            << ", y=" << T_target_source.getRotation().y()
            << ", z=" << T_target_source.getRotation().z()
            << ", w=" << T_target_source.getRotation().w() << ")" << std::endl;
  return T_target_source;
}