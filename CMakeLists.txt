cmake_minimum_required(VERSION 3.0.2)
project(gpd_pick_and_place_bak_1)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  # pcl_conversions
  # pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  # gpd_ros
  control_msgs
  anygrasp_generation
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(gpd_grasping_node
# 	src/nodes/gpd_grasping_node.cpp
# 	src/functions/manipulation_class.cpp
# 	src/functions/perception_class.cpp
#   src/functions/gpd.cpp
# )
# target_link_libraries(gpd_grasping_node ${catkin_LIBRARIES})
# add_dependencies(gpd_grasping_node ${catkin_EXPORTED_TARGETS})

# add_executable(pick_place_tests
#   src/nodes/perception_test.cpp
# 	src/functions/perception_class.cpp
#   src/functions/manipulation_class.cpp
#   src/functions/gpd.cpp
# )
# target_link_libraries(pick_place_tests ${catkin_LIBRARIES})
# add_dependencies(pick_place_tests ${catkin_EXPORTED_TARGETS})

add_executable(angrasp_object_grasping
	src/nodes/angrasp_object_grasping.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(angrasp_object_grasping ${catkin_LIBRARIES})
add_dependencies(angrasp_object_grasping ${catkin_EXPORTED_TARGETS})


add_executable(initial
	src/nodes/initial.cpp
	src/functions/manipulation_class.cpp
  src/functions/gpd.cpp
	# src/functions/perception_class.cpp
  # src/functions/gpd.cpp
)
target_link_libraries(initial ${catkin_LIBRARIES})
add_dependencies(initial ${catkin_EXPORTED_TARGETS})


# add_executable(pickup_twice
# 	src/nodes/pickup_twice.cpp
# 	src/functions/manipulation_class.cpp
# 	src/functions/perception_class.cpp
#   src/functions/gpd.cpp
# )
# target_link_libraries(pickup_twice ${catkin_LIBRARIES})
# add_dependencies(pickup_twice ${catkin_EXPORTED_TARGETS})