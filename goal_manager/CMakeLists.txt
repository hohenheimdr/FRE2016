cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  msgs 
  geometry_msgs 
  tf 
)

include_directories(   ${catkin_INCLUDE_DIRS} )

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/GoalManager.cfg
  cfg/FrameGoalManager.cfg
)

#find_package(Boost REQUIRED )

###########
## Build ##
###########

# build goal_creaternode_for point navigation
add_executable(goal_manager src/goal_creater.cpp src/pathParser.cpp)
target_link_libraries(goal_manager ${catkin_LIBRARIES})
#this publish points
add_executable(frame_goal_manager src/frame_goal_creater.cpp src/pathParser.cpp)
target_link_libraries(frame_goal_manager ${catkin_LIBRARIES})
#this publish poses
add_executable(robot_goal_manager src/robot_goal_creater.cpp src/pathParser.cpp)
target_link_libraries(robot_goal_manager ${catkin_LIBRARIES})

# build path_ writer
add_executable(path_writer src/path_writer.cpp src/pathParser.cpp)
target_link_libraries(path_writer ${catkin_LIBRARIES})
