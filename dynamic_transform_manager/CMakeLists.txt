cmake_minimum_required(VERSION 2.8.3)
project(dynamic_transform_manager)

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


## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/DynamicTransform.cfg
  
)

include_directories(   ${catkin_INCLUDE_DIRS} )

#find_package(Boost REQUIRED )

###########
## Build ##
###########

# build goal_creaternode_for point navigation
add_executable(dynamic_transform_node src/dynamic_transform_node.cpp)
target_link_libraries(dynamic_transform_node ${catkin_LIBRARIES})


