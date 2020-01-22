cmake_minimum_required(VERSION 2.8.3)
project(sick_laser_filter)

## Find catkin macros and libraries
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

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
laser_geometry
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/SickLaserFilter.cfg
  cfg/ApplyFilter.cfg
)

include_directories(   
${catkin_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#find_package(Boost REQUIRED )

###########
## Build ##
###########

# build goal_creaternode_for point navigation
add_executable(sick_laser_filter_node src/sick_laser_filter_node.cpp)
target_link_libraries(sick_laser_filter_node ${catkin_LIBRARIES})

add_executable(apply_filter_node src/apply_filter_node.cpp)
target_link_libraries(apply_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
