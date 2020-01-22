cmake_minimum_required(VERSION 2.8.3)
project(basic_navigation)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	tf 
	sensor_msgs
	std_msgs
	)

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs

)
###########
## Build ##
###########

add_executable(point_follower src/Point_Follower.cpp)
target_link_libraries(point_follower ${catkin_LIBRARIES}  )





