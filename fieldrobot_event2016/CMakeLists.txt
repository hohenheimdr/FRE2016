cmake_minimum_required(VERSION 2.8.3)
project(fieldrobot_event2016)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
	cv_bridge
	image_transport
 	dynamic_reconfigure
	geometry_msgs 
	tf
	msgs 
	
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	#${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	cfg/cpp
)

generate_dynamic_reconfigure_options(

)

catkin_package(
	
  CATKIN_DEPENDS roscpp dynamic_reconfigure
)


SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# build differential ifk
add_executable(differential_odometry src/odometry/differential_odometry_node.cpp)
target_link_libraries(differential_odometry ${catkin_LIBRARIES})

# build motorcontroller core
add_executable(sdc2130_core src/SDC2130/sdc2130_core.cpp src/SDC2130/SDC2130.cpp src/SDC2130/RoboteqDevice.cpp)
target_link_libraries(sdc2130_core ${catkin_LIBRARIES}  )

# build mode changer
add_executable(mode_changer_twist src/mode_changer/mode_changer_twist.cpp)
target_link_libraries(mode_changer_twist ${catkin_LIBRARIES})

# build mode control task1
add_executable(mode_control_task1 src/mode_changer/mode_control_task1.cpp)
target_link_libraries(mode_control_task1 ${catkin_LIBRARIES})

# build mode control task2
add_executable(mode_control_task2 src/mode_changer/mode_control_task2.cpp)
target_link_libraries(mode_control_task2 ${catkin_LIBRARIES})

# build mode control task3
add_executable(mode_control_task3 src/mode_changer/mode_control_task3.cpp)
target_link_libraries(mode_control_task3 ${catkin_LIBRARIES})

# build mode control task4
add_executable(mode_control_task4 src/mode_changer/mode_control_task4.cpp)
target_link_libraries(mode_control_task4 ${catkin_LIBRARIES})

# build mode control task2
add_executable(tape_follower src/tape_follower/tape_follower.cpp)
target_link_libraries(tape_follower ${catkin_LIBRARIES})

# build task3 sprayer
add_executable(spray_tf src/sprayer/spray_tf.cpp)
target_link_libraries(spray_tf ${catkin_LIBRARIES})

# build task3 ball_ detector
add_executable(detect_ball src/sprayer/detect_ball.cpp)
target_link_libraries(detect_ball ${catkin_LIBRARIES})

