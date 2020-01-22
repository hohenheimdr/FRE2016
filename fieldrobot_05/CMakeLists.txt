cmake_minimum_required(VERSION 2.8.3)
project(fieldrobot_05)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
 	dynamic_reconfigure
	geometry_msgs 
	tf
	msgs 
	laser_geometry 
	differential_ifk_lib
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	#${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS}
	cfg/cpp
)

generate_dynamic_reconfigure_options(
cfg/lidar_row_extractor.cfg
cfg/rabbit_follow_params.cfg
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

## create lidar row extractor out of laserdata
add_executable(lidar_row_extractor_node src/lidar_row_extractor/ransac_row_node.cpp src/lidar_row_extractor/ransac_extractor.cpp)
## Add dependencies to the executable
add_dependencies(lidar_row_extractor_node ${PROJECT_NAME} msgs_gencpp geometry_msgs_gencpp sensor_msgs_gencpp visualization_msgs_gencpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(lidar_row_extractor_node  ${catkin_LIBRARIES} )

# build rabbit follower
add_executable(rabbit_follower_node src/rabbit_follower/rabbit_follower_node.cpp src/rabbit_follower/RabbitFollow.cpp)
target_link_libraries(rabbit_follower_node ${catkin_LIBRARIES}  )

# build motorcontroller core
add_executable(sdc2130_core src/SDC2130/sdc2130_core.cpp src/SDC2130/SDC2130.cpp src/SDC2130/RoboteqDevice.cpp)
target_link_libraries(sdc2130_core ${catkin_LIBRARIES}  )

# build joystick_core
add_executable(joystick_core src/joystick_Logitech/joystick_core.cpp src/joystick_Logitech/Logitech.cpp)
target_link_libraries(joystick_core ${catkin_LIBRARIES}  )

# build path_ writer
add_executable(path_writer src/Navigation/path_writer.cpp)
target_link_libraries(path_writer ${catkin_LIBRARIES})

# build differential ifk
add_executable(differential_odometry src/odometry/differential_odometry_node.cpp)
target_link_libraries(differential_odometry ${catkin_LIBRARIES})

# build mode changer
add_executable(mode_changer src/mode_changer/mode_changer.cpp)
target_link_libraries(mode_changer ${catkin_LIBRARIES})

# build mode changer
add_executable(mode_changer_freestile src/mode_changer/mode_changer_freestile.cpp)
target_link_libraries(mode_changer_freestile ${catkin_LIBRARIES})

# build row_speed_extractor
add_executable(row_speed_extractor src/mode_changer/row_speed_extractor.cpp)
target_link_libraries(row_speed_extractor ${catkin_LIBRARIES})

# build row_speed_extractor
add_executable(metal_speed_extractor src/mode_changer/metal_speed_extractor.cpp)
target_link_libraries(metal_speed_extractor ${catkin_LIBRARIES})


# build kinect_row_speed_extractor
add_executable(kinect_row_speed_extractor src/mode_changer/kinect_row_speed_extractor.cpp)
target_link_libraries(kinect_row_speed_extractor ${catkin_LIBRARIES}${Boost_LIBRARIES} ${PCL_LIBRARIES})

# build sensortester
add_executable(sensor_test src/sensor_test/sensor_test.cpp)
target_link_libraries(sensor_test ${catkin_LIBRARIES}${Boost_LIBRARIES} ${PCL_LIBRARIES})

# build laser_navigation_node
add_executable(laser_navigation src/Navigation/laser_navigation_node.cpp)
target_link_libraries(laser_navigation ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )
# Add dependencies to the executable


# build sender_node
add_executable(sender_node2 src/SDC2130/sender_node2.cpp)
target_link_libraries(sender_node2 ${catkin_LIBRARIES} )

# build laser error cmd
add_executable(laser_error_cmd src/Navigation/laser_error_cmd_vel2.cpp)
target_link_libraries(laser_error_cmd ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )



