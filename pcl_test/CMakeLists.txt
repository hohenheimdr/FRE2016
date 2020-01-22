cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	tf 
	laser_geometry 
	pcl_ros
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp

)

###########
## Build ##
###########

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# build laser_error_navigation_node
add_executable(outlier_removal2 src/outlier_filter.cpp)
target_link_libraries(outlier_removal2 ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(pointcloud_to_pcd2 src/point_cloud_to_pcd.cpp)
target_link_libraries(pointcloud_to_pcd2 pcl_io ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_assembler src/cloud_assembler.cpp)
target_link_libraries(cloud_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(voxel_grid src/voxel_grid_filter.cpp)
target_link_libraries(voxel_grid ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(magnet_sensor_assembler src/magnet_sensor_assembler.cpp)
target_link_libraries(magnet_sensor_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(cloud_assembler ${PROJECT_NAME}_gencpp)

add_executable(icp_matcher src/icp_matcher.cpp)
target_link_libraries(icp_matcher ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(cloud_assembler ${PROJECT_NAME}_gencpp)
add_executable(robot_ransac_row_filter src/robot_ransac_row_filter.cpp)
target_link_libraries(robot_ransac_row_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ransac_line_generator src/ransac_line_generator.cpp)
target_link_libraries(ransac_line_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})





