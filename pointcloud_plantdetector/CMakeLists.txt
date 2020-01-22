cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_plantdetector)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	tf 
	laser_geometry 
	pcl_ros
	dynamic_reconfigure
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )



## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS})

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
    cfg/Filter.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp

)




###########
## Build ##
###########

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_row_filter src/laser_row_filter.cpp)
target_link_libraries(laser_row_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


add_executable(3D_plant_detector src/3D_plant_detector.cpp)
target_link_libraries(3D_plant_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(kinect_plant_detector src/kinect_plant_detector.cpp)
target_link_libraries(kinect_plant_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


