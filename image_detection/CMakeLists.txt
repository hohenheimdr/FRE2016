cmake_minimum_required(VERSION 2.8.3)
project(image_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  tf
  sensor_msgs
  std_msgs
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_detection
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(image_detection src/image_detection.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(image_detection   ${catkin_LIBRARIES}  )

