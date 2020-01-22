cmake_minimum_required(VERSION 2.8.3)
project(kinect_aux)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp sensor_msgs 
	std_msgs 
	dynamic_reconfigure
	tf 	
	)


find_path(LIBUSB_INCLUDEDIR 
          NAMES libusb.h
          HINTS /usr/include/libusb-1.0)
find_library(LIBUSB_LIBRARIES
             NAMES usb-1.0
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
             PATH_SUFFIXES lib)

message(STATUS ${LIBUSB_INCLUDEDIR})
message(STATUS ${LIBUSB_LIBRARIES})


generate_dynamic_reconfigure_options(
  cfg/KinectAux.cfg

)


catkin_package(
#  INCLUDE_DIRS include LIBUSB_INCLUDEDIRS
#  LIBRARIES kinect_aux
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS LIBUSB
  DEPENDS LIBUSB_LIBRARIES
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
   ${catkin_INCLUDE_DIRS}
   ${LIBUSB_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(kinect_aux_node src/kinect_aux_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_aux_node
   ${catkin_LIBRARIES}
   ${LIBUSB_LIBRARIES}
)

#############
## Install ##
#############


# Mark executables and/or libraries for installation
install(TARGETS kinect_aux_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect_aux.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
