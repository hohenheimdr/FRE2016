cmake_minimum_required(VERSION 2.8.3)
project(trimble_scs900)

find_package(catkin REQUIRED COMPONENTS msgs roscpp sensor_msgs tf )

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp

)
SET(CMAKE_CXX_FLAGS "-std=c++11")

add_executable(totalstation_node src/totalstation_node.cpp)

## Add dependencies to the executable
add_dependencies(totalstation_node ${PROJECT_NAME} msgs_gencpp sensor_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(totalstation_node
   ${catkin_LIBRARIES}
 )
