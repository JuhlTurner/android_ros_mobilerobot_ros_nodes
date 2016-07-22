cmake_minimum_required(VERSION 2.8.3)
project(perception_vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs)

find_package(OpenCV 2 REQUIRED)
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(perception_vision src/perception_vision_node.cpp)
add_dependencies(perception_vision ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(perception_vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
