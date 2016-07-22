cmake_minimum_required(VERSION 2.8.3)
project(vision_control)
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
add_executable(vision_control src/vision_control_node.cpp)
add_dependencies(vision_control ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vision_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
