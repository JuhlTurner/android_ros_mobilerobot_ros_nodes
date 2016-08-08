cmake_minimum_required(VERSION 2.8.3)
project(control_vision)
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(control_vision src/control_vision_node.cpp src/PID.cpp)
add_dependencies(control_vision ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control_vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
