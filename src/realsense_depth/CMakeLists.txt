cmake_minimum_required(VERSION 2.8.3)
project(realsense_depth)

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

find_package(
  OpenCV 3.4.1 EXACT REQUIRED
)

# locate OpenCV in system
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS roscpp std_msgs
  #DEPENDS
)

###########
## Build ##
###########

## NON-ROS CMAKELISTS.TXT CODE

#set(CMAKE_CXX_STANDARD 11)
#find_package( OpenCV 3.4.1 EXACT REQUIRED ) # locate OpenCV in system
#include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers
#add_executable(depth 
#  src/depth.cpp)
#target_link_libraries( depth ${OpenCV_LIBS} /usr/local/lib/librealsense2.so )
#MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS} )  #display opencv libs found

## ROSIFIED CMAKELISTS.TXT CODE

set(CMAKE_CXX_STANDARD 11)

# provide library headers
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  realsense_depth.h
  ) 

add_executable(realsense_depth
  src/realsense_depth.cpp
)

target_link_libraries(realsense_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /usr/local/lib/librealsense2.so 
)

add_dependencies(realsense_depth
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

