cmake_minimum_required(VERSION 3.0.2)
project(feature_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED PATHS "/usr/local/include")

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include/feature_tracker
    LIBRARIES feature_tracker
    CATKIN_DEPENDS roscpp
                   rospy
                   sensor_msgs
                   std_msgs
                   cv_bridge
                   image_transport
                   nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/feature_tracker
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/feature_tracker.py scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_executable(${PROJECT_NAME}_node src/feature_tracker_node.cpp src/feature_tracker.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${roscpp_LIBRARIES})
