cmake_minimum_required(VERSION 2.8.3)
project(pa_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy geometry_msgs sensor_msgs nav_msgs tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pa_slam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pa_slam_node pa_slam_node.cpp
    src/PaSlam_Costmap.cpp
    src/PaSlam_Map.cpp
    src/PaSlam.cpp
    )


## Specify libraries to link a library or executable target against
###          !!! IMPORTANT !!!

target_link_libraries(pa_slam_node
    ${catkin_LIBRARIES}
    )

