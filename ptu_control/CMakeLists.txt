cmake_minimum_required(VERSION 2.8.3)
project(ptu_control)
find_package(catkin REQUIRED COMPONENTS rospy roscpp actionlib actionlib_msgs sensor_msgs std_msgs nav_msgs tf)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_action_files(
    DIRECTORY action 
    FILES 
        PtuGoto.action
        PtuReset.action
        PtuSetVel.action)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs
)
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rospy roscpp actionlib actionlib_msgs sensor_msgs std_msgs nav_msgs tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
