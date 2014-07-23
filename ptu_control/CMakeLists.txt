cmake_minimum_required(VERSION 2.8.3)
project(ptu_control)
find_package(catkin REQUIRED COMPONENTS rospy roscpp actionlib actionlib_msgs sensor_msgs geometry_msgs std_msgs nav_msgs tf  message_runtime message_generation )

add_action_files(
    DIRECTORY action 
    FILES 
        PtuGoto.action
        PtuReset.action
        PtuSetVel.action)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy roscpp actionlib actionlib_msgs sensor_msgs std_msgs nav_msgs tf geometry_msgs message_runtime message_generation 
)
