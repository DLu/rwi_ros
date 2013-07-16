cmake_minimum_required(VERSION 2.8.3)
project(ptu46)
find_package(catkin REQUIRED COMPONENTS tf sensor_msgs roscpp rospy actionlib ptu_control diagnostic_updater)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} src/ptu46_driver.cc)
add_executable(${PROJECT_NAME} src/ptu46_node.cc)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS tf sensor_msgs roscpp rospy actionlib ptu_control diagnostic_updater
#    CATKIN_DEPENDS # TODO
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
)
