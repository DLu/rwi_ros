cmake_minimum_required(VERSION 2.8.3)
project(ptu46)
find_package(catkin REQUIRED COMPONENTS tf sensor_msgs roscpp rospy actionlib ptu_control diagnostic_updater)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(ptu_46_driver src/ptu46_driver.cc)
add_executable(${PROJECT_NAME} src/ptu46_node.cc)
target_link_libraries(${PROJECT_NAME} ptu_46_driver  ${catkin_LIBRARIES})

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS tf sensor_msgs roscpp rospy actionlib ptu_control diagnostic_updater
    CATKIN_DEPENDS
        roscpp
    INCLUDE_DIRS include
#    LIBRARIES # TODO
)
