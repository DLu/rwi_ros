cmake_minimum_required(VERSION 2.8.3)
project(b21_teleop)
find_package(catkin REQUIRED COMPONENTS sensor_msgs geometry_msgs roscpp )

catkin_package(
    CATKIN_DEPENDS sensor_msgs geometry_msgs roscpp 
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(b21_teleop src/b21_teleop.cpp)
target_link_libraries(b21_teleop ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS b21_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
