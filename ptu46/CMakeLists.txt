cmake_minimum_required(VERSION 2.8.3)
project(ptu46)
find_package(catkin REQUIRED COMPONENTS tf sensor_msgs roscpp rospy actionlib ptu_control diagnostic_updater)

catkin_package(
    CATKIN_DEPENDS
        roscpp tf sensor_msgs rospy actionlib ptu_control diagnostic_updater
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(ptu_46_driver src/ptu46_driver.cc)
add_executable(${PROJECT_NAME} src/ptu46_node.cc)
target_link_libraries(${PROJECT_NAME} ptu_46_driver  ${catkin_LIBRARIES})

install(FILES ptu46.launch ptu46_actions.launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ptu_46_driver ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
