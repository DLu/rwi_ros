cmake_minimum_required(VERSION 2.8.3)
project(rflex)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf angles dynamic_reconfigure diagnostic_updater)

include_directories(include ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/B21.cfg)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf angles dynamic_reconfigure diagnostic_updater
)

add_library(rflex src/rflex_driver.cc)


add_library(b21_driver src/b21_driver.cc)
add_executable(b21 src/b21_node.cc)
add_dependencies(b21 ${PROJECT_NAME}_gencfg)
target_link_libraries(b21 b21_driver rflex  ${catkin_LIBRARIES})

add_library(atrvjr_driver src/atrvjr_driver.cc)
add_executable(atrvjr src/atrvjr_node.cc)
target_link_libraries(atrvjr atrvjr_driver rflex  ${catkin_LIBRARIES})

install(TARGETS rflex b21_driver b21 atrvjr_driver atrvjr
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
