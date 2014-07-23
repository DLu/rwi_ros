cmake_minimum_required(VERSION 2.8.3)
project(b21_description)
find_package(catkin REQUIRED)
catkin_package()

install(FILES b21.xacro upload_b21.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
