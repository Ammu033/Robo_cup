cmake_minimum_required(VERSION 2.8.3)
project(bayes_people_tracker_bringup)
find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
