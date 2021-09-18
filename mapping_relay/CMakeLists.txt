cmake_minimum_required(VERSION 3.5.1)
project(mapping_relay)

set(PKG_DEPENDS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  )
set(BUILD_DEPENDS
  cmake_modules
  message_generation
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  message_runtime
  ${PKG_DEPENDS}
  )

find_package(catkin REQUIRED ${BUILD_DEPENDS})

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

catkin_install_python(PROGRAMS mapping_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
