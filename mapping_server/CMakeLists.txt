cmake_minimum_required(VERSION 3.5.1)
project(mapping_server)
set(CMAKE_BUILD_TYPE Debug)

set(PKG_DEPENDS
  roscpp
  json
  cbor
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  )
set(BUILD_DEPENDS
  cmake_modules
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS})
find_package(Boost REQUIRED iostreams program_options regex system thread)
find_package(Eigen3 REQUIRED NO_MODULE)

# Zlib
find_package(ZLIB)

IF (NOT ZLIB_FOUND)
  add_subdirectory(external/restinio/third_party/zlib)
  message( STATUS "USE OWN ZLIB SOURCES")
  SET(ZLIB_LIBRARIES zlibstatic)
  SET(ZLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/restinio/third_party/zlib)
ENDIF ()

#######################################
## Build configuration
#######################################

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIRS}
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ftemplate-backtrace-limit=0")

#######################################
## Build
#######################################

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
)

set(RESTINIO_USE_BOOST_ASIO "none")
set(RESTINIO_STAND_ALONE_ASIO_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/external/asio/include )
set(RESTINIO_STAND_ALONE_ASIO_DEFINES -DASIO_STANDALONE  -DASIO_HAS_STD_CHRONO -DASIO_DISABLE_STD_STRING_VIEW)

add_subdirectory(external/nodejs/http_parser)
add_subdirectory(external/fmt)
add_subdirectory(external/restinio)

add_executable(mapping_server_node src/mapping_server.cpp)

target_link_libraries(mapping_server_node
  restinio::restinio
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ZLIB_LIBRARIES}
  )

#######################################
## Install
#######################################

install(TARGETS mapping_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
