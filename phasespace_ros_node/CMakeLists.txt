cmake_minimum_required(VERSION 2.8.3)
project(phasespace)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp message_generation roslib)
find_package(PkgConfig)

add_message_files(
  DIRECTORY msg
  FILES
  Camera.msg
  Cameras.msg
  Marker.msg
  Markers.msg
  Rigid.msg
  Rigids.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp message_runtime roslib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

link_directories(${catkin_LIBRARY_DIRS} bin)

add_library(owlsock SHARED IMPORTED)
set_target_properties(owlsock PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/libowlsock.so)

add_library(${PROJECT_NAME} src/psnode.cpp)
add_dependencies(${PROJECT_NAME} phasespace_gencpp)

# publisher example
add_executable(${PROJECT_NAME}_node src/psnode.cpp)
target_link_libraries(${PROJECT_NAME}_node owlsock ${catkin_LIBRARIES})

# subscriber test
#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
