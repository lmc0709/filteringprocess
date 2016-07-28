#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-fPIC -std=c++0x -fpermissive")

cmake_minimum_required(VERSION 2.8.3)
project(abandonedmarina)

include(/home/clarisse/Downloads/dlib-18.18/dlib/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  message_generation
  mapclasses
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_message_files(
  FILES
  amDVL.msg
  amMTi.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS message_runtime
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(createBagFile src/createBagFile.cpp)
target_link_libraries(createBagFile ${catkin_LIBRARIES})

add_executable(scanFilterAB src/scanFilter.cpp)
target_link_libraries(scanFilterAB scanFilter ${catkin_LIBRARIES})

add_executable(mapBuilderAB src/mapBuilder.cpp)
target_link_libraries(mapBuilderAB map scanBuffer dlib ${catkin_LIBRARIES})
