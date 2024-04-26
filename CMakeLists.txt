cmake_minimum_required(VERSION 3.0.2)
project(robosense_filter)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common io)# find_package(Boost REQUIRED COMPONENTS system)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(ring_filter_node src/ring_filter.cpp src/ring_filter_node.cpp)
target_link_libraries(ring_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})



install(
  TARGETS
  ring_filter_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


