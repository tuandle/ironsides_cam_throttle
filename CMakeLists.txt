cmake_minimum_required(VERSION 2.8.3)
project(ironsides_reduce)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_filters
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  src/nodelets/ironsides_reduce_nodelet.cpp
  src/ironsides_reduce.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})


add_executable(ironsides_reduce_node src/nodes/ironsides_reduce_node.cpp)
target_link_libraries(ironsides_reduce_node ${catkin_LIBRARIES})

MESSAGE("lib path: " ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)