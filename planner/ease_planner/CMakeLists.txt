cmake_minimum_required(VERSION 3.0.2)
project(ease_planner)

set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  building_generator_interfaces
  roslib
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp std_msgs nav_msgs geometry_msgs visualization_msgs
    building_generator_interfaces roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ease_planner_node
  src/ease_planner_node.cpp
  src/ease_planner.cpp
)
add_dependencies(ease_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ease_planner_node ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS ease_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ config/ rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
