cmake_minimum_required(VERSION 3.0.2)
project(hybrid_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  message_filters
  cv_bridge
  message_generation
  building_generator_interfaces
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ObjectNode.msg
  ObjectNodeList.msg
  RoomNode.msg
  RoomNodeList.msg
  RoomType.msg
  ViewpointRep.msg
  DetectionResult.msg
  RoomEarlyStop1.msg
  VlmAnswer.msg
  ObjectType.msg
  NavigationQuery.msg
  TargetObjectInstruction.msg
  TargetObject.msg
  TargetObjectWithSpatial.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    hybrid_lidar_model hybrid_graph hybrid_misc_utils hybrid_pointcloud_utils
    hybrid_tsp_solver hybrid_viewpoint hybrid_rolling_grid hybrid_viewpoint_manager
    hybrid_local_coverage_planner hybrid_grid_world hybrid_pointcloud_manager
    hybrid_keypose_graph hybrid_rolling_occupancy_grid hybrid_planning_env
    hybrid_exploration_path hybrid_tare_visualizer hybrid_representation
    hybrid_sensor_coverage_planner_ground
  CATKIN_DEPENDS
    roscpp std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs
    tf2 tf2_ros pcl_ros pcl_conversions message_filters cv_bridge message_runtime
    building_generator_interfaces
  DEPENDS PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  or-tools/include
)

link_directories(or-tools/lib)

set(TARE_CATKIN_LIBS ${catkin_LIBRARIES})
set(TARE_PCL_LIBS ${PCL_LIBRARIES})
set(TARE_ORTOOLS ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so)

macro(tare_add_lib name src)
  add_library(${name} ${src})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${TARE_CATKIN_LIBS})
endmacro()

tare_add_lib(hybrid_lidar_model src/lidar_model/lidar_model.cpp)
tare_add_lib(hybrid_graph src/graph/graph.cpp)
tare_add_lib(hybrid_misc_utils src/utils/misc_utils.cpp)
target_link_libraries(hybrid_misc_utils ${TARE_PCL_LIBS} ${OpenCV_LIBRARIES})
tare_add_lib(hybrid_pointcloud_utils src/utils/pointcloud_utils.cpp)
target_link_libraries(hybrid_pointcloud_utils ${TARE_PCL_LIBS} hybrid_misc_utils)
tare_add_lib(hybrid_tsp_solver src/tsp_solver/tsp_solver.cpp)
target_link_libraries(hybrid_tsp_solver ${TARE_ORTOOLS})
tare_add_lib(hybrid_viewpoint src/viewpoint/viewpoint.cpp)
target_link_libraries(hybrid_viewpoint hybrid_lidar_model)
tare_add_lib(hybrid_rolling_grid src/rolling_grid/rolling_grid.cpp)
target_link_libraries(hybrid_rolling_grid hybrid_misc_utils hybrid_pointcloud_utils)
add_library(hybrid_viewpoint_manager STATIC src/viewpoint_manager/viewpoint_manager.cpp)
add_dependencies(hybrid_viewpoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_viewpoint_manager ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} hybrid_rolling_grid hybrid_viewpoint hybrid_grid_world)
tare_add_lib(hybrid_local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
target_link_libraries(hybrid_local_coverage_planner hybrid_viewpoint_manager hybrid_planning_env)
add_library(hybrid_grid_world STATIC src/grid_world/grid_world.cpp)
add_dependencies(hybrid_grid_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_grid_world ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} hybrid_viewpoint_manager hybrid_tsp_solver hybrid_keypose_graph hybrid_exploration_path hybrid_misc_utils)
tare_add_lib(hybrid_pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
target_link_libraries(hybrid_pointcloud_manager hybrid_misc_utils)
add_library(hybrid_keypose_graph STATIC src/keypose_graph/keypose_graph.cpp)
add_dependencies(hybrid_keypose_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_keypose_graph ${TARE_CATKIN_LIBS} ${TARE_PCL_LIBS} hybrid_viewpoint_manager)
tare_add_lib(hybrid_rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
target_link_libraries(hybrid_rolling_occupancy_grid hybrid_misc_utils hybrid_rolling_grid)
tare_add_lib(hybrid_planning_env src/planning_env/planning_env.cpp)
target_link_libraries(hybrid_planning_env ${TARE_PCL_LIBS} hybrid_rolling_occupancy_grid hybrid_pointcloud_manager hybrid_lidar_model hybrid_viewpoint_manager hybrid_grid_world)
tare_add_lib(hybrid_exploration_path src/exploration_path/exploration_path.cpp)
target_link_libraries(hybrid_exploration_path ${TARE_PCL_LIBS})
tare_add_lib(hybrid_tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
target_link_libraries(hybrid_tare_visualizer ${TARE_PCL_LIBS} hybrid_grid_world hybrid_misc_utils hybrid_planning_env)
tare_add_lib(hybrid_representation src/representation/representation.cpp)
target_link_libraries(hybrid_representation ${OpenCV_LIBRARIES} hybrid_misc_utils hybrid_pointcloud_utils)
tare_add_lib(hybrid_sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
target_link_libraries(hybrid_sensor_coverage_planner_ground ${OpenCV_LIBRARIES} hybrid_graph hybrid_planning_env hybrid_keypose_graph hybrid_viewpoint_manager hybrid_pointcloud_manager hybrid_grid_world hybrid_local_coverage_planner hybrid_tare_visualizer hybrid_misc_utils hybrid_representation)

add_executable(hybrid_navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
add_dependencies(hybrid_navigationBoundary ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_navigationBoundary ${TARE_CATKIN_LIBS} ${TARE_PCL_LIBS})

add_executable(hybrid_planner_node src/tare_planner_node/tare_planner_node.cpp)
add_dependencies(hybrid_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_planner_node hybrid_sensor_coverage_planner_ground)

add_executable(hybrid_room_segmentation src/room_segmentation/room_segmentation.cpp)
add_dependencies(hybrid_room_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_room_segmentation ${TARE_CATKIN_LIBS} ${OpenCV_LIBRARIES} hybrid_pointcloud_utils hybrid_representation hybrid_misc_utils)

install(TARGETS
  hybrid_lidar_model hybrid_graph hybrid_misc_utils hybrid_pointcloud_utils hybrid_tsp_solver
  hybrid_viewpoint hybrid_rolling_grid hybrid_viewpoint_manager hybrid_local_coverage_planner
  hybrid_grid_world hybrid_pointcloud_manager hybrid_keypose_graph hybrid_rolling_occupancy_grid
  hybrid_planning_env hybrid_exploration_path hybrid_tare_visualizer hybrid_representation
  hybrid_sensor_coverage_planner_ground hybrid_navigationBoundary hybrid_planner_node
  hybrid_room_segmentation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ config/ rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
