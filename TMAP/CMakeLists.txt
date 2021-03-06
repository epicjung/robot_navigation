cmake_minimum_required(VERSION 3.0.2)
project(TMAP)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  pcl_conversions
  pcl_ros
  cv_bridge
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
  tMap_msgs
  lvi_sam
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src
  utils
)

set(algorithm_src
    ../SLAM/src/lidar_odometry/utility.h
    utils/imu_handle.hpp
    utils/ptcloud_handle.hpp
    utils/point_types.hpp
    utils/map_handle.hpp
    utils/signal_handle.hpp
    src/include/node_gen.hpp
    src/include/patchwork_obstacle.hpp
    src/include/traversability_map.hpp
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# ptCloud_deskewer_node
add_executable(lig_node_gen src/node_gen.cpp ${algorithm_src})
target_link_libraries(lig_node_gen ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lig_node_gen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# patchwork
add_executable(gSeg_node src/gSeg_node.cpp ${algorithm_src})
target_link_libraries(gSeg_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gSeg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # traversability_map_node
add_executable(tMap_node src/tMap_node.cpp ${algorithm_src})
target_link_libraries(tMap_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(tMap_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

