cmake_minimum_required(VERSION 3.0.2)
project(segmatch_ros)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    std_msgs
    segmatch
    pcl_conversions
)

find_package(PCL REQUIRED)
######################
### Catkin
######################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    DEPENDS PCL 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node src/matcher.cpp src/matcher_node.cpp)
# add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
