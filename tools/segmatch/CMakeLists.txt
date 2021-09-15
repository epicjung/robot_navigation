cmake_minimum_required(VERSION 3.0.2)
project(segmatch)

add_definitions(-std=c++11)

# find_package(OpenCV 2 REQUIRED)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    pcl_conversions
)

catkin_package(
    INCLUDE_DIRS include
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/features.cpp
    src/matcher.cpp
    # src/opencv_random_forest.cpp
    src/segmented_cloud.cpp
)
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBS})

find_package(Boost REQUIRED COMPONENTS system thread)

#add_doxygen(NOT_AUTOMATIC)

cs_install()
cs_export()

