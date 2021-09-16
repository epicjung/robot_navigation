cmake_minimum_required(VERSION 3.0.2)
project(segmatch)

add_definitions(-std=c++11)

# find_package(OpenCV 2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    pcl_conversions
    pcl_ros
)
find_package(PCL REQUIRED)
find_package(libnabo REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# set(OpenCV_CONFIG_PATH "/usr/local/share/OpenCV")
# set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV 2 REQUIRED)

# message(STATUS "OpenCV library status:")
# message(STATUS "    config: ${OpenCV_DIR}")
# message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libs: ${OpenCV_LIBS} ")
# message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
# message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(LIB_SOURCES 
src/features.cpp 
src/opencv_random_forest.cpp
src/segmented_cloud.cpp)

add_library(${PROJECT_NAME} ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} libnabo::nabo glog::glog)

catkin_package(
    CATKIN_DEPENDS pcl_ros pcl_conversions
    INCLUDE_DIRS include
    DEPENDS PCL
    LIBRARIES ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE)

# catkin_simple(ALL_DEPS_REQUIRED)

# cs_add_library(${PROJECT_NAME} 
#     src/features.cpp
#     # src/descriptors/autoencoder.cpp
#     # src/descriptors/descriptors.cpp
#     # src/descriptors/eigenvalue_based.cpp
#     # src/descriptors/ensemble_shape_functions.cpp
#     src/opencv_random_forest.cpp
#     src/segmented_cloud.cpp
# )
# target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


#add_doxygen(NOT_AUTOMATIC)

# cs_install()
# cs_export()

