cmake_minimum_required(VERSION 2.8.3)
project(laser_slam_ros)

add_definitions(-std=c++11 -DBENCHMARK_ENABLE)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    message_generation
    tf
    rosbag
    visualization_msgs
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/laser_slam_worker.cpp
    src/visual_view.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

cs_install()
cs_export()

