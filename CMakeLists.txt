cmake_minimum_required(VERSION 2.8.3)
project(lidar_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_omp
  rosbag
  roscpp
  velodyne_msgs
  ndt_omp
)

find_package(Eigen3 REQUIRED)

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_odometry
#  CATKIN_DEPENDS ndt_omp rosbag roscpp velodyne_msgs
#  DEPENDS system_lib
)

include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)



add_executable(lidar_odometry_node src/lidar_odometry_node.cpp src/lidar_odometry.cpp)

target_link_libraries(lidar_odometry_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

