cmake_minimum_required(VERSION 2.8.3)
project(close_area_semantic_map)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  pcl_conversions
)

include_directories("/usr/local/include/eigen3")
set(PCL_DIR "/usr/share/pcl-1.8")
set(OpenCV_DIR "/usr/share/OpenCV") 
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs pcl_conversions
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

# 接收一个二维地图，并在其上进行死角搜索与死角语义地图构建
add_executable(corner_map2d_node src/corner_map2d.cpp)
target_link_libraries(corner_map2d_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBRARIES}
)