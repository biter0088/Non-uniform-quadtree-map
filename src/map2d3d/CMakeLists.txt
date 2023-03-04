cmake_minimum_required(VERSION 2.8.3)
project(map2d3d)

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

# include_directories("/usr/include/eigen3")
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

# 针对kitti数据集，进行四叉树地图匹配
# 1. 在创建离线全局四叉树地图时，将四叉树地图中的占据节点信息保存在txt中
# 2. 实时创建四叉树地图时，将四叉树地图中的占据节点信息也保存在txt中
# 3. 提取两个txt中的占据节点信息，将其转化为点云进行粗匹配和精匹配
add_executable(map_match_quadtree_kitti src/map_match_quadtree_kitti.cpp)
target_link_libraries(map_match_quadtree_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
# 多生成一个节点
add_executable(map_match_quadtree_kitti2 src/map_match_quadtree_kitti.cpp)
target_link_libraries(map_match_quadtree_kitti2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable(map_match_quadtree_kitti3 src/map_match_quadtree_kitti.cpp)
target_link_libraries(map_match_quadtree_kitti3 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable(map_match_quadtree_kitti4 src/map_match_quadtree_kitti.cpp)
target_link_libraries(map_match_quadtree_kitti4 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable(map_match_quadtree_kitti5 src/map_match_quadtree_kitti.cpp)
target_link_libraries(map_match_quadtree_kitti5 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})