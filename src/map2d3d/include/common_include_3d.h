#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/passthrough.h>    ///直通滤波相关
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <map>  
#include <algorithm>
#include<mutex>
#include <math.h> //hxz
#include<cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
using namespace std; //hxz

std::mutex maplock;
std::mutex odomlock;
std::mutex odomlock1;
std::mutex gpslock;
std::mutex imulock;
std::mutex pub_marker_lock;//以marker形式显示3d栅格的锁
std::mutex add_marker_lock;//以marker形式添加3d栅格的锁
std::mutex grid_map_3d_lock;//创建和显示3d小栅格的锁

std::deque<nav_msgs::Odometry> odomVec;
std::deque<sensor_msgs::NavSatFix> gpsVec;
std::deque<sensor_msgs::Imu> imuVec;

std::deque<sensor_msgs::PointCloud2> laserVec;
geometry_msgs::Point cur_Lidarposition;//当前车辆(雷达)在栅格地图中的位置
//media/meng/5418189112144B70/kitti/residential/kitti_2011_09_26_drive_0079_synced.bag

int obs_prob;//被激光雷达打中多少次视为占据
double max_height;//直通滤波区域
double min_height;//
double max_width;//
double min_width;//
double max_len;//
double min_len;//


double UTME0;//东  //utm坐标系下车辆初始位置
double UTMN0;//北
double UTME;//当前位置
double UTMN;
double car_x0;//x轴，对应东向  //utm坐标系下车辆初始位置
double car_y0;//y轴，对应北向
double car_x;//当前位置
double car_y;

std::mutex alock;
double robot_x;
double robot_y;

// 未知：-1， 障碍物：100， 可通行：0
int passvalue = 0;
Eigen::Quaterniond q_odom_curr_now;
Eigen::Vector3d t_odom_curr_now;
//车辆位置、基本滤波后的点云、世界坐标系下的点云、某一帧或某几帧特定的点云,3d栅格可视化方式
ros::Publisher  pubCarpositiopn,pubBasicFilterCloud,pubWorldCloud,pubFrameCloud,marker_pub;


//点云基本操作相关
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInRaw(new pcl::PointCloud<pcl::PointXYZ>);//原始点云(单帧)
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_basic_filter(new pcl::PointCloud<pcl::PointXYZ>);//基本滤波之后的点云（单帧）
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filter(new pcl::PointCloud<pcl::PointXYZ>);//体素滤波之后的点云（单帧）
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_mapping(new pcl::PointCloud<pcl::PointXYZ>);//从其他节点接收来的（单帧）点云，仅仅为了建图
sensor_msgs::PointCloud2 basic_filterCloudMsg;//基本滤波之后发布的点云消息
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWorld(new pcl::PointCloud<pcl::PointXYZ>);//世界坐标系（车辆初始位置）下的点云（单帧）
sensor_msgs::PointCloud2 worldCloudMsg;//基本滤波之后发布的点云消息
bool use_outlier_remove_with_radius;//基于半径----离群点滤除
float filter_radius;//基于半径进行离群点滤除的参数
int filter_number;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_filter(new pcl::PointCloud<pcl::PointXYZ>);// 基于半径滤波后的点云指针
sensor_msgs::PointCloud2 frameCloudMsg;//某一帧或某几帧特定的点云
ros::Time rostime;

//3d 地图相关
float width_3d;//3d地图的宽-x轴、长-y轴、高-z轴
float length_3d;
float height_3d;
float resolution_xy;//xy方向内分辨率
float resolution_z;//z方向分辨率
std::map<std::tuple<int, int, int>, std::vector<pcl::PointXYZ>> gridMap_3d_data;//存放单帧点云数据的栅格
//统计小三维栅格中点云数量，并标记小栅格的序号；vector里面第一个是marker的id号，第二个是小栅格中点云数量
std::map<std::tuple<int, int, int>, std::vector<int>> gridMap_3d_num_id;
int id_num=0;//marker的id编号
bool map_init_3d=true;//3d 地图是否需要初始化
double robot_x0,robot_y0,robot_z0;//开始建立3d地图时车辆位置
visualization_msgs::Marker marker;//单个3d栅格可视化方式
visualization_msgs::MarkerArray markerArray;//3d栅格可视化方式
int color_order=0;//颜色顺序
int color_num=15;//颜色总数
#define M_PI 3.1415926