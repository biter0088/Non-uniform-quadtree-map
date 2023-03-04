#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

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

std::deque<nav_msgs::Odometry> odomVec;
std::deque<sensor_msgs::NavSatFix> gpsVec;
std::deque<sensor_msgs::Imu> imuVec;

std::deque<sensor_msgs::PointCloud2> laserVec;
nav_msgs::OccupancyGrid map_prob;
nav_msgs::OccupancyGrid map_;
geometry_msgs::Point cur_Lidarposition;//当前车辆(雷达)在栅格地图中的位置
//media/meng/5418189112144B70/kitti/residential/kitti_2011_09_26_drive_0079_synced.bag
float P_prior = 0.5;	//# Prior occupancy probability  //先验占据概率
float P_occ = 0.9	; //# Probability that cell is occupied with total confidence //栅格被完全置信度下占用的概率
float P_free = 0.3;	//# Probability that cell is free with total confidence        //栅格被完全置信度下空闲的概率
float P_free_ray = 0.4;	 //经过ray cast 推理后，栅格被完全置信度下空闲的概率
float P_occ_ray = 0.8;	 //经过ray cast 推理后，栅格被完全置信度下占据的概率
float TRESHOLD_P_FREE = 0.3;
float TRESHOLD_P_OCC = 0.6;
int obs_prob;//被激光雷达打中多少次视为占据
double width;//地图宽度
double height;//地图高度
double resolution;//地图分辨率
double differ_height;//高度差
double min_differ_height;//悬浮物体的最小高度差
double max_height;//直通滤波区域
double min_height;//
double max_width;//
double min_width;//
double max_len;//
double min_len;//

//将占据概率信息存储在图像矩阵中
cv::Mat bayes_image;
// bool bayes_use = false;
bool bayes_use ;
bool use_ray_cast;
bool use_odom_filter;
double UTME0;//东  //utm坐标系下车辆初始位置
double UTMN0;//北
double UTME;//当前位置
double UTMN;
double car_x0;//x轴，对应东向  //utm坐标系下车辆初始位置
double car_y0;//y轴，对应北向
double car_x;//当前位置
double car_y;

bool RT = true;
std::mutex alock;
std::mutex multi_callback_lock;
double robot_x;
double robot_y;

// 未知：-1， 障碍物：100， 可通行：0
int passvalue = 0;
Eigen::Quaterniond q_odom_curr_now;
Eigen::Vector3d t_odom_curr_now;
float yaw_angle;//当前时刻绕z轴旋转角度
#define M_PI 3.1415926;
double max_x = -10000;
double max_y = -10000;
double min_x = 10000;
double min_y = 10000;
bool map_init = true;

ros::Publisher  pubCarpositiopn,pubBasicFilterCloud,pubWorldCloud,pubGridMap;//车辆位置、基本滤波后的点云、世界坐标系下的点云、2d栅格地图


//点云基本操作相关
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInRaw(new pcl::PointCloud<pcl::PointXYZ>);//原始点云(单帧)
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_basic_filter(new pcl::PointCloud<pcl::PointXYZ>);//基本滤波之后的点云（单帧）
sensor_msgs::PointCloud2 basic_filterCloudMsg;//基本滤波之后发布的点云消息
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInWorld(new pcl::PointCloud<pcl::PointXYZ>);//世界坐标系（车辆初始位置）下的点云（单帧）
sensor_msgs::PointCloud2 worldCloudMsg;//基本滤波之后发布的点云消息
bool use_outlier_remove_with_radius;//基于半径----离群点滤除
float filter_radius;//基于半径进行离群点滤除的参数
int filter_number;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_filter(new pcl::PointCloud<pcl::PointXYZ>);// 基于半径滤波后的点云指针

//栅格地图相关
std::map<std::pair<int, int>, std::vector<double>> gridMap;//存放单帧点云的栅格
//栅格地图后处理/优化相关，地图只需要在栅格地图创建完成后优化一次
int laserVec_size_flags=0;//计算接收点云的个数的标志位
int map_optimizer=0;//地图是否优化，没有优化为0，优化为1

//地图匹配---------------
ofstream outfile;//保存开始建图时的gps和imu信息
string path;//保存开始建图时的gps和imu信息
bool map_info_write=0;//是否保存地图的基本信息
class mapheader {//地图的基本信息
    public:
        float latitude;
        float longitude;
        float qx;//初始姿态，四元数
        float qy;
        float qz;
        float qw;
        int map_width;//地图的宽度高度
        int map_height;       
        float map_resolution;//分辨率
        int origin_x;//为了尽可能使地图充满栅格所平移的距离
        int origin_y;
        int x_offset;//为了尽可能使地图充满栅格所平移的距离
        int y_offset;
};
mapheader mapheader_tmp;
int x_offset_total;//显示地图需要
int y_offset_total;