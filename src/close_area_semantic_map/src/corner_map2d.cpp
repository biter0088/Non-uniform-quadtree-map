#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define M_PI 3.1415926
#define range_threshold 1.0//射线搜索距离阈值(最大值)，单位m
// #define range_threshold 4//射线搜索距离阈值(最大值)，单位m   0901-carla-town01
#define range_threshold_min 0.1//射线搜索距离阈值(最小值)，单位m
#define occgrid_value -128//占据栅格的属性值
#define freegrid_value 0//空闲栅格的属性值
using namespace std;

//地图参数与地图发布相关------------------------------------------------------------
int width;//地图宽度
int height;//地图高度
float resolution;//地图分辨率
float map_origin[2]={0,0};
int map_width_inc ;//地图栅格数
int map_height_inc;
ros::Publisher map_pub,keypoints_pub,samplepoints_pub;
visualization_msgs::Marker keypoint_marker;//单个关键点
visualization_msgs::MarkerArray keypoints_markerArray;//所有关键点
visualization_msgs::MarkerArray samplepoints_markerArray;//所有采样点
int marker_id=0;
int samplemarker_id=0;
nav_msgs::OccupancyGrid map_;
int receive_map=0;//是否接收到地图
int search_end=0;//是否进行完一次搜索
#define search_angle 2//射线搜索法的角度间隔
#define search_lines 180//射线搜索法的线的条数
// 寻找地图中的半包围结构相关-----------------------------------------------------
#define sampling_interval 3//间隔多少个栅格选出一个关键点，10、5、3(6925)
// #define sampling_interval 20//间隔多少个栅格选出一个关键点，10、5、3(6925)   0901调试carla ：town1
#define keypoints_threshold 0.2//判断是否为关键点，有些边界比较参差不齐，到边界最远距离过短需要筛除，这里设为4个像素远即为0.20m
float keypoints_max_distance=0;
float keypoints_min_distance=0;
#define sample_close_area_interval 1//在关键点与占据栅格包围区域内，多少个分辨率去一个点来代表附近区域，2、3、1.5
#define fill_one_grid_threshold 10//用最远多远周围占据或语义栅格的属性值来填充空闲栅格
int keypoints[20000]={0};//存放关键点的x,y坐标,+10是为了防止溢出,初始化为0--------这里需要注意，初始化为1000个，不要溢出，溢出的话增加即可
int keypoints_num=0;
int keypoints_border[20000]={0};//存放为死角的关键点的边界的x,y坐标，最多有地图大小个边界点
int keypoints_border_num=0;


int sample_points[100000][2]={0};//采样点的坐标x和y不会都为0
int valid_sample_points[50000][2]={0};//去除采样点中相同的点
int num_valid_sample_points=0;//有效采样点个数
#include "visualize.h"//用于可视化
#include "close_area_2d.h"
#include "map_change_2d.h"
// #include "map_change.h"
#include "map_fill.h"

// 生成充满地图的关键点（这里生成的关键点不是随机的，之后可以考虑随机生成）
void produce_keypoints(){
  for(int i=sampling_interval;i<map_.info.width;i=i+sampling_interval){
    for(int j=sampling_interval;j<map_.info.height;j=j+sampling_interval){
      if(map_.data[i+j*map_.info.width]==100 || map_.data[i+j*map_.info.width]==-1){
        continue;
      }
      keypoints[2*keypoints_num]=i;
      keypoints[2*keypoints_num+1]=j;      
      keypoints_num++;
      // cout<<"keypoints_num: "<<keypoints_num<<endl;
    }
  }
  cout<<"keypoints_num: "<<keypoints_num<<endl;
}


// 扩展一个关键点区域
// 判断条件：
    // 关键点被大于180°的角度包围
void extend_one_keypoint(int x,int y){
  int * max_angle_range;
  max_angle_range=max_close_angle_range(x,y);
  cout<<"包围角度范围:("<<*(max_angle_range+0)*2<<", "<<*(max_angle_range+1)*2<<")"<<endl;
  cout<<"包围射线的最远距离: "<<keypoints_max_distance<<endl;
  if(*(max_angle_range+0)!=400&&*(max_angle_range+1)!=400&&keypoints_max_distance>(keypoints_threshold/resolution)){
    sample_close_area(x,y,*(max_angle_range+0),*(max_angle_range+1));
    // visualize_sample_points();
    valid_sample_points_filter();//滤除相同的采样点，减少运算量
    // mapchange_with_samplepoints();//移动到其他位置，将多个关键点的采样点集合在一起进行处理
    // map_fill();
    sample_points_init();//将采样点的数组集合进行初始化
  }
}


// 扩展关键点区域
void extend_keypoints(){
  // for(int i=3100;i<3200;i++){
  //   cout<<"i:  "<<i<<endl;
  //   extend_one_keypoint(keypoints[2*i],keypoints[2*i+1]);
  //   if(num_valid_sample_points>=45000){//50000是有效采样点数组初始化的大小
  //     mapchange_with_samplepoints();//将多个关键点的采样点放在一起进行处理
  //     valid_sample_points_init();
  //   }
  // }
  // extend_one_keypoint(keypoints[2*3113],keypoints[2*3113+1]);//先测试一个点

  for(int i=0;i<keypoints_num;i++){
    cout<<"i:  "<<i<<endl;
    extend_one_keypoint(keypoints[2*i],keypoints[2*i+1]);
    if(num_valid_sample_points>=45000){//50000是有效采样点数组初始化的大小
      mapchange_with_samplepoints();//将多个关键点的采样点放在一起进行处理
      valid_sample_points_init();
    }
  }
  mapchange_with_samplepoints();//使用剩余的有效采样点处理地图
  valid_sample_points_init();
}

// 寻找地图中的半包围结构，更改该区域内地图属性值
void search_close_area(){
  // 生成充满地图的关键点（这里生成的关键点不是随机的，之后可以考虑）
  produce_keypoints();
  // visualize_keypoints();
  extend_keypoints();
  search_end=1;
}

void map_simple_optimize(){
  //如果未知栅格周边有四个栅格是可通行的，那么设置它为可通行
  std::cout<<"进行简单地图优化"<<std::endl;
  for (int i=1; i<map_.info.width-1; i++) 
  {
    for (int j=1; j<map_.info.height-1; j++){
      auto prob = map_.data[i + j*map_.info.width];
      auto prob_up = map_.data[i + (j+1)*map_.info.width];
      auto prob_down = map_.data[i + (j-1)*map_.info.width];
      auto prob_left = map_.data[i-1 + j*map_.info.width];
      auto prob_right = map_.data[i+1 + j*map_.info.width];

      if(prob==-1&&prob_up==0&&prob_down==0&&prob_left==0&&prob_right==0)
        map_.data[i + j*map_.info.width]=0;
    }
  }
}

void map_handler(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  map_=*map_msg;
  map_origin[0]=map_.info.origin.position.x;
  map_origin[1]=map_.info.origin.position.y;
  width=map_.info.width;
  height=map_.info.height;
  resolution=map_.info.resolution;
  map_width_inc = width;//有些多次一举
  map_height_inc = height;

  std::cout<<"地图原点坐标:"<<"("<<map_origin[0]<<","<<map_origin[1]<<")"<<std::endl;
  std::cout<<"地图宽度,高度,分辨率:("<<width<<","<<height<<","<<resolution<<")"<<std::endl;
  receive_map=1;//接收到地图
  // map_simple_optimize();//进行简单的地图优化
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_map");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/global_map", 1, map_handler);//1000改为1
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/create_map", 100);
  keypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("keypoints", 1);//可视化关键点
  samplepoints_pub = nh.advertise<visualization_msgs::MarkerArray>("samplepoints", 1);//可视化有效采样点

  ros::Rate rate_1hz(1);  
  while(ros::ok())
  {
    if(receive_map==0){
      ros::spinOnce();
    }
    else 
      if(search_end==0){
        search_close_area();
      }
      else{
        map_pub.publish(map_);
        keypoints_pub.publish(keypoints_markerArray);
        samplepoints_pub.publish(samplepoints_markerArray);
      }
    rate_1hz.sleep();
  }

  return 0;
}
