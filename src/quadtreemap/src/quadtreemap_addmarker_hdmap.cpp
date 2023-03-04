#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <string>
// #include "ptm2ogm/common.h"
#include "ptm2ogm/tic_toc.h"

#include "quadtree/Quadtree.h"//hxz
#include "quadtree/Stopwatch.h"//hxz

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp> //hxz

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <map>  
#include <algorithm>
#include<mutex>
#include <math.h> //hxz
#include <fstream>//hxz
#include <iostream>//hxz
using namespace std; //hxz
using namespace brandonpelfrey;

nav_msgs::OccupancyGrid globalmap;
nav_msgs::OccupancyGrid globalmap_origin;

std::vector<Vector2> points;
Quadtree *quadtree;
QuadtreePoint *quadtreePoints;

std::mutex quadtree_init_lock;
cv::Mat quadtree_img;
int sameflag[]={0,0};//栅格属性一致的标志位
//根据论文处理非均匀四元树地图
int ratio_width=0;//约分比例
int ratio_height=0;
int quad_width=0;//四元树地图的宽度
int quad_height=0;//四元树地图的高度
int depth=0;//四叉树地图的深度，用于进行迭代
int max_depth=0;//四叉树地图的最大深度
float resolution;//四叉树地图的分辨率
ofstream outfile;
string path;//四叉树地图中占据观测数据写入文件地址
int d_lw;//最小公约数
ros::Publisher  pubGlobalmap_tmp;
int quad_constructor_once=0;//控制回调函数只在第一次回调时进行建图
long int node_total_num=0;//非均匀四元树创建的节点总数

class quadnode {//节点的数据格式
public:
  int origin_x;
  int origin_y;
  int halfDimension_x;
  int halfDimension_y;
  int depth;
  int type;
};

//使用marker显示相关参量-----------------------------------------------------------
long int marker_id;//每一个marker的id号
visualization_msgs::Marker marker;//单个3d栅格可视化方式
visualization_msgs::MarkerArray markerArray;//3d栅格可视化方式
ros::Publisher marker_pub;
std_msgs::ColorRGBA color_passive;
std_msgs::ColorRGBA color_occ;
std_msgs::ColorRGBA color_unknown;
std_msgs::ColorRGBA color_deadend_inner;
std_msgs::ColorRGBA color_deadend_middle;
std_msgs::ColorRGBA color_deadend_outer;
ros::Time rostime;
bool use_resolution_005;//地图分辨率是否为0.05m，用于更清楚地显示出不同地图节点的分届框
bool use_resolution_01;//地图分辨率是否为0.2m，用于更清楚地显示出不同地图节点的分届框
bool use_resolution_02;//地图分辨率是否为0.2m，用于更清楚地显示出不同地图节点的分届框
bool use_resolution_05;//地图分辨率是否为0.5m，用于更清楚地显示出不同地图节点的分届框

void add_marker(const ros::Time& rostime, int id, float x_marker, float y_marker, float z_marker, \
                                     float x_scale, float y_scale, float z_scale,std_msgs::ColorRGBA color){
    // marker.header.frame_id = "/map";
    marker.header.frame_id = "/camera_init";
    marker.header.stamp = rostime;
    marker.ns = "quadtreemap_addmarker";
    marker.id = id;//int32
    marker_id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // marker.pose.position.x = x_marker;//float64
    // marker.pose.position.y = y_marker;
    // marker.pose.position.x = x_marker-globalmap_origin.info.width*globalmap_origin.info.resolution/2.0;//float64
    // marker.pose.position.y = y_marker-globalmap_origin.info.height*globalmap_origin.info.resolution/2.0;  
    // 为了对齐栅格地图和marker显示  2022/12/05修改 
    marker.pose.position.x = x_marker;//float64
    marker.pose.position.y = y_marker;      
    marker.pose.position.z = z_marker-0.5;
    // marker.pose.position.z = z_marker;

    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = x_scale;
    marker.scale.y = y_scale;
    marker.scale.z = z_scale;
    marker.color=color;//
    marker.lifetime = ros::Duration();
    // marker.lifetime = ros::Duration(0.2);

    markerArray.markers.push_back(marker);
}

//设置最低级指定区域的颜色
void leastnodecolor(Quadtree* quadtree_target){
  int least_x=quadtree_target->origin.x;
  int least_y=quadtree_target->origin.y;
  int width_least=quadtree_target->halfDimension.x;
  int height_least=quadtree_target->halfDimension.y;

  for (int col=least_x-width_least; col<least_x+width_least; col++) 
  {
    for (int row=least_y-height_least; row<least_y+height_least; row++){
      switch (globalmap.data[col + row*globalmap.info.width])
     {
      case 0://可通行-灰白色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 178;
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 178;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 178;
        break;
      case 100://障碍物-黑色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 0;
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 0;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 0;
        break;
      case -1://未知区域-灰色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 94;//rviz中的灰色//at<cv::Vec3b>顺序BGR
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 96;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 78;
        break;
      default:
        break;
      }
    }
  }
}


//在指定区域增加marker
void currentnodemarker(Quadtree* quadtree_target){
  float x_marker=quadtree_target->origin.x*resolution;
  float y_marker=quadtree_target->origin.y*resolution;
  // float x_scale=quadtree_target->halfDimension.x*resolution*2;
  // float y_scale=quadtree_target->halfDimension.y*resolution*2; 
  //下面操作是为了在大尺度地图上相对比较明显地表示出边界--------------
  float x_scale;
  float y_scale;
  /*
  x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//针对比较小的地图，用于显示
  y_scale=quadtree_target->halfDimension.y*resolution*2-0.02; 
  */

  // std::cout<<"resolution: "<<resolution<<std::endl;

  if(use_resolution_05){  //分辨率为0.5m
    if(quadtree_target->halfDimension.x>=100){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15; 
    }
    else if(quadtree_target->halfDimension.x>=30){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15;    
    }
    else if(quadtree_target->halfDimension.x>=15){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15; 
    }
    else if(quadtree_target->halfDimension.x>=8){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15; 
      // x_scale=quadtree_target->halfDimension.x*resolution*2;//凸显边界
      // y_scale=quadtree_target->halfDimension.y*resolution*2; 
    }
    else if(quadtree_target->halfDimension.x>=4){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15; 
      // x_scale=quadtree_target->halfDimension.x*resolution*2;//凸显边界
      // y_scale=quadtree_target->halfDimension.y*resolution*2;       
    }
    else {
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.15;//
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.15;     
      // x_scale=quadtree_target->halfDimension.x*resolution*2;//
      // y_scale=quadtree_target->halfDimension.y*resolution*2;    
    }
  }

  if(use_resolution_02){  //分辨率为0.2m
    if(quadtree_target->halfDimension.x>=100){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=30){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1;    
    }
    else if(quadtree_target->halfDimension.x>=15){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=8){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=4){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else {
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.05;//
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.05;     
    }
  }
  
  if(use_resolution_01){  //分辨率为0.1m
    if(quadtree_target->halfDimension.x>=100){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=30){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1;    
    }
    else if(quadtree_target->halfDimension.x>=15){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=8){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else if(quadtree_target->halfDimension.x>=4){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.1;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.1; 
    }
    else {
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.05;//
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.05;     
    }
  } 

  // 分辨率0.05m  局部地图
  if(use_resolution_005){
    if(quadtree_target->halfDimension.x>=100){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02; 
    }
    else if(quadtree_target->halfDimension.x>=30){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02;    
    }
    else if(quadtree_target->halfDimension.x>=15){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02; 
    }
    else if(quadtree_target->halfDimension.x>=4){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02; 
    }
    else if(quadtree_target->halfDimension.x>=2){
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//凸显边界
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02; 
    }
    else {
      x_scale=quadtree_target->halfDimension.x*resolution*2-0.02;//
      y_scale=quadtree_target->halfDimension.y*resolution*2-0.02;     
    }

  }

  switch (sameflag[0])
  {
  case 0://可通行-灰白色
    quadtree_target->type=0;
    add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.05, color_passive);
    break;
  case 100://障碍物-黑色
    quadtree_target->type=1;
    // outfile<<quadtree_target->origin.x<<"  "<<quadtree_target->origin.y<<"  " \
    //              <<quadtree_target->halfDimension.x<<"  "<<quadtree_target->halfDimension.y<<"  " \
    //              <<quadtree_target->depth<<"  "<<quadtree_target->type<<"  "<< endl;
    
    /*
    quadnode node_tmp;
    node_tmp.origin_x=quadtree_target->origin.x;
    node_tmp.origin_y=quadtree_target->origin.y;
    node_tmp.halfDimension_x=quadtree_target->halfDimension.x;
    node_tmp.halfDimension_y=quadtree_target->halfDimension.y;
    node_tmp.depth=quadtree_target->depth;
    node_tmp.type=quadtree_target->type;
    outfile.write((char*)&node_tmp, sizeof(node_tmp));
    */
    // add_marker(rostime, marker_id,  x_marker,  y_marker,  0,  x_scale,  y_scale,  0.05, color_occ);    
    add_marker(rostime, marker_id,  x_marker,  y_marker,  0,  x_scale,  y_scale,  0.2, color_occ);   //占据节点高度较高，显示出立体感 
    break;
  case -1://未知区域-灰色
    quadtree_target->type=-1;
    add_marker(rostime, marker_id,  x_marker,  y_marker,  0,  x_scale,  y_scale,  0.05, color_unknown);
    break;
  default:
    if(sameflag[0]<-60){
      quadtree_target->type=2;//死角区域内侧
      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.5, color_deadend_inner);
      }     
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale, y_scale,  0.5, color_deadend_inner);
      }        
      if(use_resolution_005){// 分辨率0.05m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale, y_scale,  0.5, color_deadend_inner);
      }                      
      quadnode node_tmp;
      node_tmp.origin_x=quadtree_target->origin.x;
      node_tmp.origin_y=quadtree_target->origin.y;
      node_tmp.halfDimension_x=quadtree_target->halfDimension.x;
      node_tmp.halfDimension_y=quadtree_target->halfDimension.y;
      node_tmp.depth=quadtree_target->depth;
      node_tmp.type=quadtree_target->type;
      outfile.write((char*)&node_tmp, sizeof(node_tmp));
    }
    else  if(sameflag[0]<-30){
      quadtree_target->type=3;//死角区域中间
      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.5, color_deadend_middle);
      }   
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.5, color_deadend_middle);
      }       
      if(use_resolution_005){// 分辨率0.05
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.5, color_deadend_middle);
      }           
      quadnode node_tmp;
      node_tmp.origin_x=quadtree_target->origin.x;
      node_tmp.origin_y=quadtree_target->origin.y;
      node_tmp.halfDimension_x=quadtree_target->halfDimension.x;
      node_tmp.halfDimension_y=quadtree_target->halfDimension.y;
      node_tmp.depth=quadtree_target->depth;
      node_tmp.type=quadtree_target->type;
      outfile.write((char*)&node_tmp, sizeof(node_tmp));         
    }
    else{
      quadtree_target->type=4;//死角区域外侧
      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale,  y_scale,  0.5, color_deadend_outer);
      }   
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale, y_scale,  0.5, color_deadend_outer);
      }   
      if(use_resolution_005){// 分辨率0.05m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, x_scale, y_scale,  0.5, color_deadend_outer);
      }         
      quadnode node_tmp;
      node_tmp.origin_x=quadtree_target->origin.x;
      node_tmp.origin_y=quadtree_target->origin.y;
      node_tmp.halfDimension_x=quadtree_target->halfDimension.x;
      node_tmp.halfDimension_y=quadtree_target->halfDimension.y;
      node_tmp.depth=quadtree_target->depth;
      node_tmp.type=quadtree_target->type;
      outfile.write((char*)&node_tmp, sizeof(node_tmp));        
    }  
    break;
  }
}


//设置图片指定区域的颜色
void currentnodecolor(Quadtree* quadtree_target){
  int row_min=quadtree_target->origin.y-quadtree_target->halfDimension.y;
  int row_max=quadtree_target->origin.y+quadtree_target->halfDimension.y-1;
  int col_min=quadtree_target->origin.x-quadtree_target->halfDimension.x;
  int col_max=quadtree_target->origin.x+quadtree_target->halfDimension.x-1;

  // if(quadtree_target->halfDimension.x>=4){//比较大的四元分区绘制矩形边框
  if(quadtree_target->halfDimension.x>=2){//比较大的四元分区绘制矩形边框
    //绘制当前节点的边界
    cv::Scalar color;
    color[0]=0;color[1]=0;color[2]=255;//红色    绿色
    cv::rectangle(quadtree_img,cvPoint(col_min,row_min),cvPoint(col_max,row_max),color,1,1,0);
    row_min++;//防止矩形边框被覆盖
    row_max--;
    col_min++;
    col_max--;
  }


  for (int row = row_min; row <=row_max; row++)
	{	
		for (int col = col_min; col <=col_max; col++)
		{
      switch (sameflag[0])
     {
      case 0://可通行-灰白色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 178;
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 178;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 178;
        quadtree_target->type=0;
        break;
      case 100://障碍物-黑色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 0;
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 0;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 0;
        quadtree_target->type=1;
        // outfile<<quadtree_target->origin.x<<"  "<<quadtree_target->origin.y<<"  " \
        //              <<quadtree_target->halfDimension.x<<"  "<<quadtree_target->halfDimension.y<<"  " \
        //              <<quadtree_target->depth<<"  "<<quadtree_target->type<<"  "<< endl;
        quadnode node_tmp;
        node_tmp.origin_x=quadtree_target->origin.x;
        node_tmp.origin_y=quadtree_target->origin.y;
        node_tmp.halfDimension_x=quadtree_target->halfDimension.x;
        node_tmp.halfDimension_y=quadtree_target->halfDimension.y;
        node_tmp.depth=quadtree_target->depth;
        node_tmp.type=quadtree_target->type;
        outfile.write((char*)&node_tmp, sizeof(node_tmp));
        break;
      case -1://未知区域-灰色
        quadtree_img.at<cv::Vec3b>(row,col)[0] = 94;//rviz中的灰色//at<cv::Vec3b>顺序BGR
        quadtree_img.at<cv::Vec3b>(row,col)[1] = 96;
        quadtree_img.at<cv::Vec3b>(row,col)[2] = 78;
        quadtree_target->type=-1;
        break;
      default:
        break;
      }
    }
	}
}

//查询当前节点对应的所有栅格是否属性一致
bool currentnodecheck(Quadtree* quadtree_cur)
{
  // double start = stopwatch();//用于计时
  int tree_x=quadtree_cur->origin.x;
  int tree_y=quadtree_cur->origin.y;
  int width_range=quadtree_cur->halfDimension.x;
  int height_range=quadtree_cur->halfDimension.y;

  sameflag[0]=globalmap.data[tree_x-width_range +(tree_y-height_range)*globalmap.info.width];
  for (int i=tree_x-width_range; i<tree_x+width_range; i++) 
  {
    for (int j=tree_y-height_range; j<tree_y+height_range; j++){
      if((i==tree_x-width_range) && (j==tree_y-height_range)){//不跟自己比
        continue;
      }
      sameflag[1]=globalmap.data[i + j*globalmap.info.width];
      if (sameflag[0]!=sameflag[1]){
        // std::cout<<"当前节点内包括不同属性的栅格，需要创建子节点"<<std::endl;
        // double T = stopwatch() - start;
	      // printf("当前节点查询耗时 in %.5f sec.\n", T);
        return false;
      }
    }
  }
  // double T = stopwatch() - start;
	// printf("当前节点查询耗时 in %.5f sec.\n", T);
  // std::cout<<"当前节点内包括相同属性的栅格，不需要创建子节点"<<std::endl;
  return true;
}

// 给最低一级四叉树节点添加属性
void quadtree_end(Quadtree* quadtree_,int depth){
  depth--;//此时depth应该是0
  // std::cout<<"depth:"<<depth<<std::endl;
  for(int i=0; i<4; ++i) {//给最低一级的
      /*// 四个节点序号分布（按照地图坐标系x右y上，相对应图像坐标系x右y下）如下，此时0号节点应该和父节点的原点相同
				// ------------  ros地图坐标系x右y上
				// 1--+-- 0
				// 3 --+-- 2
				// ----+------

    Vector2 newOrigin = quadtree_->origin;
    newOrigin.x += quadtree_->halfDimension.x * (i&1 ? -1.f : .0f);//i为1、3时为+，0、2时为 0
    newOrigin.y += quadtree_->halfDimension.y * (i>1 ?  -1.f : .0f);//i为2、3时为+，0、1时为 0 */

      // 四个节点序号分布（按照地图坐标系x右y上，相对应图像坐标系x右y下）如下，此时3号节点应该和父节点的原点相同
				// ------------  ros地图坐标系x右y上
				// 2--+-- 3
				// 0 --+-- 1
				// ----+------

    Vector2 newOrigin = quadtree_->origin;
    newOrigin.x += quadtree_->halfDimension.x * (i&1 ? .0f : -1.f);//i为1、3时为0，0、2时为 -1
    newOrigin.y += quadtree_->halfDimension.y * (i>1 ?  .0f : -1.f);//i为2、3时为0，0、1时为 -1
    quadtree_->children[i] = new Quadtree(newOrigin, Vector2(0,0),depth);//最低级节点没有半高半宽
    switch (globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width])
    {
    case 0://可通行
      quadtree_->children[i]->type=0;
      break;
    case 100://障碍物
      quadtree_->children[i]->type=1;
      // outfile<<quadtree_->children[i]->origin.x<<"  "<<quadtree_->children[i]->origin.y<<"  " \
      //              <<quadtree_->children[i]->halfDimension.x<<"  "<<quadtree_->children[i]->halfDimension.y<<"  " \
      //              <<quadtree_->children[i]->depth<<"  "<<quadtree_->children[i]->type<<"  "<< endl;
      quadnode node_tmp;
      node_tmp.origin_x=quadtree_->children[i]->origin.x;
      node_tmp.origin_y=quadtree_->children[i]->origin.y;
      node_tmp.halfDimension_x=quadtree_->children[i]->halfDimension.x;
      node_tmp.halfDimension_y=quadtree_->children[i]->halfDimension.y;
      node_tmp.depth=quadtree_->children[i]->depth;
      node_tmp.type=quadtree_->children[i]->type;
      outfile.write((char*)&node_tmp, sizeof(node_tmp));
      break;
    case -1://未知区域
      quadtree_->children[i]->type=-1;
      break;
    default:
      break;
    }
    /*
    std::cout<<"第"<<i<<"个最低一级四叉树节点的 x:"<<quadtree_->children[i]->origin.x<< \
                                                                                             "    y:"<<quadtree_->children[i]->origin.y<< \
                                                                     "    x_dimension:"<<quadtree_->children[i]->halfDimension.x<<\
                                                                     "    y_dimension:"<<quadtree_->children[i]->halfDimension.y<<\
                                                                                    "    depth:"<<quadtree_->children[i]->depth<<\
                                                                                       "    type:"<<quadtree_->children[i]->type<<std::endl;                 */                                                                   
  }
}

// 给最低一级四叉树节点添加属性
void quadtree_end_with_marker(Quadtree* quadtree_,int depth){
  depth--;//此时depth应该是0
  // std::cout<<"depth:"<<depth<<std::endl;
  for(int i=0; i<4; ++i) {//给最低一级的
      /*// 四个节点序号分布（按照地图坐标系x右y上，相对应图像坐标系x右y下）如下，此时0号节点应该和父节点的原点相同
				// ------------  ros地图坐标系x右y上
				// 1--+-- 0
				// 3 --+-- 2
				// ----+------

    Vector2 newOrigin = quadtree_->origin;
    newOrigin.x += quadtree_->halfDimension.x * (i&1 ? -1.f : .0f);//i为1、3时为+，0、2时为 0
    newOrigin.y += quadtree_->halfDimension.y * (i>1 ?  -1.f : .0f);//i为2、3时为+，0、1时为 0 */

      // 四个节点序号分布（按照地图坐标系x右y上，相对应图像坐标系x右y下）如下，此时3号节点应该和父节点的原点相同
				// ------------  ros地图坐标系x右y上
				// 2--+-- 3
				// 0 --+-- 1
				// ----+------

    Vector2 newOrigin = quadtree_->origin;
    newOrigin.x += quadtree_->halfDimension.x * (i&1 ? .0f : -1.f);//i为1、3时为0，0、2时为 -1
    newOrigin.y += quadtree_->halfDimension.y * (i>1 ?  .0f : -1.f);//i为2、3时为0，0、1时为 -1
    quadtree_->children[i] = new Quadtree(newOrigin, Vector2(0,0),depth);//最低级节点没有半高半宽

    float x_marker=newOrigin.x*resolution+resolution*0.5;
    float y_marker=newOrigin.y*resolution+resolution*0.5;

    switch (globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width])
    {
    case 0://可通行
      quadtree_->children[i]->type=0;

      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.05, color_passive);
      }          
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.05, color_passive);
      }    
      if(use_resolution_005){// 分辨率0.05m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.05, color_passive);
      }
      if(use_resolution_01){// 分辨率0.1m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.05, color_passive);
      }
      break;
    case 100://障碍物
      quadtree_->children[i]->type=1;
      // outfile<<quadtree_->children[i]->origin.x<<"  "<<quadtree_->children[i]->origin.y<<"  " \
      //              <<quadtree_->children[i]->halfDimension.x<<"  "<<quadtree_->children[i]->halfDimension.y<<"  " \
      //              <<quadtree_->children[i]->depth<<"  "<<quadtree_->children[i]->type<<"  "<< endl;
      
      /*
      quadnode node_tmp;
      node_tmp.origin_x=quadtree_->children[i]->origin.x;
      node_tmp.origin_y=quadtree_->children[i]->origin.y;
      node_tmp.halfDimension_x=quadtree_->children[i]->halfDimension.x;
      node_tmp.halfDimension_y=quadtree_->children[i]->halfDimension.y;
      node_tmp.depth=quadtree_->children[i]->depth;
      node_tmp.type=quadtree_->children[i]->type;
      outfile.write((char*)&node_tmp, sizeof(node_tmp));
      */

      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.2, color_occ);//占据节点高度较高，显示出立体感 
      }
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.2, color_occ);//占据节点高度较高，显示出立体感 
      }
      if(use_resolution_01){// 分辨率0.1m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.2, color_occ);//占据节点高度较高，显示出立体感 
      }
      if(use_resolution_005){// 分辨率0.05m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.2, color_occ);//占据节点高度较高，显示出立体感 
      }
      break;
    case -1://未知区域
      quadtree_->children[i]->type=-1;
      if(use_resolution_05){// 分辨率0.5m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.05, color_unknown);
      }      
      if(use_resolution_02){// 分辨率0.2m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.05, color_unknown);
      }
      if(use_resolution_01){// 分辨率0.1m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.05, color_unknown);
      }
      if(use_resolution_005){// 分辨率0.05m
        add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution,  resolution,  0.05, color_unknown);
      }
      break;
    default:
      // if(globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width]<-60){
      if(globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width]<=-60){        
        quadtree_->children[i]->type=2;//死角区域内侧
        if(use_resolution_05){// 分辨率0.5m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.5, color_deadend_inner);
        }      
        if(use_resolution_02){// 分辨率0.2m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.5, color_deadend_inner);
        }   
        if(use_resolution_005){// 分辨率0.05m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.01,  resolution-0.01,  0.5, color_deadend_inner);
        }           
        quadnode node_tmp;
        node_tmp.origin_x=quadtree_->children[i]->origin.x;
        node_tmp.origin_y=quadtree_->children[i]->origin.y;
        node_tmp.halfDimension_x=quadtree_->children[i]->halfDimension.x;
        node_tmp.halfDimension_y=quadtree_->children[i]->halfDimension.y;
        node_tmp.depth=quadtree_->children[i]->depth;
        node_tmp.type=quadtree_->children[i]->type;
        outfile.write((char*)&node_tmp, sizeof(node_tmp));        
      }
      // else  if(globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width]<-30){
      else  if(globalmap.data[newOrigin.x + newOrigin.y*globalmap.info.width]<=-20){        
        quadtree_->children[i]->type=3;//死角区域中间
        if(use_resolution_05){// 分辨率0.5m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.5, color_deadend_middle);
        }      
        if(use_resolution_02){// 分辨率0.2m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.5, color_deadend_middle);
        }   
        if(use_resolution_005){// 分辨率0.05m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.01,  resolution-0.01,  0.5, color_deadend_middle);
        }           
        quadnode node_tmp;
        node_tmp.origin_x=quadtree_->children[i]->origin.x;
        node_tmp.origin_y=quadtree_->children[i]->origin.y;
        node_tmp.halfDimension_x=quadtree_->children[i]->halfDimension.x;
        node_tmp.halfDimension_y=quadtree_->children[i]->halfDimension.y;
        node_tmp.depth=quadtree_->children[i]->depth;
        node_tmp.type=quadtree_->children[i]->type;
        outfile.write((char*)&node_tmp, sizeof(node_tmp));        
      }
      else{
        quadtree_->children[i]->type=4;//死角区域外侧
        if(use_resolution_05){// 分辨率0.5m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.15,  resolution-0.15,  0.5, color_deadend_outer);
        }    
        if(use_resolution_02){// 分辨率0.2m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.02,  resolution-0.02,  0.5, color_deadend_outer);
        }       
        if(use_resolution_005){// 分辨率0.05m
          add_marker(rostime, marker_id,  x_marker,  y_marker,  0, resolution-0.01,  resolution-0.01,  0.5, color_deadend_outer);
        }                        
        quadnode node_tmp;
        node_tmp.origin_x=quadtree_->children[i]->origin.x;
        node_tmp.origin_y=quadtree_->children[i]->origin.y;
        node_tmp.halfDimension_x=quadtree_->children[i]->halfDimension.x;
        node_tmp.halfDimension_y=quadtree_->children[i]->halfDimension.y;
        node_tmp.depth=quadtree_->children[i]->depth;
        node_tmp.type=quadtree_->children[i]->type;
        outfile.write((char*)&node_tmp, sizeof(node_tmp));
      }
      break;
    }
    /*
    std::cout<<"第"<<i<<"个最低一级四叉树节点的 x:"<<quadtree_->children[i]->origin.x<< \
                                                                                             "    y:"<<quadtree_->children[i]->origin.y<< \
                                                                     "    x_dimension:"<<quadtree_->children[i]->halfDimension.x<<\
                                                                     "    y_dimension:"<<quadtree_->children[i]->halfDimension.y<<\
                                                                                    "    depth:"<<quadtree_->children[i]->depth<<\
                                                                                       "    type:"<<quadtree_->children[i]->type<<std::endl;                 */                                                                   
  }
}

void quadtree_init( Quadtree* quadtree_,int depth){
  // quadtree_init_lock.lock();
  // std::cout<<"开始初始化四叉树"<<std::endl;
  //初始化一个根节点，并创建多级子节点，使最后一级子节点的分辨率是原始栅格地图分辨率的4倍----即宽度和高度方向均为2倍
  if(currentnodecheck(quadtree_)==false){
    // std::cout<<std::endl;
    if (quadtree_->halfDimension.x==1){//倒数第二低的一级即depth=1
      // leastnodecolor(quadtree_);
      // quadtree_end(quadtree_,depth);
      quadtree_end_with_marker(quadtree_,depth);
      node_total_num=node_total_num+4;//增加了四个节点
      /*
      cv::namedWindow("quadtree_img", cv::WINDOW_NORMAL);
      cv::imshow("quadtree_img",quadtree_img);
      cv::waitKey(1);
      */
    } 
    else {//不是倒数第二低的一级
      depth--;
      for(int i=0; i<4; ++i) {
		    // Compute new bounding box for this child 为子节点创建新的边界框
        // ------------
				// 1--+-- 0
				// 3 --+-- 2
				// ----+------
			  Vector2 newOrigin = quadtree_->origin;
			  // newOrigin.x += quadtree_->halfDimension.x * (i&1 ? -.5f : .5f);//i为0、2时为+，1、3时为-
			  // newOrigin.y += quadtree_->halfDimension.y * (i>1 ? -.5f : .5f);//i为0、1时为+，2、3时为-


        // ------------
				// 2--+-- 3
				// 0 --+-- 1
				// ----+------       
			  newOrigin.x += quadtree_->halfDimension.x * (i&1 ? .5f : -.5f);//i为1、3时为+，0、2时为-
			  newOrigin.y += quadtree_->halfDimension.y * (i>1 ? .5f : -.5f);//i为1、3时为+，0、2时为-
			  quadtree_->children[i] = new Quadtree(newOrigin, quadtree_->halfDimension/2,depth);
        /*
        std::cout<<"第"<<i<<"个子节点的 x:"<<quadtree_->children[i]->origin.x<< \
                                                                      "    y:"<<quadtree_->children[i]->origin.y<< \
                                              "    x_dimension:"<<quadtree_->children[i]->halfDimension.x<<\
                                              "    y_dimension:"<<quadtree_->children[i]->halfDimension.y<<\
                                                             "    depth:"<<quadtree_->children[i]->depth;
                                                            //  "    depth:"<<quadtree_->children[i]->depth<<std::endl;*/
        node_total_num++;//增加子节点
        quadtree_init(quadtree_->children[i],depth);
		  }
    }
  }
    // std::cout<<"children[0] 的 x:"<<quadtree_->children[0]->origin.x<<"   y:"<<quadtree_->children[0]->origin.y<<"  x dimension:"<< \
    //     quadtree_->children[0]->halfDimension.x<<"  y dimension:"<<quadtree_->children[0]->halfDimension.y<<std::endl;
    // std::cout<<"children[1] 的 x:"<<quadtree_->children[1]->origin.x<<"   y:"<<quadtree_->children[1]->origin.y<<"  x dimension:"<< \
    //     quadtree_->children[1]->halfDimension.x<<"  y dimension:"<<quadtree_->children[1]->halfDimension.y<<std::endl;
    // std::cout<<"children[2] 的 x:"<<quadtree_->children[2]->origin.x<<"   y:"<<quadtree_->children[2]->origin.y<<"  x dimension:"<< \
    //     quadtree_->children[2]->halfDimension.x<<"  y dimension:"<<quadtree_->children[2]->halfDimension.y<<std::endl;
    // std::cout<<"children[3] 的 x:"<<quadtree_->children[3]->origin.x<<"   y:"<<quadtree_->children[3]->origin.y<<"  x dimension:"<< \
    //     quadtree_->children[3]->halfDimension.x<<"  y dimension:"<<quadtree_->children[3]->halfDimension.y<<std::endl;
  if(currentnodecheck(quadtree_)==true) {
    currentnodemarker(quadtree_);
    // currentnodecolor(quadtree_);
    // std::cout<<"    type:"<<quadtree_->type<<std::endl;     
    /*
    cv::namedWindow("quadtree_img", cv::WINDOW_NORMAL);
    cv::imshow("quadtree_img",quadtree_img);
    cv::waitKey(1);
    */
    /*----------------------------------------------------------------------------------------------
    //绘制矩形
    cv::Scalar color;
    if( sameflag[0]==0)//可通行区域
    {
      color[0]=0;color[1]=255;color[2]=0;//红色
      cv::rectangle(quadtree_img,cvPoint(quadtree_->origin.y-quadtree_->halfDimension.y,quadtree_->origin.x-quadtree_->halfDimension.x),\
        cvPoint(quadtree_->origin.y+quadtree_->halfDimension.y,quadtree_->origin.x+quadtree_->halfDimension.x),color,1,1,0);
    }
    if( sameflag[0]==-1)//未知区域
    {
      color[0]=100;color[1]=100;color[2]=100;//灰色
    }
    if( sameflag[0]==100)//障碍物区域
    {
      color[0]=0;color[1]=0;color[2]=0;//黑色
      cv::rectangle(quadtree_img,cvPoint(quadtree_->origin.y-quadtree_->halfDimension.y,quadtree_->origin.x-quadtree_->halfDimension.x),\
        cvPoint(quadtree_->origin.y+quadtree_->halfDimension.y,quadtree_->origin.x+quadtree_->halfDimension.x),color,1,1,0);
    }
    // cv::rectangle(quadtree_img,cvPoint(quadtree_->origin.y-quadtree_->halfDimension.y,quadtree_->origin.x-quadtree_->halfDimension.x),\
    //     cvPoint(quadtree_->origin.y+quadtree_->halfDimension.y,quadtree_->origin.x+quadtree_->halfDimension.x),color,1,1,0);
    --------------------------------------------------------------------------------------------------*/
	}

  // quadtree_init_lock.unlock();
}

//计算约分比例
void fractionReduction(int a,int b){
  int x,y,big;
	x=a,y=b,big=(a>b?a:b);
	for(int i=big;i>=1;i--){
		if(x%i==0&&y%i==0){
			x/=i; //分子（前项）
			y/=i; //分母（后项）
      ratio_width=x;//约分比例
      ratio_height=y;
		}
	} 
}
//优化约分比例,保证比例中的任何一个值是2或3或4的乘积或乘方
int ratio_optimize(int ratio){
  std::cout<<"优化约分比例"<<std::endl;
  if(ratio==2||ratio==3||ratio==4){
    return ratio;
  }

  int ratio_tmp=0;  
  for(int count=0;;count++){
    ratio_tmp=ratio+count;

    //判断比例中是否是2或3或4的乘积或乘方即能够被2、3、4除尽
    while(ratio_tmp%2==0){
      ratio_tmp=ratio_tmp/2;
    }
    while(ratio_tmp%3==0){
      ratio_tmp=ratio_tmp/3;
    }
    if(ratio_tmp==1){
      ratio=ratio+count;
      break;
    }
  }
  return ratio;
}
//四元树地图宽度、高度初始化
void quadmap_size_init(int width,int height){
  if(width<height){
    for(int n=1;;n++){
      if(width<=pow(2,n)){
        quad_width=pow(2,n);
        break;
      }
    }
    quad_height=quad_width/ratio_width*ratio_height;
  }
  else  if(height<width){
    for(int n=1;;n++){
      if(height<=pow(2,n)){
        quad_height=pow(2,n);
        break;
      }
    }
    quad_width=quad_height/ratio_height*ratio_width;
  }
}

//四元树地图宽度、高度初始化 0621   d_lw
void quadmap_lw_init(int width,int height){
  //计算2^n
  for(int n=1;;n++){
    if(width<=pow(2,n)){
      quad_width=pow(2,n);
      break;
    }
  }
  for(int n=1;;n++){
    if(height<=pow(2,n)){
      quad_height=pow(2,n);
      break;
    }
  }
  //计算符合条件的最小公约数
  int d_lw_tmp=0;
  int width_divide=width/8;
  int height_divide=height/8;
  d_lw_tmp=width_divide>height_divide?width_divide:height_divide;
  for(;;d_lw_tmp++){
    if((quad_width%d_lw_tmp==0)&&(quad_height%d_lw_tmp==0)){
      d_lw=d_lw_tmp;
      break;
    }
  }
  //看地图在宽度和高度方向上可以分为多少个最小公约数----以及对应的宽度和高度
  for(int i=1;;i++){
    if(d_lw*i>=width){
      ratio_width=i;
      quad_width=d_lw*i;
      break;
    }
  }
  for(int i=1;;i++){
    if(d_lw*i>=height){
      ratio_height=i;
      quad_height=d_lw*i;
      break;
    }
  }  

}
// 计算地图深度
int getdepth(int side_length){
  int num=1;
  for(int i=1;;i++){
    num=num*2;
    if(num==side_length){
      return i;
    }
  }
  return 0;
}

void globalmapHandler(const nav_msgs::OccupancyGridConstPtr & globalmapMsg)
{
  // 雷达坐标系： x轴--前向--height，y轴-左向--width
  // 栅格地图坐标系：x轴-右向-width，y轴-上向/前向-height，原点为左下角
  //-----------将回调进来的全局地图进行预处理,  得到四元树地图的高和宽---------
  // globalmap = *globalmapMsg;//2022/12/05注释掉
  globalmap_origin=*globalmapMsg;

  if(quad_constructor_once==1){//只在第一次回调时进行四元树地图构建
    // cout<<"quad_constructor_once"<<quad_constructor_once<<endl;
    return ;
  }
  // cout<<"quad_constructor_once"<<quad_constructor_once<<endl;

  std::cout<<"全局地图的:  宽度:"<<globalmap_origin.info.width<<"  高度:"<< globalmap_origin.info.height<<  "  分辨率:"<<globalmap_origin.info.resolution<<std::endl; 
  fractionReduction(globalmap_origin.info.width,globalmap_origin.info.height);//这个比值需要传递给最底层的几层节点
  std::cout<<"近似约分比例为:"<<ratio_width<<":"<<ratio_height<<endl;
  quadmap_lw_init(globalmap_origin.info.width,globalmap_origin.info.height);
  max_depth=getdepth(quad_width/ratio_width)+1;//四叉树地图的总深度
  resolution=globalmap_origin.info.resolution;
  depth=max_depth;
  std::cout<<"初始化四元树地图的宽度、高度、深度、最小公约数分别为:"<<quad_width<<"、"<<quad_height<<"、"<<max_depth<<"、"<<d_lw<<std::endl;
  std::cout<<"进一步的近似约分比例为:"<<ratio_width<<":"<<ratio_height<<endl;
  // return;
  //--------------根据得到四元树的高和宽更新全局地图globalmap数据-----------------------------
    //初始化--
  globalmap.header.frame_id=globalmap_origin.header.frame_id;
  globalmap.header.stamp = globalmapMsg->header.stamp; 
  globalmap.info.resolution = resolution;         // float32
  globalmap.info.origin.position.x = 0;
  globalmap.info.origin.position.y = 0;
  globalmap.info.origin.position.z = 0.0;
  globalmap.info.origin.orientation.x = 0.0;
  globalmap.info.origin.orientation.y = 0.0;
  globalmap.info.origin.orientation.z = 0.0;
  globalmap.info.origin.orientation.w = 1.0;  
  globalmap.info.width      = quad_width;           // uint32
  globalmap.info.height     = quad_height;           // uint32
  globalmap.data.resize(globalmap.info.width * globalmap.info.height);
  for (int i=0; i<globalmap.info.width * globalmap.info.height; i++)//重新初始化
  {
    globalmap.data[i] = -1;  // 均为未知状态
  }

  rostime = ros::Time::now();
  for (int i=0; i<globalmap_origin.info.width ; i++) {
    for(int j=0; j<globalmap_origin.info.height; j++){
      //原始地图和四叉树地图地图中心对齐：
      // globalmap.data[i+(quad_width-globalmap_origin.info.width)/2 + (j+(quad_height-globalmap_origin.info.height)/2)*globalmap.info.width] = globalmap_origin.data[i + j*globalmap_origin.info.width];  // 移动数据的位置
      //原始地图在四叉树地图地图的左下角：
      globalmap.data[i+ j*globalmap.info.width] = globalmap_origin.data[i + j*globalmap_origin.info.width];  // 移动数据的位置
    }
  }
  // pubGlobalmap_tmp.publish(globalmap);//2022/12/02挪到后面

  /*
  //----------------------初始化用于显示四元树地图的图像
  //图像坐标系:右-x轴，下-y轴-------上下颠倒后：右-x轴，上-y轴
  quadtree_img = cv::Mat::zeros(cv::Size(globalmap.info.width, globalmap.info.height), CV_8UC3);
  // cv::flip(quadtree_img, quadtree_img, 0); //   //小于0（例如-1）代表左右上下颠倒；0代表上下颠倒；大于0（例如1）代表左右颠倒。
  //逆时针旋转90°
  // cv::Point2f quadtree_img_center(quadtree_img.cols/2.0F, quadtree_img.rows/2.0F);
  // cv::Mat rot_mat=cv::getRotationMatrix2D(quadtree_img_center, 90, 1.0);
  // cv::warpAffine(quadtree_img, quadtree_img, rot_mat, quadtree_img.size());

  //把图像初始化为灰色
  for (int row = 0; row <globalmap.info.height; row++)
	{	
		for (int col = 0; col < globalmap.info.width; col++)
		{
      // quadtree_img.at<cv::Vec3b>(row,col)[0] = 128;
      // quadtree_img.at<cv::Vec3b>(row,col)[1] = 128;
      // quadtree_img.at<cv::Vec3b>(row,col)[2] = 128;
      quadtree_img.at<cv::Vec3b>(row,col)[0] = 94;//rviz中的灰色//at<cv::Vec3b>顺序BGR
      quadtree_img.at<cv::Vec3b>(row,col)[1] = 96;
      quadtree_img.at<cv::Vec3b>(row,col)[2] = 78;
    }
	}
  */

  //---------------------------根节点与一级子地图节点初始化-----------------------
  //栅格地图小栅格的顶点为 四叉树的节点；四叉树节点的索引比对应栅格地图小栅格的索引在x、y方向上均多1
  // quadtree = new Quadtree(Vector2(quad_width/2,quad_height/2), Vector2(quad_width/2,quad_height/2));
  quadtree = new Quadtree(Vector2(quad_width/2,quad_height/2), Vector2(quad_width/2,quad_height/2),depth,true);
  // outfile.open(path);//准备写入map0_3的占据观测数据
  outfile.open(path, ios::out | ios::binary);
  depth--;
  node_total_num++;
  std::cout<<"初始根节点的 x:"<<quadtree->origin.x<<"    y:"<<quadtree->origin.y<< \
      "    x_dimension:"<<quadtree->halfDimension.x<< "    y_dimension:"<<quadtree->halfDimension.y<< "    depth:"<<quadtree->depth<<std::endl;
  //生成子图节点（这里先写生成一层子图节点的代码，二层稍后再说）
  			// ------------
				// 4 --+-- 5...
				// 0 --+-- 1--+---2--+--3...
				// ----+------
  for(int i=0;i<ratio_width;i++){
    for(int j=0;j<ratio_height;j++){
		    // Compute new bounding box for this child 为子节点创建新的边界框
        Vector2 newhalfDimension_1 = Vector2(quad_width/ratio_width/2,quad_height/ratio_height/2);
			  Vector2 newOrigin_1;
			  newOrigin_1.x = (quad_width/ratio_width)*(i+0.5);//
			  newOrigin_1.y = (quad_height/ratio_height)*(j+0.5);//
        quadtree->children_root[i+j*ratio_width] = new Quadtree(newOrigin_1, newhalfDimension_1,depth);
        node_total_num++;
        /*
        std::cout<<"第"<<i+j*ratio_width<<"个子图节点的 x:"<<quadtree->children_root[i+j*ratio_width]->origin.x<< \
                                                                                                         "    y:"<<quadtree->children_root[i+j*ratio_width]->origin.y<< \
                                                                                 "    x_dimension:"<<quadtree->children_root[i+j*ratio_width]->halfDimension.x<<\
                                                                                 "    y_dimension:"<<quadtree->children_root[i+j*ratio_width]->halfDimension.y<<\
                                                                                 "    depth:"<<quadtree->children_root[i+j*ratio_width]->depth<<std::endl;
                                                                                 */
        quadtree_init(quadtree->children_root[i+j*ratio_width],depth);
        /*
        std::cout<<"------------------------------------------------------------------------------"<<std::endl;
        std::cout<<"------------------------------------------------------------------------------"<<std::endl;
        */
    }
  }
  outfile.close();
  // marker_pub.publish(markerArray);//2022/12/02挪到后面      
  // std::cout<<"地图创建完毕"<<std::endl;

  if(globalmap_origin.data.size()!=0){//只在第一次回调时进行四元树地图构建
    quad_constructor_once=1;
  }
  //---------------------------------------------------------------------------

  // double start = stopwatch();
  // quadtree_init(quadtree);
  // double T = stopwatch() - start;
	// printf("初始化四叉树耗时 %.5f sec.\n", T);

  /*
  double image_time =ros::Time::now().toSec(); 
  std::string image_time_=to_string(image_time);
  cv::imwrite(image_time_+".png",quadtree_img);
  //小于0（例如-1）代表左右上下颠倒；0代表上下颠倒；大于0（例如1）代表左右颠倒。
  cv::flip(quadtree_img, quadtree_img, 0);//取消翻转，看看原始地图的数据结构
  cv::namedWindow("quadtree_img_end", cv::WINDOW_NORMAL);
  cv::imshow("quadtree_img_end",quadtree_img);
  cv::waitKey();
*/
  cout<<"创建的最小环境块的总数为:"<<node_total_num<<endl;
  
  return ;//调试用
}

void color_set(){
  // color_occ.r = 1;                    color_occ.g = 0;                   color_occ.b = 0;                   color_occ.a=1;//红色 占据
  // color_occ.r = 0;                    color_occ.g = 0;                   color_occ.b = 0;                   color_occ.a=1;//黑色 占据
  // color_occ.r = 1;                    color_occ.g = 1;                   color_occ.b = 0;                   color_occ.a=1;//黄色 占据
  color_occ.r = 1;                    color_occ.g = 0;                   color_occ.b = 1;                   color_occ.a=1;//紫色 占据

  // color_passive.r = 0;            color_passive.g = 1;           color_passive.b = 0;           color_passive.a=1;//绿色 可通行
  // color_passive.r = 0;            color_passive.g = 1;           color_passive.b = 0;           color_passive.a=1;//绿色 可通行
  color_passive.r = 1;            color_passive.g = 1;           color_passive.b = 1;           color_passive.a=1;//白色 可通行
  // color_passive.r = 0.85;    color_passive.g = 0.85;   color_passive.b = 0.85;   color_passive.a=1;//灰白色 可通行

  color_unknown.r = 0.5;    color_unknown.g = 0.5;   color_unknown.b = 0.5;   color_unknown.a=1;//灰色 未知
  // color_unknown.r = 0.753;    color_unknown.g = 0.753;   color_unknown.b = 0.753;   color_unknown.a=1;//灰白色 未知

  color_deadend_inner.r = 1;    color_deadend_inner.g = 0;   color_deadend_inner.b = 0;   color_deadend_inner.a=1;//死角区域内侧  红色
  // color_deadend_middle.r = 1;    color_deadend_middle.g = 0.388;   color_deadend_middle.b = 0.2784;   color_deadend_middle.a=1;//死角区域中间  红色
  color_deadend_middle.r = 1;    color_deadend_middle.g = 0.2;   color_deadend_middle.b = 0.2;   color_deadend_middle.a=1;//死角区域中间  红色  
  color_deadend_outer.r = 1;    color_deadend_outer.g = 1;   color_deadend_outer.b = 0;   color_deadend_outer.a=1;//死角区域外侧 黄色
}

int main(int argc, char * argv[]) {

  // ros::init(argc, argv, "quadtree_Map");
  ros::init(argc, argv, "quadtreemap_addmarker_hdmap");//2022/12/01

  ros::NodeHandle nh("~");

  color_set();//颜色设置
  nh.param<string>("path", path, ""); 
  nh.param<bool>("use_resolution_005", use_resolution_005, "false"); 
  nh.param<bool>("use_resolution_01", use_resolution_01, "false"); 
  nh.param<bool>("use_resolution_02", use_resolution_02, "false"); 
  nh.param<bool>("use_resolution_05", use_resolution_05, "false"); 

  //这里需要注意一下，接收的栅格地图最好宽度和高度方向的栅格数都是2^n，对应节点数为奇数个----便于建立四叉树
  ros::Subscriber sub_globalmap = nh.subscribe<nav_msgs::OccupancyGrid>("/global_path/grid_map", 1, globalmapHandler);
  pubGlobalmap_tmp= nh.advertise<nav_msgs::OccupancyGrid>("/quadtree_map", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("quadtree_map_marker", 1);//3d栅格可视化方式

  // ros::spinOnce();
  // ros::spin();
  //2022/12/02新增
  ros::Rate loop_rate(1);
  while(ros::ok())
  {
      ros::spinOnce();
      pubGlobalmap_tmp.publish(globalmap);
      marker_pub.publish(markerArray);

      loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
