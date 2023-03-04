#include <cmath>
#include<mutex>
#include <fstream>//hxz
#include <iostream>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std; //hxz
string globalmappath;//离线四叉树地图的占据节点信息
string localmappath;//实时四叉树地图的占据节点信息
string globalmap_header_path;//离线四叉树地图的基本信息文件地址
string localmap_header_path;//实时四叉树地图的基本信息
int globalmap_width;//离线四叉树地图的宽度
int globalmap_height;//离线四叉树地图的高度
float globalmap_latitude;
float globalmap_longitude;
int globalmap_x_offset;
int globalmap_y_offset;

int localmap_width;//离线四叉树地图的宽度
int localmap_height;//离线四叉树地图的高度
float localmap_latitude;
float localmap_longitude;
int localmap_x_offset;
int localmap_y_offset;
ifstream globalmapinfile; 
ifstream localmapinfile; 
ifstream globalmapheader_infile; //地图基本信息
ifstream localmapheader_infile; 
#include "lonlat2utm.h"//经纬度转utm坐标系
float localmap2gloabalmap_offset_x;//实时地图相对于离线地图的绝对位置偏差，这里先只考虑xy方向的平移
float localmap2gloabalmap_offset_y;
Eigen::Quaterniond localmap_quaternion;//局部地图初始姿态
Eigen::Quaterniond globalmap_quaternion;
Eigen::Vector3d t_localmap2globalmap;

class node {//地图节点的数据格式
  public:
    int origin_x;
    int origin_y;
    int halfDimension_x;
    int halfDimension_y;
    int depth;
    int type;
};

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
        int x_offset;//地图初始化时原点相对坐标系位置
        int y_offset;
};

float resolution=0.05;//地图分辨率

std::mutex looplock;
std::mutex loop_refine_lock;
std::mutex check_one_point_lock;
nav_msgs::OccupancyGrid gridmap0, gridmap;
int receive_map0=0;//是否接收到地图gridmap0
int receive_map=0;//是否接收到地图gridmap
int map_match_my_flag=0;//粗匹配程序是否执行过
int map_match_refine_my_flag=0;
float mapmatch_rough_x=0;//粗匹配平移距离
float mapmatch_rough_y=0;
float point_threshold_rough_x=0;//粗匹配是判断点重合的阈值
float point_threshold_rough_y=0;
int iteration_interval=0;//粗匹配迭代步长
bool rough_match=1;//粗查找是否查询整个离线地图

float mapmatch_refine_x=0;//精确匹配平移距离
float mapmatch_refine_y=0;
float point_threshold_refine_x=0;//精匹配是判断点重合的阈值
float point_threshold_refine_y=0;
int range_rough_x_min;//粗匹配查找范围
int range_rough_y_min;
int range_rough_x_max;//粗匹配查找范围
int range_rough_y_max;
int range_refine_x_min;//精匹配查找范围
int range_refine_y_min;
int range_refine_x_max;//精匹配查找范围
int range_refine_y_max;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);// 用于匹配的输入点云，可能只是输入点云的部分
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_all(new pcl::PointCloud<pcl::PointXYZ>);// 输入点云，用于最终匹配结果的显示
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_match(new pcl::PointCloud<pcl::PointXYZ>);// 用于粗匹配的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_refine_match(new pcl::PointCloud<pcl::PointXYZ>);// 精确匹配后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_icp(new pcl::PointCloud<pcl::PointXYZ>);// 用于icp精确匹配的点云
pcl::PointCloud<pcl::PointXYZ> cloud_after_icp;// icp精确匹配后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);// 目标点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_verify(new pcl::PointCloud<pcl::PointXYZ>);// 用于验证
sensor_msgs::PointCloud2 targetCloudMsg,sourceCloudMsg,sourceAllCloudMsg, verifyCloudMsg,refineverifyCloudMsg,foricpCloudMsg,aftericpCloudMsg;//
ros::Publisher  pubTargetCloud,pubSourceCloud,pubSourceAllCloud,pubVerifyCloud,pubRefineVerifyeCloud,pubForIcpCloud,pubAfterIcpCloud;

// 将离线四叉树地图的基本信息提取出来
void globalmap_header_info(){
  globalmapheader_infile.open(globalmap_header_path,ios::in|ios::binary); //二进制读方式打开
  if(!globalmapheader_infile) {
    cout << "error" <<endl;
    return ;
  }

  mapheader mapheader_tmp;
  while(globalmapheader_infile.read((char *)&mapheader_tmp, sizeof(mapheader_tmp))) { //一直读到文件结束
    globalmap_width=mapheader_tmp.map_width;
    globalmap_height=mapheader_tmp.map_height;
    globalmap_latitude=mapheader_tmp.latitude;
    globalmap_longitude=mapheader_tmp.longitude;
    globalmap_x_offset=mapheader_tmp.x_offset;
    globalmap_y_offset=mapheader_tmp.y_offset; 
    globalmap_quaternion.x()=mapheader_tmp.qx;
    globalmap_quaternion.y()=mapheader_tmp.qy;
    globalmap_quaternion.z()=mapheader_tmp.qz;
    globalmap_quaternion.w()=mapheader_tmp.qw;
  }
  // std::cout<<"离线地图的经度: "<<setprecision(20)<<globalmap_longitude<<"    "<<"离线地图的纬度: "<<setprecision(20)<<globalmap_latitude<<std::endl;
  std::cout<<"离线地图的经度: "<<globalmap_longitude<<"    "<<"离线地图的纬度: "<<globalmap_latitude<<std::endl;
  globalmapheader_infile.close();
}

// 将实时四叉树地图的基本信息提取出来
void localmap_header_info(){
  localmapheader_infile.open(localmap_header_path,ios::in|ios::binary); //二进制读方式打开
  if(!localmapheader_infile) {
    cout << "error" <<endl;
    return ;
  }
  mapheader mapheader_tmp;
  while(localmapheader_infile.read((char *)&mapheader_tmp, sizeof(mapheader_tmp))) { //一直读到文件结束
    localmap_width=mapheader_tmp.map_width;
    localmap_height=mapheader_tmp.map_height;
    localmap_latitude=mapheader_tmp.latitude;
    localmap_longitude=mapheader_tmp.longitude;
    localmap_x_offset=mapheader_tmp.x_offset;
    localmap_y_offset=mapheader_tmp.y_offset; 
    localmap_quaternion.x()=mapheader_tmp.qx;
    localmap_quaternion.y()=mapheader_tmp.qy;
    localmap_quaternion.z()=mapheader_tmp.qz;
    localmap_quaternion.w()=mapheader_tmp.qw;
  }
  // std::cout<<"实时地图的经度: "<<setprecision(20)<<localmap_longitude<<"    "<<"实时地图的纬度: "<<setprecision(20)<<localmap_latitude<<std::endl;
  std::cout<<"实时地图的经度: "<<localmap_longitude<<"    "<<"实时地图的纬度: "<<localmap_latitude<<std::endl;
  localmapheader_infile.close();
}

// 计算实时地图相对于离线地图的绝对位置偏差
void localmap2globalmap_offset(){
  double UTME_globalmap;
  double UTMN_globalmap;  
  double UTME_localmap;
  double UTMN_localmap;  

  LonLat2UTM(globalmap_longitude,  globalmap_latitude,  UTME_globalmap,  UTMN_globalmap);
  LonLat2UTM(localmap_longitude,  localmap_latitude,  UTME_localmap,  UTMN_localmap);
  localmap2gloabalmap_offset_x=UTME_localmap-UTME_globalmap+(globalmap_x_offset-localmap_x_offset);
  localmap2gloabalmap_offset_y=UTMN_localmap-UTMN_globalmap+(globalmap_y_offset-localmap_y_offset);
  // localmap2gloabalmap_offset_x=UTME_localmap-UTME_globalmap+17;//调试使用
  // localmap2gloabalmap_offset_y=UTMN_localmap-UTMN_globalmap+14;
  t_localmap2globalmap.x()=localmap2gloabalmap_offset_x;
  t_localmap2globalmap.y()=localmap2gloabalmap_offset_y;
  t_localmap2globalmap.z()=0;
  std::cout<<"实时地图相对于离线地图的绝对位置偏差: "<<localmap2gloabalmap_offset_x<<"    y: "<<localmap2gloabalmap_offset_y<<std::endl;
}

// 将离线四叉树地图的占据节点信息转化为点云信息（栅格的属性对应点云的 z 坐标）
void globalmap2cloud(){
  globalmapinfile.open(globalmappath,ios::in|ios::binary); //二进制读方式打开
  // globalmapinfile.open("/home/meng/ideas/论文稿及材料0608/自己采集的数据/0705/数据包-1/地图匹配/四叉树地图匹配/map0.dat",ios::in|ios::binary); //二进制读方式打开
  if(!globalmapinfile) {
    cout << "error" <<endl;
    return ;
  }
  node node_tmp;
  pcl::PointXYZ point_tmp;
  while(globalmapinfile.read((char *)&node_tmp, sizeof(node_tmp))) { //一直读到文件结束
    // cout << node_tmp.origin_x << "--" << node_tmp.origin_y << "--" \
    //           << node_tmp.halfDimension_x << "--" << node_tmp.halfDimension_y << "--" \
    //           << node_tmp.depth << "--" << node_tmp.type << endl;     
    if(node_tmp.depth==0){
      point_tmp.x=node_tmp.origin_x*resolution+resolution*0.5;          
      point_tmp.y=node_tmp.origin_y*resolution+resolution*0.5;       
      point_tmp.z=0;
      cloud_out->points.push_back(point_tmp);     
    }
    else {
      for(int i=node_tmp.origin_x-node_tmp.halfDimension_x;i<node_tmp.origin_x+node_tmp.halfDimension_x;i++){      
        for(int j=node_tmp.origin_y-node_tmp.halfDimension_y;j<node_tmp.origin_y+node_tmp.halfDimension_y;j++){      
          point_tmp.x=i*resolution+resolution*0.5;          
          point_tmp.y=j*resolution+resolution*0.5;       
          point_tmp.z=0;
          cloud_out->points.push_back(point_tmp);            
        }
      }
    }
  }
  globalmapinfile.close();

  pcl::toROSMsg(*cloud_out, targetCloudMsg);
  targetCloudMsg.header.stamp = ros::Time::now();
  targetCloudMsg.header.frame_id = "map";
  std::cout<<"离线四叉树地图转换为点云成功"<<std::endl;
  // pubTargetCloud.publish(targetCloudMsg);

}

// 将实时四叉树地图的占据节点信息转化为点云信息（栅格的属性对应点云的 z 坐标）
void localmap2cloud(){
  localmapinfile.open(localmappath,ios::in|ios::binary); //二进制读方式打开
  // localmapinfile.open("/home/meng/ideas/论文稿及材料0608/自己采集的数据/0705/数据包-1/地图匹配/四叉树地图匹配/map1.dat",ios::in|ios::binary); //二进制读方式打开
  if(!localmapinfile) {
    cout << "error" <<endl;
    return ;
  }
  node node_tmp;
  pcl::PointXYZ point_tmp;
  int count_node=0;//计数地图节点个数，减少实时地图数据规模的大小，进而减少后续查找时间
  while(localmapinfile.read((char *)&node_tmp, sizeof(node_tmp)) /*&& (count_node<=500)*/) { //一直读到文件结束
    // cout << node_tmp.origin_x << "--" << node_tmp.origin_y << "--" \
    //           << node_tmp.halfDimension_x << "--" << node_tmp.halfDimension_y << "--" \
    //           << node_tmp.depth << "--" << node_tmp.type << endl;     
    count_node++;
    if(node_tmp.depth==0){
      // point_tmp.x=node_tmp.origin_x*resolution+resolution*0.5;          
      // point_tmp.y=node_tmp.origin_y*resolution+resolution*0.5;       
      point_tmp.x=node_tmp.origin_x*resolution+resolution*0.5+localmap2gloabalmap_offset_x;          
      point_tmp.y=node_tmp.origin_y*resolution+resolution*0.5+localmap2gloabalmap_offset_y;       
      point_tmp.z=0;
      if(count_node%8==0){//减少实时地图数据规模的大小
        cloud_in->points.push_back(point_tmp);     
      }
      cloud_in_all->points.push_back(point_tmp);  
    }
    else {
      for(int i=node_tmp.origin_x-node_tmp.halfDimension_x;i<node_tmp.origin_x+node_tmp.halfDimension_x;i++){      
        for(int j=node_tmp.origin_y-node_tmp.halfDimension_y;j<node_tmp.origin_y+node_tmp.halfDimension_y;j++){      
          // point_tmp.x=i*resolution+resolution*0.5;  //        
          // point_tmp.y=j*resolution+resolution*0.5;       
          point_tmp.x=i*resolution+resolution*0.5+localmap2gloabalmap_offset_x;  //将偏差计算进来        
          point_tmp.y=j*resolution+resolution*0.5+localmap2gloabalmap_offset_y;       
          point_tmp.z=0;
          if(count_node%8==0){//减少实时地图数据规模的大小
            cloud_in->points.push_back(point_tmp);     
          }
          cloud_in_all->points.push_back(point_tmp);      
        }
      }
    }
  }
  localmapinfile.close();

  pcl::toROSMsg(*cloud_in, sourceCloudMsg);
  sourceCloudMsg.header.stamp = ros::Time::now();
  sourceCloudMsg.header.frame_id = "map";
  // pubSourceCloud.publish(sourceCloudMsg);

  pcl::toROSMsg(*cloud_in_all, sourceAllCloudMsg);
  sourceAllCloudMsg.header.stamp = ros::Time::now();
  sourceAllCloudMsg.header.frame_id = "map";
  std::cout<<"cloud_in->points.size(): "<<cloud_in->points.size()<<std::endl;
  std::cout<<"cloud_in_all->points.size(): "<<cloud_in_all->points.size()<<std::endl;  
  std::cout<<"实时四叉树地图转换为点云成功"<<std::endl;
}


// 根据精匹配和粗匹配的结果, 将实时四叉树地图的占据节点信息转化的点云信息, 进行坐标转换
void cloud_for_icp_transform(){
  std::cout<<"mapmatch_rough_x:  "<<mapmatch_rough_x<<"     mapmatch_rough_y:  "<<mapmatch_rough_y<<std::endl;
  std::cout<<"mapmatch_refine_x:  "<<mapmatch_refine_x<<"     mapmatch_refine_y:  "<<mapmatch_refine_y<<std::endl;
  pcl::PointXYZ point_tmp;
  /*
  for(int i=0;i<cloud_in->points.size();i++){
    point_tmp.x=cloud_in->points[i].x+mapmatch_refine_x;
    point_tmp.y=cloud_in->points[i].y+mapmatch_refine_y;
    point_tmp.z=0;
    cloud_for_icp->points.push_back(point_tmp);     
  }
  */
    
  for(int i=0;i<cloud_in_all->points.size();i++){
    point_tmp.x=cloud_in_all->points[i].x+mapmatch_refine_x;
    point_tmp.y=cloud_in_all->points[i].y+mapmatch_refine_y;
    point_tmp.z=0;
    cloud_for_icp->points.push_back(point_tmp);     
  }

  pcl::toROSMsg(*cloud_for_icp, foricpCloudMsg);
  foricpCloudMsg.header.stamp = ros::Time::now();
  foricpCloudMsg.header.frame_id = "map";
  // std::cout<<"用于icp匹配的点云创建成功"<<std::endl;

}

// 查询一个点是否在目标点云中存在，存在返回true----用于粗匹配
bool check_one_point(pcl::PointXYZ one_point){
  check_one_point_lock.lock();
  for(int i=0;i<cloud_out->points.size();i++){
    // if(fabs(one_point.x-cloud_out->points[i].x)<1e-1 && fabs(one_point.y-cloud_out->points[i].y)<1e-1){
    // if(fabs(one_point.x-cloud_out->points[i].x)<0.025 && fabs(one_point.y-cloud_out->points[i].y)<0.025){//调试使用，找到精确平移量
    // if(fabs(one_point.x-cloud_out->points[i].x)<0.25 && fabs(one_point.y-cloud_out->points[i].y)<0.25){//加快查找速度
    // if(fabs(one_point.x-cloud_out->points[i].x)<0.5 && fabs(one_point.y-cloud_out->points[i].y)<0.5){//加快查找速度------0720-1
    if(fabs(one_point.x-cloud_out->points[i].x)<point_threshold_rough_x && \
        fabs(one_point.y-cloud_out->points[i].y)<point_threshold_rough_y){//加快查找速度------0720-1
    // if(fabs(one_point.x-cloud_out->points[i].x)<1 && fabs(one_point.y-cloud_out->points[i].y)<1){//加快查找速度------
    // if(fabs(one_point.x-cloud_out->points[i].x)<2.5 && fabs(one_point.y-cloud_out->points[i].y)<2.5){//加快查找速度
    // if(fabs(one_point.x-cloud_out->points[i].x)<5 && fabs(one_point.y-cloud_out->points[i].y)<5){//加快查找速度
    // if(fabs(one_point.x-cloud_out->points[i].x)<15 && fabs(one_point.y-cloud_out->points[i].y)<15){//加快查找速度
      check_one_point_lock.unlock();
      return true;
    }
  }
  check_one_point_lock.unlock();
  return false;
}

// 查询一个点是否在目标点云中存在，存在返回true-----用于精确匹配
bool check_one_point_refine(pcl::PointXYZ one_point){
  check_one_point_lock.lock();
  for(int i=0;i<cloud_out->points.size();i++){
    // if(fabs(one_point.x-cloud_out->points[i].x)<0.025 && fabs(one_point.y-cloud_out->points[i].y)<0.025){
    // if(fabs(one_point.x-cloud_out->points[i].x)<0.05 && fabs(one_point.y-cloud_out->points[i].y)<0.05){
    if(fabs(one_point.x-cloud_out->points[i].x)<point_threshold_refine_x && fabs(one_point.y-cloud_out->points[i].y)<point_threshold_refine_y){
      check_one_point_lock.unlock();
      return true;
    }
  }
  check_one_point_lock.unlock();
  return false;
}

// 使用逐步平移输入点云/(即实时地图的原点)的方法进行配准----进行粗匹配----从离线地图左下角一直遍历到右上角，比较耗时
void map_match_my(){
  map_match_my_flag=1;
  static int xynum_iter[10000][3]={0};
  int count_xynum_iter=0;
  int cloud_in_size=cloud_in->points.size();

  // 在离线地图上查找有多少点和实时地图重合--------------------
  int x_iter=0;//将输入点云平移
  int y_iter=0;
  for(int num=0;;num++){
    looplock.lock();
    double begin_time = ros::Time::now().toSec();
    std::cout<<"开始进行第"<<num<<"次粗查找---------------------------------------"<<std::endl;
    float x_iter_range=x_iter*resolution;//将平移栅格值转化为平移距离值
    float y_iter_range=y_iter*resolution;    
    std::cout<<"平移栅格数目:  x: "<<x_iter<<";"<<"   y: "<<y_iter<<std::endl;
    std::cout<<"平移量:  x: "<<x_iter_range<<";"<<"   y: "<<y_iter_range<<std::endl;
    pcl::PointXYZ point_tmp;
    for(int k=0;k<cloud_in_size;k++){
      point_tmp.x=cloud_in->points[k].x+x_iter_range;
      point_tmp.y=cloud_in->points[k].y+y_iter_range;     
      point_tmp.z=0;
      cloud_match->points.push_back(point_tmp);       
    }
    int count_include=0;//计数转换后的点云有多少属于目标点云
    int count_uninclude=0;//计数转换后的点云有多少不属于目标点云    
    for(int k=0;k<cloud_in_size;k++){
      if(check_one_point(cloud_match->points[k])){
        count_include++;
      }
      else {
        count_uninclude++;
      }
      if(count_uninclude>=cloud_in_size*0.1){//为了减少计算资源消耗
        break;
      }
    }

    std::cout<<"输入点云总数: "<<cloud_in_size<<"输入点云总数的90%: "<<cloud_in_size*0.9<<";"<<"属于目标点云总数: "<<count_include<<std::endl;
    if(count_include>cloud_in_size*0.8){
      xynum_iter[count_xynum_iter][0]=x_iter;
      xynum_iter[count_xynum_iter][1]=y_iter;
      xynum_iter[count_xynum_iter][2]=count_include;      
      count_xynum_iter++;
    }
    // x_iter++;
    // x_iter=x_iter+5;//加快查找速度
    // x_iter=x_iter+10;//加快查找速度
    // x_iter=x_iter+20;//加快查找速度------------------------0720-1
    x_iter=x_iter+iteration_interval;//加快查找速度------------------------0720-1
  //   x_iter=x_iter+25;//加快查找速度
    // x_iter=x_iter+40;//加快查找速度-------0717-2
  //   // x_iter=x_iter+100;//加快查找速度
  //   // x_iter=x_iter+200;//加快查找速度
    if(x_iter>=globalmap_width-localmap_width){//说明此时已经在离线地图gridmap0上遍历一行了，于是开始遍历下一行
      x_iter=0;
  // //     // y_iter++;
  // //     // y_iter=y_iter+5;//加快查找速度
  //     y_iter=y_iter+10;//加快查找速度
      // y_iter=y_iter+20;//加快查找速度---------------------------0720-1
      y_iter=y_iter+iteration_interval;//加快查找速度---------------------------0720-1
  // //     y_iter=y_iter+25;//加快查找速度
      // y_iter=y_iter+40;//加快查找速度----------------0717-2
  // //     // y_iter=y_iter+100;//加快查找速度
  // //     // y_iter=y_iter+200;//加快查找速度
    }
    // if(y_iter>=gridmap0.info.height-gridmap.info.height+200){//说明此时已经在离线地图gridmap0上遍历完了
    if(y_iter>=globalmap_height-localmap_height){//说明此时已经在离线地图gridmap0上遍历完了
      break;
    }
    double end_time = ros::Time::now().toSec();
    std::cout << "一次查找耗时:" << (end_time - begin_time) << std::endl;
    if(count_include>=cloud_in_size*0.995){//粗查找提前结束标志---------
      rough_match=0;//粗查找没有查询整个离线地图
      mapmatch_rough_x=x_iter_range;//粗匹配平移距离
      mapmatch_rough_y=y_iter_range;
      std::cout<<"粗匹配查找的到平移量:  x: "<<mapmatch_rough_x<<";"<<"   y: "<<mapmatch_rough_y<<std::endl;
      looplock.unlock();
      pcl::toROSMsg(*cloud_match, verifyCloudMsg);
      verifyCloudMsg.header.stamp = ros::Time::now();
      verifyCloudMsg.header.frame_id = "map";
      return;//提前结束粗查找
    }
    cloud_match->points.clear();
    looplock.unlock();
  }
  // 查找多次平移时，有最多重合点的那次平移---------------------------
  int x_iter_max=0;
  int y_iter_max=0;
  int count_include_max=0;
  for(int i=0;i<=count_xynum_iter;i++){
    if(count_include_max<xynum_iter[i][2]){
      count_include_max=xynum_iter[i][2];
      x_iter_max=xynum_iter[i][0];
      y_iter_max=xynum_iter[i][1];
    }
  }
  // std::cout<<"888"<<std::endl;
  float x_iter_max_range=x_iter_max*resolution;//将平移栅格值转化为平移距离值
  float y_iter_max_range=y_iter_max*resolution;    
  
  for(int k=0;k<cloud_in_size;k++){
      pcl::PointXYZ point_tmp;
      point_tmp.x=cloud_in->points[k].x+x_iter_max_range;
      point_tmp.y=cloud_in->points[k].y+y_iter_max_range;     
      point_tmp.z=0;
      cloud_match->points.push_back(point_tmp);       
  }
  mapmatch_rough_x=x_iter_max_range;//粗匹配平移距离
  mapmatch_rough_y=y_iter_max_range;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"粗查找最终结果: 平移量: x: "<<mapmatch_rough_x<<"   y: "<<mapmatch_rough_y<<std::endl;

  pcl::toROSMsg(*cloud_match, verifyCloudMsg);
  verifyCloudMsg.header.stamp = ros::Time::now();
  verifyCloudMsg.header.frame_id = "map";
}


// 使用逐步平移输入点云/(即实时地图的原点)的方法进行配准----进行粗匹配----进过GPS进行坐标转换，目标点云在转换结果附近
void map_match_rough_my(){
  map_match_my_flag=1;
  static int xynum_iter[10000][3]={0};
  int count_xynum_iter=0;
  int cloud_in_size=cloud_in->points.size();

  std::cout<<"range_rough_x_min: "<<range_rough_x_min<<"  range_rough_y_min: "<<range_rough_y_min<< \
                         "range_rough_x_max: "<<range_rough_x_max<<"  range_rough_y_max: "<<range_rough_y_max<<std::endl;
  // 在离线地图上查找有多少点和实时地图重合--------------------
  int x_iter=range_rough_x_min;//将输入点云平移
  int y_iter=range_rough_y_min;
  for(int num=0;;num++){
    looplock.lock();
    double begin_time = ros::Time::now().toSec();
    std::cout<<"开始进行第"<<num<<"次粗查找---------------------------------------"<<std::endl;
    float x_iter_range=x_iter*resolution;//将平移栅格值转化为平移距离值
    float y_iter_range=y_iter*resolution;    
    std::cout<<"平移栅格数目:  x: "<<x_iter<<";"<<"   y: "<<y_iter<<std::endl;
    std::cout<<"平移量:  x: "<<x_iter_range<<";"<<"   y: "<<y_iter_range<<std::endl;
    pcl::PointXYZ point_tmp;
    for(int k=0;k<cloud_in_size;k++){
      point_tmp.x=cloud_in->points[k].x+x_iter_range;
      point_tmp.y=cloud_in->points[k].y+y_iter_range;     
      point_tmp.z=0;
      cloud_match->points.push_back(point_tmp);       
    }
    int count_include=0;//计数转换后的点云有多少属于目标点云
    int count_uninclude=0;//计数转换后的点云有多少不属于目标点云    
    for(int k=0;k<cloud_in_size;k++){
      if(check_one_point(cloud_match->points[k])){
        count_include++;
      }
      else {
        count_uninclude++;
      }
      if(count_uninclude>=cloud_in_size*0.1){//为了减少计算资源消耗
        break;
      }
    }

    std::cout<<"输入点云总数: "<<cloud_in_size<<"输入点云总数的90%: "<<cloud_in_size*0.9<<";"<<"属于目标点云总数: "<<count_include<<std::endl;
    if(count_include>cloud_in_size*0.8){
      xynum_iter[count_xynum_iter][0]=x_iter;
      xynum_iter[count_xynum_iter][1]=y_iter;
      xynum_iter[count_xynum_iter][2]=count_include;      
      count_xynum_iter++;
    }
    x_iter=x_iter+iteration_interval;//加快查找速度------------------------0720-1
    if(x_iter>=range_rough_x_max){//说明此时已经在离线地图gridmap0上遍历一行了，于是开始遍历下一行
      x_iter=range_rough_x_min;
      y_iter=y_iter+iteration_interval;//加快查找速度---------------------------0720-1
    }
    if(y_iter>=range_rough_y_max){//说明此时已经在离线地图gridmap0上遍历完了
      std::cout<<"粗查找设计的范围遍历完全. "<<std::endl;
      break;
    }
    double end_time = ros::Time::now().toSec();
    std::cout << "一次查找耗时:" << (end_time - begin_time) << std::endl;
    if(count_include>=cloud_in_size*0.995){//粗查找提前结束标志---------
      rough_match=0;//粗查找没有查询整个离线地图
      mapmatch_rough_x=x_iter_range;//粗匹配平移距离
      mapmatch_rough_y=y_iter_range;
      std::cout<<"粗匹配查找的到平移量:  x: "<<mapmatch_rough_x<<";"<<"   y: "<<mapmatch_rough_y<<std::endl;
      looplock.unlock();
      pcl::toROSMsg(*cloud_match, verifyCloudMsg);
      verifyCloudMsg.header.stamp = ros::Time::now();
      verifyCloudMsg.header.frame_id = "map";
      return;//提前结束粗查找
    }
    cloud_match->points.clear();
    looplock.unlock();
  }
  // 查找多次平移时，有最多重合点的那次平移---------------------------
  int x_iter_max=0;
  int y_iter_max=0;
  int count_include_max=0;
  for(int i=0;i<=count_xynum_iter;i++){
    if(count_include_max<xynum_iter[i][2]){
      count_include_max=xynum_iter[i][2];
      x_iter_max=xynum_iter[i][0];
      y_iter_max=xynum_iter[i][1];
    }
  }
  // std::cout<<"888"<<std::endl;
  float x_iter_max_range=x_iter_max*resolution;//将平移栅格值转化为平移距离值
  float y_iter_max_range=y_iter_max*resolution;    
  
  for(int k=0;k<cloud_in_size;k++){
      pcl::PointXYZ point_tmp;
      point_tmp.x=cloud_in->points[k].x+x_iter_max_range;
      point_tmp.y=cloud_in->points[k].y+y_iter_max_range;     
      point_tmp.z=0;
      cloud_match->points.push_back(point_tmp);       
  }
  mapmatch_rough_x=x_iter_max_range;//粗匹配平移距离
  mapmatch_rough_y=y_iter_max_range;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"粗查找最终结果: 平移量: x: "<<mapmatch_rough_x<<"   y: "<<mapmatch_rough_y<<std::endl;

  pcl::toROSMsg(*cloud_match, verifyCloudMsg);
  verifyCloudMsg.header.stamp = ros::Time::now();
  verifyCloudMsg.header.frame_id = "map";
}


// 使用逐步平移输入点云/(即实时地图的原点)的方法进行配准----在粗匹配的基础上进行精确匹配
void map_match_refine_my(){
  map_match_refine_my_flag=1;
  static int xynum_refine_iter[10000][3]={0};
  int count_xynum_iter=0;
  int cloud_in_size=cloud_in->points.size();

  // 在离线地图上查找有多少点和实时地图重合--------------------
  int x_iter=range_refine_x_min;//这组数据用作调试0720-2
  int y_iter=range_refine_y_min;//0720-2
  // int x_iter=-20;//这组数据用作调试0720-1
  // int y_iter=-10;//0720-1
  for(int num=0;;num++){
    loop_refine_lock.lock();
    double begin_time = ros::Time::now().toSec();
    std::cout<<"开始进行第"<<num<<"次精查找---------------------------------------"<<std::endl;
    float x_iter_range=x_iter*resolution+mapmatch_rough_x;//在粗查找的基础上进行查找
    float y_iter_range=y_iter*resolution+mapmatch_rough_y;    
    std::cout<<"精确查找平移栅格数目:  x: "<<x_iter<<";"<<"   y: "<<y_iter<<std::endl;
    std::cout<<"总的平移量:  x: "<<x_iter_range<<";"<<"   y: "<<y_iter_range<<std::endl;
    pcl::PointXYZ point_tmp;
    for(int k=0;k<cloud_in_size;k++){
      point_tmp.x=cloud_in->points[k].x+x_iter_range;
      point_tmp.y=cloud_in->points[k].y+y_iter_range;     
      point_tmp.z=0;
      cloud_refine_match->points.push_back(point_tmp);       
    }
    // std::cout<<"333"<<std::endl;
    int count_include=0;//计数转换后的点云有多少属于目标点云
    int count_uninclude=0;//计数转换后的点云有多少不属于目标点云    
    for(int k=0;k<cloud_in_size;k++){
      if(check_one_point_refine(cloud_refine_match->points[k])){
        count_include++;
      }
      else {
        count_uninclude++;
      }
      if(count_uninclude>=cloud_in_size*0.1){//为了减少计算资源消耗
        break;
      }
    }

    std::cout<<"输入点云总数的10%: "<<cloud_in_size*0.1<<";"<<"属于目标点云总数: "<<count_include<<std::endl;
    // std::cout<<"444"<<std::endl;
    if(count_include>cloud_in_size*0.1){
      xynum_refine_iter[count_xynum_iter][0]=x_iter;
      xynum_refine_iter[count_xynum_iter][1]=y_iter;
      xynum_refine_iter[count_xynum_iter][2]=count_include;      
      count_xynum_iter++;
    }
    x_iter=x_iter+1;//加快查找速度
    // if(x_iter>=50){//
    //   x_iter=-50;
    //   y_iter=y_iter+1;//加快查找速度
    // }
    // if(y_iter>50){//
    //   break;
    // }
    if(x_iter>range_refine_x_max){//------调试使用  0720-2
      x_iter=range_refine_x_min;
      y_iter=y_iter+1;//加快查找速度
    }
    if(y_iter>range_refine_y_max){//
      cloud_refine_match->points.clear();
      break;
    }
    // if(x_iter>20){//------调试使用  0720-1
    //   x_iter=-20;
    //   y_iter=y_iter+1;//加快查找速度
    // }
    // if(y_iter>10){//
    //   cloud_refine_match->points.clear();
    //   break;
    // }
    double end_time = ros::Time::now().toSec();
    std::cout << "一次查找耗时:" << (end_time - begin_time) << std::endl;
    if(count_include==cloud_in_size){//精确查找结束条件，这个条件比较严格，是为了避免找到不是很合适的平移
      mapmatch_refine_x=x_iter_range;//粗匹配平移距离
      mapmatch_refine_y=y_iter_range;
      std::cout<<"总的查找平移量:  x: "<<mapmatch_refine_x<<";"<<"   y: "<<mapmatch_refine_y<<std::endl;
      loop_refine_lock.unlock();

      // 发布点云
      cloud_refine_match->points.clear();
      pcl::PointXYZ point_tmp;
      for(int k=0;k<cloud_in_all->points.size();k++){
        point_tmp.x=cloud_in_all->points[k].x+x_iter_range;
        point_tmp.y=cloud_in_all->points[k].y+y_iter_range;     
        point_tmp.z=0;
        cloud_refine_match->points.push_back(point_tmp);       
      }      
      pcl::toROSMsg(*cloud_refine_match, refineverifyCloudMsg);
      refineverifyCloudMsg.header.stamp = ros::Time::now();
      refineverifyCloudMsg.header.frame_id = "map";
      return;//提前结束粗查找
    }
    cloud_refine_match->points.clear();
    loop_refine_lock.unlock();
  }
  // std::cout<<"777"<<std::endl;
  // 查找多次平移时，有最多重合点的那次平移---------------------------
  int x_iter_max=0;
  int y_iter_max=0;
  int count_include_max=0;
  for(int i=0;i<=count_xynum_iter;i++){
    if(count_include_max<xynum_refine_iter[i][2]){
      count_include_max=xynum_refine_iter[i][2];
      x_iter_max=xynum_refine_iter[i][0];
      y_iter_max=xynum_refine_iter[i][1];
    }
  }
  // std::cout<<"888"<<std::endl;
  float x_iter_max_range=x_iter_max*resolution+mapmatch_rough_x;//将平移栅格值转化为平移距离值
  float y_iter_max_range=y_iter_max*resolution+mapmatch_rough_y;    
  pcl::PointXYZ point_tmp;
  for(int k=0;k<cloud_in_all->points.size();k++){
    point_tmp.x=cloud_in_all->points[k].x+x_iter_max_range;
    point_tmp.y=cloud_in_all->points[k].y+y_iter_max_range;     
    point_tmp.z=0;
    cloud_refine_match->points.push_back(point_tmp);       
  }
  mapmatch_refine_x=x_iter_max_range;//粗匹配平移距离
  mapmatch_refine_y=y_iter_max_range;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"----------------------------------------------------------------------------------------------------------------"<<std::endl;
  std::cout<<"粗查找得到的:  平移量: x: "<<mapmatch_rough_x<<"   y: "<<mapmatch_rough_y<<std::endl;
  std::cout<<"精确查找最终结果:  平移量: x: "<<mapmatch_refine_x<<"   y: "<<mapmatch_refine_y<<std::endl;

  pcl::toROSMsg(*cloud_refine_match, refineverifyCloudMsg);
  refineverifyCloudMsg.header.stamp = ros::Time::now();
  refineverifyCloudMsg.header.frame_id = "map";
}

// 进行粗匹配和精匹配后，进行icp匹配
void map_match_icp(){

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;//创建ICP对象
  icp.setMaxCorrespondenceDistance(100);

/*
  针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_2/地图0.05m 和 /home/meng/ideas/论文稿及材料0608/kitti数据_1/栅格地图
  // icp部分参数配置 0730-1---------------------------------------------
  // icp.setMaximumIterations(100);//迭代次数已达到用户施加的最大迭代次数
  // icp.setTransformationEpsilon(1e-5);//先前转换和当前估计转换（即两次位姿转换）之间的 epsilon（差异）小于用户施加的值
  // icp.setEuclideanFitnessEpsilon(1e-5);//欧几里得平方误差的总和小于用户定义的阈值
  // icp部分参数配置 0730-2---------------------------------------------
  // icp.setMaximumIterations(20);
  // icp.setTransformationEpsilon(1e-3);
  // icp.setEuclideanFitnessEpsilon(1e-3);
  // icp部分参数配置 0730-3---------------------------------------------
  icp.setMaximumIterations(10);
  icp.setTransformationEpsilon(1e-3);
  icp.setEuclideanFitnessEpsilon(1e-3);
*/

  //针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_2/地图0.1m
  // icp部分参数配置 0730-1---------------------------------------------
  // icp.setMaximumIterations(100);
  // icp.setTransformationEpsilon(1e-2);
  // icp.setEuclideanFitnessEpsilon(1e-2);

  //针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_2/地图0.2m
  // icp部分参数配置 0730-1---------------------------------------------
  // icp.setMaximumIterations(100);
  // icp.setTransformationEpsilon(1e-2);
  // icp.setEuclideanFitnessEpsilon(1e-2);

  //针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_1/地图0.05m--------------------------------------
  // icp部分参数配置 0814-1
  // icp.setMaximumIterations(100);
  // icp.setTransformationEpsilon(1e-3);
  // icp.setEuclideanFitnessEpsilon(1e-3);
  // icp部分参数配置 0814-2
  // icp.setMaximumIterations(20);
  // icp.setTransformationEpsilon(1e-3);
  // icp.setEuclideanFitnessEpsilon(1e-3);
  // icp部分参数配置 0814-3
  icp.setMaximumIterations(15);
  icp.setTransformationEpsilon(1e-3);
  icp.setEuclideanFitnessEpsilon(1e-3);

  //针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_1/地图0.1m--------------------------------------
  // icp部分参数配置0815-1
  // icp.setMaximumIterations(15);
  // icp.setTransformationEpsilon(1e-3);
  // icp.setEuclideanFitnessEpsilon(1e-3);
  // icp部分参数配置0820-2
  // icp.setMaximumIterations(15);
  // icp.setTransformationEpsilon(1e-2);
  // icp.setEuclideanFitnessEpsilon(1e-2);

  //针对地图：/home/meng/ideas/论文稿及材料0608/kitti数据_1/地图0.2m--------------------------------------
  // icp部分参数配置0815-1
  // icp.setMaximumIterations(15);
  // icp.setTransformationEpsilon(1e-3);
  // icp.setEuclideanFitnessEpsilon(1e-3);
  // icp部分参数配置0820--2
  // icp.setMaximumIterations(15);
  // icp.setTransformationEpsilon(1e-2);
  // icp.setEuclideanFitnessEpsilon(1e-2);

  icp.setRANSACIterations(0);// 设置RANSAC运行次数
	icp.setInputCloud(cloud_for_icp);//为ICP算法设置输入点云
	icp.setInputTarget(cloud_out);//设置目标点云
	icp.align(cloud_after_icp);//cloud_after_icp为验证的结果

  pcl::toROSMsg(cloud_after_icp, aftericpCloudMsg);
  aftericpCloudMsg.header.stamp = ros::Time::now();
  aftericpCloudMsg.header.frame_id = "map";

	//如果hasConverged() == 1,说明输入点云和目标点云之间的是刚性变换，并返回ICP的评估分数
	std::cout << "has converged:" << icp.hasConverged() << "      score: " <<icp.getFitnessScore() << std::endl;
 
	//打印出输入点云和目标点云之间的转换矩阵
	std::cout << icp.getFinalTransformation() << std::endl;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "map_match_quadtree");
  ros::NodeHandle nh("~");

  nh.param<string>("globalmappath", globalmappath, ""); 
  nh.param<string>("localmappath", localmappath, ""); 
  nh.param<string>("globalmap_header_path", globalmap_header_path, ""); 
  nh.param<string>("localmap_header_path", localmap_header_path, ""); 
  nh.param<float>("resolution", resolution, 0.05); 

  // 粗匹配和精匹配时点重合的距离阈值
  nh.param<float>("point_threshold_rough_x", point_threshold_rough_x, 0.5); 
  nh.param<float>("point_threshold_rough_y", point_threshold_rough_y, 0.5); 
  nh.param<float>("point_threshold_refine_x", point_threshold_refine_x, 0.05); 
  nh.param<float>("point_threshold_refine_y", point_threshold_refine_y, 0.05); 
  
  // 粗匹配迭代步长与精匹配迭代范围
  nh.param<int>("iteration_interval", iteration_interval, 20); 
  nh.param<int>("range_rough_x_min", range_rough_x_min, -400); 
  nh.param<int>("range_rough_y_min", range_rough_y_min, -400); 
  nh.param<int>("range_rough_x_max", range_rough_x_max, 400); 
  nh.param<int>("range_rough_y_max", range_rough_y_max, 400); 

  nh.param<int>("range_refine_x_min", range_refine_x_min, -20); 
  nh.param<int>("range_refine_y_min", range_refine_y_min, -20); 
  nh.param<int>("range_refine_x_max", range_refine_x_max, 20); 
  nh.param<int>("range_refine_y_max", range_refine_y_max, 20); 

  pubSourceCloud = nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 1);//
  pubSourceAllCloud = nh.advertise<sensor_msgs::PointCloud2>("/source_all_cloud", 1);//
  pubTargetCloud = nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);//
  pubVerifyCloud = nh.advertise<sensor_msgs::PointCloud2>("/verify_cloud", 1);//
  pubRefineVerifyeCloud = nh.advertise<sensor_msgs::PointCloud2>("/refine_verify_cloud", 1);//
  pubForIcpCloud = nh.advertise<sensor_msgs::PointCloud2>("/for_icp_cloud", 1);//
  pubAfterIcpCloud = nh.advertise<sensor_msgs::PointCloud2>("/after_icp_cloud", 1);//


  double begin_time_total = ros::Time::now().toSec();
  double begin_time_change2cloud = ros::Time::now().toSec();
  globalmap_header_info();
  localmap_header_info();
  localmap2globalmap_offset();
  globalmap2cloud();
  localmap2cloud();
  double end_time_change2cloud = ros::Time::now().toSec();

  // double begin_time_rough_match = ros::Time::now().toSec();
  // map_match_rough_my();//粗匹配
  // double end_time_rough_match = ros::Time::now().toSec();



  //调试使用-------------------------
  mapmatch_rough_x=0;
  mapmatch_rough_y=0;
  // mapmatch_refine_x=13.7;//精匹配结果
  // mapmatch_refine_y=28.9;
  // double begin_time_refine_match = ros::Time::now().toSec();
  // map_match_refine_my();//精匹配
  // double end_time_refine_match = ros::Time::now().toSec();

  
  double begin_time_icp_match = ros::Time::now().toSec();
  // 调试使用--
  mapmatch_refine_x=0;
  // mapmatch_refine_y=0.35;
  mapmatch_refine_y=0; 
  cloud_for_icp_transform();//用于调试
  map_match_icp();//使用icp进行进一步的精匹配
  double end_time_icp_match = ros::Time::now().toSec();

  double end_time_total = ros::Time::now().toSec();

  std::cout << "将地图转换为点云耗时:  " << (end_time_change2cloud - begin_time_change2cloud) <<"  秒"<< std::endl;  
  // std::cout << "粗匹配耗时:  " << (end_time_rough_match - begin_time_rough_match) <<"  秒"<< std::endl;  
  // std::cout << "精匹配耗时:" << (end_time_refine_match - begin_time_refine_match) <<"秒"<< std::endl;  
  std::cout << "icp匹配耗时:" << (end_time_icp_match - begin_time_icp_match) <<"秒"<< std::endl;  
  std::cout << "查找总耗时:  " << (end_time_total - begin_time_total) <<"  秒"<< std::endl;

  if(rough_match==0){
    std::cout<<"粗查找没有查询整个区域"<<std::endl;
  }
  else {
    std::cout<<"粗查找查询了整个区域"<<std::endl;    
  }
  // 发布粗匹配、精匹配之后的点云数据
  ros::Rate rate_10hz(10);  
  while(ros::ok()){
    pubSourceCloud.publish(sourceCloudMsg);
    pubSourceAllCloud.publish(sourceAllCloudMsg);
    pubTargetCloud.publish(targetCloudMsg);
    pubForIcpCloud.publish(foricpCloudMsg);
    pubAfterIcpCloud.publish(aftericpCloudMsg);

    if(map_match_my_flag==1){
      //粗匹配点云
      pubVerifyCloud.publish(verifyCloudMsg);
    }

    if(map_match_refine_my_flag==1){
      //精确匹配点云
      pubRefineVerifyeCloud.publish(refineverifyCloudMsg);      
    }

    rate_10hz.sleep();
  }

  ros::shutdown();
  return 0;
}