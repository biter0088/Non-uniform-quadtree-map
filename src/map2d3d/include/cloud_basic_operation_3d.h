/*
    滤除无效点云及感兴趣区域选择
    输入：laserCloudInRaw
    输出：cloud_basic_filter
*/
void basic_filter_3d(){
  // std::cout<<"原始点云 laserCloudInRaw->points.size():"<<laserCloudInRaw->points.size()<<std::endl;
  for (int i=0; i<laserCloudInRaw->points.size(); i++)
  {
    if (isnan(laserCloudInRaw->points[i].x)){    //滤除无效点
      continue;  }

    if (fabs(laserCloudInRaw->points[i].x)<=1.25&&fabs(laserCloudInRaw->points[i].y)<=1.75){
      continue;//滤除车体周围点云
    }
  // 直通滤波，选取感兴趣区域
    if (( laserCloudInRaw->points[i].z < max_height)&&(laserCloudInRaw->points[i].z > min_height) 
      &&  laserCloudInRaw->points[i].x < max_width &&  laserCloudInRaw->points[i].x > min_width 
      &&  laserCloudInRaw->points[i].y < max_len &&  laserCloudInRaw->points[i].y > min_len )
      cloud_basic_filter->points.push_back(laserCloudInRaw->points[i]);
  }

  laserCloudInRaw->clear();//清空原始点云
  std::cout<<"滤除无效点云及感兴趣区域选择 cloud_basic_filter->points.size():"<<cloud_basic_filter->points.size()<<std::endl;
}

/*
    发布基本滤波之后的点云：cloud_basic_filter
*/
void pub_basic_filter_cloud_3d(const ros::Time& rostime){

  pcl::toROSMsg(*cloud_basic_filter, basic_filterCloudMsg);
  basic_filterCloudMsg.header.stamp = rostime;
  basic_filterCloudMsg.header.frame_id = "velo_link";
  pubBasicFilterCloud.publish(basic_filterCloudMsg);

}

/* 基于条件或半径进行离群点滤除
     input:  cloud_basic_filter
    output:  cloud_radius_filter
*/
void outlier_remove_with_radius_3d(){
  // 创建滤波器对象
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // 设置要滤波的点云
  outrem.setInputCloud(cloud_basic_filter);
  //滤除输入点云在一定范围内（半径 R）没有至少达到足够多近邻（K）的所有数据点；如果没有，就删除这个点
  // 设置滤波半径
  outrem.setRadiusSearch(filter_radius);//单位应该是m
  // 设置滤波最少近邻数
  outrem.setMinNeighborsInRadius (filter_number);
  // 执行半径滤波，将结果给cloud_filtered_radius
  outrem.filter (*cloud_radius_filter);

  // std::cout<<"基于条件或半径的离群点过滤效果:"<<std::endl;
  // std::cout<<"滤波前 cloud_basic_filter->points.size():"<<cloud_basic_filter->points.size()<<std::endl; 
  std::cout<<"基于条件或半径滤波后 cloud_radius_filter->points.size():"<<cloud_radius_filter->points.size()<<std::endl; 
  cloud_basic_filter->clear();
}


// 在车辆坐标系x前y左中计算点云与x轴的夹角，范围[0~360°)
double angle_compute(float y_coordinate ,float x_coordinate){
	double theta = atan2(y_coordinate, x_coordinate); //弧度
	// cout<<"theta:"<<theta<<endl;
	if (theta < 0){
		theta += 2*M_PI;
	}
	return theta * 180/M_PI; //角度
}

/*
  输出 / 分割出：                       还没有想好-----------------------------------------
    与车辆行驶有关的点云 valid_cloud_point
    与车辆行驶关系不大的点云 invalid_cloud_point
  输入：
    经过基本滤波，条件/半径滤波，地面点云分割之后的非地面点云 
  原理：
    1.在车辆坐标系x前y左中，将点云分为24份扇区，每份中心角度为0°、15°、30°...以15°为间隔，相邻份之间有50%的重叠
    2.评估每一份中有多少个点符合区域中心点的要求：
        该点周围半径r的区域内至少有4个点
    3.取每一个扇区中距离车辆位置最近的前两个区域中心点作为种子点，
        如果当前扇区中其他种子点中，有种子点距离第二近（只有一个种子点则不需要考虑这一步）的种子点距离为4*r，则也将其加入进来
        将各个扇区的种子点合并成种子点集
    4.基于"连通域"的概念进行聚类，将与种子点集距离为r的点加入种子点集； 接着将距离种子点集为r的点继续加入种子点集，直到没有新的点了
*/
void segment_valid_and_invalid(){
  for (int q=0; q<cloud_radius_filter->points.size(); q++)
  {
    // cloud_radius_filter->points[q].x, cloud_radius_filter->points[q].y, cloud_radius_filter->points[q].z;

  }
}


/*
  利用pcl自带的体素voxel grid进行滤波----弃用
  输入：cloud_radius_filter
  输出：cloud_voxel_filter
  弊端：结果中点云过于均匀分布，引起创建的三维栅格gridMap_3d_num_id数量增多
                暂时没找到非均匀体素滤波，这里先不用
*/
void down_smaple_with_voxelgrid_pcl(){
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(cloud_radius_filter);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_voxel_filter);
    std::cout<<"体素voxel grid滤波后 cloud_voxel_filter->points.size():"<<cloud_voxel_filter->points.size()<<std::endl; 
    cloud_radius_filter->clear();
}


/*
    将基本滤波、半径滤波、体素滤波后的点云转换到世界坐标系下（即车辆初始位姿）
    输入：cloud_radius_filter    /// cloud_voxel_filter    cloud_basic_filter
    输出：cloudInWorld
*/
void cloud2world_3d(){
  for (int q=0; q<cloud_radius_filter->points.size(); q++)
  {
    Eigen::Vector3d point(cloud_radius_filter->points[q].x, cloud_radius_filter->points[q].y, cloud_radius_filter->points[q].z);
    Eigen::Vector3d pointinworld;
    pointinworld = q_odom_curr_now * point + t_odom_curr_now;  

    pcl::PointXYZ pointinworld2;
    pointinworld2.x = pointinworld.x();
    pointinworld2.y = pointinworld.y();
    pointinworld2.z = pointinworld.z();

    if (fabs(pointinworld2.x) < 0.5*width_3d && fabs(pointinworld2.y) < 0.5*length_3d)
    	cloudInWorld->points.push_back(pointinworld2);
  }
  std::cout<<"转换到世界坐标系下点云数目 cloudInWorld->points.size():"<<cloudInWorld->points.size()<<std::endl; 
  cloud_radius_filter->clear();
  // cloud_voxel_filter->clear();
}

/*
    将基本滤波、半径滤波、体素滤波后的点云转换到世界坐标系下（即车辆初始位姿）
    输入：cloud_radius_filter    /// cloud_voxel_filter    cloud_basic_filter
    输出：cloudInWorld
*/
void cloud2world_3d_byslam_corner(){
  for (int q=0; q<cloud_basic_filter->points.size(); q++)
  {
    Eigen::Vector3d point(cloud_basic_filter->points[q].x, cloud_basic_filter->points[q].y, cloud_basic_filter->points[q].z);
    Eigen::Vector3d pointinworld;
    pointinworld = q_odom_curr_now * point + t_odom_curr_now;  

    pcl::PointXYZ pointinworld2;
    pointinworld2.x = pointinworld.x();
    pointinworld2.y = pointinworld.y();
    pointinworld2.z = pointinworld.z();

    if (fabs(pointinworld2.x) < 0.5*width_3d && fabs(pointinworld2.y) < 0.5*length_3d)
    	cloudInWorld->points.push_back(pointinworld2);
  }
  std::cout<<"转换到世界坐标系下点云数目 cloudInWorld->points.size():"<<cloudInWorld->points.size()<<std::endl; 
  cloud_basic_filter->clear();
  // cloud_voxel_filter->clear();
}

/*
    发布转换到世界坐标系下的点云：cloudInWorld
*/
void pub_world_cloud_3d(const ros::Time& rostime){

  pcl::toROSMsg(*cloudInWorld, worldCloudMsg);
  worldCloudMsg.header.stamp = rostime;
  // worldCloudMsg.header.frame_id = "velo_link";
  worldCloudMsg.header.frame_id = "camera_init";
  pubWorldCloud.publish(worldCloudMsg);

}


/*
    发布某一帧或某几帧转换到世界坐标系下的点云：cloudInWorld
*/
void pub_frame_cloud_3d(const ros::Time& rostime){
  if(laserVec.size()==10){//点云帧选择
    pcl::toROSMsg(*cloudInWorld, frameCloudMsg);
  }
  frameCloudMsg.header.stamp = rostime;
  frameCloudMsg.header.frame_id = "velo_link";
  // if(pubFrameCloud.getNumSubscribers()){
    pubFrameCloud.publish(frameCloudMsg);
  // }
}

