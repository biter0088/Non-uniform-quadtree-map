/*
    滤除无效点云及感兴趣区域选择
*/
void basic_filter(){
  // std::cout<<"原始点云 laserCloudInRaw->points.size():"<<laserCloudInRaw->points.size()<<std::endl;
  for (int i=0; i<laserCloudInRaw->points.size(); i++)
  {
    if (isnan(laserCloudInRaw->points[i].x)||isnan(laserCloudInRaw->points[i].y)||isnan(laserCloudInRaw->points[i].z)){    
      continue;  }//滤除无效点

  // 直通滤波，选取感兴趣区域;
    if (( laserCloudInRaw->points[i].z < max_height)&&(laserCloudInRaw->points[i].z > min_height) 
      &&  laserCloudInRaw->points[i].x < max_width &&  laserCloudInRaw->points[i].x > min_width 
      &&  laserCloudInRaw->points[i].y < max_len &&  laserCloudInRaw->points[i].y > min_len )
      cloud_basic_filter->points.push_back(laserCloudInRaw->points[i]);
  }

  laserCloudInRaw->clear();//清空原始点云
  // std::cout<<"滤除无效点云及感兴趣区域选择 cloud_basic_filter->points.size():"<<cloud_basic_filter->points.size()<<std::endl;
}

/*
    发布基本滤波之后的点云
*/
void pub_basic_filter_cloud(const ros::Time& rostime){

  pcl::toROSMsg(*cloud_basic_filter, basic_filterCloudMsg);
  basic_filterCloudMsg.header.stamp = rostime;
  basic_filterCloudMsg.header.frame_id = "velo_link";
  pubBasicFilterCloud.publish(basic_filterCloudMsg);

}

/* 基于条件或半径进行离群点滤除
     input:  cloud_basic_filter
    output:  cloud_radius_filtered   
*/
void outlier_remove_with_radius(){
  // 创建滤波器对象
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // 设置要滤波的点云
  outrem.setInputCloud(cloud_basic_filter);
  // 滤除输入点云在一定范围内（半径 R）没有至少达到足够多近邻（K）的所有数据点；如果没有，就删除这个点
  // 设置滤波半径
  outrem.setRadiusSearch(filter_radius);//单位应该是m
  // 设置滤波最少近邻数
  outrem.setMinNeighborsInRadius (filter_number);
  // 执行半径滤波，将结果给cloud_filtered_radius
  outrem.filter (*cloud_radius_filter);

  // std::cout<<"基于条件或半径的离群点过滤效果:"<<std::endl;
  // std::cout<<"滤波前 cloud_basic_filter->points.size():"<<cloud_basic_filter->points.size()<<std::endl; 
  // std::cout<<"滤波后 cloud_radius_filtered->points.size():"<<cloud_radius_filter->points.size()<<std::endl; 
  cloud_basic_filter->clear();
}

/*
    将基本滤波后的点云转换到世界坐标系下（即车辆初始位姿）
*/
void cloud2world(){
  for (int q=0; q<cloud_radius_filter->points.size(); q++)
  {
    Eigen::Vector3d point(cloud_radius_filter->points[q].x, cloud_radius_filter->points[q].y, cloud_radius_filter->points[q].z);
    Eigen::Vector3d pointinworld;
    pointinworld = q_odom_curr_now * point + t_odom_curr_now;  

    pcl::PointXYZ pointinworld2;
    pointinworld2.x = pointinworld.x();
    pointinworld2.y = pointinworld.y();
    pointinworld2.z = pointinworld.z();

    // if (fabs(pointinworld2.x) < 0.5*width && fabs(pointinworld2.y) < 0.5*height)
    	cloudInWorld->points.push_back(pointinworld2);
  }
  // std::cout<<"转换到世界坐标系下点云数目 cloudInWorld->points.size():"<<cloudInWorld->points.size()<<std::endl; 
  cloud_radius_filter->clear();
}

/*
    发布转换到世界坐标系下的点云
*/
void pub_world_cloud(const ros::Time& rostime){

  pcl::toROSMsg(*cloudInWorld, worldCloudMsg);
  worldCloudMsg.header.stamp = rostime;
  worldCloudMsg.header.frame_id = "velo_link";
  pubWorldCloud.publish(worldCloudMsg);

}

