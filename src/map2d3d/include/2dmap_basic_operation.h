/*
    将全局坐标系下的单帧点云存放到栅格gridMap中
*/
void cloud2gridmap(){
  for (int i=0; i<cloudInWorld->points.size(); i++)
  {
    pcl::PointXYZ thisPoint = cloudInWorld->points[i];
    // int x_cor = round((width*0.5+cloudInWorld->points[i].x - robot_x) / resolution) ;// 计算每个点所在坐标（可以理解为栅格）
    // int y_cor = round((height*0.5+cloudInWorld->points[i].y - robot_y) / resolution) ;
    // int x_cor = round((width*0.25+cloudInWorld->points[i].x - robot_x) / resolution) ;// 计算每个点所在坐标（可以理解为栅格）
    // int y_cor = round((height*0.25+cloudInWorld->points[i].y - robot_y) / resolution) ;//width*0.25用于平移点云，使点云表示在地图中合适的位置
    // int x_cor = round((max_width*1.2+cloudInWorld->points[i].x - robot_x) / resolution) ;// max_width、max_len为一帧点云中感兴趣点云的x、y方向上的范围
    // int y_cor = round((max_len*1.2+cloudInWorld->points[i].y - robot_y) / resolution) ;//
    int x_cor = round((45+cloudInWorld->points[i].x - robot_x) / resolution) ;// max_width、max_len为一帧点云中感兴趣点云的x、y方向上的范围
    int y_cor = round((35+cloudInWorld->points[i].y - robot_y) / resolution) ;//

    std::pair<int,int> xy_index =std::make_pair(x_cor, y_cor);
    // 循环查找
    if(gridMap.count(xy_index)>0)
    {
      // 找到了
      gridMap[xy_index].push_back(thisPoint.z);
    }
    else
    {
      // 新的栅格
      std::vector<double> newGrid;
      newGrid.push_back(thisPoint.z);
      gridMap.insert({xy_index,  newGrid });
    }
  }
  cloudInWorld->clear();
}


/*
    将全局坐标系下的单帧点云存放到栅格gridMap中  
    输入：cloud_radius_filter
*/
void cloud2gridmap_static(){
  for (int i=0; i<cloud_radius_filter->points.size(); i++)
  {
    pcl::PointXYZ thisPoint = cloud_radius_filter->points[i];
    int x_cor = round((width*0.5+cloud_radius_filter->points[i].x ) / resolution) ;// 计算每个点所在坐标（可以理解为栅格）
    int y_cor = round((height*0.5+cloud_radius_filter->points[i].y ) / resolution) ;
    std::pair<int,int> xy_index =std::make_pair(x_cor, y_cor);
    // 循环查找
    if(gridMap.count(xy_index)>0)
    {
      // 找到了
      gridMap[xy_index].push_back(thisPoint.z);
    }
    else
    {
      // 新的栅格
      std::vector<double> newGrid;
      newGrid.push_back(thisPoint.z);
      gridMap.insert({xy_index,  newGrid });
    }
  }
  cloud_radius_filter->clear();
}

/*
    利用高度差法，将gridmap内小栅格的高度差信息转化为占用信息，
    同时更新全局地图map_和贝叶斯更新数据bayes_image
*/
void gridmap2map(){
  //kv.first代表(x,y)坐标对，kv.second代表一个坐标对里的值
  for (auto &kv : gridMap) 
  {
    double maxz = -10000;
    double minz = 10000;
    for (int j=0; j<kv.second.size(); j++) {//遍历一个坐标对里的高度z值，找出最大最小高度
      if (kv.second[j] < minz) {
        minz = kv.second[j] ;
      }
      if (kv.second[j] > maxz) {
        maxz = kv.second[j] ;
      }
    }
    //-----------针对kitti数据集的环境特点，设计障碍物识别算法-----------------------------
    if (maxz - minz > differ_height)  // 认为kv中有一般类型的障碍物
    {
      map_prob.data[kv.first.first + kv.first.second*map_prob.info.width] += 1;
      bayes_image.at<float>( kv.first.second,kv.first.first)=bayes_image.at<float>( kv.first.second,kv.first.first)+bayes_update(P_occ);
      // 连续obs_prob帧扫描到才计算为障碍物
      if ( map_prob.data[kv.first.first + kv.first.second*map_prob.info.width] >= obs_prob){
        map_.data[kv.first.first + kv.first.second*map_.info.width] = 100;
        // bayes_image.at<float>( kv.first.second,kv.first.first)+=bayes_update(P_occ);
      }
    }
    else  // 认为kv中没有一般障碍物以及其他障碍物
    {
      double middle_z=(maxz+minz)/2.0;
      //  if ((maxz >= minz)&&((middle_z>-1.42)&&(middle_z<0)||(minz>=0))){//和车辆同样高度且车辆不能越过（30cm）的悬浮障碍物，或者路侧比车辆高的悬空障碍物（楼宇、房屋等）
       if ((maxz >= minz)&&(minz>=-0.20)){//路侧比车辆高的悬空障碍物（楼宇、房屋等），minz调参：0、-0.2
          map_prob.data[kv.first.first + kv.first.second*map_prob.info.width] += 1;
          bayes_image.at<float>( kv.first.second,kv.first.first)=bayes_image.at<float>( kv.first.second,kv.first.first)+bayes_update(P_occ);
          if ( map_prob.data[kv.first.first + kv.first.second*map_prob.info.width] >= obs_prob){
            map_.data[kv.first.first + kv.first.second*map_.info.width] = 100;
          }
       }
       else if(maxz >= minz){
          map_.data[kv.first.first + kv.first.second*map_.info.width] = 0; 
          bayes_image.at<float>(kv.first.second,kv.first.first)= bayes_image.at<float>(kv.first.second,kv.first.first)+bayes_update(P_free); 
        }
    }    
  }
  //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  gridMap.clear();
  // std::cout<<"判断是否有障碍物结束"<<std::endl;
  // cv::namedWindow("bayes_image_1", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_1", 255*bayes_image);
  // cv::waitKey(1);
}

/*
    如果未知栅格周边有三个栅格是可通行的，那么设置它为可通行/或贝叶斯更新里面增加空闲概率
*/
void surround_free_grid_vote(){
  cout<<"如果未知栅格周边有三个栅格是可通行的，那么设置它为可通行或贝叶斯更新里面增加空闲概率"<<endl;
  for (int i=1; i<map_.info.width-1; i++) 
  {
    for (int j=1; j<map_.info.height-1; j++){
      auto prob = map_.data[i + j*map_.info.width];
      auto prob_up = map_.data[i + (j+1)*map_.info.width];
      auto prob_down = map_.data[i + (j-1)*map_.info.width];
      auto prob_left = map_.data[i-1 + j*map_.info.width];
      auto prob_right = map_.data[i+1 + j*map_.info.width];

      if(prob==-1&&prob_up==0&&prob_down==0&&prob_left==0||
          prob==-1&&prob_up==0&&prob_down==0&&prob_right==0||
          prob==-1&&prob_up==0&&prob_left==0&&prob_right==0||
          prob==-1&&prob_down==0&&prob_left==0&&prob_right==0){
        // cout<<"更新一次"<<endl;
        map_.data[i + j*map_.info.width]=0;
        //这里如果更新贝叶斯概率，会让整个地图变成P_free。。。
        bayes_image.at<float>(j,i)=bayes_image.at<float>(j,i)+bayes_update(P_free);
      }
    }
  }
  // std::cout<<"用周围栅格投票结束"<<std::endl;
  // cv::namedWindow("bayes_image_2", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_2", 255*bayes_image);
  // cv::waitKey(1);
}

/*
    如果未知栅格周边有三个栅格是不可通行的，那么设置它为不可通行/或贝叶斯更新里面增加占据概率
*/
void surround_occ_grid_vote(){
  cout<<"如果未知栅格周边有三个栅格是不可通行的，那么设置它为不可通行或贝叶斯更新里面增加占据概率"<<endl;
  for (int i=1; i<map_.info.width-1; i++) 
  {
    for (int j=1; j<map_.info.height-1; j++){
      auto prob = map_.data[i + j*map_.info.width];
      auto prob_up = map_.data[i + (j+1)*map_.info.width];
      auto prob_down = map_.data[i + (j-1)*map_.info.width];
      auto prob_left = map_.data[i-1 + j*map_.info.width];
      auto prob_right = map_.data[i+1 + j*map_.info.width];

      if(prob==-1&&prob_up==100&&prob_down==100&&prob_left==100||
          prob==-1&&prob_up==100&&prob_down==100&&prob_right==100||
          prob==-1&&prob_up==100&&prob_left==100&&prob_right==100||
          prob==-1&&prob_down==100&&prob_left==100&&prob_right==100){
        map_.data[i + j*map_.info.width]=100;
        // cout<<"更新一次"<<endl;
        bayes_image.at<float>(j,i)=bayes_image.at<float>(j,i)+bayes_update(P_occ);
      }
    }
  }
  // std::cout<<"用周围栅格投票结束"<<std::endl;
  // cv::namedWindow("bayes_image_2", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_2", 255*bayes_image);
  // cv::waitKey(1);
}

/*
    如果未知或空闲栅格周边有四个栅格是不可通行的，那么设置它为不可通行/或贝叶斯更新里面增加占据概率
*/
void surround_occ_all_grid_vote(){
  cout<<"如果未知或空闲栅格周边有四个栅格是不可通行的，那么设置它为不可通行/或贝叶斯更新里面增加占据概率"<<endl;
  for (int i=1; i<map_.info.width-1; i++) 
  {
    for (int j=1; j<map_.info.height-1; j++){
      auto prob = map_.data[i + j*map_.info.width];
      auto prob_up = map_.data[i + (j+1)*map_.info.width];
      auto prob_down = map_.data[i + (j-1)*map_.info.width];
      auto prob_left = map_.data[i-1 + j*map_.info.width];
      auto prob_right = map_.data[i+1 + j*map_.info.width];

      if((prob==-1||prob==0)&&prob_up==100&&prob_down==100&&prob_left==100&&prob_right==100){
        map_.data[i + j*map_.info.width]=100;
        // cout<<"更新一次"<<endl;
        bayes_image.at<float>(j,i)=bayes_image.at<float>(j,i)+bayes_update(P_occ);
      }
    }
  }
  // std::cout<<"用周围栅格投票结束"<<std::endl;
  // cv::namedWindow("bayes_image_2", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_2", 255*bayes_image);
  // cv::waitKey(1);
}


/*
    如果可通行栅格周边有三个栅格是未知的，那么设置它为未知
*/
void surround_unknown_grid_vote(){
  for (int i=1; i<map_.info.width-1; i++) 
  {
    for (int j=1; j<map_.info.height-1; j++){
      auto prob = map_.data[i + j*map_.info.width];
      auto prob_up = map_.data[i + (j+1)*map_.info.width];
      auto prob_down = map_.data[i + (j-1)*map_.info.width];
      auto prob_left = map_.data[i-1 + j*map_.info.width];
      auto prob_right = map_.data[i+1 + j*map_.info.width];

      if(prob==0&&prob_up==-1&&prob_down==-1&&prob_left==-1||
          prob==0&&prob_up==-1&&prob_down==-1&&prob_right==-1||
          prob==0&&prob_up==-1&&prob_left==-1&&prob_right==-1||
          prob==0&&prob_down==-1&&prob_left==-1&&prob_right==-1){
        map_.data[i + j*map_.info.width]=-1;
        //这里如果更新贝叶斯概率，会让整个地图变成P_free。。。
        bayes_image.at<float>(j,i)=bayes_image.at<float>(j,i)+bayes_update(P_occ);
      }
    }
  }
  // std::cout<<"用周围栅格投票结束"<<std::endl;
  // cv::namedWindow("bayes_image_2", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_2", 255*bayes_image);
  // cv::waitKey(1);
}

/*
    使用里程计，车辆经过的位置都可通行
*/
void through_filter_with_odom(){
  for (int o=0; o<odomVec.size(); o++)
  {
    double pos_x = odomVec[o].pose.pose.position.x;
    double pos_y = odomVec[o].pose.pose.position.y;

    // int x_cor_car = round((0.5*width  + pos_x - robot_x) / resolution) ;
    // int y_cor_car = round((0.5*height + pos_y - robot_y) / resolution) ;
    // int x_cor_car = round((0.25*width  + pos_x - robot_x) / resolution) ;//0.25*width  和cloud2gridmap函数对应一下
    // int y_cor_car = round((0.25*height + pos_y - robot_y) / resolution) ;
    // int x_cor_car = round((max_width*1.2  + pos_x - robot_x) / resolution) ;//和cloud2gridmap函数对应一下
    // int y_cor_car = round((max_len*1.2 + pos_y - robot_y) / resolution) ;
    int x_cor_car = round((45 + pos_x - robot_x) / resolution) ;//和cloud2gridmap函数对应一下
    int y_cor_car = round((35 + pos_y - robot_y) / resolution) ;

    // 车体范围
    // double x_len_half = 0.6;//c2车体
    // double y_len_half = 0.45;
    double x_len_half = 0.8;//kitti数据集采集的车辆
    double y_len_half = 1.75;
    int x_len_d = x_len_half / resolution;
    int y_len_d = y_len_half / resolution;
    for (int px = -x_len_d ; px<=x_len_d; px++)
    {
      for (int py = -y_len_d ; py<=y_len_d; py++)
      {
        if ( ((x_cor_car+px)>=map_.info.width) ||  ((y_cor_car+py)>=map_.info.height)   )
        {
          continue;
        }
        map_.data[x_cor_car+px + (y_cor_car+py)*map_.info.width] = passvalue;   
        bayes_image.at<float>(y_cor_car+py , x_cor_car+px)=bayes_image.at<float>(y_cor_car+py , x_cor_car+px)+bayes_update(P_free);
      }
    }
  }
  // std::cout<<  "在地图中过滤车辆走过的位置"<<std::endl;
  // cv::namedWindow("bayes_image_3", cv::WINDOW_NORMAL);
  // cv::imshow("bayes_image_3", 255*bayes_image);
  // cv::waitKey(1);
}

/*
    使用bayes更新地图    
*/
void bayes_update_map(){
    // std::cout<<"使用 bayes 概率更新更新全局地图------------------------------------------ "<<std::endl;
    // std::cout<<"开始打印bayes_image数据:";
    for (int row=0; row<bayes_image.rows; row++) {
        for (int col=0; col<bayes_image.cols; col++) {
          if (bayes_image.at<float>(row,col)<bayes_update(TRESHOLD_P_FREE)){
              map_.data[col + row*map_.info.width] =0;  
          }
          else if(bayes_image.at<float>(row,col)>bayes_update(TRESHOLD_P_OCC)){
              map_.data[col + row*map_.info.width] =100;  
          }
          else{
              map_.data[col + row*map_.info.width] =-1;  
          }
          // std::cout<<bayes_image.at  <float>(row,col);
        }
    }
    // std::cout<<"结束打印bayes_image数据"<<std::endl;
}