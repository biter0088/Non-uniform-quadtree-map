/*
    地图初始化
    贝叶斯更新图片bayes_image初始化
*/
#include<typeinfo> //查看数据类型
void map_init_function(){
  if (map_init)
  {
    robot_x = odomVec[odomVec.size()-1].pose.pose.position.x;
    robot_y = odomVec[odomVec.size()-1].pose.pose.position.y;
    cout<<"地图宽度、高度、分辨率分别为:"<<width<<"、"<<height<<"、"<<resolution<<endl;
    int map_width_inc = round(width / resolution);
    int map_height_inc = round(height / resolution);
    
    // map_.header.frame_id="imu_link";
    map_.header.frame_id="/velo_link";
    // map_.header.frame_id="camera_init";
    map_.header.stamp = ros::Time::now(); 
    map_.info.resolution = resolution;         // float32
    map_.info.width      = map_width_inc;           // uint32
    map_.info.height     = map_height_inc;           // uint32
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    // map_.info.origin.orientation.x = q_odom_curr_now.x();
    // map_.info.origin.orientation.y = q_odom_curr_now.y();
    // map_.info.origin.orientation.z = q_odom_curr_now.z();
    // map_.info.origin.orientation.w = q_odom_curr_now.w();
    // map_.info.origin.position.x = robot_x;
    // map_.info.origin.position.y = robot_y;
    // map_.info.origin.position.x = -width/2.0;
    // map_.info.origin.position.y = -height/2.0;
    map_.info.origin.position.x = -50;
    map_.info.origin.position.y = -50;
    // map_.info.origin.position.x =0;
    // map_.info.origin.position.y = 0;
    // map_.info.origin.position.x = -width/2.0+30;
    // map_.info.origin.position.y = -height/2.0+30;
    map_.info.origin.position.z = -1.8;

    map_.data.resize(map_.info.width * map_.info.height);

    for (int i=0; i<map_.info.width * map_.info.height; i++)
    {
      map_.data[i] = -1;  // 均为未知状态
    }

    map_prob.header.frame_id="/velo_link";
    // map_.header.frame_id="camera_init";
    map_prob.header.stamp = ros::Time::now(); 
    map_prob.info.resolution = resolution;         // float32
    map_prob.info.width      = map_width_inc;           // uint32
    map_prob.info.height     = map_height_inc;           // uint32
    map_prob.info.origin.orientation.x = 0.0;
    map_prob.info.origin.orientation.y = 0.0;
    map_prob.info.origin.orientation.z = 0.0;
    map_prob.info.origin.orientation.w = 1.0;
    // map_prob.info.origin.orientation.x = q_odom_curr_now.x();
    // map_prob.info.origin.orientation.y = q_odom_curr_now.y();
    // map_prob.info.origin.orientation.z = q_odom_curr_now.z();
    // map_prob.info.origin.orientation.w = q_odom_curr_now.w(); 
    // map_prob.info.origin.position.x = robot_x;
    // map_prob.info.origin.position.y = robot_y;
    // map_prob.info.origin.position.x = -width/2.0;
    // map_prob.info.origin.position.y = -height/2.0;
    map_prob.info.origin.position.x = -50;
    map_prob.info.origin.position.y = -50;
    // map_prob.info.origin.position.x = 0;
    // map_prob.info.origin.position.y = 0;
    // map_prob.info.origin.position.x = -width/2.0+30;
    // map_prob.info.origin.position.y = -height/2.0+30;
    map_prob.info.origin.position.z = -1.8;


    map_prob.data.resize(map_prob.info.width * map_prob.info.height);
    for (int i=0; i<map_prob.info.width * map_prob.info.height; i++)
    {
      map_prob.data[i] = 0;  // 均为可通行状态
    }

    map_init = false;
    std::cout<<"地图初始化完成"<<std::endl;
    bayes_image = bayes_update(P_prior)*cv::Mat::ones(cv::Size(map_width_inc, map_height_inc), CV_32F);
    // std::cout<<"bayes_image.at<float>(5,5)  "<<bayes_image.at<float>(5,5)<<std::endl;
    // std::cout<<"typeid(bayes_image.at<float>(5,5)).name()   "<<typeid(bayes_image.at<float>(5,5)).name()<<std::endl;    
  
    // cv::namedWindow("bayes_image_0", 0);
    // cv::imshow("bayes_image_0", bayes_image);
    // cv::waitKey(1);
  }
}


void map_init_function_static(){
  if (map_init)
  {
    int map_width_inc = round(width / resolution);
    int map_height_inc = round(height / resolution);
    
    map_.header.frame_id="/rslidar";
    map_.header.stamp = ros::Time::now(); 
    map_.info.resolution = resolution;         // float32
    map_.info.width      = map_width_inc;           // uint32
    map_.info.height     = map_height_inc;           // uint32
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    map_.info.origin.position.x = -width/2.0;
    map_.info.origin.position.y = -height/2.0;
    map_.info.origin.position.z = -1.2;

    map_.data.resize(map_.info.width * map_.info.height);

    for (int i=0; i<map_.info.width * map_.info.height; i++)
    {
      map_.data[i] = -1;  // 均为未知状态
    }

    map_prob.header.frame_id="/rslidar";
    map_prob.header.stamp = ros::Time::now(); 
    map_prob.info.resolution = resolution;         // float32
    map_prob.info.width      = map_width_inc;           // uint32
    map_prob.info.height     = map_height_inc;           // uint32
    map_prob.info.origin.orientation.x = 0.0;
    map_prob.info.origin.orientation.y = 0.0;
    map_prob.info.origin.orientation.z = 0.0;
    map_prob.info.origin.orientation.w = 1.0;
    map_prob.info.origin.position.x = -width/2.0;
    map_prob.info.origin.position.y = -height/2.0;
    map_prob.info.origin.position.z = -1.2;


    map_prob.data.resize(map_prob.info.width * map_prob.info.height);
    for (int i=0; i<map_prob.info.width * map_prob.info.height; i++)
    {
      map_prob.data[i] = 0;  // 均为可通行状态
    }

    map_init = false;
    std::cout<<"地图初始化完成"<<std::endl;
    bayes_image = bayes_update(P_prior)*cv::Mat::ones(cv::Size(map_width_inc, map_height_inc), CV_32F);
    std::cout<<"bayes_image.at<float>(5,5)  "<<bayes_image.at<float>(5,5)<<std::endl;
    std::cout<<"typeid(bayes_image.at<float>(5,5)).name()   "<<typeid(bayes_image.at<float>(5,5)).name()<<std::endl;    
  
    // cv::namedWindow("bayes_image_0", 0);
    // cv::imshow("bayes_image_0", bayes_image);
    // cv::waitKey(1);
  }
}