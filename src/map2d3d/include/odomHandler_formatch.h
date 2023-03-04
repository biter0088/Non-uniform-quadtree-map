#include "lonlat2utm.h"//经纬度转utm坐标系

/*
  结合gps和imu数据生成odom数据
*/
void odomHandler()
{
  // odomlock1.lock();
  // std::cout<<"开始结合gps和imu数据生成odom数据"<<std::endl;
  if (odomVec.size() > 9000)
  {
    odomVec.pop_front();
  }
  if((gpsVec.size()>=1)&&(imuVec.size()>=1)&&(map_info_write==0)){
    // outfile.open(path, ios::out | ios::binary);
    mapheader_tmp.latitude=gpsVec[0].latitude;
    mapheader_tmp.longitude=gpsVec[0].longitude;
    // std::cout<<"mapheader_tmp.latitude: "<<setprecision(40)<<mapheader_tmp.latitude<<",   "<<"mapheader_tmp.longitude: "<<setprecision(40)<<mapheader_tmp.longitude<<"."<<std::endl;
    std::cout<<"mapheader_tmp.latitude: "<<mapheader_tmp.latitude<<",   "<<"mapheader_tmp.longitude: "<<mapheader_tmp.longitude<<"."<<std::endl;
    mapheader_tmp.qx=imuVec[0].orientation.x;
    mapheader_tmp.qy=imuVec[0].orientation.y;
    mapheader_tmp.qz=imuVec[0].orientation.z;
    mapheader_tmp.qw=imuVec[0].orientation.w;
    mapheader_tmp.x_offset=x_offset_total;
    mapheader_tmp.y_offset=y_offset_total;
    // outfile.write((char*)&mapheader_tmp, sizeof(mapheader_tmp));
    // outfile.close();
    map_info_write=1;
  }
  if((gpsVec.size()>=1)&&(imuVec.size()>=1)){
    nav_msgs::Odometry odom_tmp;
    odom_tmp.header.frame_id = gpsVec[gpsVec.size()-1].header.frame_id;
    // odom_tmp.child_frame_id ="";// /velo_link
    odom_tmp.child_frame_id ="/velo_link";// 
    odom_tmp.header.stamp = gpsVec[gpsVec.size()-1].header.stamp;
    double latitude=gpsVec[gpsVec.size()-1].latitude;
    double longitude=gpsVec[gpsVec.size()-1].longitude;   
    LonLat2UTM(longitude,  latitude,  UTME,  UTMN);
    if (gpsVec.size()==1){
      UTME0=UTME;
      UTMN0=UTMN;     
      car_x0 = UTME-UTME0;
      car_y0 = UTMN-UTMN0;           
    }
    car_x = UTME-UTME0;//根据点云播放效果，x方向应该是车辆前进方向
    car_y = UTMN-UTMN0;//y方向应该是车辆右侧
    //当前位姿相对初始位姿的差值
    t_odom_curr_now.x() = car_x;
  	t_odom_curr_now.y() = car_y;
  	t_odom_curr_now.z() = 0;
    //以初始位姿基准，其他时刻位姿与初始时刻相比
    Eigen::Matrix3d R_odom_curr0 ;
    // if(imuVec.size()==1){
    //   q_odom_curr_now.x() = 0;
	  //   q_odom_curr_now.y() = 0;
    // 	q_odom_curr_now.z() = 0;
	  //   q_odom_curr_now.w() =1;
    // }
    if(imuVec.size()>=1){
      // cout<<"111------"<<endl;
      // Eigen::Quaterniond q_odom_curr0;
	    // q_odom_curr0.x() = imuVec[0].orientation.x;//计算初始姿态--四元数的逆
	    // q_odom_curr0.y() = imuVec[0].orientation.y;
    	// q_odom_curr0.z() = imuVec[0].orientation.z;
	    // q_odom_curr0.w() =imuVec[0].orientation.w;

      Eigen::Quaterniond q_odom_curr_tmp;
	    q_odom_curr_tmp.x() = imuVec[imuVec.size()-1].orientation.x;
	    q_odom_curr_tmp.y() = imuVec[imuVec.size()-1].orientation.y;
    	q_odom_curr_tmp.z() = imuVec[imuVec.size()-1].orientation.z;
	    q_odom_curr_tmp.w() =imuVec[imuVec.size()-1].orientation.w;
      q_odom_curr_now=q_odom_curr_tmp;

      /*---------------使用变换矩阵将点云转换到第一帧的车载坐标系下：失败
      Eigen::Isometry3d T_1=Eigen::Isometry3d::Identity(); 
      T_1.rotate ( q_odom_curr0.toRotationMatrix() );  
      T_1.pretranslate ( Eigen::Vector3d ( 0,0,0 ) ); 

      Eigen::Isometry3d T_2=Eigen::Isometry3d::Identity(); 
      T_2.rotate ( q_odom_curr_now.toRotationMatrix() );  
      T_2.pretranslate ( Eigen::Vector3d ( car_x,-car_y,0 ) ); 

      Eigen::Isometry3d T_delta=Eigen::Isometry3d::Identity(); 
      T_delta=T_1*T_2.inverse();
      
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix = T_delta.rotation();
      t_odom_curr_now = T_delta.translation();
      q_odom_curr_now = Eigen::Quaterniond ( rotation_matrix );
      ------------------------------------------*/ 

      // cout<<"222------"<<endl;
    }


    // odom_tmp.pose.pose.position.x=car_x+ width/2.0;
    // odom_tmp.pose.pose.position.y=car_y+ height/2.0;
    odom_tmp.pose.pose.position.x=car_x;
    odom_tmp.pose.pose.position.y=car_y;
    odom_tmp.pose.pose.position.z=0;
    odom_tmp.pose.pose.orientation.x=imuVec[imuVec.size()-1].orientation.x;
    odom_tmp.pose.pose.orientation.y=imuVec[imuVec.size()-1].orientation.y;
    odom_tmp.pose.pose.orientation.z=imuVec[imuVec.size()-1].orientation.z;
    odom_tmp.pose.pose.orientation.w=imuVec[imuVec.size()-1].orientation.w;
    // std::cout<<"开始读入odom数据"<<std::endl;
    odomVec.push_back(odom_tmp);
    // std::cout<<"odomVec.size() "<<odomVec.size()<<std::endl;
    pubCarpositiopn.publish(odom_tmp);//当前车辆(雷达)在栅格地图中的位置


    // std::cout<<"相对旋转和相对平移"<<std::endl;
    // std::cout<<"q_odom_curr_now:"<< q_odom_curr_now.x()   <<q_odom_curr_now.y()   <<q_odom_curr_now.z()   <<q_odom_curr_now.w()   <<std::endl;
    // std::cout<<"t_odom_curr_now:"<<"x:"<< t_odom_curr_now.x()     <<"y:"<<t_odom_curr_now.y()     <<"z:"<<t_odom_curr_now.z()     <<std::endl;    
  }


  // std::cout << "同步完成！"<< std::endl;
  // odomlock1.unlock();
}



/*
    滤除俯仰角过大的里程计信息
*/
void bad_odom_filter(){
  if(odomVec.size()>5)
	{ //pitch俯仰角过大，放弃这样的跳跃式雷达数据
	  if(fabs(getpitch(odomVec[odomVec.size()-1].pose.pose.orientation) - getpitch(odomVec[odomVec.size()-3].pose.pose.orientation)) >= 0.2)	{     
      ros::Duration(2.0).sleep();
		  return;
		}
	}
}