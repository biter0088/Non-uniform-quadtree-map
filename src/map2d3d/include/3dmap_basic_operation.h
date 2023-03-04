/*
  初始化3d栅格地图
*/
void map_init_function_3d(){
  if (map_init_3d)
  {
    robot_x0 = odomVec[odomVec.size()-1].pose.pose.position.x;//开始建立3d地图时车辆位置
    robot_y0 = odomVec[odomVec.size()-1].pose.pose.position.y;
    robot_z0 = odomVec[odomVec.size()-1].pose.pose.position.z;//是否转化为轮子与地面接触位置的高度会好一些？？？
    std::cout<<"robot_x0:  "<<robot_x0<<"robot_y0:  "<<robot_y0<<"robot_z0:  "<<robot_z0<<std::endl;
    width_3d=100;
    length_3d=100;
    height_3d=20;
    map_init_3d=false;
    // int map_3d_width_inc = round(width_3d / resolution_xy);//3d地图各方向栅格数
    // int map_3d_length_inc = round(length_3d / resolution_xy);
    // int map_3d_height_inc = round(height_3d / resolution_z);  
  }
}

/*
    将全局坐标系下的单帧点云存放到栅格gridMap_3d_data中
*/
void cloud2gridmap_3d_data()
{
  for (int i=0; i<cloudInWorld->points.size(); i++)
  {
    pcl::PointXYZ thisPoint = cloudInWorld->points[i];
    // int x_cor = floor((width_3d*0.5+cloudInWorld->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    // int y_cor = floor((length_3d*0.5+cloudInWorld->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    // int z_cor = floor((height_3d*0.5+cloudInWorld->points[i].z - robot_z0) / resolution_z) ;
    int x_cor = floor((cloudInWorld->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    int y_cor = floor((cloudInWorld->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    int z_cor = floor((cloudInWorld->points[i].z - robot_z0) / resolution_z) ;
    std::tuple<int,int,int> xyz_index =std::make_tuple(x_cor, y_cor,z_cor);
    // 循环查找
    if(gridMap_3d_data.count(xyz_index)>0)
    {
      gridMap_3d_data[xyz_index].push_back(thisPoint);      // 找到了
    }
    else {
      // 新的栅格
      std::vector<pcl::PointXYZ> newGrid;
      newGrid.push_back(thisPoint);
      gridMap_3d_data.insert({xyz_index,  newGrid });
    }
  }
}




//改变颜色，使颜色有尽可能多的种类
std_msgs::ColorRGBA change_color(){
  std_msgs::ColorRGBA color_my;//颜色
  int case_order=abs(color_order)%color_num;//color_num为颜色总的种类，根据下面case总数进行修改
  switch (case_order) {
    case 0:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 0;   color_my.a=0.3;//红色+不透明度0.3
      // std::cout<<"红色+不透明度0.3"<<std::endl;
      break;
    case 1:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 0;   color_my.a=0.6;//红色+不透明度0.6
      break;
    case 2:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 0;   color_my.a=1;//红色+不透明度1
      break;

    case 3:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 0;   color_my.a=0.3;//绿色+不透明度0.3
      break;
    case 4:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 0;   color_my.a=0.6;//绿色+不透明度0.6
      break;
    case 5:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 0;   color_my.a=1;//绿色+不透明度1
      break;

    case 6:
      color_my.r = 0;   color_my.g = 0;   color_my.b = 1;   color_my.a=0.3;//蓝色+不透明度0.3
      break;
    case 7:
      color_my.r = 0;   color_my.g = 0;   color_my.b = 1;   color_my.a=0.6;//蓝色+不透明度0.6
      break;
    case 8:
      color_my.r = 0;   color_my.g = 0;   color_my.b = 1;   color_my.a=1;//蓝色+不透明度1
      break;

    case 9:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 1;   color_my.a=0.3;//紫色+不透明度0.3
      break;
    case 10:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 1;   color_my.a=0.6;//紫色+不透明度0.6
      break;
    case 11:
      color_my.r = 1;   color_my.g = 0;   color_my.b = 1;   color_my.a=1;//紫色+不透明度1
      break;

    case 12:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 1;   color_my.a=0.3;//青色+不透明度0.3
      break;
    case 13:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 1;   color_my.a=0.6;//青色+不透明度0.6
      break;
    case 14:
      color_my.r = 0;   color_my.g = 1;   color_my.b = 1;   color_my.a=1;//青色+不透明度1
      break;
  }
  return color_my;
}

// 按高度进行着色
std_msgs::ColorRGBA heightMapColor(double h) {

  std_msgs::ColorRGBA color;//std_msgs::ColorRGBA包含: r、g、b、a四个参数
  color.a = 1.0;
  // blend over HSV-values (more colors) 混合HSV值（更多颜色）

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);// 向下取整
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;//这里rgb的值均在0-100%之间
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}



/*
  将非空闲的3d栅格显示出来
*/
void add_marker(const ros::Time& rostime,int id,float x_marker,float y_marker,float z_marker){
    add_marker_lock.lock();
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/velo_link";
    marker.header.stamp = rostime;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "global_Map_3D";
    marker.id = id;//int32

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    // ROS_WARN("marker at rostime be add");
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_marker;//float64
    marker.pose.position.y = y_marker;
    marker.pose.position.z = z_marker;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = resolution_xy;
    marker.scale.y = resolution_xy;
    marker.scale.z = resolution_z;

    // Set the color -- be sure to set alpha to something non-zero!
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    // marker.color.a = 1.0;
    // marker.color=heightMapColor(z_marker);//按高度进行着色
    marker.color=change_color();//按列表进行着色
    // marker.color=heightMapColor(sqrt(x_marker*x_marker+y_marker*y_marker+z_marker*z_marker));//按到原点的距离进行着色

    // marker.lifetime = ros::Duration();
    marker.lifetime = ros::Duration(0.2);
    if(laserVec.size()==98){//保留最后一帧进行显示，用于观察
      marker.lifetime = ros::Duration();
    }
    markerArray.markers.push_back(marker);
    // Publish the marker
    add_marker_lock.unlock();
}

/*
    判断3d栅格中数据点的数目，并根据数目不同选择visualization_msgs::Marker的颜色
*/
void gridmap2map_3d_data(){
  pub_marker_lock.lock();
  //kv.first代表(x,y,z)坐标对，kv.second代表一个坐标对里的值
  int id=0;//marker的id编号
  for (auto &kv : gridMap_3d_data) {//将坐标对中心和坐标对里面点数目传入到显示函数
    float x_center,y_center,z_center;
    x_center=(std::get<0>(kv.first)+0.5)*resolution_xy;
    y_center=(std::get<1>(kv.first)+0.5)*resolution_xy;
    z_center=(std::get<2>(kv.first)+0.5)*resolution_z;
    color_order=std::get<2>(kv.first);
    //之后可以根据3d小栅格中点数量进行分级
    add_marker(rostime,id,x_center,y_center,z_center);
    id++;
  }
  // marker_pub.publish(markerArray);
  std::cout<<"当前帧点云对应的3d栅格数 id:  "<<id<<std::endl;
  pub_marker_lock.unlock();
}

/*
    将全局坐标系下的点云分别存在不同的小三维栅格gridMap_3d中
*/
void cloud2gridmap_3d_num()
{
  for (int i=0; i<cloudInWorld->points.size(); i++)
  {
    pcl::PointXYZ thisPoint = cloudInWorld->points[i];
    // int x_cor = floor((width_3d*0.5+cloudInWorld->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    // int y_cor = floor((length_3d*0.5+cloudInWorld->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    // int z_cor = floor((height_3d*0.5+cloudInWorld->points[i].z - robot_z0) / resolution_z) ;
    int x_cor = floor((cloudInWorld->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    int y_cor = floor((cloudInWorld->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    int z_cor = floor((cloudInWorld->points[i].z - robot_z0) / resolution_z) ;
    std::tuple<int,int,int> xyzid_index =std::make_tuple(x_cor, y_cor,z_cor);
    // 循环查找
    if(gridMap_3d_num_id.count(xyzid_index)>0) // 找到了
    {
      int point_num_in_gridmap=gridMap_3d_num_id[xyzid_index].back();//这个位置存放的是小栅格中点的数量
      point_num_in_gridmap++;
      gridMap_3d_num_id[xyzid_index].pop_back();  
      gridMap_3d_num_id[xyzid_index].push_back(point_num_in_gridmap);   
    }
    else {// 新的栅格
      std::vector<int> newGrid;
      newGrid.push_back(id_num);
      newGrid.push_back(1);//新的栅格初始化时里面只有一个点
      gridMap_3d_num_id.insert({xyzid_index,  newGrid });
      id_num++;
    }
  }
}

/*
    判断3d栅格中数据点的数目，如果数目不为0将其视为占用
*/
void gridmap2map_3d_num(){
  int id;//marker唯一编号
  //kv.first代表(x,y,z)坐标对，kv.second代表一个坐标对里的值
  for (auto &kv : gridMap_3d_num_id) {//将坐标对中心和坐标对里面点数目传入到显示函数
    float x_center,y_center,z_center;
    x_center=(std::get<0>(kv.first)+0.5)*resolution_xy;
    y_center=(std::get<1>(kv.first)+0.5)*resolution_xy;
    z_center=(std::get<2>(kv.first)+0.5)*resolution_z;
    color_order=std::get<2>(kv.first);
    id=kv.second.front();
    //之后可以根据3d小栅格中点数量进行分级
    add_marker(rostime,id,x_center,y_center,z_center);
  }
  // marker_pub.publish(markerArray);
  std::cout<<"当前帧点云对应的3d栅格数 id:  "<<id<<std::endl;
}

/////////////////////  globalmap3d_only_mapping.cpp 程序需要的建图函数    ------起始


/*
  初始化3d栅格地图
*/
void map_init_function_3d_only_map(){
  if (map_init_3d)
  {
    robot_x0 = 0;//只有一帧点云，车辆就在点云局部坐标系原点
    robot_y0 = 0;
    robot_z0 = 0;//是否转化为轮子与地面接触位置的高度会好一些？？？
    std::cout<<"robot_x0:  "<<robot_x0<<"robot_y0:  "<<robot_y0<<"robot_z0:  "<<robot_z0<<std::endl;
    width_3d=100;
    length_3d=100;
    height_3d=20;
    map_init_3d=false;
    // int map_3d_width_inc = round(width_3d / resolution_xy);//3d地图各方向栅格数
    // int map_3d_length_inc = round(length_3d / resolution_xy);
    // int map_3d_height_inc = round(height_3d / resolution_z);  
  }
}

/*
    将全局坐标系下的点云分别存在不同的小三维栅格gridMap_3d中
*/
void cloud2gridmap_3d_num_only_map()
{
  std::cout<<"进入到函数 cloud2gridmap_3d_num_only_map 中"<<std::endl;
  for (int i=0; i<cloud_for_mapping->points.size(); i++)
  {
    pcl::PointXYZ thisPoint = cloud_for_mapping->points[i];
    // int x_cor = floor((width_3d*0.5+cloudInWorld->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    // int y_cor = floor((length_3d*0.5+cloudInWorld->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    // int z_cor = floor((height_3d*0.5+cloudInWorld->points[i].z - robot_z0) / resolution_z) ;
    int x_cor = floor((cloud_for_mapping->points[i].x - robot_x0) / resolution_xy) ;// 计算每个点所在坐标（可以理解为栅格）
    int y_cor = floor((cloud_for_mapping->points[i].y - robot_y0) / resolution_xy) ;//floor向下取整,意思是xyz_index为小栅格的左下角
    int z_cor = floor((cloud_for_mapping->points[i].z - robot_z0) / resolution_z) ;
    std::tuple<int,int,int> xyzid_index =std::make_tuple(x_cor, y_cor,z_cor);
    // 循环查找
    if(gridMap_3d_num_id.count(xyzid_index)>0) // 找到了
    {
      int point_num_in_gridmap=gridMap_3d_num_id[xyzid_index].back();//这个位置存放的是小栅格中点的数量
      point_num_in_gridmap++;
      gridMap_3d_num_id[xyzid_index].pop_back();  
      gridMap_3d_num_id[xyzid_index].push_back(point_num_in_gridmap);   
    }
    else {// 新的栅格
      std::vector<int> newGrid;
      newGrid.push_back(id_num);
      newGrid.push_back(1);//新的栅格初始化时里面只有一个点
      gridMap_3d_num_id.insert({xyzid_index,  newGrid });
      id_num++;
    }
  }
}

/*
    判断3d栅格中数据点的数目，如果数目不为0将其视为占用
*/
void gridmap2map_3d_num_only_map(){
  std::cout<<"进入到函数 gridmap2map_3d_num_only_map 中"<<std::endl;
  int id;//marker唯一编号
  //kv.first代表(x,y,z)坐标对，kv.second代表一个坐标对里的值
  for (auto &kv : gridMap_3d_num_id) {//将坐标对中心和坐标对里面点数目传入到显示函数
    float x_center,y_center,z_center;
    x_center=(std::get<0>(kv.first)+0.5)*resolution_xy;
    y_center=(std::get<1>(kv.first)+0.5)*resolution_xy;
    z_center=(std::get<2>(kv.first)+0.5)*resolution_z;
    color_order=std::get<2>(kv.first);
    id=kv.second.front();
    //之后可以根据3d小栅格中点数量进行分级
    add_marker(rostime,id,x_center,y_center,z_center);
  }
  // marker_pub.publish(markerArray);
  std::cout<<"当前帧点云对应的3d栅格数 id:  "<<id<<std::endl;
}


/////////////////////  globalmap3d_only_mapping.cpp 程序需要的建图函数   --------截止