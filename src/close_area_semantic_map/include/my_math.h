double getyaw(Eigen::Quaterniond qua){//计算偏航角
  tf::Quaternion q(qua.x(),
    qua.y(),
    qua.z(),
    qua.w());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw*180.0/M_PI;
}

//计算坐标索引，输入参数：点的坐标
int * compute_point_index(double point[]){
  static int point_index[2];
  point_index[0]=(point[0]-map_origin[0])/resolution;//注意这里可能有略微超出地图边界的索引，比如-0.5可能会被取值为-1
  point_index[1]=(point[1]-map_origin[1])/resolution;
  return point_index;
}

//计算两点之间的距离
double getLength(double x1,double y1,double x2,double y2){
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}