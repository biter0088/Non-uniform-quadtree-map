#include "geometry_msgs/Quaternion.h"

double getpitch(geometry_msgs::Quaternion qua){
  tf::Quaternion q(qua.x,
    qua.y,
    qua.z,
    qua.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return pitch;
}


double getyaw(Eigen::Quaterniond qua){
  tf::Quaternion q(qua.x(),
    qua.y(),
    qua.z(),
    qua.w());
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw*180.0/M_PI;
}

double getyaw2(geometry_msgs::Quaternion qua){
  tf::Quaternion q(qua.x,
    qua.y,
    qua.z,
    qua.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}