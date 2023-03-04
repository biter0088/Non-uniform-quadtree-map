#pragma once

#include <ros/ros.h>
 #include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher  pubGridMap;
double width;
double height;
double differ_height;
double max_height;
double min_height;
double max_width;
double min_width;
double max_len;
double min_len;
double resolution;
double min_differ_height;
geometry_msgs::Point start_point;
int obs_prob;
int scale;

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
