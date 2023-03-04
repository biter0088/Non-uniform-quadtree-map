// 外推
void extrapolatePose(double exceedDuration)
{
  // 获取最近两帧里程计：
  nav_msgs::Odometry odom1 = odomVec[odomVec.size()-2];
  nav_msgs::Odometry odom2 = odomVec[odomVec.size()-1];
  Eigen::Quaterniond q_odom_curr1;
	Eigen::Vector3d t_odom_curr1;
	q_odom_curr1.x() = odom1.pose.pose.orientation.x;
	q_odom_curr1.y() =  odom1.pose.pose.orientation.y;
	q_odom_curr1.z() =  odom1.pose.pose.orientation.z;
	q_odom_curr1.w() =  odom1.pose.pose.orientation.w;
	t_odom_curr1.x() = odom1.pose.pose.position.x;
	t_odom_curr1.y() =  odom1.pose.pose.position.y;
	t_odom_curr1.z() =  odom1.pose.pose.position.z;

  Eigen::Quaterniond q_odom_curr2;
	Eigen::Vector3d t_odom_curr2;
	q_odom_curr2.x() = odom2.pose.pose.orientation.x;
	q_odom_curr2.y() =  odom2.pose.pose.orientation.y;
	q_odom_curr2.z() =  odom2.pose.pose.orientation.z;
	q_odom_curr2.w() =  odom2.pose.pose.orientation.w;
	t_odom_curr2.x() = odom2.pose.pose.position.x;
	t_odom_curr2.y() =  odom2.pose.pose.position.y;
	t_odom_curr2.z() =  odom2.pose.pose.position.z;

  // 平移部分线性外推：
  double interDuration = odom2.header.stamp.toSec() - odom1.header.stamp.toSec();
  double vtx = 	(t_odom_curr2.x() - 	t_odom_curr1.x()) / interDuration;
  double vty = 	(t_odom_curr2.y() - 	t_odom_curr1.y()) / interDuration;
  double vtz = 	(t_odom_curr2.z() - 	t_odom_curr1.z()) / interDuration;
  t_odom_curr_now.x() = t_odom_curr2.x() + vtx * exceedDuration;
	t_odom_curr_now.y() =  t_odom_curr2.y() + vty * exceedDuration;
	t_odom_curr_now.z() =  t_odom_curr2.z() + vtz * exceedDuration;
  // 旋转部分球面插值外推
  double s = (exceedDuration + interDuration) / interDuration;
  q_odom_curr_now = q_odom_curr1.slerp(s, q_odom_curr2);  // 这一帧起点到当前点的位姿增量
}

// 内插
void interpolatePose(nav_msgs::Odometry odom1,nav_msgs::Odometry odom2,double  lasertime)
{

  Eigen::Quaterniond q_odom_curr1;
	Eigen::Vector3d t_odom_curr1;
	q_odom_curr1.x() = odom1.pose.pose.orientation.x;
	q_odom_curr1.y() =  odom1.pose.pose.orientation.y;
	q_odom_curr1.z() =  odom1.pose.pose.orientation.z;
	q_odom_curr1.w() =  odom1.pose.pose.orientation.w;
	t_odom_curr1.x() = odom1.pose.pose.position.x;
	t_odom_curr1.y() =  odom1.pose.pose.position.y;
	t_odom_curr1.z() =  odom1.pose.pose.position.z;

  Eigen::Quaterniond q_odom_curr2;
	Eigen::Vector3d t_odom_curr2;
	q_odom_curr2.x() = odom2.pose.pose.orientation.x;
	q_odom_curr2.y() =  odom2.pose.pose.orientation.y;
	q_odom_curr2.z() =  odom2.pose.pose.orientation.z;
	q_odom_curr2.w() =  odom2.pose.pose.orientation.w;
	t_odom_curr2.x() = odom2.pose.pose.position.x;
	t_odom_curr2.y() =  odom2.pose.pose.position.y;
	t_odom_curr2.z() =  odom2.pose.pose.position.z;


  // 平移部分线性插值
  double exceedDuration = lasertime -  odom1.header.stamp.toSec();
  double interDuration = odom2.header.stamp.toSec() - odom1.header.stamp.toSec();
  double vtx = 	(t_odom_curr2.x() - 	t_odom_curr1.x()) / interDuration;
  double vty = 	(t_odom_curr2.y() - 	t_odom_curr1.y()) / interDuration;
  double vtz = 	(t_odom_curr2.z() - 	t_odom_curr1.z()) / interDuration;
  t_odom_curr_now.x() = t_odom_curr1.x() + vtx * exceedDuration;
	t_odom_curr_now.y() =  t_odom_curr1.y() + vty * exceedDuration;
	t_odom_curr_now.z() =  t_odom_curr1.z() + vtz * exceedDuration;
  // 旋转部分球面插值
  double s = exceedDuration / interDuration;
  q_odom_curr_now = q_odom_curr1.slerp(s, q_odom_curr2);  // 这一帧起点到当前点的位姿增量

}
