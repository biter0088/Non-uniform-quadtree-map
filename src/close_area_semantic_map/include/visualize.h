// 可视化关键点----------------------------------------------

// 将单个关键点加入到marker中
void add_single_keypoint(float x_index,float y_index,ros::Time rostime,std_msgs::ColorRGBA color_tmp){
    keypoint_marker.header.frame_id = "/map";
    keypoint_marker.header.stamp = rostime;
    keypoint_marker.ns = "points";
    keypoint_marker.id = marker_id;//int32
    marker_id++;
    // Set the keypoint_marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    keypoint_marker.type = visualization_msgs::Marker::SPHERE;

    keypoint_marker.action = visualization_msgs::Marker::ADD;
    keypoint_marker.pose.position.x = x_index*resolution;//float64
    keypoint_marker.pose.position.y = y_index*resolution;
    keypoint_marker.pose.position.z = 1;
    keypoint_marker.pose.orientation.x = 0.0;
    keypoint_marker.pose.orientation.y = 0.0;
    keypoint_marker.pose.orientation.z = 0.0;
    keypoint_marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // keypoint_marker.scale.x = 0.02;
    // keypoint_marker.scale.y = 0.02;
    // keypoint_marker.scale.z = 0.02;

    keypoint_marker.scale.x = 0.1;
    keypoint_marker.scale.y = 0.1;
    keypoint_marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    // keypoint_marker.color.r = 1.0f;
    // keypoint_marker.color.g = 0.0f;
    // keypoint_marker.color.b = 0.0f;
    // keypoint_marker.color.a = 1.0;
    keypoint_marker.color=color_tmp;

    keypoint_marker.lifetime = ros::Duration();

    keypoints_markerArray.markers.push_back(keypoint_marker);
}


// 可视化关键点
void visualize_keypoints(){
    ros::Time rostime=ros::Time::now();
    std_msgs::ColorRGBA color_1;//颜色
    color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=1;//红色+不透明度1
    for(int i=0;keypoints[2*i]!=0;i++){
        add_single_keypoint(float(keypoints[2*i]),float(keypoints[2*i+1]),rostime,color_1);
    }
}

// 可视化包围空间边界
void visualize_sample_points(){
    ros::Time rostime=ros::Time::now();
    std_msgs::ColorRGBA color_1;//颜色
    color_1.r = 0;   color_1.g = 0;   color_1.b = 1;   color_1.a=1;//红色+不透明度1
    for(int i=0;(sample_points[i][0]!=0)&&(sample_points[i][1]!=0);i++){
        add_single_keypoint(float(sample_points[i][0]),float(sample_points[i][1]),rostime,color_1);
    }
}


