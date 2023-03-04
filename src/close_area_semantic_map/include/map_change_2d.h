
// 将单个采样点对应的高度空间加入到marker中
void add_single_samplepoint_3d(float x_index,float y_index,ros::Time rostime,std_msgs::ColorRGBA color_tmp, float h){
    visualization_msgs::Marker samplepoint_marker;//单个采样点
    samplepoint_marker.header.frame_id = "/camera_init";
    samplepoint_marker.header.stamp = rostime;
    samplepoint_marker.ns = "samplepoints";
    samplepoint_marker.id = samplemarker_id;//int32
    samplemarker_id++;
    // Set the keypoint_marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    samplepoint_marker.type = visualization_msgs::Marker::SPHERE;

    samplepoint_marker.action = visualization_msgs::Marker::ADD;
    samplepoint_marker.pose.position.x = x_index*resolution-7.0;//float64
    samplepoint_marker.pose.position.y = y_index*resolution-10.0;
    samplepoint_marker.pose.position.z = h;
    samplepoint_marker.pose.orientation.x = 0.0;
    samplepoint_marker.pose.orientation.y = 0.0;
    samplepoint_marker.pose.orientation.z = 0.0;
    samplepoint_marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    samplepoint_marker.scale.x = 0.05;
    samplepoint_marker.scale.y = 0.05;
    samplepoint_marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    // keypoint_marker.color.r = 1.0f;
    // keypoint_marker.color.g = 0.0f;
    // keypoint_marker.color.b = 0.0f;
    // keypoint_marker.color.a = 1.0;
    samplepoint_marker.color=color_tmp;

    samplepoint_marker.lifetime = ros::Duration();

    samplepoints_markerArray.markers.push_back(samplepoint_marker);
}


/*
    imput:
        关键点的坐标索引（类型转化为float类型）
    功能：
        1：求出一个采样点与最近占据栅格的距离
        2：对采样点对应的栅格属性重新赋值，属性值与距离成反比
*/
void change_one_samplepoint(int x,int y){
    float step=resolution/2.0;
    float radius=0;
    float x_tmp,y_tmp;
    int getminradius=0;//是否找到最短半径
    // 确定采样点是否在占据栅格上，理论上不会在占据栅格上，但是由于采样间距比较小，可能某个采样点对应在某个占据栅格范围内-------------
    int index_tmp[2];
    index_tmp[0]=x;  index_tmp[1]=y;    
    if(((index_tmp[0]<0)||(index_tmp[0]>map_width_inc-1)||(index_tmp[1]<0)||(index_tmp[1]>map_height_inc-1))|| \
            map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]==100 ){
            // ||            map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]==-1){
        cout<<"采样点是占据栅格或地图边界"<<endl;
        return ;
    }
    // 找到最小半径
    for(int i=0;;i++){
        radius=i*step;
        for(int angle=0;angle<360;angle++){//遍历一个点周围半径为 radius的圆
            x_tmp=x+radius*cos(angle*1.0/180.0*M_PI);
            y_tmp=y+radius*sin(angle*1.0/180.0*M_PI);
            int xy_tmp[2];
            xy_tmp[0]=int(x_tmp);  xy_tmp[1]=int(y_tmp);    

            if((xy_tmp[0]<0)||(xy_tmp[0]>map_width_inc-1)||(xy_tmp[1]<0)||(xy_tmp[1]>map_height_inc-1)){
                // cout<<"搜索到地图边界"<<endl;
                // getminradius=1;
                continue;//0902   避免死角区域在地图边界附近，导致角度没有搜索完全即没有找到最小半径
                // break;
            }
            if( map_.data[xy_tmp[0]+xy_tmp[1]*map_.info.width]==100 ){
                // ||map_.data[xy_tmp[0]+xy_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
                // cout<<"搜索到占据栅格"<<endl;
                getminradius=1;
                break;
            }
        }
        if(getminradius){
            break;
        }
    }

    // 更改地图属性值：100/radius
    // map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]=100/radius;//与距离成反比
    // std::cout<<"radius:  "<<radius<<endl;
    //随着距离增加而递减  radius最大为range_threshold/0.05=30，根据当前地图种子点设计，最大值应该为20
    // 0~20范围内：
        // 1:  0~5  k1=-13.6
        // 2:  5~10  k2=-8
        // 3:  10~20  k3=-2
    float k1=16;
    float k2=6;
    float k3=1.8;
    float data_tmp;
    // if(radius>=0 && radius<=5){
    //     data_tmp=-128+k1*radius;
    // }
    // else if(radius>5 && radius<=10){
    //     data_tmp=-60+k2*(radius-5);
    // }
    // else if(radius>10){
    //     data_tmp=-20+k3*(radius-10);
    // }
    // 或者
    // data_tmp=-48+2.4*radius;

        std_msgs::ColorRGBA color_1;//颜色
        /*if(radius<=5){
            data_tmp=-128+16*radius;
            // color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=1;//红色+不透明度1
            color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=1;//红色+不透明度1
        }
        else if(radius<=10){
            data_tmp=-48+3.6*(radius-5);      
            //  color_1.r = 1;   color_1.g = 1;   color_1.b = 0;   color_1.a=1;//黄色+不透明度1 
            color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=0.4;//红色+不透明度1
        }
        else if(radius<=15){
            data_tmp=-30+2*(radius-10);       
            //  color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=0.5;//红色+不透明度0.5
            color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=0.1;//红色+不透明度1
        }
        else if(radius<=30){//30=range_threshold/resolution即死角区域最远点栅格距离
            data_tmp=-20+1*(radius-15);       
            //  color_1.r = 1;   color_1.g = 1;   color_1.b = 0;   color_1.a=0.5;//黄色+不透明度0.5
            color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=0.04;//红色+不透明度1
        }*/

/*      //0901添加
        if(radius<=1){
            data_tmp=-95;
        }
        else if(radius<=2){
            data_tmp=-60;
        }
        else if(radius<=3){
            data_tmp=-30;  
        }
        else if(radius<=5){
            data_tmp=-20;   
        }
        else {
            data_tmp=-10;   
        }
*/  
        //0913添加   仅仅为了显示调试
        if(radius<=1){
            data_tmp=-128;
        }
        else if(radius<=2){
            data_tmp=-128;
        }
        else if(radius<=3){
            data_tmp=-120;  
        }
        else if(radius<=5){
            data_tmp=-70;   
        }
        else {
            data_tmp=-10;   
        }

    if(0){//是否可视化三维采样点
        ros::Time rostime=ros::Time::now();
        float h=0;
        h=(x*resolution-7.0)*0.05-0.3;
        for(int i=0;i<5;i++){//3代表死角边界高度，这里没有计算，稍后计算
            h=h+0.10;
            add_single_samplepoint_3d(float(x),float(y),rostime,color_1,h);
        }
    }
    // data_tmp=-48+5*radius;
    map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]=data_tmp;
}


// 在一个有效采样点正上方添加一系列maker
void show_one_3d_samplepoint(int x,int y){
    // std::cout<<"在有效采样点上方添加三维采样点"<<std::endl;
    ros::Time rostime=ros::Time::now();
    std_msgs::ColorRGBA color_1;//颜色
    color_1.r = 1;   color_1.g = 0;   color_1.b = 0;   color_1.a=1;//红色+不透明度1
    for(int i=0;i<3;i++){//3代表死角边界高度，这里没有计算，稍后计算
        float h;
        h=-0.2+i*0.5;
        add_single_samplepoint_3d(float(x),float(y),rostime,color_1,h);
    }
}

/*
    功能：
        1：将采样点对应的二维地图栅格选出来，
        2：求出采样点距离占据栅格的最近距离，属性值与距离成反比(或者用100-距离)，距离越近其栅格属性值越高，以占据的100为上限；

        1：三维语义死角区域构建--在选出的二维栅格上方堆放maker
*/
void mapchange_with_samplepoints(){
    for(int i=0;(valid_sample_points[i][0]!=0)&&(valid_sample_points[i][1]!=0);i++){
        change_one_samplepoint(valid_sample_points[i][0],valid_sample_points[i][1]);
        // show_one_3d_samplepoint(valid_sample_points[i][0],valid_sample_points[i][1]);
    }
    // std::cout<<"三维采样点有"<<samplemarker_id<<"个."<<std::endl;

}