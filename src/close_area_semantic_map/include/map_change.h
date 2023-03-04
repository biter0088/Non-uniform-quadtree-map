/*
    imput:
        关键点的坐标索引（类型转化为float类型）
    功能：
        1：求出一个采样点与最近占据栅格的距离
        2：对采样点对应的栅格属性重新赋值，属性值与距离成反比
*/
void change_one_samplepoint(int x,int y){
    float step=resolution/2.0;
    float radius;
    float x_tmp,y_tmp;
    int getminradius=0;//是否找到最短半径
    // 确定采样点是否在占据栅格上，理论上不会在占据栅格上，但是由于采样间距比较小，可能某个采样点对应在某个占据栅格范围内-------------
    int index_tmp[2];
    index_tmp[0]=x;  index_tmp[1]=y;    
    if(((index_tmp[0]<0)||(index_tmp[0]>map_width_inc-1)||(index_tmp[1]<0)||(index_tmp[1]>map_height_inc-1))|| \
            map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]==-128 ){
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
                getminradius=1;
                break;
            }
            if( map_.data[xy_tmp[0]+xy_tmp[1]*map_.info.width]==-128 ){
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
    data_tmp=-48+2.4*radius;
    map_.data[index_tmp[0]+index_tmp[1]*map_.info.width]=data_tmp;
}


/*
    功能：
        1：将采样点对应的地图栅格选出来，
        2：求出采样点距离占据栅格的最近距离，属性值与距离成反比(或者用100-距离)，距离越近其栅格属性值越高，以占据的100为上限；
*/
void mapchange_with_samplepoints(){
    for(int i=0;(sample_points[i][0]!=0)&&(sample_points[i][1]!=0);i++){
        change_one_samplepoint(sample_points[i][0],sample_points[i][1]);
    }
}