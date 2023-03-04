//计算坐标索引，输入参数：点的坐标
int * compute_point_index(double point[]){
  static int point_index[2];
  point_index[0]=(point[0]-map_origin[0]);//注意这里可能有略微超出地图边界的索引，比如-0.5可能会被取值为-1
  point_index[1]=(point[1]-map_origin[1]);
  return point_index;
}

int * compute_point_index(float x,float y){
  static int point_index[2];
  point_index[0]=(x-map_origin[0]);//注意这里可能有略微超出地图边界的索引，比如-0.5可能会被取值为-1
  point_index[1]=(y-map_origin[1]);
  return point_index;
}

//计算两点之间的距离
double getLength(int x1,int y1,int x2,int y2){
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

// 返回连续角度中的最大角度范围
// 步骤：
//  1.以某点为射线起点构造一组射线(间隔为5°)
int * max_close_angle_range(int x,int y){
  std::cout<<"射线起点坐标:("<<x*resolution<<","<<y*resolution<<")"<<std::endl;
  static int angle_range[2];
  int angle_select[72];//记录在一定距离内到达地图边界或占据栅格的角度，初始化为400，作为标志位
  int angle_select_tmp[72];
  for(int i=0;i<72;i++){
    angle_select[i]=400;
  }
  int count_angle_select=0;//计数器

  // 求出一定距离内能够到达地图边界或占据栅格的搜索角度-----------------------------------------------
  double gradient;
  int flag_x;//斜率存在时x轴坐标增加，不存在时不增加
  for(int num=0;num<72;num++){
    // std::cout<<"射线角度为: "<<num*5<<std::endl;
    if(num*5==90){
      flag_x=0;
      gradient=1;
    }
    else if(num*5==270){
      flag_x=0;
      gradient=-1;      
    }
    else if(num*5>270||num*5<90){
      gradient=tan(num*5.0/180.0*M_PI);
      flag_x=1;
    }
    else if(num*5>90&&num*5<270){
      gradient= -tan(num*5.0/180.0*M_PI);
      flag_x=-1;
    }    

    // std::cout<<"222"<<std::endl;
    double point_tmp[2];
    int point_index_tmp[2];
    int * point_index_tmp_;
    double step=resolution/4.0;
    //寻找射线线段另一个端点（地图边界或占据栅格）----------------------------------------------------------
    // std::cout<<"333"<<std::endl;
    for(int i=0;;i++){
      // std::cout<<"333444"<<std::endl;
      point_tmp[0]=x+i*flag_x*1.0*step;
      point_tmp[1]=y+i*gradient*1.0*step;
      // if(abs(num*5)==45){
      //   std::cout<<"gradient:"<<gradient<<"point_tmp[0]:"<<point_tmp[0]<<"point_tmp[1]:"<<point_tmp[1]<<std::endl;
      // }
      // std::cout<<"444"<<std::endl;
      point_index_tmp_=compute_point_index(point_tmp);
      point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    

    //   cout<<"Length:  "<<getLength(x,y,point_index_tmp[0],point_index_tmp[1])<<endl;
    //   cout<<"range_threshold/resolution:   "<<range_threshold/resolution<<endl;
      if(getLength(x,y,point_index_tmp[0],point_index_tmp[1])>=(range_threshold/resolution)){//搜索方向较远
        // std::cout<<"搜索到较远处,角度为:"<<num*5<<std::endl;      
        break;
      }
    //   std::cout<<"555"<<std::endl;
      //地图边界,这里使用<和>是为了避免compute_point_index计算出来的坐标索引在边界值外侧
      if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
        // std::cout<<"搜索到地图边界,角度为:"<<num*5<<std::endl;
        // angle_select[count_angle_select]=num;
        // count_angle_select++;
        break;
      }
      // std::cout<<"666"<<std::endl;
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-128) {
          // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
        // std::cout<<"搜索到占据栅格,角度为:"<<num*5<<std::endl;
        angle_select[count_angle_select]=num;
        count_angle_select++;
        break;
      }
    } 
  }

  // 根据选中的角度/方向范围，求出连续且大于180度的角度范围（最小与最大角度）
  // -----这里还需要考虑上面得到的角度被360°截断的情况（360°与0°对应射线相同）
  // 1: 0°和355°是否包含在查找到的角度范围angle_select内
  int flag_0_355=0;
  for(int i=0;i<count_angle_select;i++){
    if(angle_select[i]==0 || angle_select[i]==71){
      flag_0_355++;
    }
  }
  if(flag_0_355>=2){
    std::cout<<"0°和355°是否包含在查找到的角度范围angle_select内"<<std::endl;
 
    // 2: 将从0°开始的递增连续角度变成从360°开始的连续角度
    int change_num=1000;//1000作为标志位
    for(int i=0;i<count_angle_select;i++){
      if(angle_select[i]==i){
        angle_select[i]=angle_select[i]+72;
        change_num=i;
      }
      else{
        break;
      }
    }  
    // 3: 将angle_select从小到大排序  angle_select_tmp
    if(change_num!=1000){
      for(int i=0;i<=change_num;i++){
        angle_select_tmp[count_angle_select-1-change_num+i]=angle_select[i];
      }
      for(int i=change_num+1;i<count_angle_select;i++){
        angle_select_tmp[i-change_num-1]=angle_select[i];
      }
    }
    for(int i=0;i<count_angle_select;i++){
      angle_select[i]=angle_select_tmp[i];
      // std::cout<<angle_select[i]<<std::endl;
    }
  }

  int consecutive_num=1;// 连续角度的个数,初始化为1
  angle_range[0]=angle_select[0];
  if(count_angle_select>=36){
    for(int i=0;i<count_angle_select-1;i++){
        if(angle_select[i+1]-angle_select[i]==1){
            consecutive_num++;
            angle_range[1]=angle_select[i+1];
        }
        else{
            consecutive_num=1;
            angle_range[0]=angle_select[i+1];
            if(count_angle_select-i<36){//剩余的角度数目即使是连续的，也不可能构成180度的范围
                break;
            }
        }
    }
  }

  if(consecutive_num<36){
    angle_range[0]=400;//把400当作失败的标志位
    angle_range[1]=400;
  }

  return angle_range;
}

// 对半包围及以上区域进行离散化采样------------------------------------------------------------------------------------------------------
void sample_close_area(int x,int y,int angle_min,int angle_max){
  int count_sample=0;//计数器
  double gradient;
  double hypotenuse;//直角三角形--斜边长度
  int flag_x;//斜率存在时x轴坐标增加，不存在时不增加
  for(int angle=angle_min;angle<=angle_max;angle++){
    // std::cout<<"射线角度为: "<<num*5<<std::endl;
    float num;
    if(angle==angle_max){//避开半包围区域边界
      num=angle-0.4;
      cout<<"num: "<<num<<endl;
    }
    if(angle==angle_min){
      num=angle+0.4;
      cout<<"num: "<<num<<endl;
    }
    if(num*5>360){
      num=num-72;
    }

    if(num*5==90){
      flag_x=0;
      gradient=1;
    }
    else if(num*5==270){
      flag_x=0;
      gradient=-1;      
    }
    else if(num*5>270||num*5<90){
      gradient=tan(num*5.0/180.0*M_PI);
      flag_x=1;
    }
    else if(num*5>90&&num*5<270){
      gradient= -tan(num*5.0/180.0*M_PI);
      flag_x=-1;
    }    
    hypotenuse=sqrt(gradient*gradient+1);

    double point_tmp[2];
    int point_index_tmp[2];
    int * point_index_tmp_;
    double step=sample_close_area_interval*resolution;
    //寻找射线线段另一个端点（地图边界或占据栅格）----------------------------------------------------------

    for(int i=0;;i++){
      point_tmp[0]=x+i*flag_x*1.0/hypotenuse*step;
      point_tmp[1]=y+i*gradient/hypotenuse*step;
      point_index_tmp_=compute_point_index(point_tmp);
      point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    

      //地图边界,这里使用<=和>=是为了避免compute_point_index计算出来的坐标索引在边界值外侧
      if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
        // std::cout<<"搜索到地图边界"<<std::endl;
        break;
      }
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-128) {
          // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
        // std::cout<<"搜索到占据栅格"<<std::endl;
        break;
      }

      sample_points[count_sample][0]=point_index_tmp[0];
      sample_points[count_sample][1]=point_index_tmp[1];
      count_sample++;
    } 
  }

  // // 增加最大最小角度附近的采样精度；一般最大最小角度发生在搜索线段到达长度阈值的地方
  for(int angle=angle_min*5+3;angle<=angle_max*5-3;angle=angle+1){
    // std::cout<<"射线角度为: "<<num*5<<std::endl;
    int num;
    num=angle;
    if(num-angle_min*5==10){
      num=angle_max*5-10;
      continue;
    }
    if(angle>360){
      num=angle-360;
    }

    if(num==90){
      flag_x=0;
      gradient=1;
    }
    else if(num==270){
      flag_x=0;
      gradient=-1;      
    }
    else if(num>270||num<90){
      gradient=tan(num/180.0*M_PI);
      flag_x=1;
    }
    else if(num>90&&num<270){
      gradient= -tan(num/180.0*M_PI);
      flag_x=-1;
    }    
    hypotenuse=sqrt(gradient*gradient+1);

    double point_tmp[2];
    int point_index_tmp[2];
    int * point_index_tmp_;
    double step=sample_close_area_interval*resolution;
    //寻找射线线段另一个端点（地图边界或占据栅格）----------------------------------------------------------

    for(int i=0;;i++){
      point_tmp[0]=x+i*flag_x*1.0/hypotenuse*step;
      point_tmp[1]=y+i*gradient/hypotenuse*step;
      point_index_tmp_=compute_point_index(point_tmp);
      point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    

      //地图边界,这里使用<=和>=是为了避免compute_point_index计算出来的坐标索引在边界值外侧
      if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
        // std::cout<<"搜索到地图边界"<<std::endl;
        break;
      }
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-128) {
          // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
        // std::cout<<"搜索到占据栅格"<<std::endl;
        break;
      }

      sample_points[count_sample][0]=point_index_tmp[0];
      sample_points[count_sample][1]=point_index_tmp[1];
      count_sample++;
    } 
  }
  cout<<"count_sample:  "<<count_sample<<endl;


}


