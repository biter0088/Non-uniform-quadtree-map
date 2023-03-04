//计算坐标索引，输入参数：点的坐标
int * compute_point_index(double point[]){
  static int point_index[2];
  point_index[0]=(point[0]-map_origin[0])/resolution;//注意这里可能有略微超出地图边界的索引，比如-0.5可能会被取值为-1
  point_index[1]=(point[1]-map_origin[1])/resolution;
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

//计算两点之间的距离
double getLengthFloat(float x1,float y1,float x2,float y2){
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

// 返回连续角度中的最大角度范围
// 步骤：
//  1.以某点为射线起点构造一组射线(间隔为5°)
int * max_close_angle_range(int x,int y){
  // std::cout<<"射线起点坐标:("<<x*resolution<<","<<y*resolution<<")"<<std::endl;
  // std::cout<<"射线起点坐标索引:("<<x<<","<<y<<")"<<std::endl;//0901
  // std::cout<<"射线起点坐标:("<<(x+0.5)*resolution<<","<<(y+0.5)*resolution<<")"<<std::endl;//0901
  keypoints_max_distance=0;//每次进入函数都是新的关键点，需要初始化为0
  keypoints_min_distance=0;//关键点到障碍物的最小距离，避免关键点太靠近障碍物的情况
  keypoints_max_distance_valid=0;
  static int angle_range[2];
  int angle_select[search_lines];//记录在一定距离内到达地图边界或占据栅格的角度，初始化为400，作为标志位
  int angle_select_tmp[search_lines];
  for(int i=0;i<search_lines;i++){
    angle_select[i]=400;
  }
  int count_angle_select=0;//计数器

  // 求出一定距离内能够到达地图边界或占据栅格的搜索角度-----------------------------------------------
  double gradient;
  int flag_x;//斜率存在时x轴坐标增加，不存在时不增加
  for(int num=0;num<search_lines;num++){
    // std::cout<<"射线角度为: "<<num*search_angle<<std::endl;
    if(num*search_angle==90){
      flag_x=0;
      gradient=1;
    }
    else if(num*search_angle==270){
      flag_x=0;
      gradient=-1;      
    }
    else if(num*search_angle>270||num*search_angle<90){
      gradient=tan(num*search_angle/180.0*M_PI);
      flag_x=1;
    }
    else if(num*search_angle>90&&num*search_angle<270){
      gradient= -tan(num*search_angle/180.0*M_PI);
      flag_x=-1;
    }    

    // std::cout<<"222"<<std::endl;
    double point_tmp[2];
    int point_index_tmp[2];
    int * point_index_tmp_;
    // double step=resolution/4.0;//    /home/meng/ideas/HDMaps/Town01/map7_deadend_data.dat
    // double step=resolution/5.0;//    /home/meng/ideas/HDMaps/Town01/map7_deadend_data_3.dat
    double step=resolution/20.0;//        /home/meng/ideas/HDMaps/Town01/map7_deadend_data_4.dat
                                                                          // /home/meng/ideas/HDMaps/Town01/map9_deadend_data_4.dat
    //寻找射线线段另一个端点（地图边界或占据栅格）----------------------------------------------------------
    // std::cout<<"333"<<std::endl;

    for(int i=0;;i++){
      // std::cout<<"i: "<<i<<std::endl;
      // std::cout<<"333444"<<std::endl;
      // point_tmp[0]=x+i*flag_x*1.0*step;
      // point_tmp[1]=y+i*gradient*1.0*step;
      point_tmp[0]=(x)*resolution+i*flag_x*1.0*step;//0901
      point_tmp[1]=(y)*resolution+i*gradient*1.0*step;//0901
      // std::cout<<"搜索到的坐标: ["<<point_tmp[0]<<","<<point_tmp[1]<<"]"<<std::endl;
      // if(abs(num*search_angle)==45){
      //   std::cout<<"gradient:"<<gradient<<"point_tmp[0]:"<<point_tmp[0]<<"point_tmp[1]:"<<point_tmp[1]<<std::endl;
      // }
      // std::cout<<"444"<<std::endl;
      point_index_tmp_=compute_point_index(point_tmp);
      point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    
      // std::cout<<"搜索到的坐标索引: ["<<*(point_index_tmp_+0)<<","<<*(point_index_tmp_+1)<<"]"<<std::endl;

    //   cout<<"Length:  "<<getLength(x,y,point_index_tmp[0],point_index_tmp[1])<<endl;
    //   cout<<"range_threshold/resolution:   "<<range_threshold/resolution<<endl;
      float length_tmp=0;
      // length_tmp=getLength(x,y,point_index_tmp[0],point_index_tmp[1]);
      length_tmp=getLengthFloat((x)*resolution,(y)*resolution,point_tmp[0],point_tmp[1]);
      keypoints_max_distance=  keypoints_max_distance>length_tmp?keypoints_max_distance:length_tmp;

      if(length_tmp>=(range_threshold)){//搜索方向较远
        // std::cout<<"搜索到较远处,角度为:"<<num*search_angle<<std::endl;      
        break;
      }
    //   std::cout<<"555"<<std::endl;
      //地图边界,这里使用<和>是为了避免compute_point_index计算出来的坐标索引在边界值外侧
      if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
        // std::cout<<"搜索到地图边界,角度为:"<<num*search_angle<<std::endl;
        // angle_select[count_angle_select]=num;
        // count_angle_select++;
        break;
      }
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1) {//未知栅格
        // std::cout<<"搜索到未知栅格,角度为:"<<num*search_angle<<std::endl;
        // std::cout<<"搜索到的坐标: ["<<point_tmp[0]<<","<<point_tmp[1]<<"]"<<std::endl;        
        break;
      }
      // std::cout<<"666"<<std::endl;
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==100) {//为占据栅格
          // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//未知栅格
        // std::cout<<"搜索到占据栅格,角度为:"<<num*search_angle<<std::endl;
        //关键点太靠近占据栅格，则这个射线搜索方向无效----------------------------------------------------------------------------
        // 这个判断“搜索方向无效”的条件是不是不太合适  0903
        // std::cout<<"搜索到占据栅格,角度为:"<<num*search_angle<<std::endl;                
        keypoints_min_distance=length_tmp;
        if(keypoints_min_distance<=(range_threshold_min)){
          break;
        }
        // ----------------------------------------------------------------------------
        // std::cout<<"该角度有效"<<std::endl;        
        keypoints_max_distance_valid=  keypoints_max_distance_valid>length_tmp?keypoints_max_distance_valid:length_tmp;
        angle_select[count_angle_select]=num;
        count_angle_select++;
        break;
      }
    } 
  }
  // 查看到达占据栅格且有效的角度
  // for(int i=0;i<count_angle_select;i++){
    // std::cout<<"angle_select["<<i<<"]: "<<angle_select[i]<<std::endl;
  // }  

  // 根据选中的角度/方向范围，求出连续且大于180度的角度范围（最小与最大角度）
  // -----这里还需要考虑上面得到的角度被360°截断的情况（360°与0°对应射线相同）
  // 1: 0°和355°是否包含在查找到的角度范围angle_select内
  int flag_0_355=0;
  for(int i=0;i<count_angle_select;i++){
    if(angle_select[i]==0 || angle_select[i]==search_lines-1){
      flag_0_355++;
    }
  }
  if(flag_0_355>=2){
    // std::cout<<"0°和355°包含在查找到的角度范围angle_select内"<<std::endl;
 
    // 2: 将从0°开始的递增连续角度变成从360°开始的连续角度
    int change_num=1000;//1000作为标志位
    for(int i=0;i<count_angle_select;i++){
      if(angle_select[i]==i){
        angle_select[i]=angle_select[i]+search_lines;
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
  if(count_angle_select>=search_lines/2){//所有有效搜索角度范围是否大于180°
    for(int i=0;i<count_angle_select-1;i++){
        if(angle_select[i+1]-angle_select[i]==1){//角度连续
          consecutive_num++;
          angle_range[1]=angle_select[i+1];
        }
        else if(consecutive_num>=search_lines/2){//一组连续的角度查询结束，判断该组角度是否大于180°
          break;//查询结束
        }
        else{//舍弃前面的查询，进行进一步的查询
            consecutive_num=1;
            angle_range[0]=angle_select[i+1];
            if(count_angle_select-i<search_lines/2){//剩余的角度数目即使是连续的，也不可能构成180度的范围
              break;
            }
        }
    }
  }

  std::cout<<"consecutive_num: "<<consecutive_num<<std::endl;
  std::cout<<"angle_range[0]: "<<angle_range[0]<<std::endl;
  std::cout<<"angle_range[1]: "<<angle_range[1]<<std::endl;

  if(consecutive_num<search_lines/2){
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
  
  for(int angle=angle_min+2;angle<=angle_max-2;angle++){
    // std::cout<<"射线角度为: "<<num*search_angle<<std::endl;
    float num;
    num=angle;
    if(angle==angle_max){//避开半包围区域边界
      // num=angle-0.4;
      num=angle-1;//carla-town01
      // cout<<"num: "<<num<<endl;
    }
    if(angle==angle_min){
      // num=angle+0.4;
      num=angle+1;//carla-town01
      // cout<<"num: "<<num<<endl;
    }
    if(num*search_angle>360){
      num=num-search_lines;
    }

    if(num*search_angle==90){
      flag_x=0;
      gradient=1;
    }
    else if(num*search_angle==270){
      flag_x=0;
      gradient=-1;      
    }
    else if(num*search_angle>270||num*search_angle<90){
      gradient=tan(num*search_angle/180.0*M_PI);
      flag_x=1;
    }
    else if(num*search_angle>90&&num*search_angle<270){
      gradient= -tan(num*search_angle/180.0*M_PI);
      flag_x=-1;
    }    
    hypotenuse=sqrt(gradient*gradient+1);

    // 针对/home/meng/ideas/论文稿及材料0608/自己采集的数据/0706/数据包-7/死角语义地图-报错-4-2--射线从对角遍历出去.png出现的问题---------------------
    int x_offset=0;int y_offset=0;
    if(num*search_angle>0&&num*search_angle<90){//向地图右上方开始遍历
      x_offset=1;
      y_offset=1;
    }  
    else if(num*search_angle>90&&num*search_angle<180){//向地图左上方开始遍历
      x_offset= -1;
      y_offset=1;
    }    
    else if(num*search_angle>180&&num*search_angle<270){//向地图左下方开始遍历
      x_offset= -1;
      y_offset= -1;
    }    
    else if(num*search_angle>270&&num*search_angle<360){//向地图右下方开始遍历
      x_offset= 1;
      y_offset= -1;
    }    
    // std::cout<<"角度,xy方向偏移:  "<<(num*search_angle)<<", "<<x_offset<<", "<<y_offset<<std::endl;
// ---------------------

    double point_tmp[2];
    int point_index_tmp[2];
    int * point_index_tmp_;
    double step=sample_close_area_interval*resolution;

    /*//// 避免射线进行对角线搜索：0901新增   弃用
    int last_x=0;
    int last_y=0;
    int next_x=0;
    int next_y=0;
    point_tmp[0]=x;
    point_tmp[1]=y;
    point_index_tmp_=compute_point_index(point_tmp);
    last_x=*(point_index_tmp_+0);  last_y=*(point_index_tmp_+1);    
    ////---------------------------------------
    */

    //寻找射线线段另一个端点（地图边界或占据栅格）----------------------------------------------------------
    // for(int i=0;;i++){
    // std::cout<<"keypoints_max_distance_valid: "<<keypoints_max_distance_valid<<std::endl;      
    // std::cout<<"step: "<<step<<std::endl;          
    // std::cout<<"keypoints_max_distance_valid/step: "<<int(keypoints_max_distance_valid/step)<<std::endl;              
    for(int i=0;i<int(keypoints_max_distance_valid/step);i++){   //2022/12/09新增   
      // point_tmp[0]=x+i*flag_x*1.0/hypotenuse*step;
      // point_tmp[1]=y+i*gradient/hypotenuse*step;
      point_tmp[0]=x*resolution+i*flag_x*1.0/hypotenuse*step;//0901
      point_tmp[1]=y*resolution+i*gradient/hypotenuse*step;//0901
      point_index_tmp_=compute_point_index(point_tmp);
      point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    

      /*next_x=point_index_tmp[0]; next_y=point_index_tmp[1];//0901新增
      if(abs(next_x-last_x)>=1 && abs(next_y-last_y)>=1){//0901新增
        break;
      }
      else {
        last_x=next_x;
        last_y=next_y;
      }
      ////---------------------------------------
      */

      //地图边界,这里使用<=和>=是为了避免compute_point_index计算出来的坐标索引在边界值外侧
      if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
        // std::cout<<"搜索到地图边界"<<std::endl;
        break;
      }
      // std::cout<<"111"<<std::endl;
      if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==100||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1) {
          // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
        // std::cout<<"搜索到占据栅格"<<std::endl;
        break;
      }
      // std::cout<<"222"<<std::endl;
      // 针对/home/meng/ideas/论文稿及材料0608/自己采集的数据/0706/数据包-7/死角语义地图-报错-4-2--射线从对角遍历出去.png出现的问题---------------------
      if((point_index_tmp[0]+x_offset)>=0 && (point_index_tmp[0]+x_offset)<= (map_width_inc-1) && \
          (point_index_tmp[1]+y_offset)>=0 && (point_index_tmp[1]+y_offset)<= (map_height_inc-1) && \ 
          map_.data[point_index_tmp[0]+x_offset+point_index_tmp[1]*map_.info.width]==100&& \
          map_.data[point_index_tmp[0]+(point_index_tmp[1]+y_offset)*map_.info.width]==100) {//遍历方向被堵住
          //  std::cout<<"遍历方向被堵住，角度为:"<<(num*search_angle)<<std::endl;
        sample_points[count_sample][0]=point_index_tmp[0];
        sample_points[count_sample][1]=point_index_tmp[1];
        count_sample++;
        break;
      }

      sample_points[count_sample][0]=point_index_tmp[0];
      sample_points[count_sample][1]=point_index_tmp[1];
      count_sample++;
      // std::cout<<"count_sample:  "<<count_sample<<std::endl;
    } 
  }

  // // 增加最大最小角度附近的采样精度；一般最大最小角度发生在搜索线段到达长度阈值的地方
  if(0){
    for(int angle=angle_min*search_angle+3;angle<=angle_max*search_angle-3;angle=angle+1){
      // std::cout<<"射线角度为: "<<num*search_angle<<std::endl;
      int num;
      num=angle;
      if(num-angle_min*search_angle==10){
        num=angle_max*search_angle-10;
        continue;
      }
      if(angle>360){
        num=angle-360;
      }

      int x_offset=0;int y_offset=0;
      if(num>0&&num<90){//向地图右上方开始遍历
        x_offset=1;
        y_offset=1;
      }  
      else if(num>90&&num<180){//向地图左上方开始遍历
        x_offset= -1;
        y_offset=1;
      }    
      else if(num>180&&num<270){//向地图左下方开始遍历
        x_offset= -1;
        y_offset= -1;
      }    
      else if(num>270&&num<360){//向地图右下方开始遍历
        x_offset= 1;
        y_offset= -1;
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

      // for(int i=0;;i++){
    for(int i=0;i<int(keypoints_max_distance_valid/step);i++){   //2022/12/09新增           
        // point_tmp[0]=x+i*flag_x*1.0/hypotenuse*step;
        // point_tmp[1]=y+i*gradient/hypotenuse*step;
        point_tmp[0]=x*resolution+i*flag_x*1.0/hypotenuse*step;//0901
        point_tmp[1]=y*resolution+i*gradient/hypotenuse*step;//0901
        point_index_tmp_=compute_point_index(point_tmp);
        point_index_tmp[0]=*(point_index_tmp_+0);  point_index_tmp[1]=*(point_index_tmp_+1);    

        //地图边界,这里使用<=和>=是为了避免compute_point_index计算出来的坐标索引在边界值外侧
        if((point_index_tmp[0]<0)||(point_index_tmp[0]>map_width_inc-1)||(point_index_tmp[1]<0)||(point_index_tmp[1]>map_height_inc-1)){
          // std::cout<<"搜索到地图边界"<<std::endl;
          break;
        }
        // std::cout<<"333"<<std::endl;
        if( map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==100||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1) {
            // ||map_.data[point_index_tmp[0]+point_index_tmp[1]*map_.info.width]==-1){//为占据栅格或未知栅格
          // std::cout<<"搜索到占据栅格"<<std::endl;
          break;
        }

        if((point_index_tmp[0]+x_offset)>=0 && (point_index_tmp[0]+x_offset)<= (map_width_inc-1) && \
            (point_index_tmp[1]+y_offset)>=0 && (point_index_tmp[1]+y_offset)<= (map_height_inc-1) && \
            map_.data[point_index_tmp[0]+x_offset+point_index_tmp[1]*map_.info.width]==100&& \
            map_.data[point_index_tmp[0]+(point_index_tmp[1]+y_offset)*map_.info.width]==100) {//遍历方向被堵住
            //  std::cout<<"遍历方向被堵住，角度为:"<<(num*search_angle)<<std::endl;
          sample_points[count_sample][0]=point_index_tmp[0];
          sample_points[count_sample][1]=point_index_tmp[1];
          count_sample++;
          break;
        }
        // std::cout<<"444"<<std::endl;
        sample_points[count_sample][0]=point_index_tmp[0];
        sample_points[count_sample][1]=point_index_tmp[1];
        count_sample++;
        // std::cout<<"count_sample:  "<<count_sample<<std::endl;
      } 
    }
  }

  cout<<"count_sample:  "<<count_sample<<endl;
}

// 查找一个点是否在有效采样点集valid_sample_points中
void check_one_samplepoint(int x,int y){
  int i;
  for(i=0;(valid_sample_points[i][0]!=0)&&(valid_sample_points[i][1]!=0);i++){
      if(valid_sample_points[i][0]==x&&valid_sample_points[i][1]==y){
        break;
      }
  }
  if(valid_sample_points[i][0]==0&&valid_sample_points[i][1]==0){
    valid_sample_points[i][0]=x;
    valid_sample_points[i][1]=y;      
    num_valid_sample_points++;
  }
}


// 去除采样点中相同的点
void valid_sample_points_filter(){
    for(int i=0;(sample_points[i][0]!=0)&&(sample_points[i][1]!=0);i++){
        check_one_samplepoint(sample_points[i][0],sample_points[i][1]);
    }
    std::cout<<"有效采样点有:  "<<num_valid_sample_points<<"个."<<std::endl;
}

//采样点初始化
void sample_points_init(){
  for(int i=0;i<100000;i++){
      sample_points[i][0]=0;
      sample_points[i][1]=0;
  }
}

//车辆楼后停车场环境
void sample_points_init_park(){
  for(int i=0;i<5000;i++){
      sample_points[i][0]=0;
      sample_points[i][1]=0;
  }
}

//有效采样点初始化
void valid_sample_points_init(){
  for(int i=0;i<100000;i++){
      valid_sample_points[i][0]=0;
      valid_sample_points[i][1]=0;
  }
  num_valid_sample_points=0;
}

//有效采样点初始化
void valid_sample_points_init_park(){
  for(int i=0;i<5000;i++){
      valid_sample_points[i][0]=0;
      valid_sample_points[i][1]=0;
  }
  num_valid_sample_points=0;
}




