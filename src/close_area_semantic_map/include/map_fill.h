/*
    功能：
        1：查找一个空闲栅格上下左右四个方向，是不是可以到达占据栅格或语义栅格
        2：是的话，用到达栅格的属性值的均值来代替该栅格的属性值
*/
void fill_one_grid(int i,int j){
    // 一个空闲栅格 上下左右四个方向是否为占据或语义栅格(空闲栅格属性为0，未知栅格属性为-1)---------------------------------------------------------
    if((map_.data[i + (j+1)*map_.info.width]>0)&&(map_.data[i + (j-1)*map_.info.width]>0)&& \ 
        (map_.data[i-1 + j*map_.info.width]>0)&&(map_.data[i+1 + j*map_.info.width]>0)){
        map_.data[i + j*map_.info.width]=0.25*(map_.data[i + (j+1)*map_.info.width]+map_.data[i + (j-1)*map_.info.width]+ \ 
                                                                                            map_.data[i-1 + j*map_.info.width]+map_.data[i+1 + j*map_.info.width]);
        return;
    }

    // 观察栅格上下左右四个方向是否为空闲栅格，如果是，只再判断上/下/左/右栅格是否为空闲-------------------------------------------------------------
    int flag_up=0;// 向上方向是否被占据或语义栅格堵住
    int flag_down=0;
    int flag_left=0;
    int flag_right=0;
    double sum=0;
    int num_total=0;
    if(map_.data[i + (j+1)*map_.info.width]==0 || map_.data[i + (j+1)*map_.info.width]==-1){//上
        if((map_.data[i + (j+2)*map_.info.width]>0)&& \ 
            (map_.data[i-1 + (j+1)*map_.info.width]>0)&&(map_.data[i+1 + (j+1)*map_.info.width]>0)){
            flag_up=1;
            sum=sum+map_.data[i + (j+2)*map_.info.width]+map_.data[i-1 + (j+1)*map_.info.width]+map_.data[i +1+ (j+1)*map_.info.width];
            num_total=num_total+3;
        }
    }
    else {
        flag_up=1;
        sum=sum+map_.data[i + (j+1)*map_.info.width];
        num_total++;
    }

    if(map_.data[i + (j-1)*map_.info.width]==0 || map_.data[i + (j-1)*map_.info.width]==-1){//下
        if((map_.data[i + (j-2)*map_.info.width]>0)&& \ 
            (map_.data[i-1 + (j-1)*map_.info.width]>0)&&(map_.data[i+1 + (j-1)*map_.info.width]>0)){
            flag_down=1;
            sum=sum+map_.data[i + (j-2)*map_.info.width]+map_.data[i-1 + (j-1)*map_.info.width]+map_.data[i +1+ (j-1)*map_.info.width];
            num_total=num_total+3;
        }
    }
    else {
        flag_down=1;
        sum=sum+map_.data[i + (j-1)*map_.info.width];
        num_total++;
    }

    if(map_.data[i-1 + (j+0)*map_.info.width]==0 || map_.data[i + (j+0)*map_.info.width]==-1){
        if((map_.data[i-1 + (j+1)*map_.info.width]>0)&& \ 
            (map_.data[i-1 + (j-1)*map_.info.width]>0)&&(map_.data[i-2 + (j+0)*map_.info.width]>0)){
            flag_left=1;
            sum=sum+map_.data[i-1 + (j+1)*map_.info.width]+map_.data[i-1 + (j-1)*map_.info.width]+map_.data[i -2+ (j+0)*map_.info.width];
            num_total=num_total+3;
        }
    }
    else {
        flag_left=1;
        sum=sum+map_.data[i-1 + (j+0)*map_.info.width];
        num_total++;
    }

    if(map_.data[i+1 + (j+0)*map_.info.width]==0 || map_.data[i +1+ (j+0)*map_.info.width]==-1){
        if((map_.data[i+1 + (j+1)*map_.info.width]>0)&& \ 
            (map_.data[i+1 + (j-1)*map_.info.width]>0)&&(map_.data[i+2 + (j+0)*map_.info.width]>0)){
            flag_right=1;
            sum=sum+map_.data[i+1 + (j+1)*map_.info.width]+map_.data[i+1 + (j-1)*map_.info.width]+map_.data[i +2+ (j+0)*map_.info.width];
            num_total=num_total+3;            
        }
    }
    else {
        flag_right=1;
        sum=sum+map_.data[i+1 + (j+0)*map_.info.width];
        num_total++;
    }

    if(flag_up&&flag_down&&flag_left&&flag_right){
        map_.data[i + j*map_.info.width]=sum/num_total;
    }
    return ;
}

/*
    功能：
        1：查找被占据栅格或死角语义栅格包围的连通的一个或多个空闲栅格
        2：更新空闲栅格的属性值为包围栅格的平均值
*/
void map_fill(){
    for(int i=2;i<map_.info.width-2;i++){
      for(int j=2;j<map_.info.height-2;j++){      
        if(map_.data[i + j*map_.info.width]==0){
            fill_one_grid(i,j);
        }
      }
    }
}