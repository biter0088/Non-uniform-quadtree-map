/*
    利用ray cast方法过滤掉车辆当前位置附近的未知栅格
*/
void ray_cast_function(){
    // std::cout<<"使用ray cast-----------------------------------------"<<std::endl;
    //利用ray cast方法过滤掉车辆当前位置附近的未知栅格，圆圈的半径均设为10m，圆圈的外接矩形为10m*10m的正方形------------------------------------------------
    //0616  更新发送给raycast的区域大小和形状
    double rect_height_half = 15;
    double rect_width_half = 25.0;
    int rect_height = rect_height_half / resolution;//"圆圈"外接矩形长和宽的一半
    int rect_width = rect_width_half / resolution;
    // int radius=20.0/resolution;
    //车辆当前位置及其栅格坐标，或者可以直接用"// 使用里程计，经过的位置都可通行"这一步计算好的
    double now_x = odomVec[odomVec.size()-1].pose.pose.position.x;
    double now_y = odomVec[odomVec.size()-1].pose.pose.position.y;
    // int x_now_car = round((0.5*width  + now_x - robot_x) / resolution) ;
    // int y_now_car = round((0.5*height + now_y - robot_y) / resolution) ;
    // int x_now_car = round((0.25*width  + now_x - robot_x) / resolution) ;//0.25*width  和cloud2gridmap函数对应一下
    // int y_now_car = round((0.25*height + now_y - robot_y) / resolution) ;
    // int x_now_car = round((max_width*1.2  + now_x - robot_x) / resolution) ;// 和cloud2gridmap函数对应一下
    // int y_now_car = round((max_len*1.2+ now_y - robot_y) / resolution) ;
    // int x_now_car = round((45  + now_x - robot_x) / resolution) ;// 和cloud2gridmap函数对应一下
    // int y_now_car = round((35+ now_y - robot_y) / resolution) ;
    int x_now_car = round((x_offset_total  + now_x - robot_x) / resolution) ;// 和cloud2gridmap函数对应一下
    int y_now_car = round((y_offset_total+ now_y - robot_y) / resolution) ;

    //opencv图像坐标系 x右 y下 ----全局地图坐标系 x前 y左------这里暂时不管x、y轴的方向，因为都是周围
    cv::Mat grid_image =  -1 * cv::Mat::ones(cv::Size(2*rect_width+1, 2*rect_height+1), CV_8U);//初始化为 -1
    //车辆不在地图边缘，可以取附近10m的圆圈----这个后期可以使用一个自适应的更小的圆圈
    // if( ((now_x- robot_x-10)>0)&& ((now_x- robot_x+10)<height)  && ((now_y- robot_y)>10-width)  && ((now_y- robot_y)<-10)){
      // std::cout<<"初始化grid_image "<<std::endl;
      for (int px = -rect_width ; px<=rect_width; px++)
      {
        for (int py = -rect_height ; py<=rect_height; py++)
        { //将该部分栅格属性赋值给grid_image
          if ( ((x_now_car+px)>=map_.info.width) ||  ((y_now_car+py)>=map_.info.height)   )     {
            continue;
          }
          switch (map_.data[x_now_car+px + (y_now_car+py)*map_.info.width]){
            case -1:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=UNKNOWN;
              break;
            case 0:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=PASSABLE;
              break;
            case 100:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=OBSTACLE;
              break;
            default:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=UNKNOWN;
              break;
          }
        }
      }
    // }

    cv::Mat display_image1 = cv::Mat::zeros(grid_image.rows, grid_image.cols, CV_8UC3);
    for (int row=0; row<grid_image.rows; row++) {
        for (int col=0; col<grid_image.cols; col++) {
            switch (grid_image.at<unsigned char>(row,col))
            {
            case OBSTACLE:
                display_image1.at<cv::Vec3b>(row,col)[0] = 255;//白色
                display_image1.at<cv::Vec3b>(row,col)[1] = 255;
                display_image1.at<cv::Vec3b>(row,col)[2] = 255;
                break;
            case PASSABLE:
                display_image1.at<cv::Vec3b>(row,col)[0] = 0;//绿色
                display_image1.at<cv::Vec3b>(row,col)[1] = 255;
                display_image1.at<cv::Vec3b>(row,col)[2] = 0;
                break;
              case UNKNOWN:
                display_image1.at<cv::Vec3b>(row,col)[0] = 0;//未知区域    黑色
                display_image1.at<cv::Vec3b>(row,col)[1] = 0;
                display_image1.at<cv::Vec3b>(row,col)[2] = 0;
                break;
            default:
                break;
            }
        }
    }
    // cv::namedWindow("display_image1", 0);
    // cv::imshow("display_image1", display_image1);
    // cv::waitKey(1);

    cv::Mat grid_image_clone = grid_image.clone();
    // known_area_extraction(grid_image_clone,   cv::Point(rect_width, rect_height),   radius);
    known_area_extraction(grid_image_clone,   cv::Point(rect_width, rect_height),rect_width,rect_height);//0616
    cv::Mat display_image = cv::Mat::zeros(grid_image_clone.rows, grid_image_clone.cols, CV_8UC3);
    for (int row=0; row<grid_image_clone.rows; row++) {
        for (int col=0; col<grid_image_clone.cols; col++) {
            switch (grid_image_clone.at<unsigned char>(row,col))
            {
            case OBSTACLE:
                display_image.at<cv::Vec3b>(row,col)[0] = 255;//白色
                display_image.at<cv::Vec3b>(row,col)[1] = 255;
                display_image.at<cv::Vec3b>(row,col)[2] = 255;
                break;
            case PASSABLE:
                display_image.at<cv::Vec3b>(row,col)[0] = 0;//绿色
                display_image.at<cv::Vec3b>(row,col)[1] = 255;
                display_image.at<cv::Vec3b>(row,col)[2] = 0;
                break;
              case UNKNOWN:
                display_image.at<cv::Vec3b>(row,col)[0] = 0;//未知区域    黑色
                display_image.at<cv::Vec3b>(row,col)[1] = 0;
                display_image.at<cv::Vec3b>(row,col)[2] = 0;
                break;
            default:
                break;
            }
        }
    }
    // cv::namedWindow("display_image", 0);
    // cv::imshow("display_image", display_image);
    // cv::waitKey(1);

    // 更新地图
    // if( ((now_x- robot_x-10)>0) && ((now_y- robot_y+10)<height) && ((now_x- robot_x+10)<width) && ((now_y- robot_y-10)>0)){
      // std::cout<<"开始更新地图"<<std::endl;
      for (int px = -rect_width ; px<=rect_width; px++)
      {
        for (int py = -rect_height ; py<=rect_height; py++)
        { 
          if ( ((x_now_car+px)>=map_.info.width) ||  ((y_now_car+py)>=map_.info.height)   )     {
            continue;
          }
          switch (grid_image_clone.at<unsigned char>(py+rect_height,px+rect_width)){
            case UNKNOWN:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =-1;  
              break;
            case PASSABLE:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =0;  
              bayes_image.at<float>(y_now_car+py,x_now_car+px) += bayes_update(P_free_ray);
              break;
            case OBSTACLE:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =100;  
              //如果先进行bayes更新地图，再进行ray cast，那就没有必要了
              bayes_image.at<float>(y_now_car+py,x_now_car+px) += bayes_update(P_occ_ray);
              break;
            default:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =-1;  
              break;
          }
        }
      }
      // std::cout<<"使用 ray cast 法过滤掉部分未知栅格"<<std::endl;
    // }
}



/*
    利用ray cast方法过滤掉车辆当前位置附近的未知栅格
*/
void ray_cast_function_static(){
    // std::cout<<"使用ray cast-----------------------------------------"<<std::endl;
    //利用ray cast方法过滤掉车辆当前位置附近的未知栅格，圆圈的半径均设为10m，圆圈的外接矩形为10m*10m的正方形------------------------------------------------
    //0616  更新发送给raycast的区域大小和形状
    double rect_height_half = 20;
    double rect_width_half = 25.0;
    int rect_height = rect_height_half / resolution;//"圆圈"外接矩形长和宽的一半
    int rect_width = rect_width_half / resolution;
    // int radius=20.0/resolution;
    //车辆当前位置及其栅格坐标，或者可以直接用"// 使用里程计，经过的位置都可通行"这一步计算好的

    int x_now_car = round((0.5*width  ) / resolution) ;
    int y_now_car = round((0.5*height ) / resolution) ;

    //opencv图像坐标系 x右 y下 ----全局地图坐标系 x前 y左------这里暂时不管x、y轴的方向，因为都是周围
    cv::Mat grid_image =  -1 * cv::Mat::ones(cv::Size(2*rect_width+1, 2*rect_height+1), CV_8U);//初始化为 -1
    //车辆不在地图边缘，可以取附近10m的圆圈----这个后期可以使用一个自适应的更小的圆圈
    // if( ((now_x- robot_x-10)>0)&& ((now_x- robot_x+10)<height)  && ((now_y- robot_y)>10-width)  && ((now_y- robot_y)<-10)){
      // std::cout<<"初始化grid_image "<<std::endl;
      for (int px = -rect_width ; px<=rect_width; px++)
      {
        for (int py = -rect_height ; py<=rect_height; py++)
        { //将该部分栅格属性赋值给grid_image
          if ( ((x_now_car+px)>=map_.info.width) ||  ((y_now_car+py)>=map_.info.height)   )     {
            continue;
          }
          switch (map_.data[x_now_car+px + (y_now_car+py)*map_.info.width]){
            case -1:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=UNKNOWN;
              break;
            case 0:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=PASSABLE;
              break;
            case 100:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=OBSTACLE;
              break;
            default:
              grid_image.at<unsigned char>(py+rect_height,px+rect_width)=UNKNOWN;
              break;
          }
        }
      }
    // }

    cv::Mat display_image1 = cv::Mat::zeros(grid_image.rows, grid_image.cols, CV_8UC3);
    for (int row=0; row<grid_image.rows; row++) {
        for (int col=0; col<grid_image.cols; col++) {
            switch (grid_image.at<unsigned char>(row,col))
            {
            case OBSTACLE:
                display_image1.at<cv::Vec3b>(row,col)[0] = 255;//白色
                display_image1.at<cv::Vec3b>(row,col)[1] = 255;
                display_image1.at<cv::Vec3b>(row,col)[2] = 255;
                break;
            case PASSABLE:
                display_image1.at<cv::Vec3b>(row,col)[0] = 0;//绿色
                display_image1.at<cv::Vec3b>(row,col)[1] = 255;
                display_image1.at<cv::Vec3b>(row,col)[2] = 0;
                break;
              case UNKNOWN:
                display_image1.at<cv::Vec3b>(row,col)[0] = 0;//未知区域    黑色
                display_image1.at<cv::Vec3b>(row,col)[1] = 0;
                display_image1.at<cv::Vec3b>(row,col)[2] = 0;
                break;
            default:
                break;
            }
        }
    }
    // cv::namedWindow("display_image1", 0);
    // cv::imshow("display_image1", display_image1);
    // cv::waitKey(1);

    cv::Mat grid_image_clone = grid_image.clone();
    // known_area_extraction(grid_image_clone,   cv::Point(rect_width, rect_height),   radius);
    known_area_extraction(grid_image_clone,   cv::Point(rect_width, rect_height),rect_width,rect_height);//0616
    cv::Mat display_image = cv::Mat::zeros(grid_image_clone.rows, grid_image_clone.cols, CV_8UC3);
    for (int row=0; row<grid_image_clone.rows; row++) {
        for (int col=0; col<grid_image_clone.cols; col++) {
            switch (grid_image_clone.at<unsigned char>(row,col))
            {
            case OBSTACLE:
                display_image.at<cv::Vec3b>(row,col)[0] = 255;//白色
                display_image.at<cv::Vec3b>(row,col)[1] = 255;
                display_image.at<cv::Vec3b>(row,col)[2] = 255;
                break;
            case PASSABLE:
                display_image.at<cv::Vec3b>(row,col)[0] = 0;//绿色
                display_image.at<cv::Vec3b>(row,col)[1] = 255;
                display_image.at<cv::Vec3b>(row,col)[2] = 0;
                break;
              case UNKNOWN:
                display_image.at<cv::Vec3b>(row,col)[0] = 0;//未知区域    黑色
                display_image.at<cv::Vec3b>(row,col)[1] = 0;
                display_image.at<cv::Vec3b>(row,col)[2] = 0;
                break;
            default:
                break;
            }
        }
    }
    // cv::namedWindow("display_image", 0);
    // cv::imshow("display_image", display_image);
    // cv::waitKey(1);

    // 更新地图
    // if( ((now_x- robot_x-10)>0) && ((now_y- robot_y+10)<height) && ((now_x- robot_x+10)<width) && ((now_y- robot_y-10)>0)){
      // std::cout<<"开始更新地图"<<std::endl;
      for (int px = -rect_width ; px<=rect_width; px++)
      {
        for (int py = -rect_height ; py<=rect_height; py++)
        { 
          if ( ((x_now_car+px)>=map_.info.width) ||  ((y_now_car+py)>=map_.info.height)   )     {
            continue;
          }
          switch (grid_image_clone.at<unsigned char>(py+rect_height,px+rect_width)){
            case UNKNOWN:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =-1;  
              break;
            case PASSABLE:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =0;  
              bayes_image.at<float>(y_now_car+py,x_now_car+px) += bayes_update(P_free_ray);
              break;
            case OBSTACLE:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =100;  
              //如果先进行bayes更新地图，再进行ray cast，那就没有必要了
              bayes_image.at<float>(y_now_car+py,x_now_car+px) += bayes_update(P_occ_ray);
              break;
            default:
              map_.data[x_now_car+px + (y_now_car+py)*map_.info.width] =-1;  
              break;
          }
        }
      }
      // std::cout<<"使用 ray cast 法过滤掉部分未知栅格"<<std::endl;
    // }
}