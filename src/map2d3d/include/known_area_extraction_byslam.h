#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
enum GridType {UNKNOWN, PASSABLE, OBSTACLE};

#ifndef KNOW_AREA_EXTRACTION_H
#define KNOW_AREA_EXTRACTION_H
/*
    利用ray cast方法减少未知栅格
*/
// void known_area_extraction(cv::Mat& src_image, cv::Point vehicle_position, int radius) {
void known_area_extraction(cv::Mat& src_image, cv::Point vehicle_position, int rect_width, int rect_height) {
    // std::cout<<"使用 ray cast 方法减少未知栅格"<<std::endl;
    const static int GAP_NUM_THRESHOLD =5;
    cv::Mat dilate_img;
    cv::Mat element_large = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(2,2));
    cv::Mat element_small = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(1,1));
    // cv::dilate(src_image, dilate_img, element_large);

    // cv::Mat known_img = cv::Mat::zeros(cv::Size(radius*2+1,radius*2+1), src_image.type());
    // cv::Point known_img_center(radius,radius);
    // std::vector<cv::Point> circle_points;
    // cv::ellipse2Poly(known_img_center,cv::Size(radius,radius),0,0,360,1,circle_points);

    cv::Mat known_img = cv::Mat::zeros(cv::Size(2*rect_width+1, 2*rect_height+1) , src_image.type());
    cv::Point known_img_center(rect_width,rect_height);
    std::vector<cv::Point> circle_points;
    // cv::ellipse2Poly(known_img_center,cv::Size(rect_width,rect_height),0,0,360,1,circle_points);
    cv::ellipse2Poly(known_img_center,cv::Size(rect_width,rect_height),yaw_angle,0,360,2,circle_points);

    for(const cv::Point& pt : circle_points)
    {
        cv::LineIterator lit(src_image, vehicle_position, pt-known_img_center+vehicle_position);
        cv::LineIterator litknown(known_img, known_img_center, pt);
        bool last_known = true;
        int gap_num = 0;
        int count_obstacle=0;//较远障碍物的数量阈值
        for(int i=0; i<lit.count; i++,litknown++,lit++)
        {
            if(count_obstacle>=1){
                break;
            }
            const unsigned char& srcdata = *(*lit);
            unsigned char& data = *(*litknown);
            if(last_known)
            {
                if(srcdata == OBSTACLE)
                {
                    // if(i>50){//较远距离的阈值，这里设为150，分辨率为0.05，即7.5m
                        count_obstacle++;
                    // }
                    last_known = false;
                    gap_num = 0;
                    // break;
                }
                else
                    data = PASSABLE;
            }
            else
            {
                if((gap_num >= GAP_NUM_THRESHOLD) && (srcdata == PASSABLE))
                {
                    last_known = true;
                    gap_num = 0;
                }
                else if(srcdata == OBSTACLE){
                    gap_num = 0;
                    if(i>50){
                        count_obstacle++;
                    }
                }
                    else
                    {
                        gap_num++;
                        data = UNKNOWN;
                    }
            }
        }
        // cout<<"count_obstacle"<<count_obstacle<<endl;
    }

    cv::dilate(known_img,known_img,element_small);
    for(int j=0; j<known_img.rows; j++)
    {
        int src_j = j - known_img_center.y + vehicle_position.y;
        if((src_j<0) || (src_j>=src_image.rows)) continue;
        unsigned char* pdata = known_img.ptr<unsigned char>(j);
        unsigned char* srcdata = src_image.ptr<unsigned char>(src_j);
        for(int i=0; i<known_img.cols; i++)
        {
            int src_i = i - known_img_center.x + vehicle_position.x;
            if((src_i<0) || (src_i>=src_image.cols)) continue;
            srcdata[src_i] = std::max(srcdata[src_i], pdata[i]);
        }
    }
}

#endif