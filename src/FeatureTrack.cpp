//
// Created by root on 18-10-24.
//
#include <cv.hpp>
#include"FeatureTrack.h"
#include "opencv2/imgproc/imgproc.hpp"
void FeatureTrack::processImage(cv::Mat &gray_image,int image_id){
    std::vector<cv::Mat> cur_pyr;
    cv::buildOpticalFlowPyramid(gray_image, cur_pyr, cv::Size(17, 17), 4,
            true, cv::BORDER_CONSTANT, cv::BORDER_CONSTANT, false);

    if(features.empty()){
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(gray_image,corners,max_corners,0.01,10);
        for(auto it=corners.begin(),it_end = corners.end();it!=it_end;++it){
            Eigen::Vector2d point(it->x,it->y);
            features.emplace_back(FeaturePoint_ptr(new FeaturePoint(point,image_id)));
        }
        return;
    }
    // do feature track
    std::vector<cv::Point2f> pre_points;
    for(auto feature:features){
        pre_points.emplace_back(cv::Point2f(feature->latestMeasure()[0],feature->latestMeasure()[1]));
    }
    std::vector<cv::Point2f> cur_points = pre_points;
    std::vector<float> err;
    std::vector<uchar> status;
    cv::calcOpticalFlowPyrLK(ref_pyr, cur_pyr, pre_points, cur_points, status, err, cv::Size(15, 15), 4,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.001),
                             cv::OPTFLOW_USE_INITIAL_FLOW);


}

