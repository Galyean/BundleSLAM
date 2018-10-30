//
// Created by root on 18-10-24.
//
#include <iostream>
#include"FeatureTrack.h"
#include <cv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#define MIN_DIST 20
FeaturesVector FeatureTrack::processImage(cv::Mat &gray_image,int image_id){
    FeaturesVector cur_tracked_feature;
    std::vector<cv::Mat> cur_pyr;
    cv::buildOpticalFlowPyramid(gray_image, cur_pyr, cv::Size(15, 15), 4,
            true, cv::BORDER_CONSTANT, cv::BORDER_CONSTANT, false);

    if(features.empty()){
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(gray_image,corners,max_corners,0.01,MIN_DIST);
        for(auto it=corners.begin(),it_end = corners.end();it!=it_end;++it){
            Eigen::Vector2d point(it->x,it->y);
            features.emplace_back(FeaturePoint_ptr(new FeaturePoint(point,image_id)));
        }
        ref_pyr.swap(cur_pyr);
        return cur_tracked_feature;
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
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 40, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);
/*    // show feature position
    for(auto pre_point:pre_points){
        cv::circle(ref_pyr[0], pre_point, 2, cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("pre_features",ref_pyr[0]);
    cv::waitKey(1);
    for(auto cur_point:cur_points){
        cv::circle(cur_pyr[0], cur_point, 2, cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("cur_features",cur_pyr[0]);
    cv::waitKey(1);*/

    cv::Mat mask(gray_image.rows, gray_image.cols, CV_8UC1, cv::Scalar(1));
    refineTrackedFeatures(pre_points,cur_points,status,mask,gray_image.cols,gray_image.rows,image_id);

    cur_tracked_feature = features;
    // add new feature
    std::vector<cv::Point2f> new_corners;
    if(max_corners-features.size()){
        cv::goodFeaturesToTrack(gray_image,new_corners,max_corners-features.size(),0.01,MIN_DIST,mask);
        for(auto new_point:new_corners){
            Eigen::Vector2d point(new_point.x,new_point.y);
            features.emplace_back(FeaturePoint_ptr(new FeaturePoint(point,image_id)));
        }
    }

    ref_pyr.swap(cur_pyr);
    return cur_tracked_feature;
}
void FeatureTrack::refineTrackedFeatures(std::vector<cv::Point2f> &pre_points,std::vector<cv::Point2f> &cur_points,
        std::vector<uchar> &status,cv::Mat&mask,int cols,int rows,int image_id){
    //refine track result
    for (size_t i = 0; i < cur_points.size(); i++){

        if (status[i] && (cur_points[i].x < 0 || cur_points[i].x >= cols - 1
                          || cur_points[i].y < 0 || cur_points[i].y >= rows - 1))
            status[i] = 0;
    }

    int j = 0;
    for (size_t i = 0; i < cur_points.size(); i++)
    {

        if (status[i])
        {
            features[j] = features[i];// remove out-of-view features
            pre_points[j] = pre_points[i];
            cur_points[j] = cur_points[i];
            j++;
        }
    }

    features.resize(j);
    pre_points.resize(j);
    cur_points.resize(j);

    std::vector<uchar> status2;
    //std::cout<<"refine ref_pts : "<<ref_pts.size()<<" cur_pts : "<<cur_pts.size()<<std::endl;
    if(pre_points.size() <= 10 || cur_points.size() <= 10){
        features.resize(0);
        pre_points.resize(0);
        cur_points.resize(0);
        return;
    }
    j = 0;

    cv::findFundamentalMat(pre_points, cur_points, cv::FM_RANSAC, 3, 0.99, status2);


    for (size_t i = 0; i < pre_points.size(); i++)
    {
        if (status2[i])
        {
            features[j] = features[i];
            Eigen::Vector2d cur_measure(cur_points[i].x,cur_points[i].y);
            features[j]->addMeasure(cur_measure,image_id);
            j++;
            cv::circle(mask, cur_points[i], MIN_DIST, 0, -1);

        }
    }
    features.resize(j);
}

