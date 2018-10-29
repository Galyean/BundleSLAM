//
// Created by root on 18-10-24.
//

#ifndef BUNDLESLAM_FEATURETRACK_H
#define BUNDLESLAM_FEATURETRACK_H
#include<vector>
#include<memory>
#include <opencv2/core.hpp>
#include"FeaturePoint.h"
typedef std::shared_ptr<FeaturePoint> FeaturePoint_ptr;
typedef std::vector<std::shared_ptr<FeaturePoint> > FeaturesVector;
class FeatureTrack{
public:
    FeatureTrack(int maxcorners=200):max_corners(maxcorners){};
    ~FeatureTrack(){};
    FeaturesVector processImage(cv::Mat &gray_image,int image_id);
private:
    void refineTrackedFeatures(std::vector<cv::Point2f> &pre_points,std::vector<cv::Point2f> &cur_points,
                               std::vector<uchar> &status,cv::Mat&mask,int cols,int rows,int image_id);
private:
    std::vector<std::shared_ptr<FeaturePoint> > features;
    int max_corners;
    std::vector<cv::Mat> ref_pyr;
};
#endif //BUNDLESLAM_FEATURETRACK_H
