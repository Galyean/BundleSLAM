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
class FeatureTrack{
public:
    FeatureTrack(int maxcorners=100):max_corners(maxcorners){};
    ~FeatureTrack(){};
    void processImage(cv::Mat &gray_image,int image_id);
private:
    std::vector<std::shared_ptr<FeaturePoint> > features;
    int max_corners;
    std::vector<cv::Mat> ref_pyr;
};
#endif //BUNDLESLAM_FEATURETRACK_H
