//
// Created by root on 18-10-24.
//

#ifndef BUNDLESLAM_FEATUREPOINT_H
#define BUNDLESLAM_FEATUREPOINT_H
#include<Eigen/Core>
#include<map>
class FeaturePoint{
public:
    FeaturePoint(Eigen::Vector2d&measure,int frame_id);
    ~FeaturePoint(){};
    void addMeasure(Eigen::Vector2d&measure,int frame_id);
    Eigen::Vector2d latestMeasure();
private:
    /**@brief a feature is tracked along many frames,[int] is the frame id. **/
    std::map<int,Eigen::Vector2d> measuremnets;
};
#endif //BUNDLESLAM_FEATUREPOINT_H
