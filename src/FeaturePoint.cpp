//
// Created by root on 18-10-24.
//
#include"FeaturePoint.h"
#include <iostream>
FeaturePoint::FeaturePoint(Eigen::Vector2d&measure,int frame_id){
    measuremnets.insert({frame_id,measure});
};
void FeaturePoint::addMeasure(Eigen::Vector2d&measure,int frame_id){
    measuremnets.insert({frame_id,measure});
}
Eigen::Vector2d FeaturePoint::latestMeasure(){
    if(measuremnets.empty()){
        std::cerr<<"feaure track is empty!"<<std::endl;
    }
    return measuremnets.rbegin()->second;
}

