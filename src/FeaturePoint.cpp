//
// Created by root on 18-10-24.
//
#include"FeaturePoint.h"
#include <iostream>
FeaturePoint::FeaturePoint(Eigen::Vector2d&measure,int frame_id){
	Eigen::Vector2d unit_measure = (measure - Eigen::Vector2d(367.215, 248.375)).array() / (Eigen::Vector2d(458.654, 457.296).array());
    measuremnets.insert({frame_id,unit_measure });
};
void FeaturePoint::addMeasure(Eigen::Vector2d&measure,int frame_id){
	Eigen::Vector2d unit_measure = (measure - Eigen::Vector2d(367.215, 248.375)).array() / (Eigen::Vector2d(458.654, 457.296).array());
    measuremnets.insert({frame_id,unit_measure });
}
Eigen::Vector2d FeaturePoint::latestMeasure(){
    if(measuremnets.empty()){
        std::cerr<<"feaure track is empty!"<<std::endl;
    }
    return measuremnets.rbegin()->second;
}

