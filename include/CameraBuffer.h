//
// Created by root on 18-10-29.
//

#ifndef BUNDLESLAM_CAMERABUFFER_H
#define BUNDLESLAM_CAMERABUFFER_H
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <vector>
#include <map>
#include"FeaturePoint.h"
class CameraPose{
public:
	CameraPose(int cameraId) :camera_id(cameraId) {};
    int camera_id;
	Eigen::Quaterniond q_c_g;
	double time_stamp;
	std::map<int, std::shared_ptr<FeaturePoint> > features;
};
typedef std::vector<CameraPose> CameraBuffer;
//class CameraBuffer{
//public:
//	std::vector<CameraPose> cameras;
//};
#endif //BUNDLESLAM_CAMERABUFFER_H
