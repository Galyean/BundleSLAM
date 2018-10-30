//
// Created by root on 18-10-29.
//
#include "InitialSFM.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core.hpp>
#include <cv.hpp>
#include <iostream>
bool InitialSFM::tryReConstruct(FeaturesVector &features, CameraBuffer&poses){
	// find biggest parallax 
	int start = -1;
	int end = -1;
	double average_parallax = 0;
	std::vector<cv::Point2f> points1, points2;
	for (int i = 0; i < poses.size() - 1; ++i) {
		for (int j = i + 1; j < poses.size(); ++j) {
			std::vector<cv::Point2f> points1_temp, points2_temp;
			double sum_parallax = 0;
			for (auto feature : features) {
				auto first_it  = feature->measuremnets.find(poses[i].camera_id);
				auto second_it = feature->measuremnets.find(poses[j].camera_id);
				if (first_it != feature->measuremnets.end() && second_it != feature->measuremnets.end()) {
					points1_temp.emplace_back(cv::Point2f(first_it->second[0], first_it->second[1]));
					points2_temp.emplace_back(cv::Point2f(second_it->second[0], second_it->second[1]));
					sum_parallax += (first_it->second - second_it->second).norm();
				}
			}
			if (points1_temp.size() > 30 && sum_parallax / points1_temp.size() > average_parallax) {
				average_parallax = sum_parallax / points1_temp.size();
				start = i;
				end = j;
				points1.swap(points1_temp);
				points2.swap(points2_temp);
			}
		}
	}
	// compute fundemental matrix and decompose to rotation and translation
	if (start < 0) {
		return false;
	}
	std::vector<uchar> status;
	cv::Mat essential = cv::findEssentialMat(points1, points2);
	cv::Mat rot, trans;
	int inlier_cnt = cv::recoverPose(essential, points1, points2, rot, trans);
	std::cout << "inliers count : " << inlier_cnt << std::endl;
	// reconstruct

	// optimize

	return false;
}
