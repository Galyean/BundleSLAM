#include <iostream>
#include <stdio.h>
#include <string>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "EurocIO.hpp"
#include "FeatureTrack.h"
#include "CameraBuffer.h"
using namespace std;

int main() {
	const string euroc_dir = "D:/e_data/EuRoC_MAV/MH_01_easy/";
	std::map<double, string> image_files;
	utility::Euroc_io::loadImgs(euroc_dir, image_files);

	FeatureTrack tracker;
	CameraBuffer camera_buffer;
	int image_id = 0;
	for (auto it = image_files.begin(), it_end = image_files.end();
		it != it_end; ++it) {
		image_id++;
		// read image and do undistortion
		cv::Mat image_gray = cv::imread(it->second, CV_LOAD_IMAGE_GRAYSCALE);

		cv::Mat map1, map2;
		cv::Matx33f mK(458.654, 0.f, 367.215,
			0, 457.296, 248.375,
			0.f, 0.f, 1.f);
		cv::Vec<float, 4>DistCoef(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
		cv::initUndistortRectifyMap(
			mK,
			DistCoef, cv::Mat(), mK,
			cv::Size(image_gray.cols, image_gray.rows),
			CV_16SC2, map1, map2
		);

		cv::Mat img_undistorted;
		cv::remap(image_gray, img_undistorted, map1, map2, cv::INTER_LINEAR);
		image_gray = img_undistorted;
		FeaturesVector features = tracker.processImage(image_gray, image_id);
		camera_buffer.emplace_back(CameraPose(0));
		

	}
}