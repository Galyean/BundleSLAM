//
// Created by root on 18-10-24.
//
#include <iostream>
#include <stdio.h>
#include <string>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "EurocIO.hpp"
#include "FeatureTrack.h"

using namespace std;

int main(){
    const string euroc_dir = "D:/e_data/EuRoC_MAV/MH_01_easy/";
    std::map<double,string> image_files;
    utility::Euroc_io::loadImgs(euroc_dir,image_files);

    FeatureTrack tracker;
    int image_id=0;
    cv::Mat pre_image;
    for(auto it = image_files.begin(),it_end = image_files.end();
    it!=it_end;++it){
        image_id++;
        // read image and do undistortion
        cv::Mat image_gray = cv::imread(it->second,CV_LOAD_IMAGE_GRAYSCALE);

        cv::Mat map1,map2;
        cv::Matx33f mK(458.654,0.f,367.215,
                       0,457.296,248.375,
                       0.f,0.f,1.f);
        cv::Vec<float, 4>DistCoef(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
        cv::initUndistortRectifyMap(
                mK,
                DistCoef, cv::Mat(), mK,
                cv::Size(image_gray.cols, image_gray.rows),
                CV_16SC2, map1, map2
        );
        cv::imshow("distort image",image_gray);
        cv::waitKey(1);
        cv::Mat img_undistorted;
        cv::remap(image_gray, img_undistorted, map1, map2, cv::INTER_LINEAR);
        image_gray = img_undistorted;
        FeaturesVector features = tracker.processImage(image_gray,image_id);
        if(!features.empty()){
            cv::Mat mergedImage(image_gray.rows, image_gray.cols * 2, CV_8UC1);
            pre_image.copyTo(cv::Mat(mergedImage, cv::Rect(0, 0, image_gray.cols, image_gray.rows)));
            image_gray.copyTo(cv::Mat(mergedImage, cv::Rect(image_gray.cols, 0, image_gray.cols, image_gray.rows)));
            cv::cvtColor(mergedImage, mergedImage, CV_GRAY2BGRA);
            for (size_t i = 0; i < features.size(); i++)
            {
                auto it = features[i]->measuremnets.rbegin();
                cv::Point2f cur_pt((it->second)[0],(it->second)[1]);
                it++;
                cv::Point2f ref_kpt((it->second)[0],(it->second)[1]);
                cv::line(mergedImage, ref_kpt, cv::Point(cur_pt.x + image_gray.cols, cur_pt.y), cv::Scalar(0, 255, 0), 1);
                cv::circle(mergedImage, ref_kpt, 2, cv::Scalar(0, 0, 255), 2);
                cv::circle(mergedImage, cv::Point(cur_pt.x + image_gray.cols, cur_pt.y), 2, cv::Scalar(255, 0, 0), 2);
            }
            cv::imshow("feature tracking",mergedImage);
            cv::waitKey(20);
            //std::getchar();
        }


        cv::imshow("test",image_gray);
        cv::waitKey(1);
        pre_image = image_gray;
        cv::waitKey(1);
        it++;
    }
}

