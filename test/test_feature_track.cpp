//
// Created by root on 18-10-24.
//
#include<iostream>
#include<stdio.h>
#include<string>
#include<map>
#include<opencv2/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "EurocIO.hpp"


using namespace std;
int main(){
    const string euroc_dir = "/home/galyean/new_work_space/MH_01_easy/";
    std::map<double,string> image_files;
    utility::Euroc_io::loadImgs(euroc_dir,image_files);
    for(auto it = image_files.begin(),it_end = image_files.end();
    it!=it_end;++it){
        cv::Mat image_gray = cv::imread(it->second,CV_LOAD_IMAGE_GRAYSCALE);
        cv::imshow("test",image_gray);
        cv::waitKey(1);
    }
}

