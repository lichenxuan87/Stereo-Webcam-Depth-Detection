// File: globaldata.h
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013

#ifndef GLOBALDATA_H
#define GLOBALDATA_H


#include <opencv2/opencv.hpp>


namespace global_data{
	extern unsigned cameraNum;
	extern bool isUseBM;
	extern cv::Ptr<cv::StereoSGBM> stereoSGBM;
	extern cv::Ptr<cv::StereoBM> stereoBM;
	extern cv::Size camStreamSize;
	//extern CvStereoGCState *GCState;
	extern cv::Mat image_left;
    extern cv::Mat image_right;
    extern cv::Mat image_c;
	extern cv::Mat image_left_rectified;
    extern cv::Mat image_right_rectified;
    extern cv::Mat fullScreenImage;
}


#endif //GLOBALDATA_H






