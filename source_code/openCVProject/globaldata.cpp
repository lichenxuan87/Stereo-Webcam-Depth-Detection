// File: globaldata.cc
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013


#include "globaldata.h"

#include "stdafx.h"

//Name Space to hold globally shared data.
namespace global_data{
	unsigned cameraNum;
	bool isUseBM;
	cv::Ptr<cv::StereoSGBM> stereoSGBM;
	cv::Ptr<cv::StereoBM> stereoBM;
	cv::Size camStreamSize;
	cv::Mat image_left;
	cv::Mat image_right;
	cv::Mat image_left_rectified;
	cv::Mat image_right_rectified;
}
