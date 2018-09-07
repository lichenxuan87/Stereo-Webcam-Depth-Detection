// File: globaldata.h
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013

#ifndef GLOBALDATA_H
#define GLOBALDATA_H


#include "opencv/cvaux.h"
#include <opencv/highgui.h>



namespace global_data{
	extern unsigned cameraNum;
	extern cv::StereoBM BMState;
	extern cv::Size camStreamSize;
	extern CvStereoGCState *GCState;
	extern cv::Mat image_left;
    extern cv::Mat image_right;
	extern cv::Mat image_left_rectified;
    extern cv::Mat image_right_rectified;
}


#endif //GLOBALDATA_H





