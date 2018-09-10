//CVStereoCorrespondance.h
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013


#ifndef CVSTEREOCORRESPONDANCE_H
#define CVSTEREOCORRESPONDANCE_H

#include "CVStereoWebcam.h"
#include "globaldata.h"
#include "stdafx.h"
#include <vector>


void sliderHandler(void);

void init_sliderHandler(void);

void init_correspondance_data(void);

void calculate_correspondance_data(cv::Mat& image_left_undistorted, cv::Mat& image_right_undistorted, cv::Mat& Q);

int calculate_correspondance_depth(cv::Point left_point, cv::Mat& Q);

void calculate_correspondance_depth_tracking(std::vector<cv::KeyPoint> keyPoints, cv::Mat& Q);

double calculate_correspondance_depth_tracking(cv::Rect boundary);

cv::Mat GetThresholdedImage_RED(cv::Mat& frame);

void calculate_correspondance_data_save(cv::Mat& image_left_undistorted, cv::Mat& image_right_undistorted, cv::Mat& Q);

namespace correspondance_data{
	
    extern cv::Mat disp_left ;
    extern cv::Mat vdisp_left ;
    extern cv::Mat smoothedvdisp_left ;
    extern cv::Mat Image3D_left ;
    extern cv::Mat grey_left_r ;
    extern cv::Mat grey_right_r;
    extern cv::Mat disp_right ;
    extern cv::Mat vdisp_right ;
    extern cv::Mat smoothedvdisp_right;
    extern cv::Mat Image3D_right ;
    extern cv::Mat threholded_image;
    extern cv::Mat disp_left_memory;
    extern cv::Mat vdisp_left_memory;
    extern cv::Mat pixel_norm_disp;
}



#endif
