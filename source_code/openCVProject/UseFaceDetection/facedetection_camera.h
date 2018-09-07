/*
 * facedetection_camera.h
 *
 *  Created on: Sep 7, 2018
 *      Author: peter
 */

#ifndef STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_USEFACEDETECTION_FACEDETECTION_CAMERA_H_
#define STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_USEFACEDETECTION_FACEDETECTION_CAMERA_H_



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


std::vector<cv::Rect> detectFaceLocation(cv::Mat& frame);


#endif /* STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_USEFACEDETECTION_FACEDETECTION_CAMERA_H_ */
