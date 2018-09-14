/*
 * CameraAreaSelector.h
 *
 *  Created on: Sep 14, 2018
 *      Author: peter
 */

#ifndef STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_CAMERAAREASELECTOR_H_
#define STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_CAMERAAREASELECTOR_H_

#include <opencv2/opencv.hpp>

class CameraAreaSelector
{
public:
    CameraAreaSelector() {};

    cv::Rect selectAreaByViewerPosition(cv::Vec3f position);

};



#endif /* STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_CAMERAAREASELECTOR_H_ */
