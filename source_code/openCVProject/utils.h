/*
 * utils.h
 *
 *  Created on: Sep 18, 2018
 *      Author: peter
 */

#ifndef SOURCE_CODE_OPENCVPROJECT_UTILS_H_
#define SOURCE_CODE_OPENCVPROJECT_UTILS_H_

#include <opencv2/opencv.hpp>

void drawTextInsideBlackBox(cv::Mat& img, char* text, cv::Point centerPoint, int fontFace, double fontScale = 1, int thickness = 3);


#endif /* SOURCE_CODE_OPENCVPROJECT_UTILS_H_ */
