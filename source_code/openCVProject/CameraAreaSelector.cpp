/*
 * CameraAreaSelector.cpp
 *
 *  Created on: Sep 14, 2018
 *      Author: peter
 */

#include "CameraAreaSelector.h"

using namespace cv;

Rect CameraAreaSelector::selectAreaByViewerPosition(cv::Vec3f position)
{
    return Rect(584, 0, 960, 720);
}
