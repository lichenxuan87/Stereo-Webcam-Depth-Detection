/*
 * CameraAreaSelector.cpp
 *
 *  Created on: Sep 14, 2018
 *      Author: peter
 */

#include "CameraAreaSelector.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "Camera_consts.h"

using namespace std;
using namespace cv;





cv::Rect CameraAreaSelector::selectAreaByViewerPosition(cv::Vec4f position)
{
    double x = position[0];
    double y = position[1];
    double z = position[2];
    double distance = position[3];

    double horizonAngle = asin(x/distance) * 180 / PI;
    double verticalAngle = asin(y/distance) * 180 / PI;

    if ((abs(distance - m_prevDistance) < 3)
        && (abs(horizonAngle - m_prevHorizonAngle) < 1)
        && (abs(verticalAngle - m_prevVerticalAngle) < 1) )
    {
        return m_prevArea;
    }
    else
    {
        double DISP_AREA_HEIGHT = DISP_AREA_WIDTH*tan(VERTICAL_VIEW_ANGLE)/tan(HORIZON_VIEW_ANGLE);
        cv::Rect select_rect;
        select_rect.x = (int)((CAMERA_IMAGE_DISTANCE*tan(HORIZON_VIEW_ANGLE) + (x-4.2)*CAMERA_IMAGE_DISTANCE/z - (CAMERA_IMAGE_DISTANCE+z)*DISP_AREA_WIDTH/(2*z))*CAMERA_X_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*tan(HORIZON_VIEW_ANGLE)));
        select_rect.y = (int)((CAMERA_IMAGE_DISTANCE*tan(VERTICAL_VIEW_ANGLE) - CAMERA_IMAGE_DISTANCE*y/z + (CAMERA_IMAGE_DISTANCE+z)*CAPTURE_CAMERA_HEIGHT/z)*CAMERA_Y_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*tan(VERTICAL_VIEW_ANGLE)));
        select_rect.width = (int)((CAMERA_IMAGE_DISTANCE + z)*DISP_AREA_WIDTH*CAMERA_X_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*z*tan(HORIZON_VIEW_ANGLE)));
        select_rect.height = (int)((CAMERA_IMAGE_DISTANCE + z)*DISP_AREA_HEIGHT*CAMERA_Y_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*z*tan(VERTICAL_VIEW_ANGLE)));

        std::cout<<"For input x: " << x
                << " y: " << y
                << " z: " << z
                << " d: " << distance
                << " H-Angle: " << horizonAngle
                << " V-Angle: " << verticalAngle << endl;

        std::cout<<"Selected area in Camera c "<< "x: " << select_rect.x
            << " y: " << select_rect.y
            << " width: " << select_rect.width
            << " height: " << select_rect.height << endl;
        m_prevArea = select_rect;
        m_prevDistance = distance;
        m_prevHorizonAngle = horizonAngle;
        m_prevVerticalAngle = verticalAngle;

        return select_rect;
    }
}

