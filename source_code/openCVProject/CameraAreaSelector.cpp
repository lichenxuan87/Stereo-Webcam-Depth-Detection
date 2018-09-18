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

#include "include/face_detection.h"

#define DISP_WINNANE "camera"
#define QUIT_KEY     'q'
#define CAMID         0

using namespace std;
using namespace cv;

/**
参考<相机位置标定_0917_1.0.vsdx>
参数（x,y,z）为相机检测到的人的具体坐标情况。z即为文档中对应的d1
DISP_SCREEN_WIDTH 显示区域的宽度，对应文档中的Ws
DISP_SCREEN_HEIGHT 显示区域的高度，对应文档中的Hs
CAPTURE_CAMERA_HEIGHT 外拍相机的中心点到显示屏幕顶端的距离，d对应文档中的Hc
CAMERA_IMAGE_DISTANCE 相机成像的假定距离，对应文档中的d2
X_AXIS_PIXEL_SIZE X轴方向像素大小
Y_AXIS_PIXEL_SIZE y轴方向像素大小
HORIZON_VIEW_ANGLE 横向可视角的一半 对应文档中的B角度
HORIZON_VIEW_ANGLE 纵向可视角的一半 对应文档中的a角度

select_rect.x 截取区域左上角的X像素坐标
select_rect.y 截取区域左上角的y像素坐标

select_rect.width 截取区域的X轴像素宽度
select_rect.height 截取区域的y轴像素高度



**/
#define PI 3.1415926
#define DISP_SCREEN_WIDTH 20.4
#define CAPTURE_CAMERA_HEIGHT 5.8
#define CAMERA_IMAGE_DISTANCE 100.0
#define X_AXIS_PIXEL_SIZE 640
#define Y_AXIS_PIXEL_SIZE 480
#define HORIZON_VIEW_ANGLE 26*PI/180   //b
#define VERTICAL_VIEW_ANGLE 13.5*PI/180   //a





cv::Rect CameraAreaSelector::selectAreaByViewerPosition(cv::Vec3f position)
{
    double x = position[0];
    double y = position[1];
    double z = position[2];
    double DISP_SCREEN_HEIGHT = DISP_SCREEN_WIDTH*tan(VERTICAL_VIEW_ANGLE)/tan(HORIZON_VIEW_ANGLE);
    //std::cout<<"DISP_SCREEN_HEIGHT 1: "<<DISP_SCREEN_WIDTH*tan(VERTICAL_VIEW_ANGLE) << std::endl;
    //std::cout<<"DISP_SCREEN_HEIGHT 2: "<<tan(HORIZON_VIEW_ANGLE) << std::endl;
    //std::cout<<"DISP_SCREEN_HEIGHT: "<<DISP_SCREEN_HEIGHT << std::endl;
    cv::Rect select_rect;
    select_rect.x = (int)((CAMERA_IMAGE_DISTANCE*tan(HORIZON_VIEW_ANGLE) + (x-4.2)*CAMERA_IMAGE_DISTANCE/z - (CAMERA_IMAGE_DISTANCE+z)*DISP_SCREEN_WIDTH/(2*z))*2*CAMERA_IMAGE_DISTANCE*tan(HORIZON_VIEW_ANGLE)/X_AXIS_PIXEL_SIZE);
    select_rect.y = (int)((CAMERA_IMAGE_DISTANCE*tan(VERTICAL_VIEW_ANGLE) + CAMERA_IMAGE_DISTANCE*y/z - (CAMERA_IMAGE_DISTANCE+z)*CAPTURE_CAMERA_HEIGHT/z)*2*CAMERA_IMAGE_DISTANCE*tan(VERTICAL_VIEW_ANGLE)/Y_AXIS_PIXEL_SIZE);
    select_rect.width = (int)((CAMERA_IMAGE_DISTANCE + z)*DISP_SCREEN_WIDTH*X_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*z*tan(HORIZON_VIEW_ANGLE)));
    select_rect.height = (int)((CAMERA_IMAGE_DISTANCE + z)*DISP_SCREEN_HEIGHT*Y_AXIS_PIXEL_SIZE/(2*CAMERA_IMAGE_DISTANCE*z*tan(VERTICAL_VIEW_ANGLE)));



    return select_rect;
}

