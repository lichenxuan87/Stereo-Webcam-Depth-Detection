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
    CameraAreaSelector(): m_prevDistance(0), m_prevHorizonAngle(0), m_prevVerticalAngle(0), m_prevArea()
    {};

    /*
     * 参考<相机位置标定_0917_1.0.vsdx>
     * Position: 参数（x,y,z）为相机检测到的人的具体坐标情况。z即为文档中对应的d1
     *
     * Return:
     * select_rect.x 截取区域左上角的X像素坐标
     * select_rect.y 截取区域左上角的y像素坐标
     * select_rect.width 截取区域的X轴像素宽度
     * select_rect.height 截取区域的y轴像素高度
     *
     */
    cv::Rect selectAreaByViewerPosition(cv::Vec4f position);

private:
    float m_prevDistance;
    float m_prevHorizonAngle;
    float m_prevVerticalAngle;

    cv::Rect m_prevArea;

};



#endif /* STEREO_WEBCAM_DEPTH_DETECTION_SOURCE_CODE_OPENCVPROJECT_CAMERAAREASELECTOR_H_ */
