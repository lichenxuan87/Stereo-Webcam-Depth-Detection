/*
 * utils.cpp
 *
 *  Created on: Sep 18, 2018
 *      Author: peter
 */
#include "utils.h"

using namespace cv;
void drawTextInsideBlackBox(Mat& img, char* text, Point centerPoint, int fontFace, double fontScale, int thickness)
{
    int baseline=0;
    Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);

    baseline += thickness;

    // Upper left
    Point textOrgUpper(centerPoint.x - textSize.width/2, centerPoint.y - textSize.height/2);
    // Lower left
    Point textOrgLower(centerPoint.x - textSize.width/2, centerPoint.y + textSize.height/2);

    // Draw text background to black
    Rect textBox(0, textOrgUpper.y - baseline, img.cols, textSize.height + 2*baseline);
    Mat textBoxArea = img(textBox);
    textBoxArea = Scalar(255, 0, 0);

    putText(img, text, textOrgLower, fontFace, fontScale,
            Scalar::all(255), thickness, 1);
}

