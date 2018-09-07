
#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

void StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated, float _squareSize);

int Begin_Calibration(const char* imageList,int nx,int ny, int useUncalibrated, float squareSize);

void showDisparity(std::string a, std::string b, bool c);
