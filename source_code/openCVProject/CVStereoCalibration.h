//CVStereoCalibration.h
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013

#ifndef CVSTEREOCALIBRATION_H
#define CVSTEREOCALIBRATION_H

#include <opencv/cv.h>

#include "CVStereoWebcam.h"
#include "globaldata.h"
#include "stdafx.h"
#include "stereo_calibrate.h"
#include "time.h"

const char* const CAPTURE_DIR = "Captures";

void CVStereo_Calibrate(bool recapture,int number_of_frames);

void CVStereo_Recapture(int number_of_frames);

void CVStereo_AutoCapture(int number_of_frames);

bool CVStereo_Capture(void);

bool CVStereo_Capture_Check(void);

#endif
