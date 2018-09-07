//CVStereoWebCam.cpp
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013


#include "CVStereoWebcam.h"
#include "stdafx.h"
#include <iostream>


//Namespace to hode Capture Objects
namespace webcam_data{
    cv::VideoCapture capture_left;
    cv::VideoCapture capture_right;
}

using namespace std;
using namespace cv;

//Initialises Webcams
void webcam_init(int cam){
    cout << "webcam_init(int cam) Not implemented!" <<endl;
}

//Initialises Webcams
void webcam_init(int cam1,int cam2){

	webcam_data::capture_left.open(cam1);
	webcam_data::capture_right.open(cam2);

	global_data::cameraNum = 2;

	webcam_data::capture_left.read(global_data::image_left);
	webcam_data::capture_right.read(global_data::image_right);
	webcam_data::capture_left.read(global_data::image_left_rectified);
	webcam_data::capture_right.read(global_data::image_right_rectified);

	global_data::camStreamSize = global_data::image_left.size();


}

//Will uupdate the current frame
void webcam_update(void){
    webcam_data::capture_left.read(global_data::image_left);						// Get the first frame

	if (global_data::cameraNum == 2) {
	    webcam_data::capture_right.read(global_data::image_right);
	}

	//cvWaitKey(10);
}

//Updates the display windows
void display_update(const unsigned int DISP_ID){
	
	if(DISP_ID&CAM1)     imshow(CAM1_WIND, global_data::image_left_rectified);
	if(DISP_ID&CAM1_RAW) imshow(CAM1_RAW_WIND, global_data::image_left);

	if (global_data::cameraNum == 2) {
        if(DISP_ID&CAM2_RAW) imshow(CAM2_RAW_WIND, global_data::image_right);
        if(DISP_ID&CAM2)     imshow(CAM2_WIND, global_data::image_right_rectified);
	}

	//TODO: not use memory
    //if(DISP_ID&DISP)	 imshow(DISP_WIND, correspondance_data::vdisp_left_memory);

#if BLOB_TRACKING==1
	if(DISP_ID&PROJ)	 imshow(PROJ_WIND, correspondance_data::threholded_image);
#endif
	if(DISP_ID&RAW_DISP) imshow( RAW_DISPARITY_WIND,correspondance_data::vdisp_left);
	if(DISP_ID&PARAM)    imshow( PARAM_WIND,0);
}

//Creates named widnwos for the displays.
void display_create(const unsigned int DISP_ID){
	
	if(DISP_ID&CAM1)     cvNamedWindow(CAM1_WIND,0);
	if(DISP_ID&CAM1_RAW) cvNamedWindow(CAM1_RAW_WIND,0); 

	if (global_data::cameraNum == 2) {
        if(DISP_ID&CAM2)     cvNamedWindow(CAM2_WIND,0);
        if(DISP_ID&CAM2_RAW) cvNamedWindow(CAM2_RAW_WIND,0);
	}

	if(DISP_ID&DISP)     cvNamedWindow(DISP_WIND,0); 

#if BLOB_TRACKING==1
	if(DISP_ID&PROJ)     cvNamedWindow(PROJ_WIND,0); 
#endif

	if(DISP_ID&RAW_DISP) cvNamedWindow(RAW_DISPARITY_WIND,0); 
	if(DISP_ID&PARAM)    cvNamedWindow(PARAM_WIND,0);
}


//Destroys the Windows.
void display_destroy(const unsigned int DISP_ID){
	
	if(DISP_ID&CAM1)     cvDestroyWindow(CAM1_WIND);
	if(DISP_ID&CAM1_RAW) cvDestroyWindow(CAM1_RAW_WIND); 

	if (global_data::cameraNum == 2) {
        if(DISP_ID&CAM2)     cvDestroyWindow(CAM2_WIND);
        if(DISP_ID&CAM2_RAW) cvDestroyWindow(CAM2_RAW_WIND);
	}

	if(DISP_ID&DISP)     cvDestroyWindow(DISP_WIND); 
	if(DISP_ID&PROJ)     cvDestroyWindow(PROJ_WIND); 
	if(DISP_ID&RAW_DISP) cvDestroyWindow(RAW_DISPARITY_WIND); 
	if(DISP_ID&PARAM)    cvNamedWindow(PARAM_WIND); 
}

//Release the Webcam Object
void webcam_release(void){
	
	//cvReleaseCapture( &(webcam_data::capture_left)  );				// release camera 1

	//if (global_data::cameraNum == 2)
	//    cvReleaseCapture( &(webcam_data::capture_right) );				// release camera 2
}




