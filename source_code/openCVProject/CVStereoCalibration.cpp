//CVStereoCalibration.cpp
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013


#include "CVStereoCalibration.h"
#include "stdafx.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

//Shared Calibration Data
namespace calibration_data{
	FILE* image_list_file; 
	int capture_number;
}

using namespace cv;

//Fucntion to be called from main.
void CVStereo_Calibrate(bool recapture,int number_of_frames){
	printf("Beginning Calibration ...\n");

	if(recapture == true){
		 CVStereo_Recapture(number_of_frames);
	}
	
	if(recapture == false || calibration_data::capture_number > 0){
		//Create Progress Bar
		printf("Checking Images for Chessboards...  \n");
		for(int count=0;count<(calibration_data::capture_number*2+1);count++) printf(" ");
		printf("]\r"); printf("[");

		Begin_Calibration(IMAGE_LIST_DIR,NX,NY,USE_UNCALIBRATED,(float) SQUARESIZE);
	}
}

//Capture of new images sequence.
void CVStereo_Recapture(int number_of_frames){

	// Needs To Save A bunch of Stereo images and Corresponding text file, containing image locations
	bool stillCapturing=TRUE;
	char key=0;
	int count=0;
	clock_t time_start=0;

		
	calibration_data::image_list_file = fopen(IMAGE_LIST_DIR, "w");
	printf("\nPress SPACE to Save Chessboard Capture and ESC to Finish Capturing...\n");

	while(stillCapturing==TRUE){
		key=cvWaitKey(10);
	
		// GET NEXT FRAME
		webcam_update();

		// UPDATE DISPLAY
		display_update(CAM1_RAW|CAM2_RAW);


		// END ONCE ESC KEY PRESSED OR ENOUGH CAPTURED
		if(key==ESC_KEY){
			stillCapturing=FALSE;
			printf("\nEnding Capture Sequence ... Done\n\n");
		}

		// AUTOCAPTURE SEQUENCE
		if(key==C_KEY || AUTO_CAPTURE==1){

				//AUTOCAPTURE_COUNTDOWN
				printf("\nAuto capturing in ... %i \r",count);
				for(count=(AUTO_CAPTURE_DELAY-1);count>0;count--){
					printf("Auto capturing in ...  %i \r",count);
					time_start=clock();
					while(((clock()-time_start)<ONE_SECOND)){
						webcam_update();
						display_update(CAM1_RAW|CAM2_RAW);
					}
				}

				printf("Auto capturing in ... Started.\n");
				printf("\nCapturing ...\n\n");

				CVStereo_AutoCapture(number_of_frames);
		}

		// MANAUL CAPTURE
		if(key==SPACE_BAR_KEY ){
			//CHECK FOR CHESSBOARD
			if(CVStereo_Capture_Check()==TRUE){
				CVStereo_Capture();
			}
			else{
				printf("No Chessboard Detected!\n");
			}
			cvWaitKey(500);
		}	
				
	}
	fclose(calibration_data::image_list_file);	
}

//Capture and save frame.
bool CVStereo_Capture(void){

		char capture_save_name_left[50];
		char capture_save_name_right[50];

		struct stat buf;
        if(stat(CAPTURE_DIR, &buf) != 0)
        {
             printf("Directory doesn't exists: %s\n", CAPTURE_DIR);

             if (0 == mkdir(CAPTURE_DIR, 666)) {
                 printf("Directory %s created success!\n", CAPTURE_DIR);
             } else {
                 printf("Directory %s created FAILED!\n", CAPTURE_DIR);
                 return false;
             }
        }

		sprintf(capture_save_name_left ,"%s/left%.2u.ppm", CAPTURE_DIR, ++calibration_data::capture_number);
		sprintf(capture_save_name_right,"%s/right%.2u.ppm", CAPTURE_DIR, calibration_data::capture_number  );

		fputs(capture_save_name_left,calibration_data::image_list_file );
		fputs("\n",calibration_data::image_list_file);

		fputs(capture_save_name_right,calibration_data::image_list_file);
		fputs("\n",calibration_data::image_list_file);

		printf("Saving images, ");
		printf(capture_save_name_left);
		printf(" & ");
		printf(capture_save_name_right);
		printf("\n");

		imwrite(capture_save_name_left ,global_data::image_left );

		if (global_data::cameraNum == 2)
		    imwrite(capture_save_name_right,global_data::image_right);

		return true;
}

//Check Frame for chessboard.
bool CVStereo_Capture_Check(void){

	int found_left=0,found_right=0;
	int count_chessboard=0;
	vector<cv::Point2f> corners;

	found_left = findChessboardCorners( global_data::image_left, Size(NX, NY),
	                                        corners,
											CV_CALIB_CB_ADAPTIVE_THRESH |
											CV_CALIB_CB_NORMALIZE_IMAGE);

	if (global_data::cameraNum == 2) {
	    found_right = findChessboardCorners( global_data::image_right, Size(NX, NY),
	                                        corners,
											CV_CALIB_CB_ADAPTIVE_THRESH |
											CV_CALIB_CB_NORMALIZE_IMAGE);
	}

	if (global_data::cameraNum == 2) {
        if(found_left&&found_right)
            return true;
        else
            return false;
	} else {
	    return found_left != 0;
	}
}


//Autocapture Sequence.
void CVStereo_AutoCapture(int number_of_frames){

	clock_t time_start=0;
	

	while((calibration_data::capture_number<number_of_frames)&&(cvWaitKey(10)!=ESC_KEY)){
					
		if(CVStereo_Capture_Check()){
			CVStereo_Capture();
		}
		else{
			printf("No Chessboard Detected!\n");
		}				

		time_start=clock();
		while((clock()-time_start)<SECONDS_BETWEEN_CAPTURES){

			// GET NEXT FRAME
			webcam_update();

			// UPDATE DISPLAY
			display_update(CAM1_RAW|CAM2_RAW);	
		}

	}
	printf("Ending Auto-capture!\n");
	cvWaitKey(500);
}





















