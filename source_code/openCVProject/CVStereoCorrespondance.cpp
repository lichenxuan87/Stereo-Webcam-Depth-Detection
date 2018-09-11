//CVCorrespondance.cpp
//Coder: Matt Kokshoorn
//		 University of Canterbury
//		 2013

#include "CVStereoCorrespondance.h"

#include "stdafx.h"


//Module Data
namespace module_data{
//*------------------ Slider Parameters ------------*/ 
    // For BM its value is 13
    // For SGBM its value is 3
    int nStereoSADWindowSize        =  11; // BM:x+5; SGBM:x+1

    // Increase num of disparity for nearer distance detection
    int nStereoNumDisparities       =  10; // (x+1)*16
	int nStereoPreFilterCap			=  62; // x+1
	int nStereoMinDisparity			=  180; // x-200
	int nStereoUniquenessRatio		=  5;

	// For BM its value is 2
	// For SGBM its value is 2
	int nStereoSpeckleWindowSize	=  2;

	// For BM its value is 8
	// For SGBM its value is 8
	int nStereoSpeckleRange			=  8;

	// For BM its value is 1
	// For SGBG its value -1
	int nStereoDisp12MaxDiff        =  1;
	int nWindnowMemory				=  1;

	// For BM only
    int nStereoPreFilterType        =  CV_STEREO_BM_NORMALIZED_RESPONSE;
    int nStereoPreFilterSize        =  7; //(x+5)
    int nStereoTextureThreshold     =  150; // x+1

    // For SGBM only
    int nStereoP1                   =  0;
    int nStereoP2                   =  0;
}


//Locally Held Data
namespace correspondance_data{
    cv::Mat disp_left ;
    cv::Mat vdisp_left ;
    cv::Mat smoothedvdisp_left ;
    cv::Mat Image3D_left ;
    cv::Mat grey_left_r ;
    cv::Mat grey_right_r;
    cv::Mat disp_right ;
    cv::Mat vdisp_right ;
    cv::Mat smoothedvdisp_right;
    cv::Mat Image3D_right ;
    cv::Mat threholded_image;
    cv::Mat disp_left_memory;
    cv::Mat vdisp_left_memory;
    cv::Mat pixel_norm_disp;
}

using namespace cv;

//Interupt for the block mathing correspondance paramter sliders.
void sliderHandler(void)
{
    if (global_data::isUseBM) {
        if (global_data::stereoBM == NULL) {
            global_data::stereoBM = cv::StereoBM::create(112, 9);
        }

        //Pre Filter Type
        if(module_data::nStereoPreFilterType==0)        global_data::stereoBM->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
        else if(module_data::nStereoPreFilterType==1)   global_data::stereoBM->setPreFilterType(CV_STEREO_BM_XSOBEL);

        // Pre Filter Size 5-255 (odd)
        if( module_data::nStereoPreFilterSize % 2 == 0 )
            global_data::stereoBM->setPreFilterSize(module_data::nStereoPreFilterSize + 5);
        else
            global_data::stereoBM->setPreFilterSize(module_data::nStereoPreFilterSize + 6);


        //Pre Filter Cap 1-63
        global_data::stereoBM->setPreFilterCap(module_data::nStereoPreFilterCap+1);

        //SAD Windnow Size
        if ((module_data::nStereoSADWindowSize % 2) == 0)
            global_data::stereoBM->setBlockSize(module_data::nStereoSADWindowSize+5);
        else
            global_data::stereoBM->setBlockSize(module_data::nStereoSADWindowSize+6);


        //Min Disparity -200 to 200
        global_data::stereoBM->setMinDisparity(module_data::nStereoMinDisparity-200);

        //Number of Disparity 0 to 150
        global_data::stereoBM->setNumDisparities((module_data::nStereoNumDisparities+1)*16);

        //Texture Threshold 1-500
        global_data::stereoBM->setTextureThreshold(module_data::nStereoTextureThreshold+1);

        //Uniqueness Ratio
        global_data::stereoBM->setUniquenessRatio(module_data::nStereoUniquenessRatio);


        //Speckle Widnows Size 0-100
        global_data::stereoBM->setSpeckleWindowSize(module_data::nStereoSpeckleWindowSize);

        // Speckle Range 0-100
        global_data::stereoBM->setSpeckleRange(module_data::nStereoSpeckleRange);

        // disp12MaxDiff 0-1
        global_data::stereoBM->setDisp12MaxDiff(module_data::nStereoDisp12MaxDiff);

    } else {
        if (global_data::stereoSGBM == NULL) {
            global_data::stereoSGBM = cv::StereoSGBM::create(
                -39, 128, 11, 0, 0, 1, 63, 15, 100, 32, cv::StereoSGBM::MODE_HH);
        }

        // Set P1
        global_data::stereoSGBM->setP1(module_data::nStereoP1);

        // Set P2
        global_data::stereoSGBM->setP2(module_data::nStereoP2);

        //Pre Filter Cap 1-63
        global_data::stereoSGBM->setPreFilterCap((module_data::nStereoPreFilterCap+1)*0.63);

        //SAD Windnow Size
        if ((module_data::nStereoSADWindowSize % 2) == 0)
            global_data::stereoSGBM->setBlockSize(module_data::nStereoSADWindowSize+1);
        else
            global_data::stereoSGBM->setBlockSize(module_data::nStereoSADWindowSize);


        //Min Disparity -200 to 200
        global_data::stereoSGBM->setMinDisparity(module_data::nStereoMinDisparity-200);

        //Number of Disparity 0 to 150
        global_data::stereoSGBM->setNumDisparities((module_data::nStereoNumDisparities+1)*16);

        //Uniqueness Ratio
        global_data::stereoSGBM->setUniquenessRatio(module_data::nStereoUniquenessRatio);

        //Speckle Windows Size 0-100
        global_data::stereoSGBM->setSpeckleWindowSize(module_data::nStereoSpeckleWindowSize);

        // Speckle Range 0-100
        global_data::stereoSGBM->setSpeckleRange(module_data::nStereoSpeckleRange);

        // disp12MaxDiff 0-1
        global_data::stereoSGBM->setDisp12MaxDiff(module_data::nStereoDisp12MaxDiff);
    }


}


//Initilise the slider bars.
void init_sliderHandler(void){
	cvNamedWindow( PARAM_WIND, CV_GUI_EXPANDED);

	if (global_data::isUseBM) {
        cvCreateTrackbar("PF-Typ"		, PARAM_WIND,&module_data::nStereoPreFilterType		, 1,   (CvTrackbarCallback)sliderHandler);
        cvCreateTrackbar("PF-Size"		, PARAM_WIND,&module_data::nStereoPreFilterSize		, 250, (CvTrackbarCallback)sliderHandler);
        cvCreateTrackbar("TxtreThrsh"   , PARAM_WIND,&module_data::nStereoTextureThreshold  , 500, (CvTrackbarCallback)sliderHandler);
	} else {
	    cvCreateTrackbar("P1"           , PARAM_WIND,&module_data::nStereoP1                , 500, (CvTrackbarCallback)sliderHandler);
        cvCreateTrackbar("P2"           , PARAM_WIND,&module_data::nStereoP2                , 1000, (CvTrackbarCallback)sliderHandler);
	}


	cvCreateTrackbar("PF-Cap"		, PARAM_WIND,&module_data::nStereoPreFilterCap		, 100, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("SADWinSiz"	, PARAM_WIND,&module_data::nStereoSADWindowSize		, 100, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("Min Dspty"	, PARAM_WIND,&module_data::nStereoMinDisparity		, 400, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("# Disprty"	, PARAM_WIND,&module_data::nStereoNumDisparities	, 20,  (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("UniqRtio"		, PARAM_WIND,&module_data::nStereoUniquenessRatio	, 100, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("SpcWindSiz"	, PARAM_WIND,&module_data::nStereoSpeckleWindowSize	, 100, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("Spk Rnge"		, PARAM_WIND,&module_data::nStereoSpeckleRange		, 100, (CvTrackbarCallback)sliderHandler);
	cvCreateTrackbar("Memory"	    , PARAM_WIND,&module_data::nWindnowMemory		    , 1  , (CvTrackbarCallback)sliderHandler);

	sliderHandler();
}

//Initilise the local data.
void init_correspondance_data(void){

//	 	correspondance_data::disp_left				= cv::create( global_data::camStreamSize.height,global_data::camStreamSize.width, CV_16S );
//		correspondance_data::vdisp_left				= cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_8U );
//		correspondance_data::smoothedvdisp_left		= cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_8U );
//		correspondance_data::Image3D_left			= cv::create(global_data::camStreamSize.height, global_data::camStreamSize.width, CV_32FC3);
//		correspondance_data::grey_left_r			= cvCreateImage(global_data::camStreamSize,IPL_DEPTH_8U,1);
//		correspondance_data::grey_right_r			= cvCreateImage(global_data::camStreamSize,IPL_DEPTH_8U,1);
//	 	correspondance_data::disp_right				= cv::create( global_data::camStreamSize.height,global_data::camStreamSize.width, CV_16S );
//		correspondance_data::vdisp_right			= cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_8U );
//		correspondance_data::smoothedvdisp_right	= cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_8U );
//		correspondance_data::Image3D_right			= cv::create(global_data::camStreamSize.height, global_data::camStreamSize.width, CV_32FC3);
//		correspondance_data::threholded_image       = cvCreateImage(global_data::camStreamSize,IPL_DEPTH_8U,1);
//		correspondance_data::disp_left_memory       = cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_16S );
//		correspondance_data::vdisp_left_memory      = cv::create(global_data::camStreamSize.height,global_data::camStreamSize.width, CV_8U );
}


//Carry out block matching correspondance
void calculate_correspondance_data(cv::Mat& image_left_undistorted, cv::Mat& image_right_undistorted, cv::Mat& Q){
	
    //Convert each image to gresycale.
    cvtColor(image_left_undistorted,correspondance_data::grey_left_r, CV_RGB2GRAY);
    cvtColor(image_right_undistorted,correspondance_data::grey_right_r, CV_RGB2GRAY);

    //Calcualte the disparity map.
    if (global_data::isUseBM) {
        global_data::stereoBM->compute(correspondance_data::grey_left_r, correspondance_data::grey_right_r, correspondance_data::disp_left);
    } else {
        global_data::stereoSGBM->compute(correspondance_data::grey_left_r, correspondance_data::grey_right_r, correspondance_data::disp_left);
    }
    //cvFindStereoCorrespondenceBM( correspondance_data::grey_left_r, correspondance_data::grey_right_r, correspondance_data::disp_left, global_data::BMState);

    correspondance_data::disp_left.convertTo(correspondance_data::disp_left, -1, 1.0/16, 0);
    //cvConvertScale(correspondance_data::disp_left, correspondance_data::disp_left, 1.0/16, 0 );
    //cvConvertScale(correspondance_data::disp_left, correspondance_data::disp_left, -16, 0 );

    //Implement Memory Image from correspondance
    //TODO: not use memory map
//		cv::Mat in = correspondance_data::disp_left;
//		int k=0;
//		int null_value=in.at<signed short>(0,0);
//
//		for(int i=0; i< (480);i++){
//			for(int j=0;j<640;j++){
//				if(module_data::nWindnowMemory==1){
//					if(
//					        (in.at<signed short>(i,j)!=null_value)
//					        &&
//					        ( !((-1*(Q.data.db[15])/(Q.data.db[14])) >in.at<signed short>(i,j))))
//
//					{
//						correspondance_data::disp_left_memory.data.s[k++] =
//						        (in.at<signed short>(i,j) * NEW_WEIGHTING +
//						         correspondance_data::disp_left_memory.data.s[k] * MEMORY_WEIGHTING
//						         ) / 100.0;
//					}
//					else{
//						k++;
//					}
//				}
//				else{
//					correspondance_data::disp_left_memory.data.s[k++]=in.at<signed short>(i,j);
//				}
//			}
//		}


    //Normalise the images for display
    //normalize( correspondance_data::disp_left_memory, correspondance_data::vdisp_left_memory, 0, 256, NORM_MINMAX, CV_8U );
    normalize( correspondance_data::disp_left, correspondance_data::vdisp_left, 0, 256, NORM_MINMAX, CV_8U );

    //Reporject the image to 3D using calibration matrix Q.
    //cvReprojectImageTo3D(correspondance_data::disp_left_memory, correspondance_data::Image3D_left, Q, true);
    reprojectImageTo3D(correspondance_data::disp_left, correspondance_data::Image3D_left, Q, false);
}


//Fucntion Can be used to save saved Data by pressing the 's' key.
void calculate_correspondance_data_save(Mat& image_left_undistorted, Mat& image_right_undistorted, Mat& Q){

		printf(" Saving 3D Projection...");
		//IMPLMENT IMAGES TO SAVE HERE...

		imwrite("TestData/Left.jpg", image_left_undistorted);
		imwrite("TestData/Right.jpg", image_right_undistorted);
		imwrite("TestData/Disparity.jpg", correspondance_data::disp_left);
		imwrite("TestData/DisparityMemory.jpg", correspondance_data::disp_left_memory);


		//END SAVE DATA
		printf(" Done.\n\n");
}


int calculate_correspondance_depth(Point left_point ,Mat& Q){
        //Calculate Depth From3D Projection Matrix
        cv::Mat New_Image3D_left = correspondance_data::Image3D_left;
        cv::Size size = correspondance_data::Image3D_left.size();

        //TODO: X/Y is inverted
        cv::Vec3f Depth_from_proj =  New_Image3D_left.at<cv::Vec3f>(left_point.y,left_point.x);
        printf(" 3D Mapping: ( %.2f, %.2f , %.2f )", Depth_from_proj[0] , Depth_from_proj[1], Depth_from_proj[2]);

        return Depth_from_proj[2];
}





void calculate_correspondance_depth_tracking(std::vector<cv::KeyPoint> keyPoints, Mat& Q){

    //Calculate Depth From3D Projection Matrix
    cv::Mat New_Image3D_left=correspondance_data::Image3D_left;
    char image_text[50];

    //Overaly text onto image displayign dpeth data.
    for(int i=0; i<keyPoints.size();i++){
        cv::Vec3f Depth_from_proj =  New_Image3D_left.at<cv::Vec3f>(keyPoints[i].pt.y,keyPoints[i].pt.x);


        if (Depth_from_proj[2] > 0) {
            sprintf(image_text,".   %1.2f",Depth_from_proj[2]);

            putText(global_data::image_left_rectified, image_text , keyPoints[i].pt, CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(50,100,250));
        }
    }

}


double calculate_correspondance_depth_tracking(cv::Rect boundary)
{
    cv::Mat New_Image3D_left=correspondance_data::Image3D_left;

    //char image_text[50];

    //Overlay text onto image displaying depth data.
    double averageDepth = 0;
    int validNumber = 0;
    for(int x = boundary.x + boundary.width/4; x < boundary.x + 3*boundary.width/4; x++){
        for (int y = boundary.y + boundary.height/4; y < boundary.y + 3*boundary.height/4; y++) {
            cv::Vec3f Depth_from_proj =  New_Image3D_left.at<cv::Vec3f>(x, y);

            if (Depth_from_proj[2] > 0 && Depth_from_proj[2] < 1000) {
                averageDepth += Depth_from_proj[2];

                validNumber++;
            }
        }
    }

    if (validNumber != 0)
        averageDepth /= validNumber;

    return averageDepth;
}


//Threshold image to RED,GREEN or BLUE for blob detection and thus object tracking.
cv::Mat GetThresholdedImage_RED(Mat& frame){
    GaussianBlur(frame, frame, Size(3,3), 0, 0);
    cv::Mat imgHSV(frame.size(),CV_8UC3);;
    cvtColor(frame, imgHSV, CV_BGR2HSV);
    cv::Mat imgThresh(imgHSV.size(),CV_8U);

    //cvInRangeS(frame,  CV_RGB(180,   0,  0),  CV_RGB(255,100,100), imgThresh); //RED
    //cvInRangeS(frame,  CV_RGB(  0, 150,  0),  CV_RGB(150,255,150), imgThresh); //GREEN
    //cvInRangeS(frame,  CV_RGB(  0,   0,150),  CV_RGB(100,100,255), imgThresh); //BLUE
    inRange(frame, Scalar(130, 70, 60),  Scalar(200, 130, 100), imgThresh); // For blue file archive


    GaussianBlur(frame, frame, Size(3,3), 0, 0);
    return imgThresh;
} 


