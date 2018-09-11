/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008
 
   AVAILABLE AT: 
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130    

   OTHER OPENCV SITES:
   * The source code is on sourceforge at:
     http://sourceforge.net/projects/opencvlibrary/
   * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
     http://opencvlibrary.sourceforge.net/
   * An active user group is at:
     http://tech.groups.yahoo.com/group/OpenCV/
   * The minutes of weekly OpenCV development meetings are at:
     http://pr.willowgarage.com/wiki/OpenCV
   ************************************************** */

/*
	Modified by Martin Peris Martorell (info@martinperis.com) in order to accept some configuration
	parameters and store all the calibration data as xml files.

*/

#include "stdafx.h"
#include "stereo_calibrate.h"

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "globaldata.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace cv;

using namespace std;

//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
void StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated, float _squareSize)
{
#if 0
    int displayCorners    = DISPLAY_CORNERS;
    int showUndistorted   = USE_UNDISTORTED;
    bool isVerticalStereo = IS_VERTICAL_STEREO;		 //OpenCV can handle left-right
													 //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = _squareSize;			 //Chessboard square size in cm
    FILE* f = fopen(imageList, "rt");
    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3] = { {1,0,0}, {0,1,0}, {0,0,1}};
    double M2[3][3] = { {1,0,0}, {0,1,0}, {0,0,1}};
    double D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);



    if( displayCorners )cvNamedWindow( "Corners", 1 );



	/*------------------ READ IN THE LIST OF CHESSBOARDS: ------------------ */


    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }
    for(i=0;;i++)
    {
        char buf[1024];
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);




		//------------- FIND CHESSBOARDS AND CORNERS THEREIN ----------------:

        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result ) break;
        }


		//------------- Display Corners  ------------------------------------:

        if( displayCorners )
        {
            printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "Corners", cimg );						
            cvReleaseImage( &cimg );							
            if( cvWaitKey(0) == 27 ) exit(-1);	 //Allow ESC to quit		
                									
        } else {
			if(result==1) putchar('0');
			else putchar('X');

		}

        //-------------------------------------------------------------------:
		N = pts.size();
		pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
		//assert( result != 0 );



        if( result )
        {
			//Calibration will suffer without subpixel interpolation
//            cvFindCornerSubPix( img, &temp[0], count,
//                cvSize(11, 11), cvSize(-1,-1),
//                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
//                30, 0.01) );

            // Put these refined points at the end of corner points
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }

        cvReleaseImage( &img );
    }
	
    fclose(f);
    printf("\n");



	//----- HARVEST CHESSBOARD 3D OBJECT POINT LIST ------------------------:


    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);

    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
            objectPoints[i*nx + j] = cvPoint3D32f(j*squareSize, i*squareSize, 0);

    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(),
            objectPoints.begin() + n,
            objectPoints.begin() + i*n );

    npoints.resize(nframes,n);
    N = nframes*n;

    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );

    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);

    cvZero(&_D1);
    cvZero(&_D2);

	//-----  CALIBRATE THE STEREO CAMERAS ----------------------------------:

    printf("\nRunning stereo calibration ...");
    fflush(stdout);
    double rms = cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH );

    printf(" Done with rms error %f\n", rms);





	//----- CALIBRATION QUALITY CHECK ----------------------------------:

	// Because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0

    vector<CvPoint3D32f> lines[2];

    points[0].resize(N);
    points[1].resize(N);

    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );

    lines[0].resize(N);
    lines[1].resize(N);

    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);

	//Always work in undistorted space

    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );

    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );

    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }

    printf( "Avg err = %g\n\n", avgErr/(nframes*n) );

	//----- COMPUTE AND DISPLAY RECTIFICATION ----------------------------------:
	

    if( showUndistorted )
    {
        CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);

		//----- IF BY CALIBRATED (BOUGUET'S METHOD) ----------------------------------:
        if( useUncalibrated == 0 )
        {
			printf("Using Bougets Method!\n\n");
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);

            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &_Q,
                CV_CALIB_ZERO_DISPARITY, 1);	//CV_CALIB_ZERO_DISPARITY

            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);

			 //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
            
			//Save parameters

			printf("Saving Parameters ... ");
			
            cvSave("CalibrationData/M1.xml",&_M1);
            cvSave("CalibrationData/D1.xml",&_D1);
            cvSave("CalibrationData/R.xml",&_R);
            cvSave("CalibrationData/T.xml",&_T);
            cvSave("CalibrationData/R1.xml",&_R1);
            cvSave("CalibrationData/P1.xml",&_P1);
            cvSave("CalibrationData/M2.xml",&_M2);
            cvSave("CalibrationData/D2.xml",&_D2);
            cvSave("CalibrationData/R2.xml",&_R2);
            cvSave("CalibrationData/P2.xml",&_P2);
            cvSave("CalibrationData/Q.xml",&_Q);
            cvSave("CalibrationData/mx1.xml",mx1);
            cvSave("CalibrationData/my1.xml",my1);
            cvSave("CalibrationData/mx2.xml",mx2);
            cvSave("CalibrationData/my2.xml",my2);

			printf("Done\n");

        }
		

		// ------ OR ELSE HARTLEY'S METHOD ----------------------------------:
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix

        else if( useUncalibrated == 1 || useUncalibrated == 2 )
        {
			printf("Using Hartley's Method!\n\n");

            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);

		    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
									  &_imagePoints2, &_F);

			cvStereoRectifyUncalibrated( &_imagePoints1,
										 &_imagePoints2, &_F,
										   imageSize,
										 &_H1, &_H2, 3);

            cvInvert(&_M1, &_iM);

            cvMatMul(&_H1, &_M1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);

            cvInvert(&_M2, &_iM);

            cvMatMul(&_H2, &_M2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);

			//Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);


			//Save parameters

			printf("Saving Parameters ... ");
			
            cvSave("CalibrationData/M1.xml",&_M1);
            cvSave("CalibrationData/D1.xml",&_D1);
            cvSave("CalibrationData/R1.xml",&_R1);
            cvSave("CalibrationData/M2.xml",&_M2);
            cvSave("CalibrationData/D2.xml",&_D2);
            cvSave("CalibrationData/R2.xml",&_R2);
           // cvSave("CalibrationData/Q.xml",&_Q);
            cvSave("CalibrationData/mx1.xml",mx1);
            cvSave("CalibrationData/my1.xml",my1);
            cvSave("CalibrationData/mx2.xml",mx2);
            cvSave("CalibrationData/my2.xml",my2);

			printf("Done\n");


	

        }

        else assert(0);


#if 1
        cvNamedWindow( "rectified", 1 );


        // --- RECTIFY THE IMAGES AND FIND DISPARITY MAPS ----------------------------------:
        if( !isVerticalStereo )
            pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );
        else
            pair = cvCreateMat( imageSize.height*2, imageSize.width, CV_8UC3 );

        //Setup for finding stereo corrrespondences
        CvStereoBMState *stereoSGBM = cvCreateStereoBMState();
        assert(stereoSGBM != 0);

//        BMState->preFilterSize=41;
//        BMState->preFilterCap=31;
//        BMState->SADWindowSize=41;
//        BMState->minDisparity=-64;
//        BMState->numberOfDisparities=128;
//        BMState->textureThreshold=10;
//        BMState->uniquenessRatio=15;
        stereoSGBM->SADWindowSize = 9;
        stereoSGBM->numberOfDisparities = 112;
        stereoSGBM->preFilterSize = 5;
        stereoSGBM->preFilterCap = 61;
        stereoSGBM->minDisparity = -39;
        stereoSGBM->textureThreshold = 507;
        stereoSGBM->uniquenessRatio = 0;
        stereoSGBM->speckleWindowSize = 0;
        stereoSGBM->speckleRange = 8;
        stereoSGBM->disp12MaxDiff = 1;

        for( i = 0; i < nframes; i++ )
        { 
            IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
            IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);

            if( img1 && img2 )
            {
                CvMat part;
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );

                // When the stereo camera is oriented vertically,
                // useUncalibrated==0 does not transpose the
                // image, so the epipolar lines in the rectified
                // images are vertical. Stereo correspondence
                // function does not support such a case.

                if( !isVerticalStereo || useUncalibrated != 0 )
                {

                    cvFindStereoCorrespondenceBM( img1r, img2r, disp, stereoSGBM);
                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                    cvNamedWindow( "disparity" );
                    cvShowImage( "disparity", vdisp );
                }

                if( !isVerticalStereo )
                {
                    cvGetCols( pair, &part, 0, imageSize.width );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetCols( pair, &part, imageSize.width,
                        imageSize.width*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.height; j += 16 )
                        cvLine( pair, cvPoint(0,j),
                            cvPoint(imageSize.width*2,j),
                            CV_RGB(0,255,0));
                }

                else
                {
                    cvGetRows( pair, &part, 0, imageSize.height );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetRows( pair, &part, imageSize.height,imageSize.height*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );

                    for( j = 0; j < imageSize.width; j += 16 ){
                        cvLine( pair,
                                cvPoint(j,0),
                                cvPoint(j,imageSize.height*2),
                                CV_RGB(0,255,0));
                    }

                }
                cvShowImage( "rectified", pair ); 
                if( cvWaitKey() == 27 ) break;

            }


            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );
        }
        cvReleaseStereoBMState(&stereoSGBM);
#else

        for( i = 0; i < nframes; i++ )
        {

            showDisparity(imageNames[0][i].c_str(), imageNames[1][i].c_str(), false);
        }
#endif

        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
    }

#else
  bool displayCorners = false;
  bool showUndistorted = true;
  bool isVerticalStereo = false; // horiz or vert cams
  const int maxScale = 1;

  // actual square size
  FILE *f = fopen(imageList, "rt");
  int i, j, lr;
  int N = nx * ny;
  cv::Size board_sz = cv::Size(nx, ny);
  vector<string> imageNames[2];
  vector<cv::Point3f> boardModel;
  vector<vector<cv::Point3f> > objectPoints;
  vector<vector<cv::Point2f> > points[2];
  vector<cv::Point2f> corners[2];
  bool found[2] = {false, false};
  cv::Size imageSize;

  // READ IN THE LIST OF CIRCLE GRIDS:
  //
  if (!f) {
    cout << "Cannot open file " << imageList << endl;
    return;
  }
  for (i = 0; i < ny; i++)
    for (j = 0; j < nx; j++)
      boardModel.push_back(
          cv::Point3f((float)(j * _squareSize), (float)(i * _squareSize), 0.f));
  i = 0;
  for (;;) {
    char buf[1024];
    lr = i % 2;
    if (lr == 0)
      found[0] = found[1] = false;
    if (!fgets(buf, sizeof(buf) - 3, f))
      break;
    size_t len = strlen(buf);
    while (len > 0 && isspace(buf[len - 1]))
      buf[--len] = '\0';
    if (buf[0] == '#')
      continue;
    cv::Mat img = cv::imread(buf, 0);
    if (img.empty())
      break;
    imageSize = img.size();
    imageNames[lr].push_back(buf);
    i++;

    // If we did not find board on the left image,
    // it does not make sense to find it on the right.
    //
    if (lr == 1 && !found[0])
      continue;

    // Find circle grids and centers therein:
    for (int s = 1; s <= maxScale; s++) {
      cv::Mat timg = img;
      if (s > 1)
        resize(img, timg, cv::Size(), s, s, cv::INTER_CUBIC);
      // Just as example, this would be the call if you had circle calibration
      // boards ...
      //      found[lr] = cv::findCirclesGrid(timg, cv::Size(nx, ny),
      //      corners[lr],
      //                                      cv::CALIB_CB_ASYMMETRIC_GRID |
      //                                          cv::CALIB_CB_CLUSTERING);
      //...but we have chessboards in our images
      found[lr] = cv::findChessboardCorners(timg, board_sz, corners[lr],
              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

      if (found[lr] || s == maxScale) {
        //cornerSubPix(timg, corners[lr], Size(11, 11), Size(-1,-1),
        //                                TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 100, 1e-5));
        cv::Mat mcorners(corners[lr]);
        mcorners *= (1. / s);
      }
      if (found[lr])
        break;
    }
    if (displayCorners) {
      cout << buf << endl;
      cv::Mat cimg;
      cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);

      // draw chessboard corners works for circle grids too
      cv::drawChessboardCorners(cimg, cv::Size(nx, ny), corners[lr], found[lr]);
      cv::imshow("Corners", cimg);
      if ((cv::waitKey(0) & 255) == 27) // Allow ESC to quit
        exit(-1);
    } else
      cout << '.';
    if (lr == 1 && found[0] && found[1]) {
      objectPoints.push_back(boardModel);
      points[0].push_back(corners[0]);
      points[1].push_back(corners[1]);
    }
  }
  fclose(f);

  // CALIBRATE THE STEREO CAMERAS
  cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat D1, D2, R, T, E, F;
  cout << "\nRunning stereo calibration ...\n";
  double rms = cv::stereoCalibrate(objectPoints, points[0], points[1],
          M1, D1,
          M2, D2,
          imageSize, R, T, E, F,
          CALIB_FIX_ASPECT_RATIO +
          CALIB_ZERO_TANGENT_DIST +
          CALIB_SAME_FOCAL_LENGTH,
          TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 100, 1e-5));
  cout << "done with RMS error=" << rms << endl;
  cout << "Done! Press any key to step through images, ESC to exit\n\n";

  cv::FileStorage fs("CalibrationData/intrinsics.yml", cv::FileStorage::WRITE);
  if( fs.isOpened() )
  {
      fs << "M1" << M1 << "D1" << D1 <<
          "M2" << M2 << "D2" << D2;
      fs.release();
  }
  else
  {
      cout << "Error: can not save the intrinsic parameters\n";
  }

  // CALIBRATION QUALITY CHECK
  // because the output fundamental matrix implicitly
  // includes all the output information,
  // we can check the quality of calibration using the
  // epipolar geometry constraint: m2^t*F*m1=0
  vector<cv::Point3f> lines[2];
  double avgErr = 0;
  int nframes = (int)objectPoints.size();
  for (i = 0; i < nframes; i++) {
    vector<cv::Point2f> &pt0 = points[0][i];
    vector<cv::Point2f> &pt1 = points[1][i];
    cv::undistortPoints(pt0, pt0, M1, D1, cv::Mat(), M1);
    cv::undistortPoints(pt1, pt1, M2, D2, cv::Mat(), M2);
    cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
    cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);

    for (j = 0; j < N; j++) {
      double err = fabs(pt0[j].x * lines[1][j].x + pt0[j].y * lines[1][j].y +
                        lines[1][j].z) +
                   fabs(pt1[j].x * lines[0][j].x + pt1[j].y * lines[0][j].y +
                        lines[0][j].z);
      avgErr += err;
    }
  }
  cout << "avg err = " << avgErr / (nframes * N) << endl;

  // COMPUTE AND DISPLAY RECTIFICATION
  //
  if (showUndistorted) {
    cv::Mat R1, R2, P1, P2, Q, map11, map12, map21, map22;

    // IF BY CALIBRATED (BOUGUET'S METHOD)
    //
    if (!useUncalibrated) {
      stereoRectify(M1, D1, M2, D2,
                  imageSize, R, T,
                  R1, R2, P1, P2, Q, //outputs
                  CALIB_ZERO_DISPARITY, 1);

      fs.open("CalibrationData/extrinsics.yml", cv::FileStorage::WRITE);

      isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
      // Precompute maps for cvRemap()
      initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11,
                              map12);
      initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21,
                              map22);

      if( fs.isOpened() )
      {
          fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q
             << "map11" << map11 << "map12" << map12 << "map21" << map21 << "map22" << map22;
          fs.release();
      }
      else
          cout << "Error: can not save the extrinsic parameters\n";
    }

    // OR ELSE HARTLEY'S METHOD
    //
    else {

      // use intrinsic parameters of each camera, but
      // compute the rectification transformation directly
      // from the fundamental matrix
      vector<cv::Point2f> allpoints[2];
      for (i = 0; i < nframes; i++) {
        copy(points[0][i].begin(), points[0][i].end(),
             back_inserter(allpoints[0]));
        copy(points[1][i].begin(), points[1][i].end(),
             back_inserter(allpoints[1]));
      }
      cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
      cv::Mat H1, H2;
      cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, imageSize,
                                    H1, H2, 3);
      R1 = M1.inv() * H1 * M1;
      R2 = M2.inv() * H2 * M2;

      // Precompute map for cvRemap()
      //
      cv::initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11,
                                  map12);
      cv::initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21,
                                  map22);
    }

    // RECTIFY THE IMAGES AND FIND DISPARITY MAPS
    //
    cv::Mat pair;
    if (!isVerticalStereo)
      pair.create(imageSize.height, imageSize.width * 2, CV_8UC3);
    else
      pair.create(imageSize.height * 2, imageSize.width, CV_8UC3);

    // Setup for finding stereo corrrespondences

    if (global_data::isUseBM) {
        global_data::stereoSGBM = StereoSGBM::create(
                  -39, 128, 22, 0, 0, 1, 63, 5, 100, 32, cv::StereoSGBM::MODE_HH);
    } else {
        global_data::stereoBM = StereoBM::create(112, 9);
        global_data::stereoBM->setPreFilterSize(5);
        global_data::stereoBM->setPreFilterCap(61);
        global_data::stereoBM->setTextureThreshold(507);
        global_data::stereoBM->setUniquenessRatio(5);
    }


    for (i = 0; i < nframes; i++) {
      cv::Mat img1 = cv::imread(imageNames[0][i].c_str(), 0);
      cv::Mat img2 = cv::imread(imageNames[1][i].c_str(), 0);
      cv::Mat img1r, img2r, disp, vdisp;
      if (img1.empty() || img2.empty())
        continue;
      cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
      cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);
      if (!isVerticalStereo || !useUncalibrated) {

        // When the stereo camera is oriented vertically,
        // Hartley method does not transpose the
        // image, so the epipolar lines in the rectified
        // images are vertical. Stereo correspondence
        // function does not support such a case.
        if (global_data::isUseBM) {
            global_data::stereoBM->compute(img1r, img2r, disp);
        } else {
            global_data::stereoSGBM->compute(img1r, img2r, disp);
        }

        cv::normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX, CV_8U);
        cv::imshow("disparity", vdisp);
      }
      if (!isVerticalStereo) {
        cv::Mat part = pair.colRange(0, imageSize.width);
        cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
        part = pair.colRange(imageSize.width, imageSize.width * 2);
        cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
        for (j = 0; j < imageSize.height; j += 16)
          cv::line(pair, cv::Point(0, j), cv::Point(imageSize.width * 2, j),
                   cv::Scalar(0, 255, 0));
      } else {
        cv::Mat part = pair.rowRange(0, imageSize.height);
        cv::cvtColor(img1r, part, cv::COLOR_GRAY2BGR);
        part = pair.rowRange(imageSize.height, imageSize.height * 2);
        cv::cvtColor(img2r, part, cv::COLOR_GRAY2BGR);
        for (j = 0; j < imageSize.width; j += 16)
          line(pair, cv::Point(j, 0), cv::Point(j, imageSize.height * 2),
               cv::Scalar(0, 255, 0));
      }
      cv::imshow("rectified", pair);
      if ((cv::waitKey() & 255) == 27)
        break;
    }
  }

#endif
}


int Begin_Calibration(const char* imageList,int nx,int ny,int useUncalibrated,float squareSize)
{

    int fail = 0;

    if (nx <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: nx value can not be <= 0\n");
    }
    if (ny <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: ny value can not be <= 0\n");
    }
    if (squareSize <= 0.0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: squareSize value can not be <= 0\n");
    }

    if(fail != 0) return 1;

    StereoCalib(imageList, nx, ny, useUncalibrated, squareSize);
    return 0;
}

//void showDisparity(string leftImageName, string rightImageName, bool isUseBM)
//{
//
//
//
//    Mat img1, img2, g1, g2;
//    Mat disp, disp8;
//    img1 = imread(leftImageName);
//    img2 = imread(rightImageName);
//    cvtColor(img1, g1, CV_BGR2GRAY);
//    cvtColor(img2, g2, CV_BGR2GRAY);
//
//    if (isUseBM)
//    {
//        StereoBM sbm;
//        sbm.state->SADWindowSize = 9;
//        sbm.state->numberOfDisparities = 112;
//        sbm.state->preFilterSize = 5;
//        sbm.state->preFilterCap = 61;
//        sbm.state->minDisparity = -39;
//        sbm.state->textureThreshold = 507;
//        sbm.state->uniquenessRatio = 0;
//        sbm.state->speckleWindowSize = 0;
//        sbm.state->speckleRange = 8;
//        sbm.state->disp12MaxDiff = 1;
//        sbm(g1, g2, disp);
//    }
//    else
//    {
//        StereoSGBM sbm;
//        sbm.minDisparity = -39;
//        sbm.numberOfDisparities = 128;
//        sbm.SADWindowSize = 3;
//        sbm.P1 = 216;
//        sbm.P2 = 864;
//        sbm.disp12MaxDiff = 1;
//        sbm.preFilterCap = 63;
//        sbm.uniquenessRatio = 15;
//        sbm.speckleWindowSize = 100;
//        sbm.speckleRange = 32;
//        //sbm.mode = StereoSGBM::MODE_SGBM_HH;
//        sbm(g1, g2, disp);
//    }
//
//
//    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
//
//    imshow("left", img1);
//    imshow("right", img2);
//    imshow("disp", disp8);
//
//    waitKey(0);
//}
