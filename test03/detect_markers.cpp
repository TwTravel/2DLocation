#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <iostream>
#include "c24bitmap.h"
#include "c256bitmap.h"
//======================================================================
#include "c24bitmap.h"
#include "c256bitmap.h"
#include <vector>
#include "region.h"
#include "mregion.h"
#include "findobj.h"
#include "hzmerge.h"
//======================================================================
using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

 

 

cv::Point2f src_points[] = { 
		cv::Point2f(2212 ,345),
		cv::Point2f(2181 ,1200),
		cv::Point2f(444 ,1181),
		cv::Point2f(419 ,359) };
 
cv::Point2f dst_points[] = {
		cv::Point2f(1030, -288),
		cv::Point2f(682, -301),
		cv::Point2f(655, 439),
		cv::Point2f(1004.85, 452),
         };
 
void transformPoint(double&x,double&y, Mat&Trans)
{
   double coeffs[3][3];
    for( int i = 0; i < 3; i++ )
            for( int j = 0; j < 3; j++ )
               coeffs[i][j] = Trans.at<double>(i, j);
  double *M = &coeffs[0][0];
  double X0 = M[0]*x + M[1]*(y) + M[2];
  double Y0 = M[3]*x + M[4]*(y) + M[5];
  double W0 = M[6]*x + M[7]*(y) + M[8];
  x = X0/W0;
  y = Y0/W0;
  
 // printf("%.2lf, %.2lf\n", x,y);
}

Mat PerspectiveTrans(Point2f* scrPoints, Point2f* dstPoints)
{
   
    //Mat dst;
    Mat Trans = getPerspectiveTransform(scrPoints, dstPoints);
	
   // warpPerspective(src, dst, Trans, Size(src.cols, src.rows), CV_INTER_CUBIC);
    return Trans;
}
/**
 */
int main(int argc, char *argv[]) {
	
	InitTextMerge();
	Mat Mt = PerspectiveTrans(src_points,  dst_points);
	double xx = src_points[0].x;
	double yy = src_points[0].y;
	transformPoint(xx,yy, Mt );
	
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
     
    if(argc < 2) {
        parser.printMessage();
        return 0;
    }
    
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
        detectorParams->cornerRefinementMethod = parser.get<int>("refine");
    }
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    double totalTime = 0;
    int totalIterations = 0;
    
	int i,j;
	C24BitMap CPic;
	
	CPic.FormatF(2592, 1944);
	//CPic.FormatF(1600, 1200);
	 
	inputVideo.set(3,2592);//CV_CAP_PROP_FRAME_WIDTH, 1600);
	inputVideo.set(4,1944);//CV_CAP_PROP_FRAME_HEIGHT, 1200);
	//CPic.FormatF(2592, 1944);
	//inputVideo.set(3,1600);//CV_CAP_PROP_FRAME_WIDTH, 1600);
	//inputVideo.set(4,1200);//CV_CAP_PROP_FRAME_HEIGHT, 1200);
	
    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

		
		
		Loopi(CPic.Width)
		    Loopj(CPic.Height)
			{
				Vec3b intensity = image.at<Vec3b>(j, i);
                uchar blue      = intensity.val[0];
                uchar green     = intensity.val[1];
                uchar red       = intensity.val[2];
				C24PixVal Pix = get_pix_color(CPic,i, j);
				*Pix.r = red;
				*Pix.g = green;
				*Pix.b = blue;
				
			}
		//=================================================
        //#################################################
        vector<Region> RegionVec;
    	ProcessImg( CPic, RegionVec);
		C256BitMap  GYellowPic;
		GetGrayImageYellow( CPic, GYellowPic);
		GYellowPic.Save("yellow.bmp");
		double ObjCenterX, ObjCenterY;
		GetBallCenter(GYellowPic, ObjCenterX, ObjCenterY);
		CPic.DrawCircle(ObjCenterX, ObjCenterY,4);
		
		double xx = ObjCenterX;
	    double yy = ObjCenterY;
	     transformPoint(xx,yy, Mt );
		 MergeTxtStrNUM(CPic, ObjCenterX -20 , ObjCenterY-20,
			  23, xx, yy,0, 0, 255); 
					   
	    int  i;
				
	    //Loopi(RegionVec[0].PtVec.size())
	    //   CPic.SigDot(RegionVec[0].PtVec[i].x, RegionVec[0].PtVec[i].y);
		//--------------------------------------------------
		vector<int>      corner_idx;
        vector<RPoint>   corner;
        vector<RPoint>   shapecontour;
		
		Region ExtractBox;
		for(i=0;i<RegionVec.size();i++)
		{
			if(RegionVec[i].PtVec.size()>80000)
			{ ExtractBox =  RegionVec[i];
              break;
			}
		}
	    GetCurveCornerPoint( CPic, ExtractBox.ConvexHullPtVec, corner_idx, corner, shapecontour);
		
		C256BitMap GSubPic;
		
		
		AnalysisRegionObj(ExtractBox);
		GSubPic.FormatF(ExtractBox.right - ExtractBox.left +1, ExtractBox.bottom - ExtractBox.top  +1);
		
		
		C24BitMap DrawPic;
		DrawPic.FormatF(GSubPic.Width, GSubPic.Height);
		
		Loopi(GSubPic.Width)
		    Loopj(GSubPic.Height)
			{
				/*Vec3b intensity = image.at<Vec3b>(j, i);
                uchar blue      = intensity.val[0];
                uchar green     = intensity.val[1];
                uchar red       = intensity.val[2];*/
				C24PixVal Pix = get_pix_color(CPic,i+ExtractBox.left, j+ExtractBox.top);
				if((*Pix.r)< 80 )
				* get_pix_color(GSubPic,i,j)= 0;
			    else
				* get_pix_color(GSubPic,i,j)= 255;	
				// = red;
				//*Pix.g = green;
				//*Pix.b = blue;
				
			}
		
		vector<Region> RegionVec1;
		RegionVec1.clear();
		
		vector<int> LabelVec;
		GetBlackPicRegion(GSubPic, DrawPic, RegionVec1);
		LabelVec.clear();
        GetObjContourColor(RegionVec1, DrawPic, LabelVec);
		printf("RegionVec:%i\n",RegionVec1.size());
		
		//GetBlackPicRegion(C256BitMap &Pic, vector<Region>&RegionVec);
		
	    GSubPic.Save("Gbox.bmp");
		
		for(i=0; i< RegionVec1.size(); i++)
		{
			double ratio = double(RegionVec1[i].PtVec.size())/double(RegionVec1[i].ContourPtVec.size()); 
			printf("obj perimeter%i,  area%i,%.2lf\n", RegionVec1[i].ContourPtVec.size(), RegionVec1[i].PtVec.size(),
			ratio);
			
			
			
			if(ratio>25)
			{
				CPic.SetColor(0);
				CPic.DrawCircle(ExtractBox.left + RegionVec1[i].x, 
				                 ExtractBox.top + RegionVec1[i].y, 5, 0.8);
				CPic.SetColor(1);
				CPic.DrawCircle(ExtractBox.left + RegionVec1[i].x, 
				                 ExtractBox.top + RegionVec1[i].y, 3, 0.8);
				
                double xx = ExtractBox.left + RegionVec1[i].x;
	            double yy = ExtractBox.top  + RegionVec1[i].y;
	            transformPoint(xx,yy, Mt );
		        MergeTxtStrNUM(CPic, ExtractBox.left + RegionVec1[i].x ,
				                     ExtractBox.top  + RegionVec1[i].y ,
			                         23, xx, yy,0, 0, 255); 				
								 
			}
			MergeTxtStrNUM(DrawPic, RegionVec1[i].x,RegionVec1[i].y,36, i,255,0,0);
		}
		
		DrawPic.Save("temp.bmp");
	    //CPic.Save("dest.bmp");
		  
	    //CPic.Save("color_pic.bmp");
		//#################################################
		//#################################################
		
        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                             tvecs);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++)
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

		for(i =0; i< ids.size(); i++)
		{
		   float XX,YY;
		  // XX = corners[i][0].x;
		  // YY =;
		   CPic.DrawLine(corners[i][0].x,corners[i][0].y,
		                 corners[i][1].x,corners[i][1].y);	
		}
		CPic.SetColor(1);
		
		for(i =0; i< ids.size(); i++)
		{
		   float XX,YY;
		  // XX = corners[i][0].x;
		  // YY =;
		   CPic.DrawLine(corners[i][1].x,corners[i][1].y,
		                 corners[i][2].x,corners[i][2].y);	
		}
		
		CPic.SetColor(2);
		for(i =0; i< ids.size(); i++)
		{
		   float XX,YY;
		  // XX = corners[i][0].x;
		  // YY =;
		   CPic.DrawCircle(corners[i][3].x ,corners[i][3].y, 12 , 0.5);
		   CPic.DrawLine(corners[i][2].x,corners[i][2].y,
		                 corners[i][3].x,corners[i][3].y);
						 
		   MergeTxtStrNUM(CPic, (corners[i][2].x + corners[i][0].x)/2,
		                        (corners[i][2].y + corners[i][0].y)/2,40, ids[i],255,0,0);
								
	       double xx = corners[i][3].x;
	       double yy = corners[i][3].y;
	       transformPoint(xx,yy, Mt );
		    MergeTxtStrNUM(CPic, corners[i][3].x -20 ,corners[i][3].y,
			16, xx, yy,0,255, 0); 
		    printf("%lf,%lf,%i\n",corners[i][3].x ,corners[i][3].y,ids[i]);
		}
		CPic.Save("color_pic.bmp");
		
		exit(0);//<<<++++++++KH_Dec
		
        imshow("out", imageCopy);
		
		
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
		
		
    }

    return 0;
}
