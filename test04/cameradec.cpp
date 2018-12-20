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
		cv::Point2f(1985 , 405),
		cv::Point2f(1922 , 1033),
		cv::Point2f(590  , 919),
		cv::Point2f(641  , 308) };
 
cv::Point2f dst_points[] = {
	 cv::Point2f( 1100, -300),
     cv::Point2f( 750,  -300),
     cv::Point2f( 750,  450),
     cv::Point2f( 1100, 450),
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

void transformPoint(double & x,double &y, Mat&Trans, double height)
{
   double dx = x - 1296;//, 972
   double dy = y - 972;
   double ratio = (140 - height)/140;
   x = 1296 + dx * ratio;
   y = 972  + dy * ratio;   
   double coeffs[3][3];
    for( int i = 0; i < 3; i++ )
            for( int j = 0; j < 3; j++ )
               coeffs[i][j] = Trans.at<double>(i, j);
  double *M = &coeffs[0][0];
  double X0 = M[0]*x + M[1]*(y) + M[2];
  double Y0 = M[3]*x + M[4]*(y) + M[5];
  double W0 = M[6]*x + M[7]*(y) + M[8];
  x = X0/W0 + 3.0;
  y = Y0/W0 -2.3;
  
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
double GetAngle(double x1,double y1, double x2,double y2)
{
  double offsetY = -(y2 - y1);
  double offsetX =   x2 - x1;
  
  double angle =  acos( offsetY /sqrt( offsetY * offsetY + offsetX * offsetX)) * 180.0/3.1415826;
  if(angle<0)angle+= 360.0;
  printf("%.3lf\n",angle);
  return angle;
}

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
	int PicWidth, PicHeight;
	PicWidth  = 1920;
	PicHeight = 1080;
	
	CPic.FormatF(PicWidth, PicHeight);
	//CPic.FormatF(1600, 1200);
	 
	inputVideo.set(3,PicWidth);//CV_CAP_PROP_FRAME_WIDTH, 1600);
	inputVideo.set(4,PicHeight);//CV_CAP_PROP_FRAME_HEIGHT, 1200);
	//CPic.FormatF(PicWidth, PicHeight);
	//inputVideo.set(3,1600);//CV_CAP_PROP_FRAME_WIDTH, 1600);
	//inputVideo.set(4,1200);//CV_CAP_PROP_FRAME_HEIGHT, 1200);
	int frame_count(0);
	int   Boxleft,  Boxtop,  Boxright,  Boxbottom;
    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        
		frame_count ++;
		if(frame_count<10)
		   continue;
		
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
        
		C256BitMap  GYellowPic;
		GetGrayImageYellow( CPic, GYellowPic);
		GYellowPic.Save("yellow01.bmp");
		double ObjCenterX, ObjCenterY;
		GetBallCenter(GYellowPic, ObjCenterX, ObjCenterY, Boxleft,  Boxtop,  Boxright,  Boxbottom);
		CPic.DrawCircle(ObjCenterX, ObjCenterY,4);
		
		//=================================================
		vector<Region> RegionVec;
		pGray = GetGrayImageCamFront;
    	ProcessImg( CPic, RegionVec);
	    
		vector<int>      corner_idx;
        vector<RPoint>   corner;
        vector<RPoint>   shapecontour;
		Region ExtractBox;
		for(i=0;i<RegionVec.size();i++)
		{
			printf("%i\n", RegionVec[i].PtVec.size());
			if(RegionVec[i].PtVec.size()>60000)
			{ ExtractBox =  RegionVec[i];
              break;
			}
		}
	    GetCurveCornerPoint( CPic, ExtractBox.ConvexHullPtVec, corner_idx, corner, shapecontour);
		CPic.Rectangle( Boxleft,  Boxtop,  Boxright,  Boxbottom );
		CPic.Save("cap01.bmp");
		
		break;
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
		
		
    }

	inputVideo.open(3);
    inputVideo.set(3,PicWidth);//CV_CAP_PROP_FRAME_WIDTH, 1600);
	inputVideo.set(4,PicHeight);//CV_CAP_PROP_FRAME_HEIGHT, 1200);
	frame_count = 0;
    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        
		frame_count ++;
		if(frame_count<10)
		   continue;
		
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
	    C256BitMap  GYellowPic;
		GetGrayImageYellow( CPic, GYellowPic);
		GYellowPic.Save("yellow02.bmp");
		double ObjCenterX, ObjCenterY;
		GetBallCenter(GYellowPic, ObjCenterX, ObjCenterY, Boxleft,  Boxtop,  Boxright,  Boxbottom);
		CPic.Rectangle( Boxleft,  Boxtop,  Boxright,  Boxbottom);
		CPic.DrawCircle(ObjCenterX, ObjCenterY,4);
		CPic.Save("cap02.bmp");
		break;
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
		
		
    }
    return 0;
}
