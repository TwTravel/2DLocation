#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "HandEyeCalib.hpp"
#include <iostream>

using namespace std;
using namespace cv;


double robotpos_array[9][7]=
{
{0.75, 0.1, 0.95, 0.0, 1.0, 0.0, -1},
{0.75, 0.2, 0.95, 0.0, 1.0, 0.0, -1},	
{0.75, 0.3, 0.95, 0.0, 1.0, 0.0, -1},							  
{0.75, 0.3, 0.85, 0.0, 1.0, 0.0, -1},	
{0.80, 0.3, 0.85, 0.0, 1.0, 0.0, -1},	
{0.90, 0.3, 0.85, 0.0, 1.0, 0.0, -1},	
{0.95, 0.3, 0.85, 0.0, 1.0, 0.0, -1},
{1.00, 0.3, 0.85, 0.0, 1.0, 0.0, -1},								  
{0.75, 0.0, 0.95, 0.0, 1.0, 0.0, -1}
}
;
							  
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

 
 vpHomogeneousMatrix toVispHomogeneousMatrix(double position_x, double  position_y,double  position_z,
          double orientation_x, double orientation_y, double  orientation_z, double  orientation_w    )
 {
    vpHomogeneousMatrix mat;
    vpTranslationVector vec( position_x, position_y, position_z);
    vpQuaternionVector   q(orientation_x, orientation_y, orientation_z, orientation_w);
    mat.buildFrom(vec,q);

    return mat;
 }
 
 void quatFromAngularVelocity(Vec3d rvec, 
 double& orientation_x, double& orientation_y, 
 double& orientation_z, double& orientation_w)
{
    const float x = rvec[0];
    const float y = rvec[1];
    const float z = rvec[2];
    const float angle = sqrt(x*x + y*y + z*z);  //module of angular velocity

    if (angle > 0.0) //the formulas from the link
    {
        orientation_x = x*sin(angle/2.0f)/angle;
        orientation_y = y*sin(angle/2.0f)/angle;
        orientation_z = z*sin(angle/2.0f)/angle;
        orientation_w = cos(angle/2.0f);
    }else    //to avoid illegal expressions
    {
       orientation_x = orientation_y = orientation_z = 0;
       orientation_w = 1.0f;
    }
}

 vpHomogeneousMatrix toVispHomogeneousMatrix(Vec3d tvec, Vec3d rvec   )
{
	double  position_x, position_y, position_z,
    orientation_x,  orientation_y, orientation_z, orientation_w ;
	
   position_x = tvec[0];
   position_y = tvec[1];
   position_z = tvec[2];
   
   double rx,ry,rz;
   rx = rvec[0];
   ry = rvec[1];
   rz = rvec[2];
   
   printf("%.4lf,%.4lf,%.4lf,===%.4lf,%.4lf,%.4lf \n", position_x, position_y, position_z, rx, ry, rz);
  quatFromAngularVelocity( rvec, 
   orientation_x,   orientation_y, 
   orientation_z,   orientation_w) ;
  
 return toVispHomogeneousMatrix( position_x,  position_y,  position_z,
   orientation_x,   orientation_y, 
   orientation_z,   orientation_w );
// orientation_x, double orientation_y, double  orientation_z, double  orientation_w 
 }
/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    /*if(argc < 2) {
        parser.printMessage();
        return 0;
    }*/
    HandEyeCalib hand_eye_cal; 
    int dictionaryId = 3;//parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = 1;//parser.has("c");
    float markerLength = parser.get<float>("l");

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    //if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
        detectorParams->cornerRefinementMethod = 3;
    //}
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    //int camId = parser.get<int>("ci");

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
        bool readOk = readCameraParameters("output.yaml", camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime;
    /*if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }*/

    double totalTime = 0;
    int totalIterations = 0;
    
	const std::vector<vpHomogeneousMatrix> cMo_in; 
	const std::vector<vpHomogeneousMatrix> wMe_in;
    //while(1) 
	for(int ii=1;ii<10;ii++)
	{
        Mat image, imageCopy;
        //inputVideo.retrieve(image);
		char namebuff[30];
		sprintf(namebuff,"img/img%06i.jpg",ii);
        image = imread(namebuff);
		
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
					if(ids[i]==101)
                    {
						/*Mat rotM, rotT, rotMT, rotRobot; 
				        Rodrigues(rvecs[i], rotM);  //将旋转向量变换成旋转矩阵
                        Rodrigues(tvecs[i], rotT);
				        rotMT =  rotM*rotT;
						aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
						Vec3d robot_pos;
						robot_pos[0] = robotpos_array[ii-1][0];
						robot_pos[1] = robotpos_array[ii-1][1];
						robot_pos[2] = robotpos_array[ii-1][2];
						Rodrigues(robot_pos, rotRobot);*/
						//printf("")
						cMo_in.push_back(toVispHomogeneousMatrix(tvecs[i],rvecs[i]));
						wMe_in.push_back(toVispHomogeneousMatrix(-robotpos_array[ii-1][0],
						                                         -robotpos_array[ii-1][1],
																 -robotpos_array[ii-1][2],
																 0,0,0,1.0));
					}
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        //imshow("out", imageCopy);
        //char key = (char)waitKey(waitTime);
        //if(key == 27) break;
    }
	
    hand_eye_cal.init(  cMo_in,   wMe_in);
	Mat eMc_ou;
	hand_eye_cal.run(eMc_ou);
    return 0;
}
