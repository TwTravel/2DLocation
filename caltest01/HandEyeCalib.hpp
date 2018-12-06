#ifndef __HANDEYECALIB_H__
#define __HANDEYECALIB_H__

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <list>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <opencv2/core/eigen.hpp>

#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>
#include <stdio.h>
#include <sstream>
#include <iomanip>


class HandEyeCalib 
{
public: 
  
     HandEyeCalib();
     
     ~HandEyeCalib();
     
     int init(const std::vector<cv::Mat>& cMo_in, const std::vector<cv::Mat>& wMe_in);
     
     int run(cv::Mat& eMc_ou);

private:
  
  int N;
  
  // VISP data types
  // Input: six couple of poses used as input in the calibration proces
  std::vector<vpHomogeneousMatrix> cMo_vis; // eye (camera) to object transformation. The object frame is attached to the calibrartion grid
  std::vector<vpHomogeneousMatrix> wMe_vis; // world to hand (end-effector) transformation
  // Output: Result of the calibration
  vpHomogeneousMatrix eMc_vis; // hand (end-effector) to eye (camera) transformation 
  
  // OpenCV data types
  std::vector<cv::Mat> cMo_cv, wMe_cv;
  cv::Mat eMc_cv;
  
  int Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou);
  int ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou);
};

#endif