#include "HandEyeCalib.hpp"

HandEyeCalib::HandEyeCalib()
{
  
}
     

HandEyeCalib::~HandEyeCalib()
{
  
}


int HandEyeCalib::init(const std::vector<vpHomogeneousMatrix>& cMo_in, const std::vector<vpHomogeneousMatrix>& wMe_in)
{
	cMo_vis = cMo_in; 
	wMe_vis = wMe_in;
}	
     
int HandEyeCalib::init(const std::vector<cv::Mat>& cMo_in, const std::vector<cv::Mat>& wMe_in)
{
  int ret = 0;
  
  N = cMo_in.size();
  
  vpHomogeneousMatrix mat_vis;
  for (int i=0; i<N; i++)
  {
     Mat2ViSP(cMo_in[i], mat_vis);
     cMo_vis.push_back(mat_vis);   // this is wrong!! or not?
     
     Mat2ViSP(wMe_in[i], mat_vis);
     wMe_vis.push_back(mat_vis);   // this is wrong!! or not?
  }
  
  return ret;
}
     
     
     
/**
 * @brief copy OpenCV Mat to ViSP vpHomogeneousMatrix     
 */
int HandEyeCalib::Mat2ViSP(const cv::Mat& mat_in, vpHomogeneousMatrix& visp_ou)
{
  int ret = 0;
  
  //if (mat_in.type() != CV_32FC1 || mat_in.type() != CV_64FC1) 
  if (mat_in.type() != CV_64FC1) 
  {
    //std::cout << "[HandEyeCalib] Mat input is not floating-point number!" << std::endl;
    std::cout << "[HandEyeCalib] Mat input is not double floating-point number!" << std::endl;
    ret = 1;
    return ret;
  }
  
  for (int i=0; i<mat_in.rows; i++)
  {
    for (int j=0; j<mat_in.cols; j++)
    {
      visp_ou[i][j] = mat_in.ptr<double>(i)[j];  // new memory is created and data is copied in this line
    }
  }
  
  return ret;
}    



/**
 * @brief copy ViSP vpHomogeneousMatrix to OpenCV Mat
 */
int HandEyeCalib::ViSP2Mat(const vpHomogeneousMatrix& visp_in, cv::Mat& mat_ou)
{
  int ret = 0;
  
  mat_ou = cv::Mat::zeros(visp_in.getRows(),visp_in.getCols(),CV_64FC1);
  
  for (int i=0; i<mat_ou.rows; i++)
  {
    for (int j=0; j<mat_ou.cols; j++)
    {
      mat_ou.ptr<double>(i)[j] = visp_in[i][j];  // new memory is created and data is copied in this line
    }
  }
  
  return ret;
}    


     
     
int HandEyeCalib::run(cv::Mat& eMc_ou)
{
  int ret = 0;
  
  vpHomogeneousMatrix eMc_vis;
  vpThetaUVector erc;
  
  // Compute the eMc hand to eye transformation from six poses
  // - cMo[6]: camera to object poses as six homogeneous transformations
  // - wMe[6]: world to hand (end-effector) poses as six homogeneous transformations
  vpCalibration::calibrationTsai(cMo_vis, wMe_vis, eMc_vis) ;

  std::cout << std::endl << "Output: hand to eye calibration result: eMc estimated " << std::endl ;
  std::cout << eMc_vis << std::endl ;
  eMc_vis.extract(erc);
  std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
  
  ViSP2Mat(eMc_vis, eMc_ou);
  
  return ret;
}