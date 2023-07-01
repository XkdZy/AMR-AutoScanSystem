#pragma once
#include <iostream>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

/*opencv图像显示头文件*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

//cv::Mat_<double> CalPose;
//cv::Mat_<double> ToolPose;

void str2Mat(const string strPosition, Mat& toolPose);
void str2Mat(vector<string>& strPosition, Mat& toolPose);
cv::Mat R_T2RT(cv::Mat& R, cv::Mat& T);
void RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T);
bool isRotationMatrix(const cv::Mat& R);
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq);
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);
cv::Mat attitudeVectorToMatrix(cv::Mat m, bool useQuaternion, const std::string& seq);
void m_calibration(string calibraDataName, vector<string>& FilesName, Size board_size, Size square_size, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecsMat, vector<Mat>& tvecsMat);
void cameraCaliration(string calibraDataName, vector<Mat>& tvecsMat, vector<Mat>& rvecsMat);
void CalibrateEyeInHand(bool rgbdOrThermal = true, string calibraDataName = "E:\\Project\\C++\\Lab\\Collection\\Collection\\imgs\\rgbd\\color");