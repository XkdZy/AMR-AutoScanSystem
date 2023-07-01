#pragma once

#include <iostream>
#include <string>
#include <vector>
// opencv
#include <opencv2/opencv.hpp>
//open3d
#include <Open3D/Open3D.h>
using namespace open3d;

using namespace std;

// engin
#include "Eigen/Dense"

struct PointCloudInfo {
	// 点云参考正方向
	Eigen::Vector3d _refRx;
	Eigen::Vector3d _refRy;
	// 点云最小、最大边界
	Eigen::Vector3d _minBound;
	Eigen::Vector3d _maxBound;
	// 点云长、宽、高
	Eigen::Vector3d _extent;
};

// 对于一次红外扫查所需要的信息
struct ThermalScanInfo {
	// 相机信息
	Eigen::Vector3d _cameraPosition; // 3d位置
	Eigen::Vector3d _cameraRx; // 3d Rx
	Eigen::Vector3d _cameraRy; // 3d Ry
	Eigen::Vector3d _cameraRz; // 3d Rz
	cv::Mat _cameraPosetrue;  // 3*3 相机姿态
	// 相机信息对应的末端信息
	Eigen::Vector3d _gripperPosition; // 3d 末端位置
	Eigen::Vector6d _gripperPoseVec; // 6d 末端位姿
	cv::Mat _gripperPosetrue; // 3*3末端姿态
	cv::Mat _gripperPoseMat; // 4*4末端位姿
	string _cmdStr; // 末端6d发送命令
};

//#define DEVICE_LENGTH (double)220. // 长：29.9cm
//#define DEVICE_WIDTH (double)220. // 宽：26.5cm
#define DEVICE_LENGTH (double)299. // 长：29.9cm
#define DEVICE_WIDTH (double)265. // 宽：26.5cm
#define PLAIN2CAMERA (double)154. // 摄像头到长宽平面：15.4cm
#define REMAIN_PLAIN2CAMERA (double)50. // 摄像头到长宽平面：15.4cm
#define INTERSECTION_COFF (double)1.2 // 尺寸长宽深长系数：1.2


void InitPointCloudInfo(const std::shared_ptr<open3d::geometry::PointCloud>& pcl, PointCloudInfo& pcli);
void CalcPointCloudXYOrient(Eigen::Vector3d xOrientRec, Eigen::Vector3d yOrientRec, Eigen::Vector3d zOrient,
	Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient);
vector<std::shared_ptr<open3d::geometry::PointCloud>> PointCloudPartition(const std::shared_ptr<open3d::geometry::PointCloud>& pcl);

vector<std::shared_ptr<open3d::geometry::LineSet>> PointCloudPartitionPCA(
	const PointCloudInfo& pcli,
	vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedCloudForPCA,
	vector<ThermalScanInfo>& _scanInfo
);
void IntergateScanInfo(ThermalScanInfo& tsi);
void CalcPointCloudNormPCA(std::shared_ptr<open3d::geometry::PointCloud>& pointCloud, Eigen::Vector3d& norm, Eigen::Vector3d& center);
cv::Mat RotatePclMat(double rotateTheta);