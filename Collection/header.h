#pragma once

#include <iostream>
using namespace std;

#include <string>
#include <memory>
#include <io.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>

//astra
#include "astra/AstraCamera.h"
#include "astra/AstraCameraD2C.h"
#include "astra/d2cSwapper.h"
#include "astra/FrameListener.h"
//Thermal
#include "MagDevice.h"
#include "MagService.h"
// mind vision
#include "MindVision.h"
//opencv
#include <opencv2/opencv.hpp>
//using namespace cv;

//open3d
#include <Open3D/Open3D.h>
using namespace open3d;

#include <Windows.h>
// 调用python
#include <Python.h>

//多线程
#include <process.h>

//创建文件夹
#include <direct.h>

#include "IOFile.h"
#include "RobotArmControl.h"
#include "Shlobj.h" //文件夹

#include <comutil.h> //char* ת wchar_t 
// 相机手眼标定
#include "CalibrateEyeInHand.h"
//// 点云显示，现已不用 
//#include "Visualizer.h"
// 点云操作
#include "OperatePointCloud.h"
// 图像操作
#include "OperateImage.h"

#include "Eigen/Dense"
// 点云划分红外测量场
#include "PointCloudPartitionForScan.h"
// AGV控制
#include "Seer.h"
#include "dijkstra.h"

// 定时器
#include <time.h>
#include "timer.hpp"
// 机械臂路径规划
#include "RoutePlaning.h"

#define COLLECT_RGBD 0 // 采集RGBD数据；1：采集；0：推理后画框
#define AGV_CONTROL 1 // AGV控制
#define RGBD_CAMERA 1 // 是否使用深度相机
#define HDRGB_CAMERA 0 // 是否使用高清视觉
#define INTERACTION 1 // 是否交互
#define VLID_INFERENCE 0 // 是否推理
// 定时器定时时间
#define TIMER_TIMING_TIME 20 // 20ms

// 深度相机句柄
extern AstraCameraD2C* astraCameraD2C;
extern string dataPath;

#define sign(x) ( ((x) <0 )? -1 : 1 )
// rgbd astra
#define HORIZON_ANGLE (double)(63.1 / 2)
#define VERTICAL_ANGLE (double)(49.4 / 2)
#define EDGE_ANGLE (double)atan(sqrt(tan(HORIZON_ANGLE * CV_PI / 180) * tan(HORIZON_ANGLE * CV_PI / 180) + tan(VERTICAL_ANGLE * CV_PI / 180) * tan(VERTICAL_ANGLE * CV_PI / 180)))
#define IMAGE_WIDTH (int)640
#define IMAGE_HEIGHT (int)480
// hd mind vision
//#define HORIZON_ANGLE_HD (double)(38.9 / 2) // 2/3''
//#define VERTICAL_ANGLE_HD (double)(29.6 / 2)
//#define EDGE_ANGLE_HD (double)(47.9 / 2)
//#define HORIZON_ANGLE_HD (double)(54.8 / 2) // 1''
//#define VERTICAL_ANGLE_HD (double)(42.3 / 2)
//#define EDGE_ANGLE_HD (double)(66.1 / 2)
#define HORIZON_ANGLE_HD (double)(71.2 / 2) // 4/3''
#define VERTICAL_ANGLE_HD (double)(56.3 / 2)
#define EDGE_ANGLE_HD (double)(83.4 / 2)
//#define EDGE_ANGLE_HD (double)atan(sqrt(tan(HORIZON_ANGLE_HD * CV_PI / 180) * tan(HORIZON_ANGLE_HD * CV_PI / 180) + tan(VERTICAL_ANGLE_HD * CV_PI / 180) * tan(VERTICAL_ANGLE_HD * CV_PI / 180)))
//#define IMAGE_WIDTH_HD (int)2048
//#define IMAGE_HEIGHT_HD (int)1500
#define IMAGE_WIDTH_HD (int)4096
#define IMAGE_HEIGHT_HD (int)3000

#define ROBOTARM_X_MIN -700
#define ROBOTARM_X_MAX 700
#define ROBOTARM_Y_MIN -1200
#define ROBOTARM_Y_MAX -10
#define ROBOTARM_Z_MIN -300
#define ROBOTARM_Z_MAX 2000

extern std::shared_ptr<open3d::geometry::PointCloud> pcl_ptr; // 全局点云
//extern std::shared_ptr<open3d::geometry::VoxelGrid> voxel_ptr; // 全局点云体素后信息
extern std::shared_ptr<open3d::geometry::VoxelGrid> pcl_voxel; // 全局点云体素后信息
extern cv::Mat_<double> CameraMatrix;
extern cv::Mat_<double> ThermalMatrix;
extern cv::Mat_<double> CameraMatrix_HD;
extern cv::Mat_<double> H_Camera2Gripper;
extern cv::Mat_<double> H_Thermal2Gripper;
extern cv::Mat_<double> H_Camera2Gripper_HD;

extern vector<cameraImgInfo> vRGBDInfo;
extern vector<cameraImgInfo> vHDRGBInfo;
extern vector<cameraImgInfo> vThermalInfo;
extern vector<bboxImgInfo> vHDBboxInfo;

extern bool exitChildThread;
extern int mouseClickRow;
extern int mouseClickCol;