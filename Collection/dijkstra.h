#pragma once

#include "Seer.h"

#include <opencv2/opencv.hpp>
#include <Open3D/Open3D.h>
#include "Eigen/Dense"

#include "RobotArmControl.h"
#include "CalibrateEyeInHand.h"
#include "RoutePlaning.h"


using namespace open3d;
using namespace cv;
using namespace std;

//#include "PointCloudPartitionForScan.h"


#define PI 3.14159265
#define X 84
#define Y 63

extern cv::Mat_<double> H_Camera2Gripper;
extern int FR[X][Y];
extern int FR1[X][Y];
extern int FR2[X][Y];

struct AgvMoveInfo {
	/*	AGV是否到达目标
	* 0：未到达;
	* 1：AGV激光被阻塞；
	* 2：RGBD检测到阻塞;
	* 3：RGBD实时检测子线程退出
	* 4：
	*/
	int _move2goal = 0; 
	Eigen::Vector3d _worldGoalPos = Eigen::Vector3d{ 0., 0., 0. }; // 世界坐标xyz
	Eigen::Vector3d _currAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // 当前AGV坐标朝向xyθ（θ弧度）
	Eigen::Vector3d _oriAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // 初始AGV坐标朝向xyθ（θ弧度）
	Eigen::Vector3d _oriArmPos = Eigen::Vector3d{ 0., 0., 0. }; // 初始机械臂末端初始坐标
};
extern AgvMoveInfo agi;

//int map_obstacles();
int map_obstacles(string path);
int* dijkstra(float x, float y, float xx, float yy);
int map_Laser_res(SOCKET& client);
void Move2Goal(const Eigen::Vector2d turegoal, const Eigen::Vector2d goal);
void SubThreadAdjustRobotArmPosture();
void RobotArmReset(); // 初始化机械臂末端rz指向x方向
void RobotArm2Goal(); // 调整相机位姿指向目标位姿
/// <summary>
/// 机械臂坐标点经过AGV旋转、平移后坐标
/// </summary>
/// <param name="agvMoveInfo">0：x方向移动（当前相对初始）；1：y方向移动（当前相对初始）；2：旋转角（当前相对初始）</param>
/// <param name="robotPos">curr2ori为ture时，为当前AGV位姿下机械臂坐标点，false为初始AGV位姿下机械臂坐标点</param>
/// <param name="curr2ori">true：机械臂当前点到初始点</param>
/// <returns>旋转、平移后机械臂坐标</returns>
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta); // 机械臂坐标点经过旋转、平移后对应初始AGV状态下的坐标
void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos); // 将AGV运动轨迹转化为机械臂运动轨迹
void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos); // 将AGV初始位置机械臂运动轨迹转化到当前位置下

cv::Mat arm2agv(const double x, const double y, const double theta);
cv::Mat arm2agv(const Eigen::Vector3d agv);
Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint);
cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint);
vector<Eigen::Vector3d> CalcAgvPosForRGBD(double radius, double safedis, double theta);