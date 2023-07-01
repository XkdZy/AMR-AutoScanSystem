#pragma once

#include <windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "Eigen/Dense"
#include "rokae/robot.h"

using namespace std;
using namespace rokae;

// 0：安川机械臂 1：珞石机械臂
#define ROBOT_TYPE 1 

#define ROBOT_OFFLINE_TEST 1 // 0：离线测试

#if ROBOT_TYPE == 1
#define PLACE_METHOD 1 // 机械臂放置方式，底座放置方式	0：垂直；1：侧放
#endif

#define USE_MULTITHREAD 0

extern mutex mutWritePos; // 多线程读写坐标互斥锁
extern string sRealTimePos; // 实时机械臂坐标

// 安川机械臂：串口通信
class SerialPort
{
public:
	SerialPort(void);
	~SerialPort(void);
	//打开串口
	
	bool Serial_open(LPCSTR COMx, int BaudRate);
	// 发送数据
	int Serial_write_string(string& Buf, int size);
	int Serial_write_char(unsigned char* Buf, int size);
	// 接收数据
	int Serial_read_string(string &OutBuf,int maxSize);
	int Serial_read_char(unsigned char* OutBuf,int maxSize);
	// 复用：读取机械臂位置
	bool ReadRobotArmPos(SerialPort& sp, vector<float>& vPos);
	void ReadRobotArmPosString(SerialPort& sp, string& strPos);
public:
	HANDLE m_hComm;//串口句柄
};

extern SerialPort sp;

// 珞石机械臂：C++ API
extern string rokaeIP;
extern XMateRobot* rokaeRobot;
extern error_code ec;

void WaitRokaeRobot(BaseRobot* robot);
void StopRokaeRobot(); 
// moveMode	0：moveL直线；1：moveJ机器人自己规划避开奇异点
void MoveRokae2OnePoint(const vector<double>& pos, INT8 moveMode);
void MoveRokae2MultiPoint(const vector<vector<double>>& pos);

// 通用方法
// 将机械臂CMD转换为6DMat
void AnalysisString26D(const string& str, vector<double>& v);
void AnalysisString26D(const string& str, cv::Mat& pose);
// 将6D坐标转化为机械臂CMD
void Convert6D2String(const vector<double>& v, string& str);
// 将4*4RT转化为机械臂CMD
void ConvertMat2String(const cv::Mat& mat, string& str);
//// 将6DMat转化为4*4Mat
//void Convert6DMat2Mat(const cv::Mat& src, cv::Mat& dst);
// 判断机械臂是否到达目标点
bool WhetherRobotArmMoveToPoint(string dstPoint);
// 旋转矩阵转欧拉角
Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R);
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat& R);
// 读取运动点坐标
int ReadPointFile(fstream* fHandle, char* ch, int length);

// 安川、珞石通用
// 阻塞当前主线程运行
void MoveToOnePoint(string point, SerialPort* sp);
//// 非阻塞主线程运行，阻塞子线程，运动到目标查询move2CurrTarget状态
//void MoveToOnePointMultiThread(string point, SerialPort* sp);