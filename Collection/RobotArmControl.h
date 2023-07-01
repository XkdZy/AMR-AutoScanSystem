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

// 0��������е�� 1����ʯ��е��
#define ROBOT_TYPE 1 

#define ROBOT_OFFLINE_TEST 1 // 0�����߲���

#if ROBOT_TYPE == 1
#define PLACE_METHOD 1 // ��е�۷��÷�ʽ���������÷�ʽ	0����ֱ��1�����
#endif

#define USE_MULTITHREAD 0

extern mutex mutWritePos; // ���̶߳�д���껥����
extern string sRealTimePos; // ʵʱ��е������

// ������е�ۣ�����ͨ��
class SerialPort
{
public:
	SerialPort(void);
	~SerialPort(void);
	//�򿪴���
	
	bool Serial_open(LPCSTR COMx, int BaudRate);
	// ��������
	int Serial_write_string(string& Buf, int size);
	int Serial_write_char(unsigned char* Buf, int size);
	// ��������
	int Serial_read_string(string &OutBuf,int maxSize);
	int Serial_read_char(unsigned char* OutBuf,int maxSize);
	// ���ã���ȡ��е��λ��
	bool ReadRobotArmPos(SerialPort& sp, vector<float>& vPos);
	void ReadRobotArmPosString(SerialPort& sp, string& strPos);
public:
	HANDLE m_hComm;//���ھ��
};

extern SerialPort sp;

// ��ʯ��е�ۣ�C++ API
extern string rokaeIP;
extern XMateRobot* rokaeRobot;
extern error_code ec;

void WaitRokaeRobot(BaseRobot* robot);
void StopRokaeRobot(); 
// moveMode	0��moveLֱ�ߣ�1��moveJ�������Լ��滮�ܿ������
void MoveRokae2OnePoint(const vector<double>& pos, INT8 moveMode);
void MoveRokae2MultiPoint(const vector<vector<double>>& pos);

// ͨ�÷���
// ����е��CMDת��Ϊ6DMat
void AnalysisString26D(const string& str, vector<double>& v);
void AnalysisString26D(const string& str, cv::Mat& pose);
// ��6D����ת��Ϊ��е��CMD
void Convert6D2String(const vector<double>& v, string& str);
// ��4*4RTת��Ϊ��е��CMD
void ConvertMat2String(const cv::Mat& mat, string& str);
//// ��6DMatת��Ϊ4*4Mat
//void Convert6DMat2Mat(const cv::Mat& src, cv::Mat& dst);
// �жϻ�е���Ƿ񵽴�Ŀ���
bool WhetherRobotArmMoveToPoint(string dstPoint);
// ��ת����תŷ����
Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R);
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat& R);
// ��ȡ�˶�������
int ReadPointFile(fstream* fHandle, char* ch, int length);

// ��������ʯͨ��
// ������ǰ���߳�����
void MoveToOnePoint(string point, SerialPort* sp);
//// ���������߳����У��������̣߳��˶���Ŀ���ѯmove2CurrTarget״̬
//void MoveToOnePointMultiThread(string point, SerialPort* sp);