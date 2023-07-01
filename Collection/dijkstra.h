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
	/*	AGV�Ƿ񵽴�Ŀ��
	* 0��δ����;
	* 1��AGV���ⱻ������
	* 2��RGBD��⵽����;
	* 3��RGBDʵʱ������߳��˳�
	* 4��
	*/
	int _move2goal = 0; 
	Eigen::Vector3d _worldGoalPos = Eigen::Vector3d{ 0., 0., 0. }; // ��������xyz
	Eigen::Vector3d _currAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ǰAGV���곯��xy�ȣ��Ȼ��ȣ�
	Eigen::Vector3d _oriAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ʼAGV���곯��xy�ȣ��Ȼ��ȣ�
	Eigen::Vector3d _oriArmPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ʼ��е��ĩ�˳�ʼ����
};
extern AgvMoveInfo agi;

//int map_obstacles();
int map_obstacles(string path);
int* dijkstra(float x, float y, float xx, float yy);
int map_Laser_res(SOCKET& client);
void Move2Goal(const Eigen::Vector2d turegoal, const Eigen::Vector2d goal);
void SubThreadAdjustRobotArmPosture();
void RobotArmReset(); // ��ʼ����е��ĩ��rzָ��x����
void RobotArm2Goal(); // �������λ��ָ��Ŀ��λ��
/// <summary>
/// ��е������㾭��AGV��ת��ƽ�ƺ�����
/// </summary>
/// <param name="agvMoveInfo">0��x�����ƶ�����ǰ��Գ�ʼ����1��y�����ƶ�����ǰ��Գ�ʼ����2����ת�ǣ���ǰ��Գ�ʼ��</param>
/// <param name="robotPos">curr2oriΪtureʱ��Ϊ��ǰAGVλ���»�е������㣬falseΪ��ʼAGVλ���»�е�������</param>
/// <param name="curr2ori">true����е�۵�ǰ�㵽��ʼ��</param>
/// <returns>��ת��ƽ�ƺ��е������</returns>
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta); // ��е������㾭����ת��ƽ�ƺ��Ӧ��ʼAGV״̬�µ�����
void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos); // ��AGV�˶��켣ת��Ϊ��е���˶��켣
void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos); // ��AGV��ʼλ�û�е���˶��켣ת������ǰλ����

cv::Mat arm2agv(const double x, const double y, const double theta);
cv::Mat arm2agv(const Eigen::Vector3d agv);
Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint);
cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint);
vector<Eigen::Vector3d> CalcAgvPosForRGBD(double radius, double safedis, double theta);