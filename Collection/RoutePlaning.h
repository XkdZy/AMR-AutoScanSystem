#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
using namespace std;

typedef struct _SearchVoxelRegion {
	Eigen::Vector3i v; // 体素块区域xyz最小顶点对应体素
	int width; // 宽
	int length; // 长
	int height; // 高
}SearchVoxelRegion;

struct Voxel2PclInfo {
	Eigen::Vector3d minPoint; // xyz最小值对应的顶点
	Eigen::Vector3d size; // 长宽高
}; // 体素转换到点云的必要信息
extern Voxel2PclInfo v2pInfo;

// 路径规划
typedef struct _vertex {
	bool occupy;
	int num; // 顶点编号
	Eigen::Vector3i v; // 顶点空间位置
	Eigen::Vector3d vPcl; // 顶点体素块中心对应点云中实际位置
	//vector<int> vVertexNum; // 连接的节点
	int* vVertexNum; // 连接的节点编号
}vertex;

extern vertex startVertex;
extern vertex goalVertex;
extern vertex* allVertexArr;
extern vertex* expandedVertexArr;

extern vector<int> vCurrentTrajectory;
extern vector<vertex> vCurrentTrajectoryVertex;
extern int CurrAGVMoveIdx; // 当前AGV移动点位索引
extern vector<Eigen::Vector3d> vAGVMovePosition; // AGV移动坐标（AGV坐标系下）
extern vector<Eigen::Vector3d> vArmMovePosition; // 机械臂移动坐标（AGV初始位置时的机械臂坐标系下）

extern vector<int> vTest;

extern Eigen::Vector3d objectCenterPos; // 粗定位目标中心坐标

// 手持式光激励红外热成像设备尺寸
#define DEVICE_LENGTH (double)299.0 // 长：29.9cm
#define DEVICE_WIDTH (double)265.0 // 宽：26.5cm
#define DEVICE_HEIGHT (double)320.0 // 长宽平面到屏幕平面高度：32.0cm
//#define DEVICE_LENGTH (double)164.4 // 长：29.9cm
//#define DEVICE_WIDTH (double)72.4 // 宽：26.5cm
//#define DEVICE_HEIGHT (double)164.4 // 长宽平面到屏幕平面高度：32.0cm
//#define DEVICE_LENGTH (double)1. // 长：29.9cm
//#define DEVICE_WIDTH (double)1. // 宽：26.5cm
//#define DEVICE_HEIGHT (double)1. // 长宽平面到屏幕平面高度：32.0cm
#define DEVICE_PLAIN2CAM (double)154.0 // 摄像头到长宽平面：15.4cm
#define DEVICE_REMAINPLAIN2CAM (double)20.0 // 预留摄像头镜面到光心
#define DEVICE_HALFHYPOTENUSE (double)(0.5 * sqrt(DEVICE_LENGTH * DEVICE_LENGTH + DEVICE_WIDTH * DEVICE_WIDTH + DEVICE_HEIGHT * DEVICE_HEIGHT)) // 长方体设备的斜边长
#define DEVICE_OCCUPYVOXEL (int)ceil(DEVICE_HALFHYPOTENUSE / VOXEL_SIZE)

#define VOXEL_SIZE (double)20. // 每个体素块大小VOXEL_SIZE*VOXEL_SIZE*VOXEL_SIZE


/// <summary>
/// 根据目标检测确定的目标位置生成相机的观测位置
/// </summary>
/// <param name="objPos"></param>
/// <param name="camPos"></param>
void FromObjPos2CamObservePos(const Eigen::Vector3d& camPos, const Eigen::Vector3d& objPos, Eigen::Vector3i camDisPos);
/// <summary>
/// 从起始点、目标点以及设备尺寸确定搜索的体素范围
/// </summary>
/// <param name="start">起始点</param>
/// <param name="goal">目标点</param>
/// <param name="svr">搜索的体素范围</param>
void GetSearchRegion(const Eigen::Vector3i& start, const Eigen::Vector3i& goal, SearchVoxelRegion& svr);
/// <summary>
/// 对搜索区域的体素块进行编号
/// </summary>
/// <param name="svr">搜索的体素范围</param>
/// <param name="start">起始点</param>
/// <param name="goal">目标点</param>
/// <param name="allVertex">所有编好号的体素顶点</param>
/// <param name="startVertex">起始顶点</param>
/// <param name="goalVertex">目标顶点</param>
void VertexNumFromSearchRegion(const SearchVoxelRegion& svr, const Eigen::Vector3i& start, const Eigen::Vector3i& goal, vertex& startVertex, vertex& goalVertex);
/// <summary>
/// 对搜索区域内的体素按设备尺寸进行膨胀
/// </summary>
/// <param name="svr">搜索的体素范围</param>
/// <param name="allVertex">所有顶点</param>
/// <param name="expandedVertex">膨胀后所有顶点</param>
void ExpandBarrierRegion(SearchVoxelRegion& svr, vertex* expandedVertex);
/// <summary>
/// 对无向、有环、无权图进行BFS搜索确定最短路径
/// </summary>
/// <param name="len">顶点个数</param>
/// <param name="start">起始点</param>
/// <param name="goal">目标点</param>
/// <param name="bfsLine">最优路线</param>
bool BreadthFirst(const int len, const vertex& start, const vertex& goal, vector<int>& bfsLine);
/// <summary>
/// 根据当前点云pclRealTime_ptr更新规划一条路径vCurrentTrajectoryVertex
/// </summary>
/// <param name="start">当前位置</param>
/// <param name="end">目标位置</param>
void UpDateRouteBaseCurrentPointCloud(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
/// <summary>
/// 判断之前路径route是否满足当前点云pclRealTime_ptr
/// </summary>
/// <param name="bfsLine">之前路径</param>
/// <returns>满足：true</returns>
bool VertifyRouteWhetherMeetRequire(const int startIndex);

/// 数据转换
// 向量计算AB = OB - OA
Eigen::Vector3d CalcVectorAB(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

void ConvertString2Mat44(const string& str, const bool gripper2cam, cv::Mat& m44);
// 根据相机rz指向（多解，选择一个）确定相机姿态
cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, Eigen::Vector3d& rx, Eigen::Vector3d& ry);
cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, const Eigen::Vector3d& rx, const Eigen::Vector3d& ry, Eigen::Vector3d& rxAdjusted, Eigen::Vector3d& ryAdjusted);
// 根据目标位置调整相机姿态
void AdjustCameraPosture(const string& nowGripperPose, const Eigen::Vector3d& objPos, string& adjustedGripperCmd, cv::Mat& adjustedR);
void AdjustCameraPosture(const Eigen::Vector3d& nowPose, const Eigen::Vector3d& objPos, cv::Mat& adjustedR);
void UpdateRealTimePointCloud();

