#pragma once

#include <iostream>
using namespace std;

#include <string>

#include <opencv2/opencv.hpp>
//open3d
#include <Open3D/Open3D.h>
using namespace open3d;
#include "OperateImage.h"

struct reconstructInfo {
	/*
		0x01：转到机械臂世界坐标系
		0x02：转到AGV世界坐标坐标
		0x04：限幅，距离设置点（set point）一定范围（dmm）内的点保留
		0x08：按掩膜进行抠图
		0x10：thermal上色
		0x20：插入世界点时，与全局点云体素化后进行比较，判断是否被占用
	*/
	int _2whichbase = 0x01;
	//_2whichbase == 0x01时有效
	bool _agvRT = false; // false：不旋转平移，ture：旋转平移
	// _agvRT=ture时有效
	Eigen::Vector3d _agvRTInfo = Eigen::Vector3d{ 0.,0.,0. }; // AGV具体旋转多少
	double _agvOriDir = 0.; // 计算diff时的AGV初始朝向
	//_2whichbase == 0x02时有效
	Eigen::Vector3d _agvCurrInfo = Eigen::Vector3d{ 0.,0.,0. }; // AGV当前位姿
	//_2whichbase == 0x04时有效
	Eigen::Vector3d _setPoint = Eigen::Vector3d{ 0.,0.,0. }; // 设置点
	double _dmm = 100000; // 到设置点距离小于<_dmm保留，默认100m	
	//_2whichbase == 0x08时有效
	cv::Mat _mask = cv::Mat::zeros(480, 640, CV_8U); // 掩膜
	//_2whichbase == 0x10时有效
	cv::Mat _thermal = cv::Mat::zeros(480, 640, CV_8UC3); // thermal上色图片
	//_2whichbase == 0x20时有效
	std::shared_ptr<open3d::geometry::VoxelGrid> _totalVoxel = std::make_shared<open3d::geometry::VoxelGrid>(); // 全局点云栅格化
};

struct PointCloudRotateInfo {
	bool _isNeedRotate = false; // 是否需要旋转点云
	//double _rotateTheta = 0.; // 旋转角度
	Eigen::Vector3d _rotateXYZ = Eigen::Vector3d{ 0.,0.,0. }; // 旋转角度，顺时针为负
	double _minRotatAngle = 2.; // 最小旋转角度
	Eigen::Vector3d _rotateCenter = Eigen::Vector3d{ 0.,0.,0. }; // 旋转中心
};

struct drawAxisInfo {
	double _xLen = 100.;
	double _yLen = 200.;
	double _zLen = 500.;
	Eigen::Vector3d _center = Eigen::Vector3d{ 0., 0., 0. };
};

/// <summary>
/// 像素坐标到世界坐标系下坐标
/// </summary>
/// <param name="row">像素行</param>
/// <param name="col">像素列</param>
/// <param name="cii">图片信息</param>
/// <param name="depth">深度</param>
/// <returns>世界坐标系坐标</returns>
Eigen::Vector3d ConvertPixel2World(int row, int col, const cameraImgInfo& cii, ushort depth = 0);
/// <summary>
/// 根据单张图片返回对应重构的局部点云
/// </summary>
/// <param name="cii">图片结构体信息</param>
/// <param name="ri">重构需要的额外信息，直接返回直接定义结构体即可</param>
/// <returns></returns>
std::shared_ptr<open3d::geometry::PointCloud> ReconstructFromOneImg(const cameraImgInfo& cii, const reconstructInfo& ri);
/// <summary>
/// 将较小的点云从较大的点云中去除
/// </summary>
/// <param name="bigPC">较大的点云</param>
/// <param name="smallPC">较小的点云</param>
/// <returns>去除后的点云</returns>
std::shared_ptr<open3d::geometry::PointCloud> RemoveSmallerPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& bigPC, const std::shared_ptr<open3d::geometry::PointCloud>& smallPC);

void CombinePartialPointCloud(std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC1,
							std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC2, 
							std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& totalPC);
/// <summary>
/// 整体旋转点云
/// </summary>
/// <param name="srcPointCloud">输入原始点云</param>
/// <param name="R">输出矩阵</param>
/// <param name="rotateCenter">旋转中心</param>
/// <param name="dstPointCloud">输出旋转后的点云</param>
std::shared_ptr<open3d::geometry::PointCloud> RotatePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& srcPcl, const PointCloudRotateInfo pcri);
/// <summary>
/// 根据点云外接矩形的尺寸和实际设备大小剪切整个点云
/// </summary>
/// <param name="srcPointCloud">输入点云</param>
/// <param name="rowScanTimes">行分割次数</param>
/// <param name="colScanTimes">列分割次数</param>
/// <param name="rowScanDis">行分割间隔</param>
/// <param name="colScanDis">列分割间隔</param>
/// <param name="vvBoxSegmentPos">分割点坐标</param>
/// <param name="vCropedPointCloud">分割后的所有点云</param>
void CropPointCloudThroughBoundingBox(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud,
	int& rowScanTimes, int& colScanTimes, double& rowScanDis, double& colScanDis, vector<vector<Eigen::Vector3d>>& vvBoxSegmentPos,
	std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedPointCloud, int& lengthMoreWidth);
/// <summary>
/// 根据OrientBoundingBox生成贴合的边界框
/// </summary>
/// <param name="srcPointCloud">输入：原始点云</param>
/// <param name="vEightPoint">输出：8个顶点</param>
void GenerateBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> srcPointCloud, vector<Eigen::Vector3d>& vEightPoint);
/// <summary>
/// 根据OrientBoundingBox，生成从z轴向xy平面的2D图像
/// </summary>
/// <param name="srcPointCloud">输入：原始点云</param>
/// <param name="vBoundingBox">输入：8个顶点</param>
/// <param name="img">输出：投影得到的图片</param>
/// <param name="validPixel">输出：有效像素的个数</param>
/// <param name="outImgType">输出：图片类型0：黑白、1：灰度、2彩色</param>
/// <param name="dir">输出：方向0、1如下，表示倾斜方向</param>
///0------  1------
/// |\       |    /
/// | \      |   /
/// |  \     |  /
/// |   \    | /
/// |    \   |/
/// --------  --------
void ProjectPointCloudFromZOrient(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud, vector<Eigen::Vector3d>& vBoundingBox, cv::Mat& img, int& validPixel, uchar outImgType, bool& dir);
/// <summary>
/// 从2D图像获得相机x、y轴指向，通过无效像素的个数（三角形面积）和行、列的正切值，确定倾斜夹角
/// </summary>
/// <param name="src">输入：z方向投影的二维图片</param>
/// <param name="validCnt">输入：有效的像素个数</param>
/// <param name="xOrient">输出：x方向的方向向量</param>
/// <param name="yOrient">输出：y方向的方向向量</param>
void FromImgGetXYOrient(cv::Mat& src, int validCnt, bool leanDir, Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient, int lengthMoreWidth);
/// <summary>
/// 绘制世界坐标系的XYZ轴，长度递增
/// </summary>
/// <param name="axis">输出轴的点云</param>
std::shared_ptr<open3d::geometry::LineSet> DrawXYZAxisAtOrient(const drawAxisInfo dai);
/// <summary>
/// 绘制设备大致3d轮廓
/// </summary>
/// <param name="device_cloud">输出轴的点云</param>
std::shared_ptr<open3d::geometry::LineSet> DrawDeviceLinePointCloud(Eigen::Vector3d position, Eigen::Vector3d xOrient, Eigen::Vector3d yOrient, Eigen::Vector3d zOrient);
/// <summary>
/// 根据相机z轴方向、FOV确定当前位置相机的视场范围
/// </summary>
/// <param name="x">相机z方向</param>
/// <param name="y">相机y方向</param>
/// <param name="z">相机x方向</param>
/// <returns>向量列表，4个向量，0/1：对应长边，2/3：对应短边（和四个顶点相对位置）</returns>
vector<Eigen::Vector3d> GetAngleBounding(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, bool hd);
/// <summary>
/// 当前向量是否在相机视角范围内
/// </summary>
/// <param name="vec">当前相机与点云点构成向量</param>
/// <param name="camZOrient">相机z方向</param>
/// <param name="maxAngle">最大角度</param>
/// <returns>true:在视场内</returns>
bool VectorInRange(Eigen::Vector3d vec, Eigen::Vector3d camZOrient);

/// <summary>
/// 从特定相机姿态将3d点云投影到2d图像
/// </summary>
/// <param name="pd">点云</param>
/// <param name="pose">相机位姿</param>
/// <returns>图像列表：索引0：二进制图片；索引1：灰度图片；索引2：彩色图片；索引3：腐蚀膨胀后的二进制图片；索引4：伪深度图</returns>
vector<cv::Mat> PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose);
vector<cv::Mat> PerspectivePointCloudFromSpecialPoseHD(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose);

controlRobotInfo ConvertMask2CameraPose(const cameraImgInfo& cami, const bboxImgInfo& bbi, int bboxIndex, int con2whichcam);
//void PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose, vector<cv::Mat>& vMat);
/// <summary>
/// 求图像掩膜和参考掩膜的交集
/// </summary>
/// <param name="src">图像列表：索引0：二进制图片；索引1：灰度图片；索引2：彩色图片；索引3：腐蚀膨胀后的二进制图片</param>
/// <param name="ref">参考掩膜</param>
/// <param name="dst"></param>
void MaskIntersection(const vector<cv::Mat>& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, vector<cv::Mat>& dst);
void MaskIntersection(const cv::Mat& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, cv::Mat& dst);
/// <summary>
/// 处理掩膜中的异常点
/// </summary>
/// <param name="src">图像列表：索引0：二进制图片；索引1：灰度图片；索引2：彩色图片；索引3：腐蚀膨胀后的二进制图片</param>
void DealMaskRegion(vector<cv::Mat>& src);
void ConvertPoint2VoxelPos(const Eigen::Vector3d p, Eigen::Vector3i& pos);