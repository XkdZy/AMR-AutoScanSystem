#pragma once

#include <iostream>
using namespace std;
#include <string>
#include <opencv2/opencv.hpp>

#include "dijkstra.h"

// 相机类型，深度图包含信息：图片信息+位置信息
typedef struct _cameraImgInfo {
	// 图片类型，false：rgb，true：rgbd
	bool _type;
	// 图片信息
	cv::Mat _RgbImg;
	cv::Mat _DepthImg;
	// 位置信息
	cv::Mat _Gripper2Base; // 机械臂读取到的Gripper位置
	Eigen::Vector3d _CameraPosition; // 相机位置
	cv::Mat _CameraPose; // 相机位姿
	Eigen::Vector3d _CameraPostureEig; // 相机姿态欧拉角形式
	cv::Mat _CameraPostureMat; // 相机姿态旋转矩阵形式
	// 6D位姿字符串形式
	string _poseStr;
	
}cameraImgInfo;

// 当前图片对应的掩膜信息
typedef struct _bboxImgInfo {
	int _imgType; // 0：RGBD；1：HD
	cv::Mat _src; // bgr
	cv::Mat _selectmask; // mask
	vector<cv::Mat> _img; // 所有掩膜图片 binary
	//vector<cv::Mat> _maskSrc; // 根据掩膜抠图 gary
	vector<string> _imgPath; // 所有掩膜图片 binary
	//vector<string> _maskSrc; // 根据掩膜抠图 gary
	vector<double> _maskArea; // 掩膜面积
	vector<cv::Rect> _bboxRect; // 所有bbox信息(x,y,width,height)
	vector<Eigen::Vector4i> _bboxAB; // 所有bbox信息(x1,y1,x2,y2)
	vector<vector<cv::Point>> _totalContours; // 当前image所有轮廓
	vector<vector<cv::Point>> _maskContour; // 每一个bbox掩膜对应最长的轮廓
	vector<vector<cv::Point>> _contour; // 每一个bbox对应最近的轮廓
	vector<double> _contourErr; // 每一个bbox对应最近轮廓的误差
}bboxImgInfo;

typedef struct _controlRobotInfo {
	cv::Mat _campose; // 4*4
	cv::Mat _camposture; // 3*3
	cv::Mat _campositionMat; // 4*1
	Eigen::Vector3d _campositionEigen;
	Eigen::Vector3d _cameuler;
	string _grippercmdStr;
}controlRobotInfo;

bool PixelInBBox(const cv::Rect& rec, const cv::Point& pixel);
/// <summary>
/// 定时器中断
/// </summary>
void TimerInterpute(void);
/// <summary>
/// 采集显示实时的RGBD数据
/// </summary>
void CollectImgAndInteraction(cameraImgInfo& cii);
/// <summary>
/// 将两张图片拼接到一张图片上
/// </summary>
/// <param name="m1">俯视图</param>
/// <param name="m2">前视图</param>
/// <param name="m2">拼接后的图片</param>
void CombinateTwoImg(const cv::Mat& mVertical, const cv::Mat& mFront, cv::Mat& dst);
/// <summary>
/// 更新实时图片信息depthImgInfo
/// </summary>
/// <param name="rgb">rgb</param>
/// <param name="depth">深度图</param>
/// <param name="imgPath">机械臂末端信息</param>
void UpdateRealTimePosInfo(const cv::Mat& rgb, const cv::Mat& depth, string imgPose, cameraImgInfo& cii);
/// <summary>
/// 将一组图像中的掩模按照大小进行排序
/// </summary>
/// <param name="vImg">图像容器</param>
/// <returns>返回从大到小的索引</returns>
vector<int> CompareImageArea(const vector<cv::Mat>& vImg);
/// <summary>
/// 根据mask统计伪深度有效点数量
/// </summary>
/// <param name="img">二值化原始图像</param>
/// <param name="depth">double深度图</param>
/// <param name="mask">输出图像掩膜</param>
/// <returns>返回图像有效面积（二值化为255且有深度值）</returns>
int CountImageWhiteArea(const cv::Mat& img, const cv::Mat& depth, const double moreThanVal, cv::Mat& mask);
double CountImageAgvDepth(const cv::Mat& depth, cv::Mat& mask);
/// <summary>
/// 整合图片信息：到末端转换关系、相机位姿、末端位姿等
/// </summary>
/// <param name="rgb">输入rgb图像</param>
/// <param name="depth">输入depth图像（type为0时，rgb=depth）</param>
/// <param name="pose">输入当前末端位姿</param>
/// <param name="type">输入图片类型：0：rgbd	1：hd rgb	2：thermal</param>
/// <param name="cameraInfo">输出：图片整合后信息</param>
void IntegrateImgInfo(const cv::Mat& rgb, const cv::Mat& depth, string pose, int type, cameraImgInfo& cameraInfo);
/// <summary>
/// 整合VILD推理BBox图片信息：bbox位置、binary掩膜路径等（高清缓存成本太大只保留图片路径）
/// </summary>
/// <param name="bboxPth">bbox跟目录位置，推理结果俺工程./imgs/hdvild放</param>
/// <param name="bboxInfo">bbox结构体</param>
/// <param name="type">输入图片类型：0：rgbd	1：hd rgb	2：thermal</param>
/// <param name="type">图片推理类型：0：vild	>=1：bts 并表示图片索引</param>
void IntegrateBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype, int inftype);
void IntegrateVildBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype);
/// <summary>
/// 将像素坐标系转换到图像坐标系
/// </summary>
/// <param name="u">像素坐标系下列</param>
/// <param name="v">像素坐标系下行</param>
/// <param name="d">当前（u，v）像素对应深度</param>
/// <param name="cameraType">相机类型：0深度；1HD；2Thermal</param>
/// <returns></returns>
Eigen::Vector3d ConvertPixel2Camera(const int& u, const int& v, const ushort& d, const int& cameraType);

bool CallPythonScriptInference(bool vildOrbts);

double FindHHOptimumExposureTime();
/// <summary>
/// 寻找图片中轮廓
/// </summary>
/// <param name="src">图片</param>
/// <param name="oneMax">1：只返回最长轮廓，0：返回所有轮廓</param>
/// <returns>轮廓列表</returns>
vector<vector<Point>> FindContours(const cv::Mat& src, bool oneMax);
/// <summary>
/// 找出目标轮廓相对所有轮廓中误差最小的轮廓
/// </summary>
/// <param name="src">所有轮廓</param>
/// <param name="mask">目标轮廓</param>
/// <param name="optimumSrc">所有轮廓中误差最小的原始轮廓</param>
/// <param name="optimumed">optimumSrc中误差最小轮廓部分点</param>
/// <returns>当前轮廓误差</returns>
double FindOptimumContours(const vector<vector<Point>>& src, const vector<vector<Point>>& mask, vector<cv::Point>& optimumSrc, vector<cv::Point>& optimumed);
bool InterSect(const cv::Mat& m1, const cv::Mat& m2);
bool WhetherUpdate(const cv::Mat& src, const cv::Mat& update);