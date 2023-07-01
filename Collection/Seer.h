#pragma once
#define _CRT_SECURE_NO_WARNINGS

#ifndef WIN32_LEAN_AND_MEAN
#include WIN32_LEAN_AND_MEAN
#endif

#include <iostream>
#include <cstdint>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "cJSON.h"
#include <string>
#include <winsock2.h>  


#include<stdlib.h>
#include <fstream>
#include<vector>
#include <time.h>

using namespace std;

#define SEERCONTROLIP "192.168.3.38"

extern SOCKET m_SockClient10;//端口19210  DO输出  抱闸打开
extern SOCKET m_SockClient04;//端口19204  查询
extern SOCKET m_SockClient06;//端口19206  导航

/// <summary>
/// 
/// </summary>
/// <param name="ms"></param>
void delay_ms(int ms, int x);
/// <summary>
/// 初始化网络环境
/// </summary>
void InitInternet();
/// <summary>
/// 连接仙工只能控制器
/// </summary>
/// <param name="port">机器人状态：19204；控制：19205；导航 API	19206；配置：19207；其他：19210；推送：19301</param>
/// <param name="client">客户端套接字</param>
/// <returns>连接是否成功true：成功</returns>
bool ConnectSeer(const u_short port, SOCKET& client);
/// <summary>
/// 查询机器人信息指令：转Seer数据帧
/// </summary>
/// <param name="m_number">指令编号</param>
/// <param name="m_type">指令类型</param>
/// <returns>Seer数据帧</returns>
char* RobotState2SeerFrame(const uint16_t m_number, const uint16_t m_type);
/// <summary>
/// 机器人导航指令：Json转Seer数据帧
/// </summary>
/// <param name="m_number">指令编号</param>
/// <param name="m_type">指令类型</param>
/// <param name="json">json数据帧</param>
/// <returns>Seer数据帧</returns>
char* RobotNavigationJSON2SeerFrame(const uint16_t m_number, const uint16_t m_type, const char* json);

/// <summary>
/// 世界坐标系下导航到指定位置
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="theta"></param>
/// <param name="client"></param>
void AMRRobotNavigation(const double x, const double y, const double theta, SOCKET& client);

/// <summary>
/// 世界坐标系下导航到指定位置  不需要指定方向
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="client"></param>
void AMRRobotNavigation(const double x, const double y, SOCKET& client);

/// <summary>
/// 自由导航
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="theta"></param>
/// <param name="client"></param>
void AMRgofree(const double x, const double y, const double theta, SOCKET& client);

#pragma region 导航指令

/// <summary>
/// AMR平动
/// </summary>
/// <param name="dist">距离</param>
/// <param name="vx">x方向速度</param>
/// <param name="vy">y方向速度</param>
/// <returns>Seer数据帧</returns>
void AMRLibration(const double dist, const double vx, const double vy, SOCKET& client, const bool mode);
/// <summary>
/// AMR转动
/// </summary>
/// <param name="angle">弧度（2Π=360度）</param>
/// <param name="vw">速度（+：顺时针   -：逆时针）</param>
/// <param name="client">导航套接字</param>
void AMRRotation(const double angle, const double vw, SOCKET& client);
/// <summary>
/// 停止导航
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* AMRstop_Navigation(const SOCKET& client);
#pragma endregion










#pragma region 读取以太网数据，解析数据

/// <summary>
/// 
/// </summary>
/// <param name="client"></param>
/// <returns></returns>

char* ReadAMRRespond(const SOCKET& client);
int ReadAMRRespond(const SOCKET& client, char*& data);
/// <summary>
/// 读取AMR位置
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRPostion(const SOCKET& client);
/// <summary>
/// 激光雷达数据读取
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRLaser_res(const SOCKET& client);
int ReadAMRLaser_res(const SOCKET& client, char*& data, bool multi);
/// <summary>
/// 导航情况读取
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRnavigation(const SOCKET& client);
/// <summary>
/// 
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRInfo(const SOCKET& client);
int ReadAMRInfo(const SOCKET& client, char*& data);

char* ReadAMblocked(const SOCKET& client);

int AMRblocked_status(SOCKET& client);

#pragma endregion

#pragma region 打开抱闸
/// <summary>
/// 打开抱闸指令
/// </summary>
/// <param name="id">DO口</param>
/// <param name="status">状态</param>
/// <param name="client"></param>
void AMRUnlock(const double id, const bool status, SOCKET& client);

#pragma endregion

#pragma region 将json转为正常格式

/// <summary>
/// 导航状态读取
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
int AMRnavigation_status(SOCKET& client);

/// <summary>
/// 读取agv目前位置
/// </summary>
/// <param name="ss"></param>   x：    y：
/// <param name="client"></param>
/// <returns></returns>
float AMRnow_position(const string ss,SOCKET& client);

#pragma endregion

void AMRStartCreateMap(int slam_tpye, bool real_time, int screen_width, int screen_height);
void AMREndCreateMap();
void AMRQueryScanMapStatus();
void AMRQueryToatlMap();

struct AMRLocalInfo {
	double _x;
	double _y;
	double _angle;
};
void RequestAMRLocal(AMRLocalInfo& amrli);
bool WhetherAMRMove2Goal(const SOCKET& client, AMRLocalInfo& goal, AMRLocalInfo& last, int which);
// 重新扫描地图
void ReScanMap();