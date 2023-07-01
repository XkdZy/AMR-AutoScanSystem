#pragma once

#include <iostream>
using namespace std;
#include <string>
#include "./Include/CameraDefine.h"
#include "./Include/CameraApi.h"

bool init_mindvision();
/// <summary>
/// 保存高清视觉图片
/// </summary>
/// <param name="fileName">路径不加文件格式</param>
void save_mindvision(char* fileName);
/// <summary>
/// 设置高清视觉曝光时间
/// </summary>
/// <param name="exptime">曝光时间单位ms</param>
void set_mindivision_exposuretime(double exptime);
