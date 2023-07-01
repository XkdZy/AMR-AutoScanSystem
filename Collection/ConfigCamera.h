#pragma once
#ifndef __CONFIGCAMERA_H
#define __CONFIGCAMERA_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

extern char dirPath[MAX_PATH];

static class ConfigCamera
{
public:
	static bool init_thermogroup();
	static void save_thermogroup(char* fileName, bool isPrintf=true);
	static void CALLBACK NewFrame(UINT, int, DWORD, DWORD, DWORD, void*);
	static void realtimeCameraDisplay(void*);
	static Mat combineImages(vector<Mat> imgs, int col, int row, bool hasMargin);

};

#endif // !__CONFIGCAMERA_H

