#include "header.h"

// -------------------- start of :  mindvision  -------------------- 
CameraHandle m_hCamera;
BYTE* m_pFrameBuffer;

int SetCameraResolution(int hCamera, int offsetx, int offsety, int width, int height) {
	tSdkImageResolution sRoiResolution = { 0 };

	// 设置成0xff表示自定义分辨率，设置成0到N表示选择预设分辨率
	sRoiResolution.iIndex = 0xff;

	// iWidthFOV表示相机的视场宽度，iWidth表示相机实际输出宽度
	// 大部分情况下iWidthFOV=iWidth。有些特殊的分辨率模式如BIN2X2：iWidthFOV=2*iWidth，表示视场是实际输出宽度的2倍

	sRoiResolution.iWidth = width;
	sRoiResolution.iWidthFOV = width * 2;

	// 高度，参考上面宽度的说明
	sRoiResolution.iHeight = height;
	sRoiResolution.iHeightFOV = height * 2;

	// 视场偏移
	sRoiResolution.iHOffsetFOV = offsetx;
	sRoiResolution.iVOffsetFOV = offsety;

	// ISP软件缩放宽高，都为0则表示不缩放
	sRoiResolution.iWidthZoomSw = 0;
	sRoiResolution.iHeightZoomSw = 0;

	// BIN SKIP 模式设置（需要相机硬件支持）
	// sRoiResolution.uBinAverageMode 设置bin AverageMode 
	sRoiResolution.uBinAverageMode = 2;
	sRoiResolution.uBinSumMode = 0;
	sRoiResolution.uResampleMask = 0;
	sRoiResolution.uSkipMode = 0;

	return CameraSetImageResolution(hCamera, &sRoiResolution);
}

bool init_mindvision() {
	tSdkCameraDevInfo sCameraList[10];
	INT iCameraNums;
	CameraSdkStatus status;
	tSdkCameraCapbility sCameraInfo;
	int FrameBufferSize;

	iCameraNums = 10;

	if (CameraEnumerateDevice(sCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
	{
		printf("No camera was found! \n");
		return FALSE;
	}

	//In this demo ,we just init the first camera.
	if ((status = CameraInit(&sCameraList[0], -1, -1, &m_hCamera)) != CAMERA_STATUS_SUCCESS)
	{
		printf("Failed to init the camera! Error code is %d. \n", status);
		return FALSE;
	}

	CameraSetOutPutIOMode(m_hCamera, 3, IOMODE_TRIG_INPUT);
	CameraSetOutPutIOMode(m_hCamera, 4, IOMODE_TRIG_INPUT);

	CameraSetTriggerMode(m_hCamera, 0);//EXTERNAL_TRIGGER

	//Get properties description for this camera.
	CameraGetCapability(m_hCamera, &sCameraInfo);

	CameraSetAeState(m_hCamera, 0);
	//CameraSetExposureTime(m_hCamera, 20. * 1000); //Ori:10*1000
	CameraSetAeState(m_hCamera, true); // 自动曝光

	CameraSetAnalogGainX(m_hCamera, 15);
#pragma region MyRegion

	// 设置分辨率2048*1500
	tSdkImageResolution pImageResolution;// = { 1 };
	pImageResolution.iIndex = 0x0;
	//pImageResolution.iIndex = 0xff;
	pImageResolution.uBinSumMode = 0;
	pImageResolution.uBinAverageMode = 0;
	pImageResolution.uSkipMode = 0;
	pImageResolution.uResampleMask = 0;

	//pImageResolution.iHOffsetFOV = 4096;
	//pImageResolution.iVOffsetFOV = 3000;
	pImageResolution.iHOffsetFOV = 0;
	pImageResolution.iVOffsetFOV = 0;
	pImageResolution.iWidthFOV = 4096; // 4096
	pImageResolution.iHeightFOV = 3000; // 3000
	pImageResolution.iWidth = 4096; // 4096
	pImageResolution.iHeight = 3000; // 3000
	pImageResolution.iWidthZoomHd = 0;
	pImageResolution.iHeightZoomHd = 0;
	pImageResolution.iWidthZoomSw = 0;
	pImageResolution.iHeightZoomSw = 0;
	CameraSetImageResolution(m_hCamera, &pImageResolution);
	//SetCameraResolution(m_hCamera, 0, 0, 4096, 3000);

#pragma endregion

	CameraPlay(m_hCamera);

	m_pFrameBuffer = (BYTE*)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iHeightMax * 4, 16);

	assert(m_pFrameBuffer);

	return TRUE;
}

void save_mindvision(char* fileName)
{
	tSdkFrameHead 	sFrameInfo;
	BYTE* pbyBuffer;
	CameraSdkStatus status;
	tSdkFrameHead   m_sFrInfo;

	if (CameraGetImageBuffer(m_hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
		status = CameraImageProcess(m_hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo);

		CameraReleaseImageBuffer(m_hCamera, pbyBuffer);

		if (status == CAMERA_STATUS_SUCCESS /*&& pThis->m_bSaveFile == FALSE*/) {
			CameraImageOverlay(m_hCamera, m_pFrameBuffer, &sFrameInfo);
			CameraDisplayRGB24(m_hCamera, m_pFrameBuffer, &sFrameInfo);
			memcpy(&m_sFrInfo, &sFrameInfo, sizeof(tSdkFrameHead));

			//CameraSaveImage(m_hCamera, fileName, m_pFrameBuffer, &sFrameInfo, FILE_BMP, 100);
			//cout << fileName << endl;
			CameraSaveImage(m_hCamera, fileName, m_pFrameBuffer, &sFrameInfo, FILE_BMP_8BIT, 100);
		}
	}
}

void set_mindivision_exposuretime(double exptime) {
	CameraSetAeState(m_hCamera, false); // 关闭自动曝光
	CameraSetExposureTime(m_hCamera, exptime * 1000); //Ori:10*1000
	Sleep(10);
}