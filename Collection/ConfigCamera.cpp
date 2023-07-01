#include "header.h"

BYTE* m_pFrameBuffer;
BOOL m_bMonoCamera;
CMagDevice device;
CMagService service;
bool ConfigCamera::init_thermogroup()
{
	if (!device.IsInitialized())
	{
		printf("-------相机初始化失败！-------\r\n");
		return FALSE;
	}

	/*printf("请输入相机IP地址: ");
	scanf("%15s", sIp);*/
	char sIp[16] = "192.168.1.160";
	printf("\r\n开始连接相机...\r\n");
	if (!device.LinkCamera(sIp))
	{
		printf("-------连接相机失败！-------\r\n");
		return FALSE;
	}

	const struct_CamInfo* pCamInfo = device.GetCamInfo();
	if (!pCamInfo)
	{
		printf("-------获取相机参数失败！-------\r\n");
		return FALSE;
	}

	OutputPara paraOut;
	paraOut.dwFPAWidth = pCamInfo->intFPAWidth;
	paraOut.dwFPAHeight = pCamInfo->intFPAHeight;
	paraOut.dwBMPWidth = pCamInfo->intVideoWidth;
	paraOut.dwBMPHeight = pCamInfo->intVideoHeight;
	paraOut.dwColorBarWidth = 16;
	paraOut.dwColorBarHeight = pCamInfo->intVideoHeight;

	if (!device.StartProcessImage(&paraOut, ConfigCamera::NewFrame, STREAM_TEMPERATURE, &device))
	{
		printf("-------传输数据失败！-------\r\n");
		device.DisLinkCamera();
		return FALSE;
	}

	service.EnableAutoReConnect(TRUE);//开启断线重连

	printf("相机正常工作中...\r\n\r\n");
	return TRUE;
}

string _path_;
char fullPath[MAX_PATH];//用于返回路径
void ConfigCamera::save_thermogroup(char* fileName, bool isPrintf)
{
#define _TO_UNICODE(y) L##y
#define TO_UNICODE(x) _TO_UNICODE(x)

#define PRODUCT_NAME "Chrome"
#define PRODUCT_NAME_W TO_UNICODE(PRODUCT_NAME)

	std::string product_name = PRODUCT_NAME;
	std::wstring product_name_w = PRODUCT_NAME_W;
	wchar_t strPath[MAX_PATH];

	_bstr_t t = fileName;
	wchar_t* pwchar = (wchar_t*)t;
	swprintf(strPath, 100, pwchar); // 20220409修改swprintf报错

	device.Lock();
	if (device.SaveBMP(0, strPath))
	{
		if (isPrintf == true)
		{
			printf("保存BMP成功！\r\n");
		}
	}
	else
	{
		if (isPrintf == true)
		{
			printf("保存BMP失败！");
		}
	}
	device.Unlock();
}

void CALLBACK ConfigCamera::NewFrame(UINT intChannelIndex, int intCameraTemperature, DWORD dwFFCCounterdown,
	DWORD dwCamState, DWORD dwStreamType, void* pUserData)
{
	CMagDevice* pDevice = (CMagDevice*)pUserData;

	if (dwStreamType == STREAM_TEMPERATURE)//temperature
	{
		const UCHAR* pIrData;
		const BITMAPINFO* pIrInfo;
		if (pDevice->GetOutputBMPdata(&pIrData, &pIrInfo))
		{
			//pIrData中保存的是BMP图像的实际数据
		}
	}
}

/*******************************************************************
* 功能			： 实时显示相机和热像仪图像，在子线程中调用该函数
* lpParamter	:
*******************************************************************/
void ConfigCamera::realtimeCameraDisplay(void*)
{
	Mat cameraImageFin;
	Mat cameraImage;
	Mat thermoImage;
	/*经测试计算   显示每一帧的时间大概是30ms*/
	while (1)
	{
		// 更新图片
		thermoImage = imread(".\\imgs\\thermal\\thermoTempImg.bmp");
		cv::imshow("image video", thermoImage); //显示当前帧
		waitKey(30); //延时30ms 

		//cameraImage = imread(".\\imgs\\thermal\\cameraTempImg.bmp");
		//thermoImage = imread(".\\imgs\\thermal\\thermoTempImg.bmp");
		// 
		//if ((cameraImage.size().height != 0) && (cameraImage.size().width != 0)
		//	&& (thermoImage.size().height != 0) && (thermoImage.size().width != 0))
		//{
		//	/*拼接两个相机的图像到一个显示框中显示*/
		//	vector<Mat> imgs(2);
		//	imgs[0] = thermoImage;
		//	// 视觉图片像素较大与红外图片像素不同，需将其像素变为相同
		//	Size thermoSize = thermoImage.size();
		//	cv::resize(cameraImage, cameraImageFin, thermoSize);
		//	imgs[1] = cameraImageFin;
		//	Mat combineImage = ConfigCamera::combineImages(imgs, 2, 1, true);

		//	//putTextZH(combineImage, "录制中...", Point(910, 230), Scalar(255, 255, 255), 35, "隶书");
		//	//line(combineImage, Point(120, 320), Point(550, 320), CV_RGB(0, 255, 0), 2);	//绿色画框
		//	namedWindow("image video");
		//	cv::imshow("image video", combineImage); //显示当前帧
		//	waitKey(30); //延时30ms  
		//}
	}
}


/*******************************************************************
* 功能	： 组合热像仪和工业相机的图像到一个MAT数据上，用于一起显示
* imgs	:  需要显示的图像组
* col	:  显示的列数
* row	:  显示的行数
* hasMargin	:是否设置边框
*******************************************************************/
Mat ConfigCamera::combineImages(vector<Mat> imgs, int col, int row, bool hasMargin)
{
	int imgAmount = imgs.size();//获取需要显示的图像数量
	int width = imgs[0].cols;//本函数默认需要显示的图像大小相同
	int height = imgs[0].rows;//获取图像宽高
	int newWidth, newHeight;//新图像宽高

	if (!hasMargin)
	{
		newWidth = col * imgs[0].cols;//无边框，新图像宽/高=原图像宽/高*列/行数
		newHeight = row * imgs[0].rows;
	}
	else
	{
		newWidth = (col + 1) * 10 + col * width;//有边框，要将上边框的尺寸，这里设置边框为20px
		newHeight = (row + 1) * 100 + row * height;
	}

	Mat newImage(newHeight, newWidth, CV_8UC3, Scalar(255, 255, 255));//显示创建设定尺寸的新的大图像；色深八位三通道；填充为白色

	int x, y, imgCount;//x列号，y行号，imgCount图片序号
	if (hasMargin) {//有边框
		imgCount = 0;
		x = 0; y = 0;
		while (imgCount < imgAmount) {
			Mat imageROI = newImage(Rect(x * width + (x + 1) * 10, y * height + (y + 1) * 100, width, height));//创建感兴趣区域
			imgs[imgCount].copyTo(imageROI);//将图像复制到大图中
			imgCount++;
			if (x == (col - 1)) {
				x = 0;
				y++;
			}
			else {
				x++;
			}//移动行列号到下一个位置
		}
	}
	else
	{//无边框
		imgCount = 0;
		x = 0; y = 0;
		while (imgCount < imgAmount) {
			Mat imageROI = newImage(Rect(x * width, y * height, width, height));
			imgs[imgCount].copyTo(imageROI);
			imgCount++;
			if (x == (col - 1)) {
				x = 0;
				y++;
			}
			else {
				x++;
			}
		}
	}
	return newImage;//返回新的组合图像
}
