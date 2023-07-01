#include "header.h"

BYTE* m_pFrameBuffer;
BOOL m_bMonoCamera;
CMagDevice device;
CMagService service;
bool ConfigCamera::init_thermogroup()
{
	if (!device.IsInitialized())
	{
		printf("-------�����ʼ��ʧ�ܣ�-------\r\n");
		return FALSE;
	}

	/*printf("���������IP��ַ: ");
	scanf("%15s", sIp);*/
	char sIp[16] = "192.168.1.160";
	printf("\r\n��ʼ�������...\r\n");
	if (!device.LinkCamera(sIp))
	{
		printf("-------�������ʧ�ܣ�-------\r\n");
		return FALSE;
	}

	const struct_CamInfo* pCamInfo = device.GetCamInfo();
	if (!pCamInfo)
	{
		printf("-------��ȡ�������ʧ�ܣ�-------\r\n");
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
		printf("-------��������ʧ�ܣ�-------\r\n");
		device.DisLinkCamera();
		return FALSE;
	}

	service.EnableAutoReConnect(TRUE);//������������

	printf("�������������...\r\n\r\n");
	return TRUE;
}

string _path_;
char fullPath[MAX_PATH];//���ڷ���·��
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
	swprintf(strPath, 100, pwchar); // 20220409�޸�swprintf����

	device.Lock();
	if (device.SaveBMP(0, strPath))
	{
		if (isPrintf == true)
		{
			printf("����BMP�ɹ���\r\n");
		}
	}
	else
	{
		if (isPrintf == true)
		{
			printf("����BMPʧ�ܣ�");
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
			//pIrData�б������BMPͼ���ʵ������
		}
	}
}

/*******************************************************************
* ����			�� ʵʱ��ʾ�����������ͼ�������߳��е��øú���
* lpParamter	:
*******************************************************************/
void ConfigCamera::realtimeCameraDisplay(void*)
{
	Mat cameraImageFin;
	Mat cameraImage;
	Mat thermoImage;
	/*�����Լ���   ��ʾÿһ֡��ʱ������30ms*/
	while (1)
	{
		// ����ͼƬ
		thermoImage = imread(".\\imgs\\thermal\\thermoTempImg.bmp");
		cv::imshow("image video", thermoImage); //��ʾ��ǰ֡
		waitKey(30); //��ʱ30ms 

		//cameraImage = imread(".\\imgs\\thermal\\cameraTempImg.bmp");
		//thermoImage = imread(".\\imgs\\thermal\\thermoTempImg.bmp");
		// 
		//if ((cameraImage.size().height != 0) && (cameraImage.size().width != 0)
		//	&& (thermoImage.size().height != 0) && (thermoImage.size().width != 0))
		//{
		//	/*ƴ�����������ͼ��һ����ʾ������ʾ*/
		//	vector<Mat> imgs(2);
		//	imgs[0] = thermoImage;
		//	// �Ӿ�ͼƬ���ؽϴ������ͼƬ���ز�ͬ���轫�����ر�Ϊ��ͬ
		//	Size thermoSize = thermoImage.size();
		//	cv::resize(cameraImage, cameraImageFin, thermoSize);
		//	imgs[1] = cameraImageFin;
		//	Mat combineImage = ConfigCamera::combineImages(imgs, 2, 1, true);

		//	//putTextZH(combineImage, "¼����...", Point(910, 230), Scalar(255, 255, 255), 35, "����");
		//	//line(combineImage, Point(120, 320), Point(550, 320), CV_RGB(0, 255, 0), 2);	//��ɫ����
		//	namedWindow("image video");
		//	cv::imshow("image video", combineImage); //��ʾ��ǰ֡
		//	waitKey(30); //��ʱ30ms  
		//}
	}
}


/*******************************************************************
* ����	�� ��������Ǻ͹�ҵ�����ͼ��һ��MAT�����ϣ�����һ����ʾ
* imgs	:  ��Ҫ��ʾ��ͼ����
* col	:  ��ʾ������
* row	:  ��ʾ������
* hasMargin	:�Ƿ����ñ߿�
*******************************************************************/
Mat ConfigCamera::combineImages(vector<Mat> imgs, int col, int row, bool hasMargin)
{
	int imgAmount = imgs.size();//��ȡ��Ҫ��ʾ��ͼ������
	int width = imgs[0].cols;//������Ĭ����Ҫ��ʾ��ͼ���С��ͬ
	int height = imgs[0].rows;//��ȡͼ����
	int newWidth, newHeight;//��ͼ����

	if (!hasMargin)
	{
		newWidth = col * imgs[0].cols;//�ޱ߿���ͼ���/��=ԭͼ���/��*��/����
		newHeight = row * imgs[0].rows;
	}
	else
	{
		newWidth = (col + 1) * 10 + col * width;//�б߿�Ҫ���ϱ߿�ĳߴ磬�������ñ߿�Ϊ20px
		newHeight = (row + 1) * 100 + row * height;
	}

	Mat newImage(newHeight, newWidth, CV_8UC3, Scalar(255, 255, 255));//��ʾ�����趨�ߴ���µĴ�ͼ��ɫ���λ��ͨ�������Ϊ��ɫ

	int x, y, imgCount;//x�кţ�y�кţ�imgCountͼƬ���
	if (hasMargin) {//�б߿�
		imgCount = 0;
		x = 0; y = 0;
		while (imgCount < imgAmount) {
			Mat imageROI = newImage(Rect(x * width + (x + 1) * 10, y * height + (y + 1) * 100, width, height));//��������Ȥ����
			imgs[imgCount].copyTo(imageROI);//��ͼ���Ƶ���ͼ��
			imgCount++;
			if (x == (col - 1)) {
				x = 0;
				y++;
			}
			else {
				x++;
			}//�ƶ����кŵ���һ��λ��
		}
	}
	else
	{//�ޱ߿�
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
	return newImage;//�����µ����ͼ��
}
