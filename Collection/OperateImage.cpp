#include "header.h"

const wchar_t* PythonEnvPath = L"F:/QMDownload/Anaconda/Anaconda3/envs/Env_VILD";

bool PixelInBBox(const cv::Rect& rec, const cv::Point& pixel) {
	if (rec.x > pixel.x) {
		return false;
	}
	if (rec.y > pixel.y) {
		return false;
	}
	if ((rec.x + rec.width) < pixel.x) {
		return false;
	}
	if ((rec.y + rec.height) < pixel.y) {
		return false;
	}
	return true;
}

template <typename T>
vector<int> sort_indexes(const vector<T>& v)
{
	// 初始化索引
	vector<int> idx(v.size());
	for (int i = 0; i != idx.size(); ++i) idx[i] = i;

	// 根据vector的值对索引排序
	//sort(idx.begin(), idx.end()); // 默认升序排列
	sort(idx.begin(), idx.end(),
		[&v](int i1, int i2) -> bool { return v[i1] > v[i2]; }); //C++11 Lambda表达式
	/*
	* [caoture] (params) opt -> ret {body;};
	* 其中：capture是捕获列表；
	* params是参数表；
	* opt是函数选项；
	* ret是返回值类型；
	* body是函数体
	*/
	return idx;
}

bool refreshImgFlag = false;
// 定时器中断
void TimerInterpute(void) {
	refreshImgFlag = true;
}

void CollectImgAndInteraction(cameraImgInfo& cii) {
	static int timer10sCnt = 0; // 计数，时间10s，10s更新一次点云俯视图
	string gripper2BaseRealTimePose;
	//string colorRealTimeSavePath = "./imgs/tmp.jpg";
	//string depthRealTimeSavePath = "./imgs/tmp.png";
	string colorRealTimeSavePath = "./imgs/rgbd_frame/rgb/current_frame.jpg"; // 保存实时图片的路径
	string depthRealTimeSavePath = "./imgs/rgbd_frame/depth/current_frame.png";
	string saveRealTimePosFilePath = "./imgs/curr.txt";
	string opencvRealTimeDisplayWindowName = "real time";
	while (!exitChildThread) {
		if (refreshImgFlag) {
			timer10sCnt++;
			refreshImgFlag = false;
			//cout << "该刷新图片了" << endl;
			cv::Mat colorImageDup(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
			cv::Mat depthImageDup(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);
			cv::Mat depthImageRealTime, colorImageRealTime;
			// 保存实时RGBD图片并显示两个视图
			if (astraCameraD2C->GetStreamData(colorImageDup, depthImageDup) == CAMERA_STATUS_SUCCESS) {
				sp.ReadRobotArmPosString(sp, gripper2BaseRealTimePose); // 读取当前机械臂6D位姿
				//cout << "current pose:" << gripper2BaseRealTimePose << endl;
				// 将实时坐标写入到文件中
				IOFile iof(saveRealTimePosFilePath);
				// 清空文件内容，写入坐标
				iof.WriteString2File(gripper2BaseRealTimePose, 1); // 写入当前机械臂末端坐标
				flip(colorImageDup, colorImageRealTime, 1); // 深度相机图片镜像
				flip(depthImageDup, depthImageRealTime, 1);
				//cv::imshow("a", colorImageRealTime);
				//cv::waitKey(0);
				imwrite(colorRealTimeSavePath, colorImageRealTime); // 保存当前RGB、深度
				imwrite(depthRealTimeSavePath, depthImageRealTime); // PNG16
				UpdateRealTimePosInfo(colorImageRealTime, depthImageRealTime, gripper2BaseRealTimePose, cii);
				cv::Mat upDateDisplayImg;
				//cout << "vVerticalImageRealTime[2]:" << vVerticalImageRealTime[2].cols << endl;
				cv::Mat verticalImageRealTimeDisplay = colorImageRealTime; // 取垂直为当前视角
				CombinateTwoImg(verticalImageRealTimeDisplay, colorImageRealTime, upDateDisplayImg);
				cv::imshow(opencvRealTimeDisplayWindowName, upDateDisplayImg); // 更新显示
				cv::waitKey(5);
				// 捕获鼠标点击信息
				cv::setMouseCallback(opencvRealTimeDisplayWindowName, 
					[](int event, int x, int y, int flags, void* para) {
						switch (event)
						{
						case EVENT_LBUTTONDOWN:
							mouseClickCol = x;
							mouseClickRow = y;
							mouseClickCol = mouseClickCol > IMAGE_WIDTH ? mouseClickCol - IMAGE_WIDTH : mouseClickCol;
							exitChildThread = true;
							cout << "鼠标点击位置在：" << mouseClickRow << " " << mouseClickCol << endl;
						}
					},
					reinterpret_cast<void*>(&upDateDisplayImg)
				);
			}
		}
		//if ((timer10sCnt >= (1000. / TIMER_TIMING_TIME)) && upDateVerticalVaild) {
		//	timer10sCnt = 0;
		//	// 10s到了，还原一次3D，更新一次点云俯视图
		//	// 子线程用于采集实时RGBD图片（暂时不用）
		//	HANDLE handle = (HANDLE)_beginthread(ReconstructRealTime, 0, NULL);
		//}
	}

	cv::destroyWindow(opencvRealTimeDisplayWindowName);

	return;
}

void CombinateTwoImg(const cv::Mat& mVertical, const cv::Mat& mFront, cv::Mat& dst) {
	int verticalViewRows = mVertical.rows;
	int verticalViewCols = mVertical.cols;
	int currentViewRows = mFront.rows;
	int currentViewCols = mFront.cols;
	dst = cv::Mat::zeros(max(verticalViewRows, currentViewRows), currentViewCols + verticalViewCols, CV_8UC3);
	//cout << "verticalView:" << verticalViewRows << "  " << verticalViewCols << endl;
	//cout << "currentView:" << currentViewRows << "  " << currentViewCols << endl;
	mVertical.copyTo(dst(cv::Rect(0, 0, verticalViewCols, verticalViewRows)));
	mFront.copyTo(dst(cv::Rect(verticalViewCols, 0, currentViewCols, currentViewRows)));

	return;
}

// 更新实时图片的位置信息
void UpdateRealTimePosInfo(const cv::Mat& rgb, const cv::Mat& depth, string imgPose, cameraImgInfo& cii) {
	// 字符串转Mat
	cv::Mat str2MatData;
	AnalysisString26D(imgPose, str2MatData);
	// _Gripper2Base
	cv::Mat gripper2base = attitudeVectorToMatrix(str2MatData.row(0), false, "xyz");
	cii._Gripper2Base = gripper2base.clone();
	//cout << "image pose:" << imgPose << endl;
	//cout << "cii._Gripper2Base read:" << cii._Gripper2Base << endl;
	// _CameraPose
	Mat cameraPose = gripper2base * H_Camera2Gripper;
	cii._CameraPose = cameraPose.clone();
	// _CameraPosition
	Mat cameraPosition = cameraPose * (Mat_<double>(4, 1) << 0, 0, 0, 1);
	cii._CameraPosition = Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0));
	// _CameraPostureEig、_CameraPostureEig
	Mat cameraPosture = cameraPose(cv::Range(0, 3), cv::Range(0, 3)); // 前3*3
	cii._CameraPostureEig = RotationMatrixToEulerAngles(cameraPosture);
	cii._CameraPostureMat = cameraPosture.clone();
	// _RgbImg
	cii._RgbImg = rgb.clone();
	// _DepthImg
	cii._DepthImg = depth.clone();
	// string
	cii._poseStr = imgPose;

	return;
}


bool CallPythonScriptInference(bool vildOrbts) {
	//指定python.exe位置  python的环境。后来在迁移环境时，将include libs放在工程目录下，指定工程目录就可以
	Py_SetPythonHome(PythonEnvPath);
	Py_Initialize();
	// 防止import tensorflow错误
	wchar_t* argv[] = { LPWSTR(L" ") }; // 正确代码;
	PySys_SetArgv(1, argv);
	// 导入VildInference.py模块 
	PyObject* pModule = PyImport_ImportModule("VildInference");//这里是要调用的文件名	   
	if (!pModule)
	{
		cout << "Can't find file (VildInference)" << endl;
		return false;
	}
	else
	{
		PyObject* mainPo = PyObject_GetAttrString(pModule, "VildInference");
		if (!mainPo || !PyCallable_Check(mainPo))
		{
			cout << "Can't find funftion (VildInference)" << endl;
			return false;
		}
		else
		{
			cout << "Get function (VildInference) succeed." << endl;
			//PyObject* pArg = Py_BuildValue("(s)", dataPath.c_str());  //一个字符串参数
			PyObject* pArg = NULL;  //一个字符串参数
			PyEval_CallObject(mainPo, pArg);
		}
	}

	Py_DECREF(pModule);

	Py_Finalize();

	return true;
}

vector<int> CompareImageArea(const vector<cv::Mat>& vImg) {
	vector<int> vIndex;
	vector<int> vArea;

	cv::Mat depthImg(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, cv::Scalar(1));
	cv::Mat mask;
	// 计算每张图片白色区域面积
	for (int i = 0; i < vImg.size(); i++) {
		int validCnt = CountImageWhiteArea(vImg[i], depthImg, 0, mask);
		vArea.push_back(validCnt);
	}
	// 对每张面积进行降序排序
	vIndex = sort_indexes(vArea);

# if 1

	cout << "*****************************************************************图像置信度比较" << endl;
	cout << "vArea：";
	for (int i = 0; i < vImg.size(); i++) {
		cout << vArea[i] << " ";
	}
	cout << endl;
	cout << "vIndex：";
	for (int i = 0; i < vImg.size(); i++) {
		cout << vIndex[i] << " ";
	}
	cout << endl;
	cout << "*******************************************************************************" << endl;

#endif

	return vIndex;
}

int CountImageWhiteArea(const cv::Mat& img, const cv::Mat& depth, const double moreThanVal, cv::Mat& mask) {
	//imshow("src", img);
	//waitKey(0);
	mask = cv::Mat(img.rows, img.cols, CV_8U, cv::Scalar(0));
	cv::Mat binary;
	if (img.channels() != 1) {
		// 二值化
		Mat gray;
		cvtColor(img, gray, COLOR_BGR2GRAY);
		threshold(gray, binary, 128, 255, THRESH_BINARY);
	}
	else
		binary = img;
	//cout << "row:" << img.rows << "	col:" << img.cols << endl;
	int validCnt = 0;
	for (int row = 0; row < img.rows; row++) {
		for (int col = 0; col < img.cols; col++) {
			//cout << (int)vImg[i].at<uchar>(row, col) << " ";
			if (binary.at<uchar>(row, col) == 255 && depth.at<double>(row, col) > moreThanVal) {
				// 前景 
				validCnt++;
				mask.at<uchar>(row, col) = 255;
			}
			else
				mask.at<uchar>(row, col) = 128;
		}
	}

	return validCnt;
}

double CountImageAgvDepth(const cv::Mat& depth, cv::Mat& mask) {
	int validCnt = 0;
	int totalVal = 0;
	for (int row = 0; row < depth.rows; row++) {
		for (int col = 0; col < depth.cols; col++) {
			//cout << (int)vImg[i].at<uchar>(row, col) << " ";
			if (mask.at<uchar>(row, col) == 255 && depth.at<UINT16>(row, col) != 0) {
				// 前景 
				validCnt++;
				totalVal += depth.at<UINT16>(row, col);
			}
		}
	}

	return double(totalVal) / validCnt;
}

// 更新实时图片的位置信息
void IntegrateImgInfo(const cv::Mat& rgb, const cv::Mat& depth, string pose, int type, cameraImgInfo& cameraInfo) {
	// 字符串转Mat
	cv::Mat str2MatData(1, 6, CV_64F, 0.0);
	AnalysisString26D(pose, str2MatData);
	//cout << str2MatData.at<double>(0, 0) << "	" << str2MatData.at<double>(0, 1) << "	" << str2MatData.at<double>(0, 2)
	//	<< "	" << str2MatData.at<double>(0, 3) << "	" << str2MatData.at<double>(0, 4) << "	" << str2MatData.at<double>(0, 5) << endl;
	// _Gripper2Base
	cameraInfo._Gripper2Base = attitudeVectorToMatrix(str2MatData.row(0), false, "xyz");
	// _CameraPose
	if (type == 0) cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Camera2Gripper;
	else if (type == 1) cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Camera2Gripper_HD;
	else cameraInfo._CameraPose = cameraInfo._Gripper2Base * H_Thermal2Gripper;
	// _CameraPosition
	Mat cameraPosition = cameraInfo._CameraPose * (Mat_<double>(4, 1) << 0, 0, 0, 1);
	cameraInfo._CameraPosition = Eigen::Vector3d(cameraPosition.at<double>(0, 0), cameraPosition.at<double>(1, 0), cameraPosition.at<double>(2, 0));
	// _CameraPostureEig、_CameraPostureEig
	Mat cameraPosture = cameraInfo._CameraPose(cv::Range(0, 3), cv::Range(0, 3)); // 前3*3
	cameraInfo._CameraPostureEig = RotationMatrixToEulerAngles(cameraPosture);
	cameraInfo._CameraPostureMat = cameraPosture.clone();
	// _RgbImg
	cameraInfo._RgbImg = rgb.clone();
	// 打印均值方差
	if (type == 1) {
		cv::Mat gray;
		cv::cvtColor(cameraInfo._RgbImg, gray, cv::COLOR_BGR2GRAY);
		cv::Mat meanval, varval;
		cv::meanStdDev(gray, meanval, varval);
		//cout << "mean:" << meanval.t() << "	var:" << varval.t() << endl;
	}
	// _DepthImg
	cameraInfo._DepthImg = depth.clone();
	// string
	cameraInfo._poseStr = pose;

	return;
}

void IntegrateVildBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype) {
	// type
	bboxInfo._imgType = imgtype;
	// src
	bboxInfo._src = cv::imread(bboxPth + "src.jpg");
	cv::Mat srcGray;
	cv::cvtColor(bboxInfo._src, srcGray, cv::COLOR_BGR2GRAY);
	// VILD bbox位置
	IOFile iof(bboxPth + "box.txt", 0);
	vector<vector<float>> vvBboxInfo;
	iof.ReadFile2FloatArr(vvBboxInfo);
	//cout << "bbox size:" << vvBboxInfo.size() << "	" << bboxPth + "bbox.txt" << endl;
	vector<string> vBboxMaskInfo(vvBboxInfo.size()); // bbox mask图像
	vector<cv::Mat> vImg(vvBboxInfo.size()); // 图片掩膜
	vector<double> vMaskArea(vvBboxInfo.size()); // 掩膜面积
	vector<Eigen::Vector4i> v4DBoxPos(vvBboxInfo.size(), Eigen::Vector4i(0, 0, 0, 0)); // bbox坐标
	vector<cv::Rect> v4DBoxPosRect(vvBboxInfo.size(), cv::Rect(0, 0, 0, 0)); // bbox坐标
	// 加载BBox对应的掩模信息
	for (int bbox = 0; bbox < vvBboxInfo.size(); bbox++) {
		v4DBoxPos[bbox] = Eigen::Vector4i((int)vvBboxInfo[bbox][0], (int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][2], (int)vvBboxInfo[bbox][3]);
		v4DBoxPosRect[bbox] = cv::Rect((int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][0], ((int)vvBboxInfo[bbox][3] - (int)vvBboxInfo[bbox][1]), ((int)vvBboxInfo[bbox][2] - (int)vvBboxInfo[bbox][0]));
		vBboxMaskInfo[bbox] = bboxPth + to_string(bbox) + ".jpg";
		cv::Mat srcMask = imread(vBboxMaskInfo[bbox]);
		cv::Mat dst(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
		Mat gray;
		cvtColor(srcMask, gray, COLOR_BGR2GRAY);
		threshold(gray, dst, 128, 255, THRESH_BINARY);
		if (imgtype == 0) {
			// RGBD才保存掩膜
			vImg[bbox] = dst.clone();
		}
	}
	bboxInfo._imgPath = vBboxMaskInfo;
	bboxInfo._bboxRect = v4DBoxPosRect;
	bboxInfo._bboxAB = v4DBoxPos;
	bboxInfo._maskArea = vMaskArea;
	bboxInfo._img = vImg;
	bboxInfo._selectmask = vImg[0];
}

void IntegrateBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype, int inftype) {
//	// BTS
//	if (inftype != 0) {
//		char buff[10];
//		sprintf(buff, "%04d", inftype - 1);
//		cv::Mat srcBgr = cv::imread(bboxPth + buff + ".png");
//		cv::Mat gray, binary;
//		cv::cvtColor(srcBgr, gray, cv::COLOR_BGR2GRAY);
//		cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
//		bboxInfo._selectmask = binary;
//
//		return;
//	}
//
//	// type
//	bboxInfo._imgType = imgtype;
//	// src
//	bboxInfo._src = cv::imread(bboxPth + "src.jpg");
//	cv::Mat srcGray;
//	cv::cvtColor(bboxInfo._src, srcGray, cv::COLOR_BGR2GRAY);
//	// VILD bbox位置
//	IOFile iof(bboxPth + "box.txt", 0);
//	vector<vector<float>> vvBboxInfo;
//	iof.ReadFile2FloatArr(vvBboxInfo);
//	//cout << "bbox size:" << vvBboxInfo.size() << "	" << bboxPth + "bbox.txt" << endl;
//	vector<string> vBboxMaskInfo(vvBboxInfo.size()); // bbox mask图像
//	vector<cv::Mat> vImg(vvBboxInfo.size()); // 图片掩膜
//	vector<double> vMaskArea(vvBboxInfo.size()); // 掩膜面积
//	vector<Eigen::Vector4i> v4DBoxPos(vvBboxInfo.size(), Eigen::Vector4i(0, 0, 0, 0)); // bbox坐标
//	vector<cv::Rect> v4DBoxPosRect(vvBboxInfo.size(), cv::Rect(0, 0, 0, 0)); // bbox坐标
//	bboxInfo._maskContour = vector<vector<Point>>(vvBboxInfo.size()); // bbox轮廓
//	bboxInfo._totalContours = vector<vector<Point>>(vvBboxInfo.size()); // bbox轮廓
//	// 加载BBox对应的掩模信息
//	for (int bbox = 0; bbox < vvBboxInfo.size(); bbox++) {
//		//cout << vvBboxInfo[bbox][0] << " " << vvBboxInfo[bbox][1] << " " << vvBboxInfo[bbox][2] << " " << vvBboxInfo[bbox][3] << endl;
//		v4DBoxPos[bbox] = Eigen::Vector4i((int)vvBboxInfo[bbox][0], (int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][2], (int)vvBboxInfo[bbox][3]);
//		v4DBoxPosRect[bbox] = cv::Rect((int)vvBboxInfo[bbox][1], (int)vvBboxInfo[bbox][0], ((int)vvBboxInfo[bbox][3] - (int)vvBboxInfo[bbox][1]), ((int)vvBboxInfo[bbox][2] - (int)vvBboxInfo[bbox][0]));
//		char bboxName[12];
//		sprintf_s(bboxName, "%d.jpg", bbox); // ".jpg"
//		string maskImagePath = bboxPth + string(bboxName);
//		vBboxMaskInfo[bbox] = maskImagePath; // mask
//		cv::Mat srcMask = imread(maskImagePath);
//		cv::Mat dst(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
//		Mat gray;
//		cvtColor(srcMask, gray, COLOR_BGR2GRAY);
//		threshold(gray, dst, 128, 255, THRESH_BINARY);
//		if (imgtype == 0) {
//			// RGBD才保存掩膜
//			vImg[bbox] = dst.clone();
//		}
//		// 轮廓面积
//		vector<vector<cv::Point> > contours;
//		vector<Vec4i> hierarchy;
//		cv::findContours(dst, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//		vector<vector<cv::Point>>::iterator itr = contours.begin();
//		double area = 0.;
//		double areaMax = 0.;
//		int optimumIdx = 0;
//		while (itr != contours.end()) {
//			double currArea = cv::contourArea(*itr);
//			if (currArea > areaMax) optimumIdx = itr - contours.begin();
//			area += currArea;
//			areaMax = max(currArea, areaMax);
//			itr++;
//		}
//		bboxInfo._maskContour[bbox] = vector<cv::Point>(contours[optimumIdx]);
//		vMaskArea[bbox] = area;
//		//cv::namedWindow("bbox", cv::WINDOW_NORMAL);
//		//cv::imshow("bbox", vMaskSrc[bbox]);
//		//cv::waitKey(0);
//	}
//	bboxInfo._imgPath = vBboxMaskInfo;
//	bboxInfo._bboxRect = v4DBoxPosRect;
//	bboxInfo._bboxAB = v4DBoxPos;
//	bboxInfo._maskArea = vMaskArea;
//	bboxInfo._img = vImg;
//	//bboxInfo._selectmask = cv::Mat(bboxInfo._src.rows, bboxInfo._src.cols, CV_8U, cv::Scalar(0));
//	bboxInfo._selectmask = vImg[0];
//	
//	// image轮廓
//	if (imgtype == 0) {
//		cv::Mat gray, binary, blur;
//		vector<vector<cv::Point>>contours;//轮廓
//		vector<cv::Vec4i>hierachy;//存放轮廓结构变量
//		cv::cvtColor(bboxInfo._src, gray, COLOR_BGR2GRAY);
//		cv::GaussianBlur(gray, gray, Size(9, 9), 2, 2);//平滑滤波
//		cv::threshold(gray, binary, 170, 255, THRESH_BINARY | THRESH_OTSU);//自适应二值化
//		cv::findContours(binary, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
//		//for (int i = 0; i < bboxInfo._totalContours.size(); i++) {
//		//	for (int k = 0; k < gray.rows; k++) {
//		//		contours[i].push_back(cv::Point(0, k));
//		//		contours[i].push_back(cv::Point(gray.rows - 1, k));
//		//	}
//		//	for (int j = 0; j < gray.cols; j++) {
//		//		contours[i].push_back(cv::Point(j, 0));
//		//		contours[i].push_back(cv::Point(j, gray.cols - 1));
//		//	}
//		//}
//		bboxInfo._totalContours = contours;
//
//		static int  num = 0;
//		cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
//		cv::drawContours(mask, contours, -1, Scalar(0, 0, 255), 2, 8);
//		cv::imwrite(dataPath + "temp/test/" + to_string(num) + ".jpg", mask);
//		//cv::imwrite(dataPath + "temp/test/" + to_string(num) + "src.jpg", bboxInfo._src);
//		//for (int i = 0; i < contours.size(); ++i) {
//		//	bboxInfo._totalContours[i] = contours[i];
//		//}
//
//		//bboxInfo._contour = vector<vector<Point>>(vvBboxInfo.size());
//		//bboxInfo._contourErr = vector<double>(vvBboxInfo.size());
//		//for (int i = 0; i < vvBboxInfo.size(); ++i) {
//		//	vector<vector<cv::Point>> vContours(1);
//		//	vContours[0] = bboxInfo._maskContour[i];
//		//	vector<cv::Point> srcOptimumContours, optimumed;
//		//	double err = FindOptimumContours(bboxInfo._totalContours, vContours, srcOptimumContours, optimumed);
//		//	//cout << "optimumed:" << optimumed.size() << "	" << bboxInfo._maskContour[i].size() << endl;
//		//	bboxInfo._contourErr[i] = err;
//		//	bboxInfo._contour[i] = vector<cv::Point>(bboxInfo._totalContours[i]);
//		//	//bboxInfo._contour[i] = vector<cv::Point>(optimumed);
//		//	//if (err < 10.) {
//		//	//	//cout << err << "	" << num << "	" << i << endl;
//		//	//	// 当误差较小时，替代foundcontour寻找的
//		//	//	bboxInfo._totalContours[i] = optimumed;
//		//	//}
//		//	vector<vector<cv::Point>> vDrawContours(1);
//		//	vDrawContours[0] = optimumed;
//		//	cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
//		//	cv::drawContours(mask, vDrawContours, 0, Scalar(0, 0, 255), 2, 8);
//		//	cv::drawContours(mask, vContours, 0, Scalar(0, 0, 255), 2, 8);
//		//	cv::imwrite(dataPath + "temp/test/" + to_string(num) + "_" + to_string(i) + ".jpg", mask);
//		//}
//		for (int i = 0; i < bboxInfo._totalContours.size(); i++) {
//			cv::Mat mask(bboxInfo._src.rows, bboxInfo._src.cols, CV_8UC3, Scalar(0, 0, 0));
//			cv::drawContours(mask, bboxInfo._totalContours, i, Scalar(0, 0, 255), 2, 8);
//			cv::imwrite(dataPath + "temp/test/" + to_string(num) + "_" + to_string(i) + "optimum.jpg", mask);
//		}
//		num++;
//	}
}

Eigen::Vector3d ConvertPixel2Camera(const int& u, const int& v, const ushort& d, const int& cameraType) {
	//分辨率缩放，这里假设标定时的分辨率分RESOULTION_X，RESOULTION_Y
	// 相机内参
	cv::Mat cameraInt;
	if (cameraType == 0) cameraInt = CameraMatrix;
	else if (cameraType == 1) cameraInt = CameraMatrix_HD;
	else cameraInt = ThermalMatrix;
	double fdx = cameraInt.at<double>(0, 0);
	double fdy = cameraInt.at<double>(1, 1);
	double u0 = cameraInt.at<double>(0, 2);
	double v0 = cameraInt.at<double>(1, 2);
	//cout << "fdx:" << fdx << "	fdy:" << fdy << "	u0:" << u0 << "	v0:" << v0 << endl;
	if (fdx == 0 || fdy == 0) return Eigen::Vector3d{ 0.,0.,0. };
	// 反透视投影
	double tx = (u - u0) / fdx;
	double ty = (v - v0) / fdy;
	//cout << Eigen::Vector3d{ (double)d * tx, (double)d * ty, (double)d }.transpose() << endl;

	return Eigen::Vector3d{ (double)d * tx, (double)d * ty, (double)d };
}

double FindHHOptimumExposureTime() {
	//static int exptime = 0;
	double optimumTime = 0.;
	char hdPath[100];
	for (int i = 10; i < 50; i++) {
		set_mindivision_exposuretime((double)i);
		sprintf(hdPath, "%s%s%04d", dataPath, "recollect/exposuretime/", i);
		save_mindvision(hdPath); // 保存高清图
		sprintf(hdPath, "%s%s", hdPath, ".bmp");
		cv::Mat currMat = cv::imread(hdPath);
		cv::Mat gray;
		cv::cvtColor(currMat, gray, cv::COLOR_BGR2GRAY);
		cv::Mat meanval, varval;
		cv::meanStdDev(gray, meanval, varval);
		cout << "exposure time:" << i << "ms	mean:" << meanval.t() << "	var:" << varval.t() << endl;
	}
	return 0.;
}

vector<vector<Point>> FindContours(const cv::Mat& src, bool oneMax) {
	cv::Mat img = src.clone();	
	cv::Mat gray, binary, blur;
	vector<vector<Point>>contours;//轮廓
	vector<Vec4i>hierachy;//存放轮廓结构变量
	if (src.channels() != 1) {
		cvtColor(src, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(9, 9), 2, 2);//平滑滤波
		threshold(gray, binary, 170, 255, THRESH_BINARY | THRESH_OTSU);//自适应二值化
	}
	else {
		// binary只保留最长外轮廓
		binary = src.clone();
	}
	static int num = 0;
	//轮廓发现与绘制
	// cv::findContours(binary, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	cv::findContours(binary, contours, hierachy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0) return contours;
	if (!oneMax) {
		//绘制轮廓 所有轮廓
		//cout << "轮廓数：" << contours.size() << endl;
		for (int i = 0; i < contours.size(); ++i) {
			int currArea = cv::contourArea(contours[i]);
			double currLen = cv::arcLength(contours[i], true);
			//bool close = cv::isContourConvex(contours[i]);
			//cout << "轮廓面积：" << currArea << "	长度：" << currLen << "	封闭：" << close << endl;
			//if (currArea < 1000 || currArea > 600 * 400) continue;
			//if (currLen < 1000) continue;
			if (currArea < 100 || currArea > 600 * 400) continue;
			cv::Mat mask(src.rows, src.cols, CV_8UC3, Scalar(0, 0, 0));
			cv::drawContours(mask, contours, i, Scalar(0, 0, 255), 2, 8);
			cv::imwrite(dataPath + "temp/contours/" + to_string(num) + "_" + to_string(i) + ".jpg", mask);
		}
	}
	else {
		// 绘制一条最长轮廓
		//cout << "轮廓数：" << contours.size() << endl;
		int maxIdx = 0;
		double maxLen = 0;
		for (int i = 0; i < contours.size(); ++i) {
			double currLen = cv::arcLength(contours[i], true);
			maxIdx = currLen > maxLen ? i : maxIdx;
			maxLen = max(maxLen, currLen);
		}
		vector<Point> optimumContours = contours[maxIdx];
		cv::Mat mask(src.rows, src.cols, CV_8UC3, Scalar(0, 0, 0));
		cv::drawContours(mask, contours, maxIdx, Scalar(0, 0, 255), 2, 8);
		cv::imwrite(dataPath + "temp/contours/" + to_string(num) + ".jpg", mask);
		contours.clear();
		contours.emplace_back(optimumContours);
		// 绘制一条最长轮廓
		//cout << "轮廓数：" << contours.size() << endl;
	}
	num++;
	return contours;
}

double FindOptimumContours(const vector<vector<Point>>& src, const vector<vector<Point>>& mask, vector<cv::Point>& optimumSrc, vector<cv::Point>& optimumed) {
	//if (src.size() == 0 || mask.size() != 1) return INT32_MAX;
	if (src.size() == 0) return INT32_MAX;
	double TotalErr = 0.;
	for (int j = 0; j < mask.size(); j++) {
		//int optimumIdx = 0;
		double minErr = INT32_MAX;
		vector<cv::Point> vResult;
		for (int i = 0; i < src.size(); i++) {
			// 计算所有轮廓与目标轮廓误差
			vector<cv::Point> currContours = src[i];
			if (currContours.size() < mask[j].size() * 0.5) continue;
			double totalSubErr = 0.;
			vector<cv::Point> vTmp;
			for (int pIdx = 0; pIdx < mask[j].size(); pIdx++) {
				// 遍历目标轮廓所有点，与参考轮廓找最近
				double minSubErr = INT32_MAX;
				int minSubIdx = 0;
				cv::Point dstPoint = mask[j][pIdx];
				//cout << "contours point:" << dstPoint.y << "	" << dstPoint.x << endl;
				//cout << dstPoint.x << "	" << dstPoint.y << endl;
				for (int pIdxSrc = 0; pIdxSrc < currContours.size(); pIdxSrc++) {
					cv::Point srcPoint = currContours[pIdxSrc];
					if (norm(srcPoint - dstPoint) < minSubErr) minSubIdx = pIdxSrc;
					minSubErr = min(norm(srcPoint - dstPoint), minSubErr);
					//cout << norm(srcPoint - dstPoint) << endl;
				}
				// 图像边界也是轮廓
				if (minSubErr > dstPoint.x && dstPoint.x < 2) { minSubErr = dstPoint.x; currContours[minSubIdx] = cv::Point{ 0 , dstPoint.y }; }
				else if (minSubErr > 639 - dstPoint.x && dstPoint.x > 637) { minSubErr = 639 - dstPoint.x; currContours[minSubIdx] = cv::Point{ 639 , dstPoint.y }; }
				else if (minSubErr > dstPoint.y && dstPoint.y < 2) { minSubErr = dstPoint.y; currContours[minSubIdx] = cv::Point{ dstPoint.x , 0 };}
				else if (minSubErr > 479 - dstPoint.y && dstPoint.y > 477) { minSubErr = 479 - dstPoint.y; currContours[minSubIdx] = cv::Point{ dstPoint.x ,479 }; }
				else vTmp.emplace_back(currContours[minSubIdx]);
				totalSubErr += minSubErr;
			}
			if (totalSubErr < minErr) {
				//optimumIdx = i;
				vResult = vTmp;
			}
			minErr = min(minErr, totalSubErr);
		}
		TotalErr += minErr;
		optimumed.insert(optimumed.begin(), vResult.begin(), vResult.end());
	}
	optimumSrc = src[0];
	return TotalErr / (double)mask[0].size();
}

//double FindOptimumContours(const vector<vector<Point>>& src, const vector<vector<Point>>& mask, vector<Point>& optimumSrc, vector<Point>& optimumed) {
//	if (src.size() == 0 || mask.size() != 1) return INT32_MAX;
//	int optimumIdx = 0;
//	double minErr = INT32_MAX;
//	//vector<cv::Point> vResult;
//	for (int i = 0; i < src.size(); i++) {
//		// 计算所有轮廓与目标轮廓误差
//		vector<cv::Point> currContours = src[i];
//		if (currContours.size() < mask[0].size() * 0.5) continue;
//		double totalSubErr = 0.;
//		vector<cv::Point> vTmp;
//		set<cv::Point> sTmp;
//		for (int pIdx = 0; pIdx < mask[0].size(); pIdx++) {
//			// 遍历目标轮廓所有点，与参考轮廓找最近
//			double minSubErr = INT32_MAX;
//			int minSubIdx = 0;
//			cv::Point dstPoint = mask[0][pIdx];
//			//cout << dstPoint.x << "	" << dstPoint.y << endl;
//			for (int pIdxSrc = 0; pIdxSrc < currContours.size(); pIdxSrc++) {
//				cv::Point srcPoint = currContours[pIdxSrc];
//				if (norm(srcPoint - dstPoint) < minSubErr) minSubIdx = pIdxSrc;
//				minSubErr = min(norm(srcPoint - dstPoint), minSubErr);
//				//cout << norm(srcPoint - dstPoint) << endl;
//			}
//			
//			//if (minSubErr > dstPoint.x || minSubErr > abs(640 - dstPoint.x)
//			//	|| minSubErr > dstPoint.y || minSubErr > abs(480 - dstPoint.y)) {
//			//	// 没有该点计算误差
//			//	minSubErr = min((double)dstPoint.x, minSubErr);
//			//	minSubErr = min((double)abs(640 - dstPoint.x), minSubErr);
//			//	minSubErr = min((double)dstPoint.y, minSubErr);
//			//	minSubErr = min((double)abs(480 - dstPoint.y), minSubErr);
//			//	totalSubErr += minSubErr;
//			//	vTmp.emplace_back(dstPoint);
//			//}
//			//else {
//				totalSubErr += minSubErr;
//				vTmp.emplace_back(currContours[minSubIdx]);
//			//}
//			//cout << pIdx << "	" << minSubIdx << "	" << minSubErr << endl;
//			//sTmp.insert(currContours[minSubIdx]);
//		}
//		if (totalSubErr < minErr) {
//			optimumIdx = i;
//			//vResult = vTmp;
//			optimumed = vTmp;
//		}
//		minErr = min(minErr, totalSubErr);
//	}
//	optimumSrc = src[optimumIdx];
//	//cout << "err:" << minErr / mask[0].size() << endl;
//	//return vResult;
//	return minErr / (double)mask[0].size();
//}

bool InterSect(const cv::Mat& m1, const cv::Mat& m2) {
	if (m1.rows != m2.rows || m1.cols != m2.cols) return false;
	if (m1.channels() != m2.channels() || m1.channels() != 1) return false;
	bool haveVildPixel = false;
	for (int row = 0; row < m1.rows; row++) {
		for (int col = 0; col < m1.cols; col++) {
			if (m1.at<uchar>(row, col) == 255 && m1.at<uchar>(row, col) == m2.at<uchar>(row, col)) return true;
			if (!haveVildPixel && m1.at<uchar>(row, col) == 255) haveVildPixel = true;
		}
	}
	return haveVildPixel ? false : true;
}

bool WhetherUpdate(const cv::Mat& src, const cv::Mat& update) {
	cv::Mat mask;
	cv::Mat depth(src.rows, src.cols, CV_64F, Scalar(0.));
	int num1 = CountImageWhiteArea(src, depth, -1., mask);
	int num2 = CountImageWhiteArea(update, depth, -1., mask);
	return (num2 > 0.5 * num1 ? true : false);
}

