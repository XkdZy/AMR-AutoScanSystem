#include "header.h"

/// <summary>
/// 初始化点云信息，获得点云参数
/// </summary>
/// <param name="pcl">输入：点云</param>
/// <param name="pcli">输出：点云参数结构体</param>
void InitPointCloudInfo(const std::shared_ptr<open3d::geometry::PointCloud>& pcl, PointCloudInfo& pcli) {
	open3d::geometry::AxisAlignedBoundingBox axisBox = pcl->GetAxisAlignedBoundingBox(); // 点云包围框
	vector<Eigen::Vector3d> axisPoint = axisBox.GetBoxPoints(); // 获取包围盒坐标（0-1：长；0-2：宽；0-3：高）
	double pclLength = axisBox.GetExtent()[0];
	double pclWidth = axisBox.GetExtent()[1];
	double pclHeight = axisBox.GetExtent()[2];
	pcli._refRx = (axisPoint[0] - axisPoint[1]).normalized();
	pcli._refRy = (axisPoint[0] - axisPoint[2]).normalized();
	if (pclLength > pclWidth) {
		pcli._refRx = (axisPoint[0] - axisPoint[2]).normalized();
		pcli._refRy = (axisPoint[0] - axisPoint[1]).normalized();
	}
	//cout << pcli._refRx.transpose() << "	" << pcli._refRy.transpose() << endl;
	pcli._minBound = axisBox.GetMinBound();
	pcli._maxBound = axisBox.GetMaxBound();
	pcli._extent = axisBox.GetExtent();
	//点云参数确定：
	//    axisBox.GetExtent()：包围盒长、宽、高
	//    inlier->GetMinBound()：包围盒8个点中的第0个axisPoint[0]---P0
	//    inlier->GetMaxBound()：包围盒8个点中的第4个axisPoint[4]---P4
	return;
}

/// <summary>
/// 点法式：判断当前法向量的指向
/// </summary>
/// <param name="vPoint"></param>
/// <returns></returns>
bool OrientEstimate(Eigen::Vector3d& norm) {
	bool status = true;

	// 最大元素对应的符号
	Eigen::Vector3d pointDup = norm.cwiseAbs(); // 取绝对值
	Eigen::MatrixXd::Index maxRow, maxCol;
	int maxComponentInd = pointDup.maxCoeff(&maxRow, &maxCol); // 绝对值最大值对应索引
	if (norm[maxRow] > 0)
		return status;
	else
		return 1 - status;
}

/// <summary>
/// 斯密特：以相机光轴z方向为β1，使点云x、y参考方向与之正交
/// </summary>
/// <param name="xOrientRec">输入：点云x参考方向</param>
/// <param name="yOrientRec">输入：点云y参考方向</param>
/// <param name="zOrient">输入：相机光轴方向</param>
/// <param name="xOrient">输出：正交后的x方向</param>
/// <param name="yOrient">输出：正交后的y方向</param>
void CalcPointCloudXYOrient(Eigen::Vector3d xOrientRec, Eigen::Vector3d yOrientRec, Eigen::Vector3d zOrient,
	Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient) {
	// 姿态：当前点云包围盒，以包围盒的x，y两个方向作为相机的候选方向，先直接选y方向（可以选与n点积最小的方向为相机一个方向）
	// 将x方向（长的设备方向）斯密特正交化
	xOrient = (xOrientRec - xOrientRec.transpose() * zOrient * zOrient).normalized();
	bool ret = OrientEstimate(xOrient); // 使x方向是指向试件同一侧（根据向量最大分量的符号）
	////cout << "第" << i << "个点云返回值：" << ret << endl;
	//if (!ret) {
	//	xOrient = -1 * xOrient; // 使整个方向都指向试件外部
	//}
	// 将y方向斯密特正交化
	yOrient = (yOrientRec - yOrientRec.transpose() * xOrient * xOrient - yOrientRec.transpose() * zOrient * zOrient).normalized();

	return;
}

/// <summary>
/// 根据设备尺寸将点云分快
/// </summary>
/// <param name="pcl">输入：整体点云</param>
/// <returns>分块后结果</returns>
vector<std::shared_ptr<open3d::geometry::PointCloud>> PointCloudPartition(const std::shared_ptr<open3d::geometry::PointCloud>& pcl) {
	// 分块：根据设备尺寸，将点云分割成小块。
	// 每个小块通过plain segment进行细分成最多三块
	vector<std::shared_ptr<open3d::geometry::PointCloud>> vSubPointCloud;

	open3d::geometry::AxisAlignedBoundingBox axisBox = pcl->GetAxisAlignedBoundingBox(); // 点云包围框
	vector<Eigen::Vector3d> axisPoint = axisBox.GetBoxPoints(); // 获取包围盒坐标（0-1：长；0-2：宽；0-3：高）
	// pclLength不一定大于pclWidth，
	double pclLength = axisBox.GetExtent()[0];
	double pclWidth = axisBox.GetExtent()[1];
	double pclHeight = axisBox.GetExtent()[2];
	//// MinBound：对应最小坐标点；MaxBound：最大坐标点；GetExtent：对应长宽高
	//cout << "MinBound：" << axisBox.GetMinBound().transpose() << endl;
	//cout << "MaxBound：" << axisBox.GetMaxBound().transpose() << endl;
	//cout << "GetExtent:" << axisBox.GetExtent().transpose() << endl;
	//cout << (axisPoint[0] - axisPoint[1]).norm() << endl;
	//cout << (axisPoint[0] - axisPoint[2]).norm() << endl;
	//cout << (axisPoint[0] - axisPoint[3]).norm() << endl;

	// 点云短/设备长
	//int lengthScanTimes = ceil(min(pclLength, pclWidth) / DEVICE_LENGTH); // 点云短/设备长
	//int widthScanTimes = ceil(max(pclLength, pclWidth) / DEVICE_WIDTH); // 点云长/设备短
	int lengthScanTimes = floor(min(pclLength, pclWidth) / DEVICE_LENGTH); // 点云短/设备长
	int widthScanTimes = floor(max(pclLength, pclWidth) / DEVICE_WIDTH); // 点云长/设备短
	//cout << "lengthScanTimes:" << lengthScanTimes << "	widthScanTimes:" << widthScanTimes << endl;
	vector<std::shared_ptr<const open3d::geometry::Geometry>> vTotal; // 用于显示
	for (int col = 0; col < lengthScanTimes; col++) {
		for (int row = 0; row < widthScanTimes; row++) {
			// 计算包围盒minBound、maxBound
			double minBoundX = col * pclLength / lengthScanTimes;
			double minBoundY = row * pclWidth / widthScanTimes;
			double maxBoundX = (col + 1) * pclLength / lengthScanTimes;
			double maxBoundY = (row + 1) * pclWidth / widthScanTimes;
			if (pclLength > pclWidth) {
				// 长宽对调
				////cout << "length > width" << endl;
				minBoundX = row * pclLength / widthScanTimes;
				minBoundY = col * pclWidth / lengthScanTimes;
				maxBoundX = (row + 1) * pclLength / widthScanTimes;
				maxBoundY = (col + 1) * pclWidth / lengthScanTimes;
			}
			Eigen::Vector3d currMinBound = axisBox.GetMinBound() + Eigen::Vector3d(minBoundX, minBoundY, 0);
			Eigen::Vector3d currMaxBound = axisBox.GetMinBound() + Eigen::Vector3d(maxBoundX, maxBoundY, axisBox.GetMaxBound()[2] - axisBox.GetMinBound()[2]);
			open3d::geometry::AxisAlignedBoundingBox cropBox = open3d::geometry::AxisAlignedBoundingBox(currMinBound, currMaxBound);
			// 按bbox分割整体点云
			std::shared_ptr<open3d::geometry::PointCloud> croped_cloud = pcl->Crop(cropBox);
			shared_ptr<open3d::geometry::PointCloud> temp_pointcloud = croped_cloud;
			//open3d::visualization::DrawGeometries({ croped_cloud }, "croped_cloud");
			//vector<std::shared_ptr<open3d::geometry::Geometry>> vPlainSegment;
			vector<std::shared_ptr<open3d::geometry::PointCloud>> vPlainSegment;
			// 平面分割最多分成3分
			for (int itr = 0; itr < 3; itr++) {
				// 1、平面分割
				tuple< Eigen::Vector4d, vector<size_t>> retPlane = temp_pointcloud->SegmentPlane(12, 5, 100); // 点距离平面模型的最大距离
				Eigen::Vector4d planeFormula = std::get<0>(retPlane); // 平面方程：ax + by + cz + d = 0 
				vector<size_t> planeIndex = std::get<1>(retPlane); // 内点索引
				//std::cout << "planeFormula：" << planeFormula.transpose() << endl;
				// 2、平面
				shared_ptr<open3d::geometry::PointCloud> in_pointcloud = temp_pointcloud->SelectByIndex(planeIndex);
				open3d::geometry::AxisAlignedBoundingBox localPlaneSegment = in_pointcloud->GetAxisAlignedBoundingBox(); // 显示平面分割后的包围盒
				shared_ptr<open3d::geometry::PointCloud> outlier_pointcloud = temp_pointcloud->SelectByIndex(planeIndex, true);
				//      去除离群量：该离群点剔除方法是指。在一个点周围选择若干个点，计算它们距离的统计参数，
				//                  如果某个点偏离平均值超过stdio_ratio倍的方差则认为是离群点，并进行删除。
				shared_ptr<open3d::geometry::PointCloud> out_withoutOutlier = std::get<0>(outlier_pointcloud->RemoveStatisticalOutliers(30, 1)); //0返回正常值点云
				//open3d::visualization::DrawGeometries({ in_pointcloud }, "Segment Plane");
				vPlainSegment.emplace_back(in_pointcloud); // 平面分割结果
				// 3、1/n原点数>剩余点->停止
				if (croped_cloud->points_.size() > 6 * out_withoutOutlier->points_.size()) { // 取n=5
					break;
				}
				// <= 更新点云
				temp_pointcloud = out_withoutOutlier;
			}
			std::shared_ptr<open3d::geometry::PointCloud> currPointCloud = std::make_shared<open3d::geometry::PointCloud>();
			for (int i = 0; i < vPlainSegment.size(); i++) {
				*currPointCloud += *vPlainSegment[i];
			}
			vTotal.emplace_back(currPointCloud);
			vSubPointCloud.emplace_back(currPointCloud); // 保存当前平面
			//open3d::visualization::DrawGeometries({ currPointCloud }, "Segment Plane");
		}
	}
	//open3d::visualization::DrawGeometries({ vTotal }, "total");

	return vSubPointCloud;
}

/// <summary>
/// 基于当前红外测量场位置、姿态，更新其余信息
/// </summary>
/// <param name="tsi"></param>
void IntergateScanInfo(ThermalScanInfo& tsi) {
	cv::Mat cmaR = tsi._cameraPosetrue;
	cv::Mat camT = (Mat_<double>(3, 1) << tsi._cameraPosition[0], tsi._cameraPosition[1], tsi._cameraPosition[2]);
	// 将R、t化为其次形式
	cv::Mat hThermal2Base = R_T2RT(cmaR, camT);
	// 机械臂末端姿态求取：手眼标定矩阵B * 相机位姿(R,t)
	cv::Mat gripperRT = hThermal2Base * H_Thermal2Gripper.inv(); // (R,t) * H_Thermal2Gripper.inv()
	tsi._gripperPoseMat = gripperRT.clone();
	ConvertMat2String(tsi._gripperPoseMat, tsi._cmdStr); // gripper cmd
	tsi._gripperPosetrue = gripperRT({ 0,0,3,3 });
	Eigen::Vector3d eulerAngles = RotationMatrixToEulerAngles(tsi._gripperPosetrue); // 旋转矩阵转欧拉角
	//std::cout << "欧拉角为：" << eulerAngles.transpose() << endl;

	// 保存当前数据
	tsi._cameraRx = Eigen::Vector3d { cmaR.at<double>(0, 0), cmaR.at<double>(1, 0), cmaR.at<double>(2, 0) }; // Rx
	tsi._cameraRy = Eigen::Vector3d{ cmaR.at<double>(0, 1), cmaR.at<double>(1, 1), cmaR.at<double>(2, 1) }; ; // Ry
	tsi._cameraRz = Eigen::Vector3d{ cmaR.at<double>(0, 2), cmaR.at<double>(1, 2), cmaR.at<double>(2, 2) }; ; // Rz
	Eigen::Vector6d nowGripperPose; // 存放机械臂末端6D位姿
	nowGripperPose << gripperRT.at<double>(0, 3), gripperRT.at<double>(1, 3), gripperRT.at<double>(2, 3),
		eulerAngles[0], eulerAngles[1], eulerAngles[2];
	tsi._gripperPoseVec = nowGripperPose;

	return;
}

/// <summary>
/// 对分块后点云进行PCA求取法线
/// </summary>
/// <param name="pcli"></param>
/// <param name="vCropedCloudForPCA"></param>
/// <param name="_scanInfo"></param>
/// <returns></returns>
vector<std::shared_ptr<open3d::geometry::LineSet>> PointCloudPartitionPCA(
	const PointCloudInfo& pcli,
	vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedCloudForPCA,
	vector<ThermalScanInfo>& _scanInfo) {
	vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient;

	// 分块点云PCA求法线
	for (int i = 0; i < vCropedCloudForPCA.size(); i++) {
		//auto currPcl = vCropedCloudForPCA[i];
		auto center_point = vCropedCloudForPCA[i]->GetCenter();
		//创建KDTree 用于K近邻搜索           
		geometry::KDTreeFlann pointTree(*vCropedCloudForPCA[i]);
		//体素中心使用KNN近邻搜索  寻找原点云中的最近点
		std::vector<int> indices;
		std::vector<double> distance;
		pointTree.SearchKNN(center_point, 1, indices, distance);
		indices.clear();
		distance.clear();
		//体素中心使用KNN近邻搜索  寻找附近最近kNum个点云数据
		int kNum = vCropedCloudForPCA[i]->points_.size();
		//int kNum = vCropedCloudForPCA[i]->points_.size() / 2;
		//int kNum = 1000;
		pointTree.SearchKNN(center_point, kNum, indices, distance);
		//添加K近邻点到矩阵中
		Eigen::MatrixXd knearestPointData(kNum, 3);
		auto currPcl = std::make_shared<open3d::geometry::PointCloud>();
		for (int pdNum = 0; pdNum < kNum; pdNum++) {
			currPcl->points_.emplace_back(vCropedCloudForPCA[i]->points_[indices[pdNum]]);
			currPcl->colors_.emplace_back(vCropedCloudForPCA[i]->colors_[indices[pdNum]]);
		}

		// 当前点云对应的是第i行第j列扫描时的点云
		// 姿态R
		Eigen::Vector3d minEigVector(0, 0, 0), nowCenter(0, 0, 0);
		CalcPointCloudNormPCA(currPcl, minEigVector, nowCenter); // 分块PCA求相机z方向
		//CalcPointCloudNormPCA(vCropedCloudForPCA[i], minEigVector, nowCenter); // 分块PCA求相机z方向
		//std::cout << "当前点云中心位置：" << nowCenter.transpose() << endl;
		// 判断当前法线方向是指向试件外部（根据法线x向量最大分量的符号）
		bool ret = OrientEstimate(minEigVector);
		//cout << "第" << i << "个点云返回值：" << ret << endl;
		if (!ret) {
			minEigVector = -1 * minEigVector; // 使整个方向都指向试件外部
		}
		minEigVector = -1 * minEigVector; // z轴反向
		Eigen::Vector3d xCamOrient, yCamOrient;
		CalcPointCloudXYOrient(pcli._refRx, pcli._refRy, minEigVector, xCamOrient, yCamOrient); // 斯密特正交化求相机x、y方向            
		cv::Mat camR = (Mat_<double>(3, 3) << xCamOrient[0], yCamOrient[0], minEigVector[0], // xyz
			xCamOrient[1], yCamOrient[1], minEigVector[1],
			xCamOrient[2], yCamOrient[2], minEigVector[2]
			);
		ThermalScanInfo tsi;
		tsi._cameraPosetrue = camR.clone();
		// 位置t
		Eigen::Vector3d camPosition(nowCenter - minEigVector * (PLAIN2CAMERA + REMAIN_PLAIN2CAMERA)); // 确定热像仪光心 = 相机光心（法线反）方向 * （热像仪尺寸 + 预留长度）
		tsi._cameraPosition = camPosition; // 存放相机光心3D位置
		//std::cout << "当前相机光心坐标：" << camPosition.transpose() << endl;
		IntergateScanInfo(tsi);
		_scanInfo.emplace_back(tsi);
		// 获取设备3D轮廓
		std::shared_ptr<open3d::geometry::LineSet> device_pointcloud = DrawDeviceLinePointCloud(camPosition, xCamOrient, yCamOrient, minEigVector);
		// 显示当前点云的法线
		vector<Eigen::Vector3d> vLinePoint;
		vLinePoint.emplace_back(camPosition); // 需要画的直线所在点
		vLinePoint.emplace_back(camPosition + minEigVector * 50);
		vLinePoint.emplace_back(camPosition + xCamOrient * 30);
		vLinePoint.emplace_back(camPosition + yCamOrient * 20);
		vector<Eigen::Vector2i> vLineIndex; // 直线点在容器中对应的索引
		vLineIndex.emplace_back(Eigen::Vector2i(0, 1));
		vLineIndex.emplace_back(Eigen::Vector2i(0, 2));
		vLineIndex.emplace_back(Eigen::Vector2i(0, 3));
		std::shared_ptr<open3d::geometry::LineSet> lineSet_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vLinePoint, vLineIndex));
		vNormOrient.emplace_back(lineSet_cloud);
		//open3d::visualization::DrawGeometries({ pcl_ptr, device_pointcloud, vCropedCloudForPCA[i] , lineSet_cloud, /*nowBox_cloud*/ }, "CloudWithDir");
	}

	return vNormOrient;
}

void CalcPointCloudNormPCA(std::shared_ptr<open3d::geometry::PointCloud>& pointCloud, Eigen::Vector3d& norm, Eigen::Vector3d& center) {
	auto sampled_cloud = pointCloud->VoxelDownSample(10);//对点云进行下采样
	auto inlier = std::get<0>(sampled_cloud->RemoveStatisticalOutliers(30, 1.0)); //0返回正常值点云
	vector<Eigen::Vector3d> vNowPoints = inlier->points_; // vCropedCloud[i]
	Eigen::Matrix3Xd inputX(3, vNowPoints.size()); // 输入数据 
	Eigen::Matrix3d convX; // 协方差矩阵
	for (int i = 0; i < vNowPoints.size(); i++) {
		// 读取数据
		inputX(0, i) = vNowPoints[i][0];
		inputX(1, i) = vNowPoints[i][1];
		inputX(2, i) = vNowPoints[i][2];
	}
	//cout << "行：" << inputX.rows() << "列：" << inputX.cols() << endl;
	// PCA：去均值、求协方差矩阵、特征分解、选择最小特征值（法线方向）
	// 01：取均值x~=1/n*sum(xi)
	Eigen::MatrixXd meanVal = inputX.rowwise().mean(); //计算每一维度均值rowwise().mean：每一行求均值
	Eigen::Vector3d meanColVal = meanVal;
	inputX.colwise() -= meanColVal; //样本均值化为0
	//cout << "meanVal.cols()：" << meanVal.cols() << "meanVal.rows()：" << meanVal.rows() << endl;
	// 02协方差矩阵：convX = X*X.T
	convX = inputX * inputX.adjoint(); // 求协方差矩阵
	convX = convX / (inputX.rows() - 1);
	//// SVD分解
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(convX, Eigen::ComputeThinU | Eigen::ComputeThinV); 
	//Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
	//Eigen::Matrix3d S = U.inverse() * convX * V.transpose().inverse();
	//cout << U << endl;
	//cout << S << endl;
	//cout << V << endl;
	//Eigen::Matrix3d eigVector = V.transpose(); // 每一列是对应的特征向量
	//Eigen::Vector3d minEigVector = eigVector.col(0);
	// 03：特征分解
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(convX); // 特征分解
	Eigen::Matrix3d eigVector = eig.eigenvectors(); // 每一列是对应的特征向量
	Eigen::Vector3d eigValue = eig.eigenvalues();
	//cout << "特征向量：" << eigVector << endl;
	//cout << "特征值：" << eigValue.transpose() << endl;
	// 04：法线：特征值最小对应的特征向量
	Eigen::Vector3d minEigVector = eigVector.col(0);
	//Eigen::Vector3d secEigVector = eigVector.col(1);
	//Eigen::Vector3d maxEigVector = eigVector.col(2);
	//cout << "最小特征值对应特征向量维为：" << minEigVector.transpose() << endl;

	norm = minEigVector;
	center = pointCloud->GetCenter();// 当前点云的形心
}

/// <summary>
/// 点云旋转回原始位置的角度，沿z轴逆时针为正
/// </summary>
/// <param name="rotateTheta"></param>
/// <returns></returns>
cv::Mat RotatePclMat(double rotateTheta) {
	return (cv::Mat_<double>(4, 4) <<
		cos(rotateTheta), -sin(rotateTheta), 0, 0,
		sin(rotateTheta), cos(rotateTheta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);
}