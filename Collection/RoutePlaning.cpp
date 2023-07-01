#include "header.h"

vertex startVertex;
vertex goalVertex;
vertex* allVertexArr;
vertex* expandedVertexArr;
Voxel2PclInfo v2pInfo;
vector<int> vCurrentTrajectory;
vector<vertex> vCurrentTrajectoryVertex;
vector<Eigen::Vector3d> vAGVMovePosition; // AGV移动坐标（AGV坐标系下）
vector<Eigen::Vector3d> vArmMovePosition; // 机械臂移动坐标（AGV初始位置时的机械臂坐标系下）

vector<int> vTest;

int CurrAGVMoveIdx; // 当前AGV移动点位索引
Eigen::Vector3d objectCenterPos; // 粗定位目标中心坐标

void FromObjPos2CamObservePos(const Eigen::Vector3d& camPos, const Eigen::Vector3d& objPos, Eigen::Vector3i camDisPos) {

	return;
}

void GetSearchRegion(const Eigen::Vector3i& start, const Eigen::Vector3i& goal, SearchVoxelRegion& svr) {
	// 设备占用体素块大小
	int occupNum = DEVICE_OCCUPYVOXEL;
	cout << "设备占有体素为：" << occupNum << endl;
	// 根据设备大小确定选用的体素块区域：有效体素区域 = abs(start - goal) + 2*occupNum
	int minX = min(start[0], goal[0]) - occupNum;
	int minY = min(start[1], goal[1]) - occupNum;
	int minZ = min(start[2], goal[2]) - occupNum;
	int height = abs(goal[2] - start[2]) + 2 * occupNum;
	int width = abs(goal[1] - start[1]) + 2 * occupNum;
	int length = abs(goal[0] - start[0]) + 2 * occupNum;
	// 限幅
	minX = minX > 0 ? minX : 0;
	minY = minY > 0 ? minY : 0;
	minZ = minZ > 0 ? minZ : 0;
	// 赋值
	svr.v = Eigen::Vector3i(minX, minY, minZ);
	svr.height = height;
	svr.width = width;
	svr.length = length;

	return;
}

void VertexNumFromSearchRegion(const SearchVoxelRegion& svr, const Eigen::Vector3i& start, const Eigen::Vector3i& goal, vertex& startVertex, vertex& goalVertex) {
	int xNorm = svr.length;
	int yNorm = svr.width;
	int zNorm = svr.height;
	cout << "xNorm：" << xNorm << "	yNorm：" << yNorm << "	zNorm：" << zNorm << endl;
	// 初始化定点编号为0，体素块在原点（minX、minY、minZ）
	// 顶点数组，vector 申请不了太长内存
	allVertexArr = (vertex*)malloc(xNorm * yNorm * zNorm * sizeof(vertex{ false, 0, svr.v, Eigen::Vector3d(0.0, 0.0, 0.0),(int*)malloc(sizeof(int) * 6) }));
	// Minxyz（原点）
	for (int z = 0; z < zNorm; z++) {
		for (int y = 0; y < yNorm; y++) {
			for (int x = 0; x < xNorm; x++) {
				// 相对坐标
				int voxelNum = (x)+xNorm * (y)+xNorm * yNorm * (z); // 体素编号，以原点开始计算：x，y，z顺序
				//cout << "z:" << z << "	y:" << y << "	x:" << x << "	voxelNum:" << voxelNum << endl;
				vertex v;
				v.num = voxelNum; // 编号
				v.v = svr.v + Eigen::Vector3i(x, y, z); // 体素坐标
				// 体素编号对应的实际点云位置 ???????????????????这可能有点问题
				v.vPcl = v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(v.v[0] + 0.9, v.v[1] - 0.1, v.v[2] + 1.6); // 根据体素顶点计算近似中心位置xyz
				//v.vPcl = v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(v.v[0] + 0.5, v.v[1] + 0.5, v.v[2] + 0.5); // 根据体素顶点计算近似中心位置xyz
				// v2pInfo.minPoint + VOXEL_SIZE * Eigen::Vector3d(nowVertex.v[0] - 0.5, nowVertex.v[1] - 0.5, nowVertex.v[2] - 0.5); // 根据体素顶点计算近似中心位置xyz
				auto itr = pcl_voxel->voxels_.find(v.v); // 占用情况
				if (itr != pcl_voxel->voxels_.end()) {
					v.occupy = true;
				}
				else {
					v.occupy = false;
				}
				v.vVertexNum = (int*)malloc(sizeof(int) * 6); // 相邻体素
				memset(v.vVertexNum, -1, sizeof(int) * 6);
				//v.vVertexNum = vector<int>(6, -1);
				// 6个相邻体素是否在搜索范围内
				if (x + 1 < xNorm) {
					int num = (x + 1) + xNorm * (y)+xNorm * yNorm * (z); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[0] = num;
				}
				if (x - 1 >= 0) {
					int num = (x - 1) + xNorm * (y)+xNorm * yNorm * (z); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[1] = num;
				}
				if (y + 1 < yNorm) {
					int num = (x)+xNorm * (y + 1) + xNorm * yNorm * (z); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[2] = num;
				}
				if (y - 1 >= 0) {
					int num = (x)+xNorm * (y - 1) + xNorm * yNorm * (z); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[3] = num;
				}
				if (z + 1 < zNorm) {
					int num = (x)+xNorm * (y)+xNorm * yNorm * (z + 1); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[4] = num;
				}
				if (z - 1 >= 0) {
					int num = (x)+xNorm * (y)+xNorm * yNorm * (z - 1); // 体素编号，以原点开始计算：x，y，z顺序
					v.vVertexNum[5] = num;
				}
				allVertexArr[voxelNum] = v;
			}
		}
	}
	startVertex = allVertexArr[(start[0] - svr.v[0]) + (start[1] - svr.v[1]) * xNorm + (start[2] - svr.v[2]) * xNorm * yNorm];
	cout << "---------------------startVertex:" << startVertex.v.transpose() << "	" << startVertex.vPcl.transpose() << endl;
	goalVertex = allVertexArr[(goal[0] - svr.v[0]) + (goal[1] - svr.v[1]) * xNorm + (goal[2] - svr.v[2]) * xNorm * yNorm];
	//cout << endl << "起始体素信息：" << endl;
	//cout << "体素编号：" << startVertex.num << endl;
	//cout << "体素位置：" << startVertex.v.transpose() << endl;
	//for (int j = 0; j < 6; j++) {
	//	cout << startVertex.vVertexNum[j] << " ";
	//}
	//cout << endl << "目标体素信息：" << endl;
	//cout << "体素编号：" << goalVertex.num << endl;
	//cout << "体素位置：" << goalVertex.v.transpose() << endl;
	//for (int j = 0; j < 6; j++) {
	//	cout << goalVertex.vVertexNum[j] << " ";
	//}

	return;
}

void FillCircleBasePoint(Eigen::Vector3i voxel, vertex* expandedVertex, SearchVoxelRegion& svr) {
	int xNorm = svr.length;
	int yNorm = svr.width;
	int zNorm = svr.height;
	// 设备占用体素块大小
	int occupNum = DEVICE_OCCUPYVOXEL;
	for (int i = -occupNum; i < occupNum; i++) {
		for (int j = -occupNum; j < occupNum; j++) {
			for (int k = -occupNum; k < occupNum; k++) {
				double distance = (Eigen::Vector3i(i, j, k) - voxel).norm();
				if (distance > DEVICE_OCCUPYVOXEL + 1) {
					continue;
				}
				Eigen::Vector3i nowVoxel = voxel + Eigen::Vector3i(i, j, k) + svr.v;
				// 限幅
				nowVoxel[0] = nowVoxel[0] > svr.v[0] + svr.length - 1 ? svr.v[0] + svr.length - 1 : nowVoxel[0];
				nowVoxel[1] = nowVoxel[1] > svr.v[1] + svr.width - 1 ? svr.v[1] + svr.width - 1 : nowVoxel[1];
				nowVoxel[2] = nowVoxel[2] > svr.v[2] + svr.height - 1 ? svr.v[2] + svr.height - 1 : nowVoxel[2];
				nowVoxel[0] = nowVoxel[0] < svr.v[0] ? svr.v[0]: nowVoxel[0];
				nowVoxel[1] = nowVoxel[1] < svr.v[1] ? svr.v[1] : nowVoxel[1];
				nowVoxel[2] = nowVoxel[2] < svr.v[2] ? svr.v[2] : nowVoxel[2];
				nowVoxel = nowVoxel - svr.v;
				// 相对坐标
				int voxelNum = (nowVoxel[0])+xNorm * (nowVoxel[1])+xNorm * yNorm * (nowVoxel[2]); // 体素编号，以原点开始计算：x，y，z顺序
				expandedVertex[voxelNum].occupy = true;
			}
		}
	}
}

void ExpandBarrierRegion(SearchVoxelRegion& svr, vertex* expandedVertex) {
	// 遍历体素块
	cout << endl << endl << endl << endl;
	int cnt = 0;
	for (int k = 0; k < svr.height; k++) {
		for (int j = 0; j < svr.width; j++) {
			for (int i = 0; i < svr.length; i++) {
				int voxelNum = (i)+svr.length * (j)+svr.length * svr.width * (k); // 体素编号，以原点开始计算：x，y，z顺序
				//cout << voxelNum << "	";
				expandedVertex[voxelNum] = allVertexArr[voxelNum];
				if (allVertexArr[voxelNum].occupy) {
					FillCircleBasePoint(Eigen::Vector3i(i, j, k), expandedVertex, svr);
					cnt++;
				}
			}
		}
	}
	cout << "总共有：" << pcl_voxel->GetVoxels().size() << "个体素，其中被占用的体素有：" << cnt << "个" << endl;
}

bool BreadthFirst(const int len, const vertex& start, const vertex& goal, vector<int>& bfsLine) {
	cout << "len:" << len << endl;
	vertex v{ start };
	bool* vertexUse = (bool*)malloc(sizeof(bool) * len); // 所有顶点是否已被搜索
	int* vertexSort = (int*)malloc(sizeof(int) * len); // 广度优先搜索结果
	memset(vertexUse, false, sizeof(bool) * len);
	memset(vertexSort, 0, sizeof(int) * len);
	int nowVertexIndex = 0; // 搜索结果
	int lastVertex = 0; // 前一时刻顶点索引
	vertexUse[v.num] = true;
	vertexSort[nowVertexIndex] = v.num; // 第一个顶点体素编号
	nowVertexIndex++;
	bool findFlag = false;
	int totalLen = 1;
	vector<int> vTotalSortIndex;
	vTotalSortIndex.emplace_back(0);
	//vector<vector<int>> vvAllLine;
 	while (1) {
		int temp = nowVertexIndex;
		if (lastVertex == temp) {
			// 当前搜索区域根本没有最优解
			return false;
		}
		for (int i = lastVertex; i < temp; i++) {
			//cout << lastVertex << "	" << temp << endl;
			v = expandedVertexArr[vertexSort[i]]; // 遍历当前广度的所有体素块

			if (v.occupy) {
				continue;
			}
			for (int j = 0; j < 6; j++) {
				int nextVoxelNum = v.vVertexNum[j];
				if (nextVoxelNum == -1) {
					// 当前方向的顶点无效
					continue;
				}
				else {
					// 有效节点
					if (vertexUse[nextVoxelNum] == true) {
						continue;
					}
					//cout << v.num << " ";
					vertexSort[nowVertexIndex] = nextVoxelNum; // 当前节点加入广度搜索序列
					nowVertexIndex++;
					vertexUse[nextVoxelNum] = true; // 指示当前节点已为最优
					//cout << "v.vVertexNum[j]:" << v.vVertexNum[j] << endl;
					if (nextVoxelNum == goal.num) {
						// 找到目标顶点
						findFlag = true;
						break;
					}
				}
			}
			if (findFlag) {
				break;
			}
			//cout << "广度优先排序结果：" << endl;
			//for (int i = 0; i < nowVertexIndex; i++) {
			//	cout << vertexSort[i] << " ";
			//}
		}
		lastVertex = temp;
		vTotalSortIndex.emplace_back(lastVertex);
		totalLen++;
		if (findFlag) {
			break;
		}
		//if (nowVertexIndex == len) {
		//	break;
		//}
	}
	vTotalSortIndex.swap(vTotalSortIndex);
	cout << "最短路径长度为：" << totalLen << endl;
	// 最短路径对应广度优先搜索的索引
	for (int i = 0; i < vTotalSortIndex.size(); i++) {
		cout << vTotalSortIndex[i] << " ";
	}
	//cout << endl << "广度优先排序结果：" << endl;
	//for (int i = 0; i < len; i++) {
	//	cout << vertexSort[i] << " ";
	//}
	cout << endl;

	// 根据广度优先层级结果还原到路径
	vCurrentTrajectoryVertex = vector<vertex>(vTotalSortIndex.size(), vertex());
	bfsLine = vector<int>(vTotalSortIndex.size(), 0);
	bfsLine[vTotalSortIndex.size() - 1] = goal.num;
	for (int i = vTotalSortIndex.size() - 2; i >= 0; i--) {
		int lastVoxelNum = bfsLine[i + 1];
		int nowFinded = false;
		cout << "vTotalSortIndex[i]:" << vTotalSortIndex[i] << "	vTotalSortIndex[i + 1]:" << vTotalSortIndex[i + 1] << endl;
		for (int j = vTotalSortIndex[i]; j < vTotalSortIndex[i + 1]; j++) {
			vertex nowVertex = expandedVertexArr[vertexSort[j]];
			//cout << "nowVertex:"<< vertexSort[j] << " " << nowVertex.vVertexNum[0] << " " << nowVertex.vVertexNum[1]
			//	<< " " << nowVertex.vVertexNum[2] << " " << nowVertex.vVertexNum[3]
			//	<< " " << nowVertex.vVertexNum[4] << " " << nowVertex.vVertexNum[5] << endl;
			// 寻找父亲节点
			for (int k = 0; k < 6; k++) {
				if (nowVertex.vVertexNum[k] == lastVoxelNum) {
					//cout << "相邻节点为：" << vVertex[j].vVertexNum[k] << endl;
					bfsLine[i] = nowVertex.num;
					nowFinded = true;
					vCurrentTrajectoryVertex[i] = nowVertex;
					break;
				}
			}
			if (nowFinded) {
				break;
			}
		}
	}
	cout << endl << "广度优先排序结果：" << endl;
	for (int i = 0; i < bfsLine.size(); i++) {
		cout << bfsLine[i] << " ";
	}

	return true;
}

void UpDateRouteBaseCurrentPointCloud(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
	// 点云体素化确定目标可行解范围
	// 起始点（相机位置）、目标点（粗定位点）
	Eigen::Vector3i startVertexVoxel = Eigen::Vector3i(0, 0, 0);
	Eigen::Vector3i endVertexVoxel = Eigen::Vector3i(0, 0, 0);
	v2pInfo.size = pcl_ptr->GetAxisAlignedBoundingBox().GetExtent(); // 更新点云信息
	v2pInfo.minPoint = pcl_ptr->GetAxisAlignedBoundingBox().GetAxisAlignedBoundingBox().GetBoxPoints()[0]; // 0对应xyz最小的顶点
	ConvertPoint2VoxelPos(start, startVertexVoxel); // 相机位置
	//cout << "*******起始点对应的顶点编号为：" << startVertexVoxel.transpose() << endl;
	ConvertPoint2VoxelPos(goal, endVertexVoxel); // 目标对应的体素位置
	// 目标顶点不应是物体实际位置，而应是相机的观测位置。（在物体上方进行画圆搜索）    
	endVertexVoxel[2] = endVertexVoxel[2] + 2 * DEVICE_OCCUPYVOXEL;
	cout << "起始体素块对应体素坐标系下坐标:" << startVertexVoxel.transpose() << "	点云坐标为：" << start.transpose() << endl;
	cout << "目标体素块对应体素坐标系下坐标:" << endVertexVoxel.transpose() << endl;
	// 体素化：从点云xyz最小的那个角进行编号，点云bbox是xyz最小角为0号
	// 根据设备尺寸，对障碍物进行膨胀，将设备抽象成为质点
	SearchVoxelRegion svr;
	GetSearchRegion(startVertexVoxel, endVertexVoxel, svr); // 确定搜索范围
	// 对搜索区域的体素块进行编号、膨胀
	vertex startVertex, endVertex;
	VertexNumFromSearchRegion(svr, startVertexVoxel, endVertexVoxel, startVertex, endVertex); // 对搜索范围进行编号
	expandedVertexArr = (vertex*)malloc(svr.height * svr.width * svr.length * sizeof(vertex));
	ExpandBarrierRegion(svr, expandedVertexArr); // 膨胀障碍物区域
	// 在点云中绘制膨胀后区域
	for (int i = 0; i < svr.length; i++) {
		for (int j = 0; j < svr.width; j++) {
			for (int k = 0; k < svr.height; k++) {
				int voxelNum = (i)+svr.length * (j)+svr.length * svr.width * (k); // 体素编号，以原点开始计算：x，y，z顺序
				if (expandedVertexArr[voxelNum].occupy) {
					pcl_voxel->AddVoxel(open3d::geometry::Voxel(expandedVertexArr[voxelNum].v, Eigen::Vector3d(255, 0, 0)));
				}
				//else {
				//    pcl_voxel->AddVoxel(open3d::geometry::Voxel(expandedVertexArr[voxelNum].v, Eigen::Vector3d(255, 0, 255)));
				//}
			}
		}
	}
	pcl_voxel->AddVoxel(open3d::geometry::Voxel(startVertexVoxel, Eigen::Vector3d(0, 0, 255)));
	pcl_voxel->AddVoxel(open3d::geometry::Voxel(endVertexVoxel, Eigen::Vector3d(0, 0, 255)));
	//open3d::visualization::DrawGeometries({ pcl_voxel }, "expanded");
	// BFS：建立无向有环图，并对无向无环图进行广度优先排序
	vCurrentTrajectory.clear();
	BreadthFirst(svr.height * svr.length * svr.width, startVertex, endVertex, vCurrentTrajectory);
	// 点云中绘制BFS结果
	cout << "bfsLine.size():" << vCurrentTrajectory.size() << endl;
	for (int i = 0; i < vCurrentTrajectory.size(); i++) {
		pcl_voxel->AddVoxel(open3d::geometry::Voxel(allVertexArr[vCurrentTrajectory[i]].v, Eigen::Vector3d(0, 255, 0)));
	}
}

bool VertifyRouteWhetherMeetRequire(const int startIndex) {
	int occupyNumX = ceil(DEVICE_WIDTH / VOXEL_SIZE / 2.);
	int occupyNumY = ceil(DEVICE_LENGTH / VOXEL_SIZE / 2.);
	int occupyNumZ = ceil(DEVICE_HEIGHT / VOXEL_SIZE / 2.);
	//for (int num = startIndex; num < startIndex + ROBOT_UPDATE_MINDIS * 2; num++) {
	for (int num = startIndex; num < vCurrentTrajectoryVertex.size(); num++) {
		//if (num >= vCurrentTrajectoryVertex.size()) return true;
		vertex tmpVertex = vCurrentTrajectoryVertex[num];
		cv::Mat adjustedNowGripperPostrue;
		AdjustCameraPosture(tmpVertex.vPcl, objectCenterPos, adjustedNowGripperPostrue); // 当前相机指向目标位置
		vector<Eigen::Vector3d> vDrawLinePoint;
		vector<Eigen::Vector2i> vDrawLineIndex;
		Eigen::Vector3d xOrient(adjustedNowGripperPostrue.at<double>(0, 0), adjustedNowGripperPostrue.at<double>(1, 0), adjustedNowGripperPostrue.at<double>(2, 0));
		Eigen::Vector3d yOrient(adjustedNowGripperPostrue.at<double>(0, 1), adjustedNowGripperPostrue.at<double>(1, 1), adjustedNowGripperPostrue.at<double>(2, 1));
		Eigen::Vector3d zOrient(adjustedNowGripperPostrue.at<double>(0, 2), adjustedNowGripperPostrue.at<double>(1, 2), adjustedNowGripperPostrue.at<double>(2, 2));
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 500 * xOrient);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 800 * yOrient);
		//vDrawLinePoint.emplace_back(tmpVertex.vPcl + 1600 * zOrient);
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
		//vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
		//std::shared_ptr<open3d::geometry::LineSet> nowCameraCloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
		//std::shared_ptr<open3d::geometry::LineSet> originandaxiscloud;
		//DrawXYZAxisAtOrient(originandaxiscloud);
		//open3d::visualization::DrawGeometries({pcl_voxel, nowCameraCloud, originandaxiscloud }, "Cam Pose"); // 显示当前相机朝向
		for (int i = -occupyNumX + 1; i < occupyNumX; i++) {
			for (int j = -occupyNumY + 1; j < occupyNumY; j++) {
				for (int k = 0; k < 2 * occupyNumZ; k++) {
					//double distance = (Eigen::Vector3i(i, j, k) - tmpVertex.vPcl).norm();
					//// 超出半径范围
					//if (distance > DEVICE_OCCUPYVOXEL + 1) {
					//	continue;
					//}
					// 查询当前位置是否被占用
					vector<Eigen::Vector3d> vCheck(1, Eigen::Vector3d(0, 0, 0));
					//vCheck[0] = tmpVertex.vPcl + Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					Eigen::Vector3d nowVoxelPos = Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					vCheck[0] = tmpVertex.vPcl + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					//cout << "当前点在点云中位置为：" << tmpVertex.vPcl.transpose() << "	查询点坐标为：" << vCheck[0].transpose() << endl;
					vector<bool> vResult = pcl_voxel->CheckIfIncluded(vCheck); // 占用情况
					//cout << "当前体素占用信息：" << vResult[0] << endl;
					if (vResult[0]) {
						// 被占用
						cout << "第" << num << "个点不满足要求！" << endl;
						return false;
					}

				}
			}
		}
	}
	return true;
}

cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, Eigen::Vector3d& rx, Eigen::Vector3d& ry) {
	double A = rz[0], B = rz[1], C = rz[2];
	if (A != 0) {
		ry = Eigen::Vector3d(-C / A, 0, 1).normalized();
		if (B == 0) {
			rx = Eigen::Vector3d(0, 1, 0);
		}
		else {
			rx = Eigen::Vector3d(1, -(C * C + A * A) / A / B, C / A).normalized();
		}
	}
	else if (A == 0 && B != 0) {
		ry = Eigen::Vector3d(0, -C / B, 1).normalized();
		rx = Eigen::Vector3d(1, 0, 0);
	}
	else {
		ry = Eigen::Vector3d(0, 1, 0);
		rx = Eigen::Vector3d(1, 0, 0);
	}

	cv::Mat rotate = (cv::Mat_<double>(3, 3) <<
		rx[0], ry[0], rz[0],
		rx[1], ry[1], rz[1],
		rx[2], ry[2], rz[2]
		);
	//cout << "rx：" << rx.transpose() << endl;
	//cout << "ry：" << ry.transpose() << endl;
	//cout << "rz：" << rz.transpose() << endl;
	//cout << "当前相机旋转矩阵为：" << rotate << "是否为旋转矩阵？："<< isRotationMatrix(rotate) << endl;

	return rotate;
}

//cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, const Eigen::Vector3d& rx, const Eigen::Vector3d& ry, Eigen::Vector3d& rxAdjusted, Eigen::Vector3d& ryAdjusted) {
//	// 保留rz方向使rx、ry与rz正交
//}

void AdjustCameraPosture(const Eigen::Vector3d& nowPose, const Eigen::Vector3d& objPos, cv::Mat& adjustedR) {
	// 相机Rz方向
	Eigen::Vector3d nowCameraRz = CalcVectorAB(nowPose, objPos);
	//cout << "当前相机应该指向的方向为：" << nowCameraRz.transpose() << endl;
	// 根据Rz计算Rx、Ry并组合成旋转矩阵
	Eigen::Vector3d nowCameraRx, nowCameraRy;
	cv::Mat nowCameraRotate = CalcRxRyBaseRz(nowCameraRz, nowCameraRx, nowCameraRy);
	adjustedR = nowCameraRotate({ 0,0,3,3 });
}

void AdjustCameraPosture(const string& nowGripperPose, const Eigen::Vector3d& objPos, string& adjustedGripperCmd, cv::Mat& adjustedR) {
	// 解析string
	cv::Mat nowCameraRT, nowGripperRT;
	ConvertString2Mat44(nowGripperPose, 1, nowCameraRT); // 字符转解析成6D坐标
	ConvertString2Mat44(nowGripperPose, 0, nowGripperRT); // 字符转解析成6D坐标
	//cout << "相机位姿为：" << nowCameraRT << endl;
	//cout << "机械臂末端位姿为：" << nowGripperRT << endl;
	//cout << "相机位姿计算机械臂末端位姿为：" << nowCameraRT * H_Camera2Gripper.inv() << endl;
	// 相机Rz方向
	Eigen::Vector3d nowCameraRz = CalcVectorAB(
		Eigen::Vector3d(nowCameraRT.at<double>(0, 3), nowCameraRT.at<double>(1, 3), nowCameraRT.at<double>(2, 3)),
		objPos
	);
	//cout << "当前相机应该指向的方向为：" << nowCameraRz.transpose() << endl;
	// 根据Rz计算Rx、Ry并组合成旋转矩阵
	Eigen::Vector3d nowCameraRx, nowCameraRy;
	cv::Mat nowCameraRotate = CalcRxRyBaseRz(nowCameraRz, nowCameraRx, nowCameraRy);
	// 相机姿态转机械臂末端
	cv::Mat nowCameraT = nowCameraRT * (Mat_<double>(4, 1) << 0, 0, 0, 1);
	//cout << "机械臂末端位置为：" << nowCameraT.t() << endl;
	//cout << "相机位置1为：" << ((H_Camera2Gripper.inv() * nowGripperRT.inv()).inv() * (Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
	//cout << "相机位置2为：" << (nowGripperRT * H_Camera2Gripper * (Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
	cv::Mat adjustedCameraRT = R_T2RT(nowCameraRotate, nowCameraT); // 合并R、T
	//cout << "Camera2Gripper的逆：" << H_Camera2Gripper.inv() << endl;
	cv::Mat adjustedGripper = adjustedCameraRT * H_Camera2Gripper.inv(); // 相机转Gipper
	adjustedR = adjustedGripper({ 0,0,3,3 }); // 相机坐标系转换到机械臂末端对应的旋转矩阵
	//cout << "调整后机械臂末端姿态为：" << adjustedGripper << endl;
	ConvertMat2String(adjustedGripper, adjustedGripperCmd); // Mat转机械臂指令
	//cout << "调整后机械臂末端姿态为：" << adjustedGripperCmd << endl;
}

Eigen::Vector3d CalcVectorAB(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	return (b - a).normalized();
}

void ConvertString2Mat44(const string& str, const bool gripper2cam, cv::Mat& m44) {
	vector<double> v; // 79.4 -446.7 357.6 179 -027 179
	AnalysisString26D(str, v); // A+0794-4467+3576+179-027+179BC ---> 79.4 -446.7 357.6 179 -027 179
	cv::Mat euler = (cv::Mat_<double>(1, 3) << v[3], v[4], v[5]);
	cv::Mat rotate = eulerAngleToRotatedMatrix(euler, "xyz");
	cv::Mat translate = (cv::Mat_<double>(3, 1) << v[0], v[1], v[2]);
	cv::Mat m = R_T2RT(rotate, translate); // 4*4旋转平移矩阵Gripper2base
	if (gripper2cam) {
		// 机械臂末端坐标系转相机坐标系
		m44 = m * H_Camera2Gripper;
	}
	else {
		m44 = m.clone();
	}

	return;
}

/// <summary>
/// 写入坐标
/// </summary>
/// <param name="v"></param>
/// <param name="trunc"></param>
void WriteTrajectory2File(const vector<vertex>& v, fstream& fs) {
	fs << "第i个点	体素编号	占用信息	空间位置	点云位置" << endl;
	for (int i = 0; i < v.size(); i++) {
		fs << i << " " << v[i].num << "	" << v[i].occupy << "	" << v[i].v.transpose() << "	" << v[i].vPcl.transpose() << endl;
	}
	fs.close();

	return;
}


bool VertifyRouteWhetherSatisfy(const vector<Eigen::Vector3d>& vPos, const Eigen::Vector3d& point, double dist) {
	int occupyNumX = ceil(DEVICE_WIDTH / VOXEL_SIZE / 2.);
	int occupyNumY = ceil(DEVICE_LENGTH / VOXEL_SIZE / 2.);
	int occupyNumZ = ceil(DEVICE_HEIGHT / VOXEL_SIZE / 2.);

	for (int num = 0; num < vPos.size(); num++) {
		if ((vPos[num] - point).norm() > dist) continue;
		// 读取当前机械臂坐标
		string currStrPose;
		sp.ReadRobotArmPosString(sp, currStrPose);
		// 解析string
		cv::Mat curr6DMatPose;
		AnalysisString26D(currStrPose, curr6DMatPose);
		cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
		Eigen::Vector3d xOrient(H_gripper2base.at<double>(0, 0), H_gripper2base.at<double>(1, 0), H_gripper2base.at<double>(2, 0));
		Eigen::Vector3d yOrient(H_gripper2base.at<double>(0, 1), H_gripper2base.at<double>(1, 1), H_gripper2base.at<double>(2, 1));
		Eigen::Vector3d zOrient(H_gripper2base.at<double>(0, 2), H_gripper2base.at<double>(1, 2), H_gripper2base.at<double>(2, 2));
		Eigen::Vector3d currPos = vPos[num];
		for (int i = -occupyNumX + 1; i < occupyNumX; i++) {
			for (int j = -occupyNumY + 1; j < occupyNumY; j++) {
				for (int k = -occupyNumZ + 1; k < occupyNumZ; k++) {
					//double distance = (Eigen::Vector3i(i, j, k) - tmpVertex.vPcl).norm();
					//// 超出半径范围
					//if (distance > DEVICE_OCCUPYVOXEL + 1) {
					//	continue;
					//}
					// 查询当前位置是否被占用
					vector<Eigen::Vector3d> vCheck(1, Eigen::Vector3d(0, 0, 0));
					//vCheck[0] = tmpVertex.vPcl + Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					Eigen::Vector3d nowVoxelPos = Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5) * VOXEL_SIZE;
					//vCheck[0] = currPos + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					vCheck[0] = currPos + DEVICE_HEIGHT * zOrient / 2 + nowVoxelPos[0] * xOrient + nowVoxelPos[1] * yOrient + nowVoxelPos[2] * zOrient;
					//vCheck[0] = currPos + nowVoxelPos;
					//cout << "当前点在点云中位置为：" << tmpVertex.vPcl.transpose() << "	查询点坐标为：" << vCheck[0].transpose() << endl;
					vector<bool> vResult = pcl_voxel->CheckIfIncluded(vCheck); // 占用情况
					//cout << "当前体素占用信息：" << vResult[0] << endl;
					if (vResult[0]) {
						// 被占用
						//cout << "第" << num << "个点不满足要求！" << endl;
						return false;
					}

				}
			}
		}
	}
	return true;
}

void UpdateRealTimePointCloud() {
	string saveRealTimePosFilePath_Route = "./imgs/routeplaning/"; // 路径规划感控阶段，文件路径位置，实时图片的6D坐标
	CreateDir(saveRealTimePosFilePath_Route);
	//while (!existUpdateSubThread) {
	static int robotMoveCnt = 0;
	while (!agi._move2goal) {
		//Sleep(20);
		if (vAGVMovePosition.size() == 0) continue;

		// 固定步长ROBOT_UPDATE_MINDIS更新3D场景信息：点云-》体素化-》路径是否适用（不适用：重新编号BFS更新路径，适用：继续走）
		cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
		cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
		if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
			long t1 = GetTickCount();		
			//cout << "读取深度相机图片成功！" << endl;
			Mat colorImageDup, depthImageDup;
			flip(colorImage, colorImageDup, 1);
			flip(depthImage, depthImageDup, 1);

			// 读取当前机械臂坐标
			string currStrPose;
			sp.ReadRobotArmPosString(sp, currStrPose);
			// 更新当前实时图片信息
			//mutWritePos.lock();
			cameraImgInfo currentDepthImgInfo;
			UpdateRealTimePosInfo(colorImageDup, depthImageDup, currStrPose, currentDepthImgInfo);
			//mutWritePos.unlock();

			// 重构当前3D场景，只保末端distance范围内(mm)的点，减少数据量
			//double distance = 800.;
			double distance = 10000.;
			// 将当前视图也反投影到3D空间 
			std::shared_ptr<open3d::geometry::PointCloud> tmpNowPointCloud = std::make_shared<open3d::geometry::PointCloud>();
			reconstructInfo ri;
			*tmpNowPointCloud = *ReconstructFromOneImg(currentDepthImgInfo, ri);
			long t2 = GetTickCount();

			// 将当前AGV状态下所有点转化到初始状态下
			//Eigen::Vector3d diff = Eigen::Vector3d{
			//	agi._currAgvPos[0] - agi._oriAgvPos[0],
			//	agi._currAgvPos[1] - agi._oriAgvPos[1],
			//	agi._currAgvPos[2] - agi._oriAgvPos[2]
			//};
			Eigen::Vector3d diff = agi._currAgvPos - agi._oriAgvPos;
			//Eigen::Vector3d diff = agi._oriAgvPos - agi._currAgvPos;
			for (int pd = 0; pd < tmpNowPointCloud->points_.size(); pd++) {
				Eigen::Vector3d convertPoint = AGVMove2ArmPos(diff, tmpNowPointCloud->points_[pd], true, agi._oriAgvPos[2]);
				tmpNowPointCloud->points_[pd] = convertPoint;
			}

			//vector<Eigen::Vector3d> vArmMovePositionRTed(vArmMovePosition.size());
			//for (int i = 0; i < vArmMovePosition.size(); i++) {
			//	vArmMovePositionRTed[i] = AGVMove2ArmPos(diff, vArmMovePosition[i], false);
			//}
			//Eigen::Vector3d goalInfo = agi._oriAgvPos;
			//Eigen::Vector3d goalInfoRT = AGVMove2ArmPos(diff, goalInfo, false);
			long t3 = GetTickCount();
			
			//open3d::visualization::DrawGeometries({ pclRealTime_ptr, originandaxis_cloud }, "Point Cloud");
			// 体素化：从点云xyz最小的那个角进行编号，点云bbox是xyz最小角为0号
			pcl_voxel = open3d::geometry::VoxelGrid::CreateFromPointCloud(open3d::geometry::PointCloud(tmpNowPointCloud->points_), VOXEL_SIZE);
			long t4 = GetTickCount();

			vector<Eigen::Vector3d> vDrawLinePoint; // 画线所需点
			vector<Eigen::Vector2i> vDrawLineIndex; // 直线点在容器中对应的索引
			std::shared_ptr<open3d::geometry::LineSet> totalLine = std::make_shared<open3d::geometry::LineSet>();
			// 画出AGV轨迹转化到机械臂运动轨迹
			for (int point = 1; point < vArmMovePosition.size(); point++) {
				if (!vTest.empty() && vTest.front() <= point) {
					// 不画连接线
					cout << vTest.front() << endl;
					vTest.erase(vTest.begin());
					continue;
				}
				//cout << vArmMovePosition[point - 1].transpose() << "	" << vArmMovePosition[point].transpose() << endl;
				vDrawLinePoint.clear();
				vDrawLineIndex.clear();
				vDrawLinePoint.emplace_back(vArmMovePosition[point]);
				vDrawLinePoint.emplace_back(vArmMovePosition[point - 1]);
				vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
				*totalLine += *std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
				//open3d::visualization::DrawGeometries({ pcl_voxel, line }, "Voxeled");

			}				
			vDrawLinePoint.clear();
			vDrawLineIndex.clear();
			vDrawLinePoint.emplace_back(Eigen::Vector3d{ 0., 0., 0. });
			vDrawLinePoint.emplace_back(agi._worldGoalPos);
			//vDrawLinePoint.emplace_back(goalInfoRT);
			vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
			*totalLine += *std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
			std::shared_ptr<open3d::geometry::LineSet> originandaxis_cloud = DrawXYZAxisAtOrient(drawAxisInfo{});
			*totalLine += *originandaxis_cloud;
			//open3d::visualization::DrawGeometries({ tmpNowPointCloud, totalLine, originandaxis_cloud }, "Voxeled");

			//string dirPath = saveRealTimePosFilePath_Route + to_string(robotMoveCnt);
			//open3d::io::WritePointCloudToPLY(dirPath + "pointcloud.ply", *tmpNowPointCloud);
			////open3d::io::WritePointCloudToPLY(dirPath + "_line.ply", totalLine);
			//robotMoveCnt++;
			
			//{
			//	// 保存数据
			//	string dirPath = saveRealTimePosFilePath_Route + to_string(robotMoveCnt) + "/";
			//	CreateDir(dirPath);
			//	string filePath = dirPath + "pos.txt";
			//	// 将实时坐标写入到文件中
			//	IOFile iof(filePath.c_str());
			//	// 清空文件内容，写入坐标
			//	iof.WriteString2File(currStrPose, 1);
			//	// 将实时轨迹写入到文件中
			//	filePath = dirPath + "trajectory.txt";
			//	// 保存RGBD数据
			//	string colorSavePath = dirPath + "rgb.jpg";
			//	string depthSavePath = dirPath + "depth.png";
			//	imwrite(colorSavePath, colorImageDup);
			//	imwrite(depthSavePath, depthImageDup); // PNG16
			//	// 保存点云
			//	open3d::io::WritePointCloudToPLY(dirPath + "pointcloud.ply", *tmpNowPointCloud);
			//	// 清空文件内容，写入当前轨迹坐标
			//	iof = IOFile(filePath.c_str());
			//	//WriteTrajectory2File(vCurrentTrajectoryVertex, iof);
			//}
			//// 判断之前路径vCurrentTrajectoryVertex是否满足当前体素要求
			//vector<Eigen::Vector3d> convertedPos;
			//ConvertOriMovePos2Curr(vArmMovePosition, convertedPos);
			// 更新当前路径对应坐标sssssssss
			Eigen::Vector3d rotatedCamera = AGVMove2ArmPos(diff, currentDepthImgInfo._CameraPosition, true, agi._oriAgvPos[2]);
			int vertifyRet = VertifyRouteWhetherSatisfy(vArmMovePosition, rotatedCamera, distance);
			long t5 = GetTickCount();
			cout << "   当前运动轨迹" << (vertifyRet ? "满足" : "不满足") << "要求!"
				<< "(" << t2 - t1 << "," << t3 - t2 << "," << t4 - t3 << "," << t5 - t4 << ")" << endl;

			if (!vertifyRet) {
				agi._move2goal = 2;
				cout << agi._move2goal << "-----------------------------------------" << (agi._move2goal == 2 ? "RGBD检测出异常" : "激光检测出异常") << endl << flush;
				break;
			}
			//robotMoveCnt++;
		}
		// Sleep(2);
	}
	agi._move2goal = 3;
}