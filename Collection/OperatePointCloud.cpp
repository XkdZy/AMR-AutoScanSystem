#include "header.h"

struct Image {
    Image(int _imageIndex, int _u, int _v, int _distance) {
        imageIndex = _imageIndex;
        distance = _distance;
        u = _u;
        v = _v;
    }
    int imageIndex;
    int distance;
    int u;
    int v;
};
void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera) {
    colorTmp << 0.0, 0.0, 0.0;
    Eigen::Vector3d pointTemp(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    vector<Image> imageVec;
    if (thermalOrCamera == "thermal") {
        for (int j = 0; j < vThermalInfo.size(); j++) {
            int distanceTmp = (vThermalInfo[j]._CameraPosition - pointTemp).norm();
            //计算热像仪下的坐标信息
            cv::Mat p_thermal = H_Thermal2Gripper.inv() * vThermalInfo[j]._Gripper2Base * p_base;
            //计算热像仪对应的像素坐标
            p_thermal = ThermalMatrix * p_thermal;

            int u = p_thermal.at<double>(0, 0) / p_thermal.at<double>(2, 0);
            int v = p_thermal.at<double>(1, 0) / p_thermal.at<double>(2, 0);
            if (u > 0 && u < 640 && v > 0 && v < 480) {
                imageVec.push_back(Image(j, u, v, distanceTmp));
            }
        }
    }
    else if (thermalOrCamera == "camera") {
        for (int j = 0; j < vRGBDInfo.size(); j++) {
            int distanceTmp = (vRGBDInfo[j]._CameraPosition - pointTemp).norm();
            //计算热像仪下的坐标信息
            cv::Mat p_camera = H_Camera2Gripper.inv() * vRGBDInfo[j]._Gripper2Base * p_base;
            //计算热像仪对应的像素坐标
            p_camera = CameraMatrix * p_camera;

            int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
            int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
            if (u > 0 && u < 640 && v > 0 && v < 480) {
                imageVec.push_back(Image(j, u, v, distanceTmp));
            }
        }
    }

    //查看该点云是否有匹配的热像仪特征图
    if (imageVec.empty()) {
        //如果没有匹配的特征图，将其标记为白色
        colorTmp << 1, 1, 1;
    }
    else {
        //如果有匹配的特征图，将特张图加权融合
        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
            return img1.distance < img2.distance; });

        int differencValue = 30;
        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
            imageVec.pop_back();
        }
        int distanceSum = differencValue;
        for (auto image_ : imageVec) {
            distanceSum += image_.distance;
        }
        int meanDistance = distanceSum / imageVec.size();

        for (auto image_ : imageVec) {
            double weight = (meanDistance - image_.distance) / (double)differencValue;
            Mat img;
            if (thermalOrCamera == "thermal") {
                img = vThermalInfo[image_.imageIndex]._RgbImg;
            }
            else if (thermalOrCamera == "camera") {
                img = vRGBDInfo[image_.imageIndex]._RgbImg;
            }
            int u = image_.u;
            int v = image_.v;
            Eigen::Vector3d colorTemp_(img.at<Vec3b>(v, u)[2] / 255.0, img.at<Vec3b>(v, u)[1] / 255.0, img.at<Vec3b>(v, u)[0] / 255.0);

            colorTmp += colorTemp_ * weight;
        }
    }
}
//void GetColorFromImageVec(Eigen::Vector3d& colorTmp, const cv::Mat& p_base, string thermalOrCamera) {
//    colorTmp << 0.0, 0.0, 0.0;
//    Eigen::Vector3d pointTemp(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
//    vector<Image> imageVec;
//    if (thermalOrCamera == "thermal") {
//        for (int j = 0; j < thermalPositionVec.size(); j++) {
//            int distanceTmp = (thermalPositionVec[j] - pointTemp).norm();
//            //计算热像仪下的坐标信息
//            cv::Mat p_thermal = H_Thermal2Gripper.inv() * H_Base2Gripper_thermalVec[j] * p_base;
//            //计算热像仪对应的像素坐标
//            p_thermal = ThermalMatrix * p_thermal;
//
//            int u = p_thermal.at<double>(0, 0) / p_thermal.at<double>(2, 0);
//            int v = p_thermal.at<double>(1, 0) / p_thermal.at<double>(2, 0);
//            if (u > 0 && u < 640 && v > 0 && v < 480) {
//                imageVec.push_back(Image(j, u, v, distanceTmp));
//            }
//        }
//    }
//    else if (thermalOrCamera == "camera") {
//        for (int j = 0; j < cameraPositionVec.size(); j++) {
//            int distanceTmp = (cameraPositionVec[j] - pointTemp).norm();
//            //计算热像仪下的坐标信息
//            cv::Mat p_camera = H_Camera2Gripper.inv() * H_Base2Gripper_cameraVec[j] * p_base;
//            //计算热像仪对应的像素坐标
//            p_camera = CameraMatrix * p_camera;
//
//            int u = p_camera.at<double>(0, 0) / p_camera.at<double>(2, 0);
//            int v = p_camera.at<double>(1, 0) / p_camera.at<double>(2, 0);
//            if (u > 0 && u < 640 && v > 0 && v < 480) {
//                imageVec.push_back(Image(j, u, v, distanceTmp));
//            }
//        }
//    }
//
//    //查看该点云是否有匹配的热像仪特征图
//    if (imageVec.empty()) {
//        //如果没有匹配的特征图，将其标记为白色
//        colorTmp << 1, 1, 1;
//    }
//    else {
//        //如果有匹配的特征图，将特张图加权融合
//        sort(imageVec.begin(), imageVec.end(), [=](Image img1, Image img2) {
//            return img1.distance < img2.distance; });
//
//        int differencValue = 30;
//        while (imageVec.back().distance - imageVec.front().distance > differencValue) {
//            imageVec.pop_back();
//        }
//        int distanceSum = differencValue;
//        for (auto image_ : imageVec) {
//            distanceSum += image_.distance;
//        }
//        int meanDistance = distanceSum / imageVec.size();
//
//        for (auto image_ : imageVec) {
//            double weight = (meanDistance - image_.distance) / (double)differencValue;
//            Mat img;
//            if (thermalOrCamera == "thermal") {
//                img = thermalImageVec[image_.imageIndex];
//            }
//            else if (thermalOrCamera == "camera") {
//                img = colorImageVec[image_.imageIndex];
//            }
//            int u = image_.u;
//            int v = image_.v;
//            Eigen::Vector3d colorTemp_(img.at<Vec3b>(v, u)[2] / 255.0, img.at<Vec3b>(v, u)[1] / 255.0, img.at<Vec3b>(v, u)[0] / 255.0);
//
//            colorTmp += colorTemp_ * weight;
//        }
//    }
//}

Eigen::Vector3d ConvertPixel2World(int row, int col, const cameraImgInfo& cii, ushort depth) {
    ushort depthNum = 0;
    if (depth == 0) depthNum = cii._DepthImg.at<ushort>(row, col);
    else depthNum = depth;
    cout << "depth num:" << depthNum << endl;
    Eigen::Vector3d position{ 0., 0., 0. };
    if (depthNum > MIN_DISTANCE && depthNum < MAX_DISTANCE) {
        //cout << depthNum << endl;
        //计算深度相机下的坐标
        double x_camera, y_camera, z_camera;
        Eigen::Vector3d camera_xyz = ConvertPixel2Camera(col, row, depthNum, 0);
        x_camera = camera_xyz[0];
        y_camera = camera_xyz[1];
        z_camera = camera_xyz[2];
        //astraCameraD2C->convertDepthToWorld(col, row, depthNum, x_camera, y_camera, z_camera);
        cv::Mat p_camera{ x_camera, y_camera, z_camera, 1.0 };
        cout << "p_camera:" << p_camera.t() << endl;
        //计算机械手下的坐标信息
        cv::Mat p_gripper = H_Camera2Gripper * p_camera;

        //计算基地下的坐标信息
        cv::Mat p_base = cii._Gripper2Base * p_gripper;
        cout << "p_base:" << p_base.t() << endl;
        cout << "_Gripper2Base:" << cii._Gripper2Base << endl;

        position = Eigen::Vector3d(p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2));
    }

    return position;
}

std::shared_ptr<open3d::geometry::PointCloud> ReconstructFromOneImg(const cameraImgInfo& cii, const reconstructInfo& ri) {
    std::shared_ptr<open3d::geometry::PointCloud> pcl = std::make_shared<open3d::geometry::PointCloud>();
    Mat depthImage = cii._DepthImg;
    // 拼接深度图为三维点云数据，匹配红外特征图到三维点云
    for (int row = 0; row < depthImage.rows; row++) {
        for (int col = 0; col < depthImage.cols; col++) {
            ushort depthNum = depthImage.at<ushort>(row, col); // 深度相机当前像素的深度值
            if (depthNum < MIN_DISTANCE) continue;
            if ((0x08 & ri._2whichbase) && ri._mask.at<uchar>(row, col) == 0) continue;

            //计算深度相机下的坐标
            double x_camera, y_camera, z_camera;
            Eigen::Vector3d camera_xyz = ConvertPixel2Camera(col, row, depthNum, 0);
            x_camera = camera_xyz[0];
            y_camera = camera_xyz[1];
            z_camera = camera_xyz[2];
            cv::Mat p_camera{ x_camera, y_camera, z_camera, 1.0 };

            //计算机械手下的坐标信息
            cv::Mat p_gripper = H_Camera2Gripper * p_camera;

            //计算基地下的坐标信息
            cv::Mat p_base = cii._Gripper2Base * p_gripper;

            Eigen::Vector3d pointTemp{ p_base.at<double>(0), p_base.at<double>(1), p_base.at<double>(2) };
            Eigen::Vector3d colorThermal(0.0, 0.0, 0.0);
            Eigen::Vector3d colorCamera(0.0, 0.0, 0.0);
            Eigen::Vector3d colorTemp(0.0, 0.0, 0.0);

            if ((0x01 & ri._2whichbase) && ri._agvRT) {
                // AGV旋转平移，机械臂世界坐标对应变化
                pointTemp = AGVMove2ArmPos(ri._agvRTInfo, pointTemp, true, ri._agvOriDir);
            }
            else if (0x02 & ri._2whichbase) {
                // 机械臂世界坐标到AGV世界坐标
                //cout << ri._agvCurrInfo.transpose() << endl;
                //cout << "1:" << pointTemp.transpose() << endl;
                pointTemp = arm2agv(ri._agvCurrInfo, pointTemp);
                //cout << "2:" << pointTemp.transpose() << endl;
            }
            else if (0x04 & ri._2whichbase) {
                // 限幅
                if ((pointTemp - ri._setPoint).norm() > ri._dmm) continue;
            }
            else if (0x10 & ri._2whichbase) {
                // thermal图片上色
                //GetColorFromImageVec(colorCamera, p_base, "camera");
                //GetColorFromImageVec(colorThermal, p_base, "thermal");
            }
            else if (0x20 & ri._2whichbase) {
                // 判断该点在全局点云中是否存在（体素网格中是否含有该点），如果存在则不保存
                vector<Eigen::Vector3d> v{ pointTemp };
                if (ri._totalVoxel->CheckIfIncluded(v)[0]) continue;
            }

            // 用当前图片颜色上色
            double weightCamera = 1., weightThermal = 0.;
            Vec3b nowImg = cii._RgbImg.at<Vec3b>(row, col);
            colorCamera = Eigen::Vector3d(nowImg[2] / 255.0, nowImg[1] / 255.0, nowImg[0] / 255.0);
            colorTemp = weightCamera * colorCamera + weightThermal * colorThermal;

            pcl->points_.push_back(pointTemp);
            pcl->colors_.push_back(colorTemp);
        }
    }

    return pcl;
}

std::shared_ptr<open3d::geometry::PointCloud> RemoveSmallerPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& bigPC, const std::shared_ptr<open3d::geometry::PointCloud>& smallPC) {
    std::shared_ptr<open3d::geometry::PointCloud> ret = bigPC;

    vector<double> dis = bigPC->ComputePointCloudDistance(open3d::geometry::PointCloud(smallPC->points_));
    vector<size_t> vIndex;
    for (int i = 0; i < dis.size(); i++) {
        if (dis[i] >= 0.1) {
            vIndex.emplace_back(i);
        }
    }
    if (vIndex.size() != 0) {
        ret = bigPC->SelectByIndex(vIndex);
        //open3d::visualization::DrawGeometries({ pclWithoutlastPointCloud }, "diff");
    }

    return ret;
}

void CombinePartialPointCloud(std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC1,
                            std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC2,
                            std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& totalPC)
{
    for (int i = 0; i < vPartialPC1.size(); i++)
    {
        totalPC.emplace_back(vPartialPC1[i]); // 点云整合一起显示
        totalPC.emplace_back(vPartialPC2[i]);
    }
}

std::shared_ptr<open3d::geometry::PointCloud> RotatePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& srcPcl, const PointCloudRotateInfo pcri) {
    // 拷贝原始点云
    std::shared_ptr<open3d::geometry::PointCloud> pclUse = std::make_shared<open3d::geometry::PointCloud>(*srcPcl);

    //cout << "欧拉角：" << pcri._rotateXYZ.transpose() << endl;
    //// 点云按某点旋转
    //cv::Mat rotMat = RotatePclMat(pcri._rotateXYZ[2]);
    ////cout << "旋转矩阵为：" << rotMat << endl;
    //auto pclPtrSLRotate = make_shared<open3d::geometry::PointCloud>();
    //for (int pd = 0; pd < pclUse->points_.size(); pd++) {
    //    Eigen::Vector3d currPosition = pclUse->points_[pd];
    //    //cout << "before rotate:" << currPosition.transpose() << endl;
    //    Eigen::Vector3d cmeraPosition = currPosition - pcri._rotateCenter;
    //    cv::Mat cameraRotatedMat = rotMat * (cv::Mat_<double>(4, 1) << cmeraPosition[0], cmeraPosition[1], cmeraPosition[2], 1);
    //    Eigen::Vector3d rotatedPosition{ pcri._rotateCenter + Eigen::Vector3d{ cameraRotatedMat.at<double>(0,0), cameraRotatedMat.at<double>(1,0),cameraRotatedMat.at<double>(2,0)} };
    //    pclPtrSLRotate->points_.emplace_back(rotatedPosition);
    //    //cout << "after rotate:" << rotatedPosition.transpose() << endl;
    //    pclPtrSLRotate->colors_.emplace_back(pclUse->colors_[pd]);
    //}
    //return pclPtrSLRotate;

    // 将点云整体旋转，open3d顺时针为负、逆时针为正
    Eigen::Matrix3d rotMat = pclUse->GetRotationMatrixFromXYZ(pcri._rotateXYZ);
    auto rotatedPointCloudTemp = pclUse->Rotate(rotMat, pcri._rotateCenter);
    return std::make_shared<open3d::geometry::PointCloud>(rotatedPointCloudTemp);
}

void GenerateBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> srcPointCloud, vector<Eigen::Vector3d>& vEightPoint) {
    // 点云最小外接矩形
    open3d::geometry::AxisAlignedBoundingBox axisBox = srcPointCloud->GetAxisAlignedBoundingBox(); // 点云包围框
    axisBox.color_ = Eigen::Vector3d(1, 0, 0);
    std::shared_ptr<geometry::AxisAlignedBoundingBox> pcl_ptr4 = std::make_shared<geometry::AxisAlignedBoundingBox>(axisBox); // 包围框对应点云
    //open3d::visualization::DrawGeometries({ inlier, pcl_ptr4 }, "BoundingBox");
    auto boxPoint = pcl_ptr4->GetBoxPoints();
    vector<Eigen::Vector3d> allPoint = srcPointCloud->points_;
    vector<Eigen::Vector3d> allPointColor = srcPointCloud->colors_;
    //for (int i = 0; i < boxPoint.size(); i++)
    //{
    //    cout << boxPoint[i].transpose() << endl;
    //}
    double BILI = 0.999999;
    double limitXLow = boxPoint[0][0]; // 顶点1
    limitXLow = limitXLow * (sign(limitXLow) > 0 ? 2 - BILI : BILI);
    double limitYLow = boxPoint[0][1]; // 顶点2
    limitYLow = limitYLow * (sign(limitYLow) > 0 ? 2 - BILI : BILI);
    double limitZLow = boxPoint[0][2];
    limitZLow = limitZLow * (sign(limitZLow) > 0 ? 2 - BILI : BILI);
    double limitXUpp = boxPoint[4][0]; // 顶点3
    limitXUpp = limitXUpp * (sign(limitXUpp) < 0 ? 2 - BILI : BILI);
    double limitYUpp = boxPoint[4][1]; // 顶点4
    limitYUpp = limitYUpp * (sign(limitYUpp) < 0 ? 2 - BILI : BILI);
    double limitZUpp = boxPoint[4][2];
    limitZUpp = limitZUpp * (sign(limitZUpp) < 0 ? 2 - BILI : BILI);
    Eigen::Vector3d vP1, vP2, vP3, vP4, vP5, vP6;
    for (int i = 0; i < allPoint.size(); i++)
    {
        if (allPoint[i][0] < limitXLow)
            vP1 = allPoint[i];
        if (allPoint[i][1] < limitYLow)
            vP2 = allPoint[i];
        if (allPoint[i][2] < limitZLow)
            vP3 = allPoint[i];
        if (allPoint[i][0] > limitXUpp)
            vP4 = allPoint[i];
        if (allPoint[i][1] > limitYUpp)
            vP5 = allPoint[i];
        if (allPoint[i][2] > limitZUpp)
            vP6 = allPoint[i];
    }
    ///*****************************open3d定义包围盒的方式和非紧密接触时自己定义的定点方式（myBoundingBox）的画线比较
    /// 包围盒8顶点定义                    包围盒(1')非紧密相接时俯视图
    ///      0 ------------------- 1    0 --------------1
    ///       /|                /|      |       /\      |
    ///      / |               / |      |      /  \     |
    ///     /  |              /  |      |     /    \    |
    ///    /   |             /   |      |    /      \   |
    /// 2 ------------------- 7  |      |   /        \  |
    ///   |    |____________|____| 6    |  /          \ |
    ///   |   /3            |   /       | /            \|7'
    ///   |  /              |  /        |/             /|
    ///   | /               | /       0'|\            / |
    ///   |/                |/          | \          /  |
    /// 5 ------------------- 4         2 --------------3
    ///                                         2'
    //cout << Eigen::Vector3d(vP1[0], vP1[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP4[0], vP4[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP5[0], vP5[1], limitZLow).transpose() << endl;
    //cout << Eigen::Vector3d(vP1[0], vP1[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP2[0], vP2[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP5[0], vP5[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP4[0], vP4[1], limitZUpp).transpose() << endl;
    //cout << Eigen::Vector3d(vP2[0], vP2[1], limitZLow).transpose() << endl;
    vEightPoint.emplace_back(Eigen::Vector3d(vP1[0], vP1[1], limitZLow)); // P0
    vEightPoint.emplace_back(Eigen::Vector3d(vP2[0], vP2[1], limitZLow)); // P1
    vEightPoint.emplace_back(Eigen::Vector3d(vP5[0], vP5[1], limitZLow)); // P2
    vEightPoint.emplace_back(Eigen::Vector3d(vP1[0], vP1[1], limitZUpp)); // P3
    vEightPoint.emplace_back(Eigen::Vector3d(vP4[0], vP4[1], limitZUpp)); // P4
    vEightPoint.emplace_back(Eigen::Vector3d(vP5[0], vP5[1], limitZUpp)); // P5
    vEightPoint.emplace_back(Eigen::Vector3d(vP2[0], vP2[1], limitZUpp)); // P6
    vEightPoint.emplace_back(Eigen::Vector3d(vP4[0], vP4[1], limitZLow)); // P7
    vector<Eigen::Vector2i> vLineIndexTemp; // 直线点在容器中对应的索引
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 1));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 2));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 3));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(1, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(1, 7));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(2, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(2, 7));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(3, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(3, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 5));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 6));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(4, 7));
    vector<Eigen::Vector2i> vLineIndexBoxTemp; // 直线点在容器中对应的索引
    vLineIndexBoxTemp.emplace_back(Eigen::Vector2i(0, 1));
    vLineIndexBoxTemp.emplace_back(Eigen::Vector2i(0, 2));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vEightPoint, vLineIndexTemp));
    std::shared_ptr<open3d::geometry::LineSet> lineSetBoxTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(boxPoint, vLineIndexBoxTemp));
    //open3d::visualization::DrawGeometries({ srcPointCloud, lineSetTemp_cloud,lineSetBoxTemp_cloud, pcl_ptr4 }, "BoundingBox");
}

void ProjectPointCloudFromZOrient(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud, vector<Eigen::Vector3d>& vBoundingBox, cv::Mat& img, int& validPixel, uchar outImgType, bool& dir)
{
    // 根据AxisBoundingBox生成z轴向下看的二维图像
    // 点云点个数少190000，2D图像分辨率不高
    vector<Eigen::Vector3d> vAllPoint = srcPointCloud->points_;
    vector<Eigen::Vector3d> vAllColor = srcPointCloud->colors_;
    // 固定x的长度360，确定y的长度
    int rowNum = 240;
    double rowNumTemp = (vBoundingBox[0] - vBoundingBox[2]).norm(); // y长度
    double colNumTemp = (vBoundingBox[0] - vBoundingBox[1]).norm(); // x长度
    double zValNumTemp = (vBoundingBox[0] - vBoundingBox[3]).norm(); // z长度
    int colNum = (int)((double)(colNumTemp * rowNum) / rowNumTemp);
    // 记录包围盒
    //Eigen::MatrixXd outImgDouble(rowNum, colNum); // 将体素归一化到rowNum*colNum的图片上
    cv::Mat binImgInv(rowNum, colNum, CV_8U); // 黑色（无点落在该位置上）、白色（有点落在该位置上）
    cv::Mat garyImgInv(rowNum, colNum, CV_8U); // 图片颜色和当前点到最高点深度差相关
    cv::Mat rgbImgInv(rowNum, colNum, CV_8UC3); // 用点云中点的颜色给图片上色
    validPixel = 0;
    for (int i = 0; i < vAllPoint.size(); i++)
    {
        double disCol = abs(vAllPoint[i][0] - vBoundingBox[0][0]); // x偏差 - col
        double disRow = abs(vAllPoint[i][1] - vBoundingBox[0][1]); // y偏差 - row
        double disZ = abs(vAllPoint[i][2] - vBoundingBox[4][2]); // z偏差 - z
        // 第i个点对应图片上的坐标（nowRow、nowCol）
        int nowRow = (disRow * rowNum) / rowNumTemp - 1;
        int nowCol = (disCol * colNum) / colNumTemp - 1;
        nowRow = nowRow < 0 ? 0 : nowRow;
        nowCol = nowCol < 0 ? 0 : nowCol;
        int nowZVal = 255 - (disZ * 255.) / zValNumTemp; // 归一化到255，离上平面越近则数值越大
        if (binImgInv.at<unsigned char>(nowRow, nowCol) == 255)
        {
            // 当前z在之前点的最上方，覆盖之前的点
            if (nowZVal > garyImgInv.at<uchar>(nowRow, nowCol))
            {
                garyImgInv.at<uchar>(nowRow, nowCol) = nowZVal;
                rgbImgInv.at<Vec3b>(nowRow, nowCol) = Vec3b(255 * vAllColor[i][0], 255 * vAllColor[i][1], 255 * vAllColor[i][2]);
            }
            if ((nowRow <= rowNum / 2) && (nowCol <= colNum / 2))
                validPixel -= 1;
        }
        else
        {
            garyImgInv.at<uchar>(nowRow, nowCol) = nowZVal;
            rgbImgInv.at<Vec3b>(nowRow, nowCol) = Vec3b(255 * vAllColor[i][0], 255 * vAllColor[i][1], 255 * vAllColor[i][2]);
        }
        binImgInv.at<unsigned char>(nowRow, nowCol) = 255;
        if((nowRow <= rowNum / 2) && (nowCol <= colNum / 2))
            validPixel += 1;
    }
    Mat binImg, garyImg, rgbImg;
    // >0水平 =0垂直 <0水平、垂直
    //flip(binImgInv, binImg, 0);
    //flip(binImgInv, garyImg, 0);
    //flip(binImgInv, rgbImg, 0);
    cv::flip(binImgInv, binImg, -1);
    cv::flip(binImgInv, garyImg, -1);
    cv::flip(binImgInv, rgbImg, -1);
    if (outImgType == 0)
        img = cv::Mat(binImg);
    else if (outImgType == 1)
        img = cv::Mat(garyImg);
    else
        img = cv::Mat(rgbImg);

    Mat blurImg, binary, dst;
    cv::GaussianBlur(binImg, blurImg, cv::Size(5, 5), 1); // 高斯模糊
    threshold(blurImg, binary, 128, 255, THRESH_BINARY); // 二值化
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, dst, cv::MORPH_CLOSE, kernel); // 闭操作 先膨胀在腐蚀

    // 抽取1/4、2/4、3/4行；1/4、2/4、3/4列数据判断黑白跳变时的位置，由此判断朝向
    dir = 0;
    vector<int> jumpRow(3, 0), jumpCol(3, 0);
    for (int j = 0; j < binImg.cols/2; j++) // 行跳变
    {
        if ((binary.at<uchar>(rowNum / 4, j) == 0) && (binary.at<uchar>(rowNum / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum / 4, j + 2) == 255) && (binary.at<uchar>(rowNum / 4, j + 3) == 255)) 
        {
            jumpRow[0] = j;
        }
        if ((binary.at<uchar>(rowNum * 2 / 4, j) == 0) && (binary.at<uchar>(rowNum * 2 / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum * 2 / 4, j + 2) == 255) && (binary.at<uchar>(rowNum * 2 / 4, j + 3) == 255))
        {
            jumpRow[1] = j;
        }
        if ((binary.at<uchar>(rowNum * 3 / 4, j) == 0) && (binary.at<uchar>(rowNum * 3 / 4, j + 1) == 255)
            && (binary.at<uchar>(rowNum * 3 / 4, j + 2) == 255) && (binary.at<uchar>(rowNum * 3 / 4, j + 3) == 255))
        {
            jumpRow[2] = j;
        }
    }
    for (int i = 0; i < binImg.rows / 2; i++) // 列跳变
    {
        if ((binary.at<uchar>(i, colNum / 4) == 0) && (binary.at<uchar>(i + 1, colNum / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum / 4) == 255) && (binary.at<uchar>(i + 3, colNum / 4) == 255))
        {
            jumpCol[0] = i;
        }
        if ((binary.at<uchar>(i, colNum * 2 / 4) == 0) && (binary.at<uchar>(i + 1, colNum * 2 / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum * 2 / 4) == 255) && (binary.at<uchar>(i + 3, colNum * 2 / 4) == 255))
        {
            jumpCol[1] = i;
        }
        if ((binary.at<uchar>(i, colNum * 3 / 4) == 0) && (binary.at<uchar>(i + 1, colNum * 3 / 4) == 255)
            && (binary.at<uchar>(i + 2, colNum * 3 / 4) == 255) && (binary.at<uchar>(i + 3, colNum * 3 / 4) == 255))
        {
            jumpCol[2] = i;
        }
    }
    if ((jumpRow[2] > jumpRow[1]) && (jumpRow[1] >= jumpRow[0]))
    {
        dir = false;
        return;
    }
    if ((jumpRow[2] <= jumpRow[1]) && (jumpRow[1] < jumpRow[0]))
    {
        dir = true;
        return;
    }
    if ((jumpCol[2] > jumpCol[1]) && (jumpCol[1] >= jumpCol[0]))
    {
        dir = false;
        return;
    }
    if ((jumpCol[2] <= jumpCol[1]) && (jumpCol[1] < jumpCol[0]))
    {
        dir = true;
        return;
    }

    //cout << "***********************************相机朝x、y平面投影以及投影、图片倾斜情况，输出测试" << endl;
    //cout << jumpRow[2] << "   " << jumpRow[1] << "   " << jumpRow[0] << endl;
    //cout << jumpCol[2] << "   " << jumpCol[1] << "   " << jumpCol[0] << endl;
    //cout << "dir：" << dir << endl;
    //cout << "rowNum：" << rowNum << endl;
    //cout << "colNum：" << colNum << endl;
    //cout << "outImg.size()：" << binImg.size() << endl;
    //cout << "vAllPoint.size()：" << vAllPoint.size() << endl;
    //cout << "validPixel：" << validPixel << endl;
    //cout << "**********************************************************************" << endl;
    ////cout << "nowRow：" << nowRow << "nowCol：" << nowCol << endl;
    //cv::imshow("binImg", binImg);
    //cv::imshow("garyImg", garyImg);
    //cv::imshow("rgbImg", rgbImg);
    //cv::waitKey(0);
}

void FromImgGetXYOrient(cv::Mat& src, int validCnt, bool leanDir, Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient, int lengthMoreWidth)
{
    // 角度放大了大约4倍（算面积没除以2、长、宽没除以2）
    // 黑边面积：recImg.size() - validCnt
    int rowNum = src.rows;
    int colNum = src.cols;
    int blackArea = src.rows * src.cols / 4 - validCnt;
    // 行列面积比：行/（行+列）**2
    double areaRatio = ((double)src.rows) / (src.cols + src.rows);    //double areaRatio = pow((recImg.rows / (recImg.cols + recImg.rows)), 2);
    areaRatio = 0.75;//1.、0.125;
    int rowValidCnt = blackArea * areaRatio;    //int rowValidCnt = blackArea * areaRatio / 2;
    int colValidCnt = blackArea * areaRatio;    //int colValidCnt = blackArea * (1 - areaRatio) / 2;
    double xAngle = atan((double)rowValidCnt / rowNum / rowNum) / CV_PI * 180;
    double yAngle = atan((double)colValidCnt / colNum / colNum) / CV_PI * 180;
    if (leanDir == true)
    {
        yOrient = Eigen::Vector3d(100 * tan(xAngle / 180. * CV_PI), 100, 0);
        yOrient = yOrient.normalized();
        xOrient = Eigen::Vector3d(100, -100 * tan(yAngle / 180. * CV_PI), 0);
        xOrient = xOrient.normalized();
    }
    else
    {
        yOrient = Eigen::Vector3d(-tan(xAngle / 180. * CV_PI) * 100, 100, 0);
        yOrient = yOrient.normalized();
        xOrient = Eigen::Vector3d(100, 100 * tan(yAngle / 180. * CV_PI), 0);
        xOrient = xOrient.normalized();
    }
    //if (!lengthMoreWidth)
    {
        // 默认要交换
        Eigen::Vector3d tempVec(yOrient); // 将相机长边对应到x
        yOrient = xOrient;
        xOrient = tempVec;
    }
    
    cout << "***********************************求相机x、y方向朝向，输出测试" << endl;
    cout << "blackArea：" << blackArea << endl;
    cout << "validCnt：" << validCnt << "   rowValidCnt：" << rowValidCnt << "   colValidCnt：" << colValidCnt << endl;
    cout << "xAngle：" << xAngle << "   yAngle：" << yAngle << endl;
    cout << "**********************************************************************" << endl;

    // 画线显示
    Point p1(100, 100), p2(tan(xAngle / 180. * CV_PI) * 100, 100), p3(100, -tan(yAngle / 180. * CV_PI) * 100);
    cv::line(src, p1, p2 + p1, Scalar(0, 0, 255));
    cv::line(src, p1, p3 + p1, Scalar(0, 0, 255));
    //cv::imshow("src", src);
    //cv::waitKey(0);

    /// 根据最小外接矩形，确定倾斜角，效果不明显
    //vector<cv::Mat> contours; 
    //Mat hierarcy;
    //cv::findContours(binary, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    //cout << "num=" << contours.size() << endl;
    //vector<Rect> boundRect(contours.size());  //定义外接矩形集合
    //vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
    //Point2f rect[4];
    //int maxIndex = 0, maxArea = 0;
    //for (int i = 0; i < contours.size(); i++)
    //{
    //    int nowArea = cv::contourArea(contours[i]);
    //    if (maxArea < nowArea)
    //    {
    //        maxIndex = i;
    //        maxArea = nowArea;
    //    }
    //}
    //cout << "maxArea：" << maxArea << "总像素" << dst.size() <<  endl;
    //int i = maxIndex;
    //box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
    //boundRect[i] = boundingRect(Mat(contours[i]));
    ////绘制最小外接矩形的中心点
    //box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
    //rectangle(dst, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x +
    //    boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
    //for (int j = 0; j < 4; j++)
    //{
    //    line(dst, rect[j], rect[(j + 1) % 4], Scalar(255, 0, 0), 2, 8);  //绘制最小外接矩形每条边
    //}
    //Eigen::Vector2d xOrient((rect[0] - rect[1]).x, (rect[0] - rect[1]).y);
    //Eigen::Vector2d yOrient((rect[0] - rect[3]).x, (rect[0] - rect[3]).y);
    //cout << "rect[0]：" << rect[0] << "   rect[1]：" << rect[1] << "rect[2]：" << rect[2] << "   rect[3]：" << rect[3] << endl;
    //cout << "xOrient：" << xOrient.normalized().transpose() << "   yOrient：" << yOrient.normalized().transpose() << endl;
    //cv::imshow("2DImage", dst);
    //cv::waitKey(0);
}

std::shared_ptr<open3d::geometry::LineSet> DrawXYZAxisAtOrient(const drawAxisInfo dai) {
    //std::shared_ptr<open3d::geometry::LineSet> ls = std::make_shared<open3d::geometry::LineSet>();
    // 绘制坐标轴位置
    vector<Eigen::Vector3d> vAxisPoint;
    vAxisPoint.emplace_back(dai._center); // 需要画的直线所在点
    vAxisPoint.emplace_back(dai._center + dai._xLen * Eigen::Vector3d(1., 0, 0));
    vAxisPoint.emplace_back(dai._center + dai._yLen * Eigen::Vector3d(0, 1., 0));
    vAxisPoint.emplace_back(dai._center + dai._zLen * Eigen::Vector3d(0, 0, 1.));
    vector<Eigen::Vector2i> vAxisIndex; // 直线点在容器中对应的索引
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 1));
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 2));
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 3));

    return std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
}

std::shared_ptr<open3d::geometry::LineSet> DrawDeviceLinePointCloud(Eigen::Vector3d position, Eigen::Vector3d xOrient, Eigen::Vector3d yOrient, Eigen::Vector3d zOrient) {
    double uppPlainLength = DEVICE_LENGTH / 4, uppPlainWidth = DEVICE_WIDTH / 4;
    //double uppPlainLength = 0., uppPlainWidth = 0.;
    vector<Eigen::Vector3d> vAxisPoint;
    //vAxisPoint.emplace_back(position); // 需要画的直线所在点
    vAxisPoint.emplace_back(position + xOrient * uppPlainLength + yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position + xOrient * uppPlainLength - yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position - xOrient * uppPlainLength + yOrient * uppPlainWidth);
    vAxisPoint.emplace_back(position - xOrient * uppPlainLength - yOrient * uppPlainWidth);
    //vAxisPoint.emplace_back(position + zOrient * plain2cam);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) + xOrient * DEVICE_LENGTH /2 + yOrient * DEVICE_WIDTH /2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) + xOrient * DEVICE_LENGTH/2 - yOrient * DEVICE_WIDTH/2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) - xOrient * DEVICE_LENGTH /2 + yOrient * DEVICE_WIDTH /2);
    vAxisPoint.emplace_back(position + zOrient * (PLAIN2CAMERA /*+ REMAIN_PLAIN2CAMERA*/) - xOrient * DEVICE_LENGTH /2 - yOrient * DEVICE_WIDTH /2);
    vector<Eigen::Vector2i> vAxisIndex; // 直线点在容器中对应的索引
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 1)); // 上平面
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 2));
    vAxisIndex.emplace_back(Eigen::Vector2i(1, 3));
    vAxisIndex.emplace_back(Eigen::Vector2i(2, 3));
    vAxisIndex.emplace_back(Eigen::Vector2i(4, 5)); // 下平面
    vAxisIndex.emplace_back(Eigen::Vector2i(4, 6));
    vAxisIndex.emplace_back(Eigen::Vector2i(5, 7));
    vAxisIndex.emplace_back(Eigen::Vector2i(6, 7)); 
    vAxisIndex.emplace_back(Eigen::Vector2i(0, 4)); // 上、下平面连接线
    vAxisIndex.emplace_back(Eigen::Vector2i(1, 5));
    vAxisIndex.emplace_back(Eigen::Vector2i(2, 6));
    vAxisIndex.emplace_back(Eigen::Vector2i(3, 7));
    //std::shared_ptr<open3d::geometry::LineSet> device_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
    //return device_cloud;
    return std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vAxisPoint, vAxisIndex));
}

vector<Eigen::Vector3d> GetAngleBounding(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, bool hd) {
    /*
    * x -> 相机z方向 z -> 相机x方向
    * 1、A0x + B0y + C0z = cosθ postrueCamZ
    * 2、A1x + B1y + C1z = sinθ postrueCamY
    * 3、A2x + B2y + C2z = 0   postrueCamX
    * 123 --> (A2B0 - A0B2)y + (A2C0 - A0C2)z = A2cosθ
    *         (A2B1 - A1B2)y + (A2C1 - A1C2)z = A2sinθ 
    *         (B2A1 - B1A2)x + (B2C1 - B1C2)z = B2sinθ 
    *  记：b0 = A2B0 - A0B2，c0 = A2C0 - A0C2，d0 = A2cosθ
    *      b1 = A2B1 - A1B2，c1 = A2C1 - A1C2，d1 = A2sinθ
    *      a2 = B2A1 - B1A2，c2 = B2C1 - B1C2，d2 = B2sinθ
    * 4、      b0y + c0z = d0
    * 5、      b1y + c1z = d1
    * 6、      a2x + c2z = d2
    * 456 ---> (c1b0 - c0b1)y = c1d0 - c0d1
    *          (b1c0 - b0c1)z = b1d0 - b0d1
    *  记：tempY = (c1b0 - c0b1)，tempD0 = c1d0 - c0d1
    *      tempZ = (b1c0 - b0c1)，tempD1 = b1d0 - b0d1
    * 7、      tempYy = tmepD0
    * 8、      tempZz = tempD1
    *          a2x = d2 - c2z
    */
    vector<Eigen::Vector3d> v; // 前四个：FOV四条边向量；后四个：FOV四个顶点位置
    Eigen::Vector3d temp = y;
    y = z;
    z = temp;
    Eigen::Vector3d tempX = x;
    Eigen::Vector3d tempY = y;
    Eigen::Vector3d tempZ = z;
    double angle;
    if (hd) angle = HORIZON_ANGLE_HD;
    else angle = HORIZON_ANGLE;
    for (int i = 0; i < 4; i++) {
        if (i == 1) {
            tempY = -y;
            tempZ = z;
            if (hd) angle = HORIZON_ANGLE_HD;
            else angle = HORIZON_ANGLE;
        }
        else if (i == 2) {
            tempY = z;
            tempZ = y;
            if (hd) angle = VERTICAL_ANGLE_HD;
            else angle = VERTICAL_ANGLE;
        }
        else if(i == 3) {
            tempY = -z;
            tempZ = y;
            if (hd) angle = VERTICAL_ANGLE_HD;
            else angle = VERTICAL_ANGLE;
        }
        double A0 = tempX[0], B0 = tempX[1], C0 = tempX[2];
        double A1 = tempY[0], B1 = tempY[1], C1 = tempY[2];
        double A2 = tempZ[0], B2 = tempZ[1], C2 = tempZ[2];
        double b0 = A2 * B0 - A0 * B2, c0 = A2 * C0 - A0 * C2;
        double b1 = A2 * B1 - A1 * B2, c1 = A2 * C1 - A1 * C2;
        double a2 = B2 * A1 - B1 * A2, c2 = B2 * C1 - B1 * C2;
        double d0 = A2 * cos(angle * CV_PI / 180), d1 = A2 * sin(angle * CV_PI / 180), d2 = B2 * sin(angle * CV_PI / 180);
        double tempY = c1 * b0 - c0 * b1;
        double tempZ = b1 * c0 - b0 * c1;
        double tempD0 = c1 * d0 - c0 * d1;
        double tempD1 = b1 * d0 - b0 * d1;
        // double y, z, x;
        //double yComp = abs(tempY) < 1e-3 ? 0 : tempD0 / tempY;
        //double zComp = abs(tempZ) < 1e-3 ? 0 : tempD1 / tempZ;
        double zComp = tempD1 / tempZ;
        //double yComp = tempD0 / tempY;
        double yComp = (d1 - c1 * zComp) / b1;
        double xComp = (d2 - c2 * zComp) / a2;
        v.emplace_back(Eigen::Vector3d(xComp, yComp, zComp).normalized());

#if 0 // 1

    cout << "hor：" << HORIZON_ANGLE << "  ver：" << VERTICAL_ANGLE << endl;
    cout << "x：" << x.transpose() << "  y：" << y.transpose() << "   z：" << z.transpose() << endl;
    cout << "b0：" << b0 << "  c0：" << c0 << "  d0：" << d0 << endl;
    cout << "b1：" << b1 << "  c1：" << c1 << "  d1：" << d1 << endl;
    cout << "a2：" << a2 << "  c2：" << c2 << "  d2：" << d2 << endl;
    cout << "tempY：" << tempY << "  tempD0：" << tempD0 << endl;
    cout << "tempZ：" << tempZ << "  tempD1：" << tempD1 << endl;
    cout << "xComp：" << xComp << "  yComp：" << yComp << "  zComp：" << zComp << endl;

#endif

    }

    // 根据GetAngleBounding四个向量，生成图片四个顶点相对位置
    Eigen::Vector3d xFovOrient; // v[0] / cos(hor * CV_PI / 180)：斜边长，xFovOrient：成像平面x方向
    Eigen::Vector3d yFovOrient; // v[2] / cos(ver * CV_PI / 180)：斜边长，yFovOrient：成像平面y方向
    if (hd) {
        xFovOrient = x - v[0] / cos(HORIZON_ANGLE_HD * CV_PI / 180);
        yFovOrient = x - v[2] / cos(VERTICAL_ANGLE_HD * CV_PI / 180);
    }
    else {
        xFovOrient = x - v[0] / cos(HORIZON_ANGLE * CV_PI / 180);
        yFovOrient = x - v[2] / cos(VERTICAL_ANGLE * CV_PI / 180);
    }
    v.emplace_back(x + xFovOrient + yFovOrient);
    v.emplace_back(x + xFovOrient - yFovOrient);
    v.emplace_back(x - xFovOrient + yFovOrient);
    v.emplace_back(x - xFovOrient - yFovOrient);
    v.emplace_back(xFovOrient.normalized());
    v.emplace_back(yFovOrient.normalized());

    //cout << "cos(hor)：" << cos(hor) << "   sin(hor)：" << sin(hor) << endl;
    //cout << "cos(ver)：" << cos(ver) << "   sin(ver)：" << sin(ver) << endl;
    //cout << "cos(45)：" << cos(45) << "   sin(45)：" << sin(45) << endl;
    //cout << "v[0]：" << v[0].transpose() << "  v[1]：" << v[1].transpose() << endl;
        
    return  v;
}

bool VectorInRange(Eigen::Vector3d vec, Eigen::Vector3d camZOrient)
{
    bool ret = true;

    double angle = acos(vec.transpose() * camZOrient) * 180 / CV_PI;
    if (abs(angle) < abs(EDGE_ANGLE))
    {
        return true;
    }
    else
    {
        return false;
    }

    return ret;
}

///*vector<cv::Mat>*/void PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose, vector<cv::Mat>& vMat)
vector<cv::Mat> PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose) {
    vector<cv::Mat> vMat;

    Eigen::Vector3d positionCam(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3)); // 位置
    Eigen::Vector3d postrueCamZ(pose.at<double>(0, 2), pose.at<double>(1, 2), pose.at<double>(2, 2)); // 朝向
    Eigen::Vector3d postrueCamY(pose.at<double>(0, 1), pose.at<double>(1, 1), pose.at<double>(2, 1)); // 朝向
    Eigen::Vector3d postrueCamX(pose.at<double>(0, 0), pose.at<double>(1, 0), pose.at<double>(2, 0)); // 朝向
    // 给定相机法线方向，根据相机视场角确定视野范围
    // 视角范围vBoundingVector[0]、[1]->长边，vBoundingVector[2]、[3]->短边
    vector<Eigen::Vector3d> vBoundingVector = GetAngleBounding(postrueCamZ, postrueCamY, postrueCamX, 0);
    double edgeAngle = EDGE_ANGLE * 180 / CV_PI; // angleEdge：顶点到相机z方向的夹角
    Eigen::Vector3d planeXAxis = vBoundingVector[8], planeYAxis = vBoundingVector[9];
    //cout << "planeXAxis：" << planeXAxis.transpose() << "planeYAxis：" << planeYAxis.transpose() << endl;
    cv::Mat binImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U, Scalar(0));
    cv::Mat garyImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_64F, Scalar(0.));
    //cv::Mat garyImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U, Scalar(0));
    cv::Mat rgbImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(0, 0, 0));
    // 相对位置
    for (int i = 0; i < pd->points_.size(); i++) {
        // 遍历所有空间点
        Eigen::Vector3d nowVec = (pd->points_[i] - positionCam);
        double zDistance = nowVec.norm() * nowVec.normalized().transpose() * postrueCamZ;
        nowVec = nowVec.normalized(); // 当前相机指向点云当前点
        double angle = acos(nowVec.transpose() * postrueCamZ) * 180 / CV_PI; // 与z轴夹角判断是否在视场范围内
        //if (abs(angle) > abs(EDGE_ANGLE * 180 / CV_PI)) {
        //    // 不在内部
        //    continue;
        //}
        //double xResult = nowVec.transpose() * planeXAxis; // 与x轴夹角判断在x正、反方向 
        //double yResult = nowVec.transpose() * planeYAxis; // 与y轴夹角判断在y正、反方向
        Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance;
        //Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance * sin(angle * CV_PI / 180); // 平面中心处dsin(angle)
        Eigen::Vector3d planeVec = pd->points_[i] - planeOri;
        double xDistance = planeVec.norm() * planeVec.normalized().transpose() * planeXAxis;
        double yDistance = planeVec.norm() * planeVec.normalized().transpose() * planeYAxis; // dcos
        double xAllDistance = zDistance * tan(HORIZON_ANGLE * CV_PI / 180);
        double yAllDistance = zDistance * tan(VERTICAL_ANGLE * CV_PI / 180);
        int xPos = ((xDistance / xAllDistance) / 2.) * IMAGE_WIDTH;
        int yPos = ((yDistance / yAllDistance) / 2.) * IMAGE_HEIGHT;
        if ((abs(xPos) >= IMAGE_WIDTH / 2) || (abs(yPos) >= IMAGE_HEIGHT / 2)) {
            // 下标越界
            continue;
        }
        //if ((pd->points_[i] - Eigen::Vector3d(628.667, -952.119, -917.092)).norm() < 0.5) {
        //    //cout << "测试点坐标为：" << xPos + IMAGE_WIDTH_HD / 2 << "  " << yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "测试点坐标为：" << -xPos + IMAGE_WIDTH / 2 << "  " << -yPos + IMAGE_HEIGHT / 2 << endl;
        //}
        if (binImage.at<uchar>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) == 255) {
            float lastDistance = garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2);
            if (lastDistance > (pd->points_[i] - positionCam).norm()) {
                // 比上次的近-》替换 
                garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = (pd->points_[i] - positionCam).norm(); // 深度
                rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            }
        }
        else {
            rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            garyImage.at<double>(yPos + IMAGE_HEIGHT / 2, xPos + IMAGE_WIDTH / 2) = (pd->points_[i] - positionCam).norm(); // 深度
        }
        binImage.at<uchar>(yPos + (IMAGE_HEIGHT / 2), xPos + (IMAGE_WIDTH / 2)) = (uchar)255;
        ////cout << "points_.size()：" << pcl_ptr->points_.size() << endl;
        ////cout << "xDistance/xAllDistance：" << xDistance / xAllDistance << "   yDistance/yAllDistance：" << yDistance / yAllDistance << endl;
        ////cout << "xPos：" << xPos << "   yPos：" << yPos << endl;
    }
    // 1
    //double maxVal1 = *max_element(vDup[1].begin<double>(), vDup[1].end<double>());
    //double minVal1 = *min_element(vDup[1].begin<double>(), vDup[1].end<double>());
    //std::cout << "最大值：" << maxVal1 << "最小值：" << minVal1 << std::endl;
    // 2
    //double minValue, maxValue;    // 最大值，最小值
    //cv::Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标     
    //cv::minMaxLoc(vDup[1], &minValue, &maxValue, &minIdx, &maxIdx);
    //std::cout << "最大值：" << maxValue << "最小值：" << minValue << std::endl;
    //std::cout << "最大值位置：" << maxIdx << "最小值位置：" << minIdx;
    // 3
    double maxVal = 0., minVal = INT32_MAX;
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            double currVal = garyImage.at<double>(row, col);
            if ((int)garyImage.at<double>(row, col) == 0) continue;
            maxVal = max(maxVal, currVal);
            minVal = min(minVal, currVal);
        }
    }
    cv::Mat gary;
    garyImage.convertTo(gary, CV_8U, 255. / (maxVal - minVal));
    //std::cout << "最大值：" << maxVal << "最小值：" << minVal << std::endl;
    garyImage = gary;

    cv::flip(binImage, binImage, -1); // 图片水平、垂直都是镜像的
    cv::flip(garyImage, garyImage, -1); // 图片水平、垂直都是镜像的
    cv::flip(rgbImage, rgbImage, -1); // 图片水平、垂直都是镜像的

    cv::Mat dst, dst1;
    //cv::Mat kernel1 = cv::getStructuringElement(MORPH_RECT, cv::Size(5, 5));
    cv::Mat kernel1 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    //cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(9, 9));
    //cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    cv::dilate(binImage, dst1, kernel1);
    //cv::erode(dst1, dst, kernel2);

    vMat.emplace_back(binImage);
    vMat.emplace_back(garyImage);
    vMat.emplace_back(rgbImage);
    //vMat.emplace_back(dst);
    vMat.emplace_back(dst1);
    //cv::imshow("rgbImage", rgbImage);
    //cv::waitKey(0);

#if 0 // 1   

    cout << "*******************************************************************点云透视测试：" << endl;

    cout << "fourPostrue.size()：" << pose.size() << endl;
    cout << "fourPostrue：" << pose << endl;
    cout << "positionCam：" << positionCam.transpose() << endl;
    cout << "postrueCamZ：" << postrueCamZ.transpose() << endl;
    cout << "vBoundingVector.size()：" << vBoundingVector.size() << endl;
    cout << "vBoundingVector[0] 与 x轴夹角：" << acos(postrueCamX.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0] 与 y轴夹角：" << acos(postrueCamY.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0] 与 z轴夹角：" << acos(postrueCamZ.transpose() * vBoundingVector[0]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] 与 x轴夹角：" << acos(postrueCamX.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] 与 y轴夹角：" << acos(postrueCamY.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[2] 与 z轴夹角：" << acos(postrueCamZ.transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
    cout << "vBoundingVector[0]：" << vBoundingVector[0].transpose() << endl;
    cout << "vBoundingVector[1]：" << vBoundingVector[1].transpose() << endl;
    cout << "vBoundingVector[2]：" << vBoundingVector[2].transpose() << endl;
    cout << "vBoundingVector[3]：" << vBoundingVector[3].transpose() << endl;
    cout << "HORIZON_ANGLE：" << HORIZON_ANGLE << "  VERTICAL_ANGLE：" << VERTICAL_ANGLE << "   EDGE_ANGLE：" << edgeAngle << endl;
    cout << "************************************************************************************：" << endl;

    vector<Eigen::Vector3d> vDrawLinePoint; // 画线所需点
    vector<Eigen::Vector2i> vDrawLineIndex; // 直线点在容器中对应的索引
    // 边界的4个方向
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + postrueCamX * 50); // 1
    vDrawLinePoint.emplace_back(positionCam + postrueCamY * 100); // 2
    vDrawLinePoint.emplace_back(positionCam + postrueCamZ * 1200);  // 3
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[0] * 1500); // x 4
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[1] * 1500); // -x 5
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[2] * 1000); // y 6
    vDrawLinePoint.emplace_back(positionCam + vBoundingVector[3] * 1000); // -y 7
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 6));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 7));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    // 边界的方形轮廓
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    double zAxisLength = 800;
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[4]); // 1
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[5]); // 2
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[6]); // 3
    vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[7]); // 4
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 2)); //
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 4));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 3));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud2 = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    open3d::visualization::DrawGeometries({ pd, lineSetTemp_cloud, lineSetTemp_cloud2 }, "CamLine");

#endif


    return vMat;
}

vector<cv::Mat> PerspectivePointCloudFromSpecialPoseHD(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose) {
    vector<cv::Mat> vMat;

    Eigen::Vector3d positionCam(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3)); // 位置
    Eigen::Vector3d postrueCamZ(pose.at<double>(0, 2), pose.at<double>(1, 2), pose.at<double>(2, 2)); // 朝向
    Eigen::Vector3d postrueCamY(pose.at<double>(0, 1), pose.at<double>(1, 1), pose.at<double>(2, 1)); // 朝向
    Eigen::Vector3d postrueCamX(pose.at<double>(0, 0), pose.at<double>(1, 0), pose.at<double>(2, 0)); // 朝向
    // 给定相机法线方向，根据相机视场角确定视野范围
    // 视角范围vBoundingVector[0]、[1]->长边，vBoundingVector[2]、[3]->短边
    vector<Eigen::Vector3d> vBoundingVector = GetAngleBounding(postrueCamZ, postrueCamY, postrueCamX, 1);
    Eigen::Vector3d planeXAxis = vBoundingVector[8], planeYAxis = vBoundingVector[9];
    //cout << "planeXAxis：" << planeXAxis.transpose() << "planeYAxis：" << planeYAxis.transpose() << endl;
    cv::Mat binImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, Scalar(0));
    cv::Mat depthImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_64F, Scalar(0.));
    //cv::Mat garyImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8U, Scalar(0));
    cv::Mat rgbImage(IMAGE_HEIGHT_HD, IMAGE_WIDTH_HD, CV_8UC3, Scalar(0, 0, 0));
    // 相对位置
    for (int i = 0; i < pd->points_.size(); i++) {
        // 遍历所有空间点
        Eigen::Vector3d nowVec = (pd->points_[i] - positionCam);
        double zDistance = 0.8 * nowVec.norm() * nowVec.normalized().transpose() * postrueCamZ;
        nowVec = nowVec.normalized(); // 当前相机指向点云当前点
        double angle = acos(nowVec.transpose() * postrueCamZ) * 180 / CV_PI; // 与z轴夹角判断是否在视场范围内
        //if (abs(angle) > abs(EDGE_ANGLE_HD * 180 / CV_PI)) {
        //    // 不在内部 EDGE_ANGLE_HD * 180 / CV_PI：顶点到相机z方向的夹角
        //    continue;
        //}
        //double xResult = nowVec.transpose() * planeXAxis; // 与x轴夹角判断在x正、反方向 
        //double yResult = nowVec.transpose() * planeYAxis; // 与y轴夹角判断在y正、反方向
        //Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance; // 平面中心处dsin(angle)
        Eigen::Vector3d planeOri = positionCam + postrueCamZ * zDistance * sin(angle * CV_PI / 180); // 平面中心处dsin(angle)
        Eigen::Vector3d planeVec = pd->points_[i] - planeOri;
        double xDistance = planeVec.norm() * planeVec.normalized().transpose() * planeXAxis;
        double yDistance = planeVec.norm() * planeVec.normalized().transpose() * planeYAxis; // dcos
        double xAllDistance = zDistance * tan(HORIZON_ANGLE_HD * CV_PI / 180);
        double yAllDistance = zDistance * tan(VERTICAL_ANGLE_HD * CV_PI / 180);
        int xPos = ((xDistance / xAllDistance) / 2.) * IMAGE_WIDTH_HD;
        int yPos = ((yDistance / yAllDistance) / 2.) * IMAGE_HEIGHT_HD;
        if ((abs(xPos) >= IMAGE_WIDTH_HD / 2) || (abs(yPos) >= IMAGE_HEIGHT_HD / 2)) {
            // 下标越界
            continue;
        }

        //if ((pd->points_[i] - Eigen::Vector3d(628.037, -974.233, -940.113)).norm() < 0.5) {
        //    //cout << "测试点坐标为：" << xPos + IMAGE_WIDTH_HD / 2 << "  " << yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "测试点在图片上的坐标为：" << -xPos + IMAGE_WIDTH_HD / 2 << "  " << -yPos + IMAGE_HEIGHT_HD / 2 << endl;
        //    cout << "测试点信息1：" << planeVec.transpose() << "  x" << planeXAxis.transpose() << " y" << planeYAxis.transpose() << " " << postrueCamZ.transpose() << " " << angle << endl;
        //    cout << "测试点信息2：" << zDistance << "  " << xDistance << " " << yDistance << " " << xAllDistance << " " << yAllDistance << endl;
        //    cout << "测试点与y夹角：" << acos((pd->points_[i] - positionCam).normalized().transpose() * vBoundingVector[2]) * 180 / CV_PI << endl;
        //    cout << "测试点与光轴夹角：" << acos((pd->points_[i] - positionCam).normalized().transpose() * postrueCamZ) * 180 / CV_PI << endl;
        //    cout << "y与光轴夹角：" << acos(vBoundingVector[2].transpose() * postrueCamZ) * 180 / CV_PI << endl;
        //}

        if (binImage.at<uchar>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) == 255) {
            double lastDistance = depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2);
            if (lastDistance > (pd->points_[i] - positionCam).norm()) {
                // 比上次的近-》替换 
                depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = (pd->points_[i] - positionCam).norm(); // 深度
                rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            }
        }
        else {
            rgbImage.at<cv::Vec3b>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = cv::Vec3b(255 * pd->colors_[i][0], 255 * pd->colors_[i][1], 255 * pd->colors_[i][2]); // rgb
            depthImage.at<double>(yPos + IMAGE_HEIGHT_HD / 2, xPos + IMAGE_WIDTH_HD / 2) = (pd->points_[i] - positionCam).norm(); // 深度
        }
        binImage.at<uchar>(yPos + (IMAGE_HEIGHT_HD / 2), xPos + (IMAGE_WIDTH_HD / 2)) = (uchar)255;
        ////cout << "points_.size()：" << pcl_ptr->points_.size() << endl;
        ////cout << "xDistance/xAllDistance：" << xDistance / xAllDistance << "   yDistance/yAllDistance：" << yDistance / yAllDistance << endl;
        ////cout << "xPos：" << xPos << "   yPos：" << yPos << endl;
    }
    // 1
    double maxVal1 = *max_element(depthImage.begin<double>(), depthImage.end<double>());
    double minVal1 = *min_element(depthImage.begin<double>(), depthImage.end<double>());
    //std::cout << "最大值：" << maxVal1 << "最小值：" << minVal1 << std::endl;
    // 2
    //double minValue, maxValue;    // 最大值，最小值
    //cv::Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标     
    //cv::minMaxLoc(vDup[1], &minValue, &maxValue, &minIdx, &maxIdx);
    //std::cout << "最大值：" << maxValue << "最小值：" << minValue << std::endl;
    //std::cout << "最大值位置：" << maxIdx << "最小值位置：" << minIdx;
    // 3
    double maxVal = 0., minVal = INT32_MAX;
    for (int row = 0; row < IMAGE_HEIGHT_HD; row++) {
        for (int col = 0; col < IMAGE_WIDTH_HD; col++) {
            double currVal = depthImage.at<double>(row, col);
            if ((int)depthImage.at<double>(row, col) == 0) continue;
            maxVal = max(maxVal, currVal);
            minVal = min(minVal, currVal);
        }
    }
    cv::Mat garyImage;
    depthImage.convertTo(garyImage, CV_8U, 255. / (maxVal - minVal));
    //depthImage.convertTo(gary, CV_8U, 255. / (maxVal1 - minVal1));

    cv::flip(binImage, binImage, -1); // 图片水平、垂直都是镜像的
    cv::flip(depthImage, depthImage, -1); // 图片水平、垂直都是镜像的
    cv::flip(rgbImage, rgbImage, -1); // 图片水平、垂直都是镜像的
    cv::flip(garyImage, garyImage, -1); // 图片水平、垂直都是镜像的

    cv::Mat dst, dst1;
    //cv::Mat kernel1 = cv::getStructuringElement(MORPH_RECT, cv::Size(5, 5));
    cv::Mat kernel1 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    //cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(9, 9));
    //cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    cv::Mat kernel2 = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));
    cv::dilate(binImage, dst1, kernel1);
    cv::erode(dst1, dst, kernel2);

    vMat.emplace_back(binImage);
    vMat.emplace_back(garyImage);
    vMat.emplace_back(rgbImage);
    vMat.emplace_back(dst);
    vMat.emplace_back(depthImage);
    //cv::imshow("rgbImage", rgbImage);
    //cv::waitKey(0);
    
    //vector<Eigen::Vector3d> vDrawLinePoint; // 画线所需点
    //vector<Eigen::Vector2i> vDrawLineIndex; // 直线点在容器中对应的索引
    //// 边界的4个方向
    //vDrawLinePoint.clear();
    //vDrawLineIndex.clear();
    //vDrawLinePoint.emplace_back(positionCam); // 0
    //vDrawLinePoint.emplace_back(positionCam + postrueCamX * 50); // 1
    //vDrawLinePoint.emplace_back(positionCam + postrueCamY * 100); // 2
    //vDrawLinePoint.emplace_back(positionCam + postrueCamZ * 500);  // 3
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[0] * 1500); // x 4
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[1] * 1500); // -x 5
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[2] * 1000); // y 6
    //vDrawLinePoint.emplace_back(positionCam + vBoundingVector[3] * 1000); // -y 7
    //vDrawLinePoint.emplace_back(Eigen::Vector3d(628.037, -974.233, -940.113)); // 测试点
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 6));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 7));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 8)); // 测试点
    //std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //// 边界的方形轮廓
    //vDrawLinePoint.clear();
    //vDrawLineIndex.clear();
    //double zAxisLength = 800;
    //vDrawLinePoint.emplace_back(positionCam); // 0
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[4]); // 1
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[5]); // 2
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[6]); // 3
    //vDrawLinePoint.emplace_back(positionCam + zAxisLength * vBoundingVector[7]); // 4
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 2)); //
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(1, 3));
    //std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud2 = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //// 点云xyz坐标轴
    //std::shared_ptr<open3d::geometry::LineSet> originandaxis_cloud;
    //DrawXYZAxisAtOrient(originandaxis_cloud);
    //open3d::visualization::DrawGeometries({ pd, lineSetTemp_cloud, lineSetTemp_cloud2, originandaxis_cloud,  }, "CamLine");

    return vMat;
}

vector<Eigen::Vector3d> SchdimtOrthogonalityForPoseture(const vector<Eigen::Vector3d>& src) {
    vector<Eigen::Vector3d> dst{ 3 };
    if (src.size() != 3) {
        cout << "错误输入，维度需为3" << endl;
        return dst;
    }
    // 三维顺序：rz ry rx
    dst[0] = src[0];
    //double k = (dst[0].transpose() * src[1]).norm() / (dst[0].norm() * dst[0].norm());
    double k = (dst[0].transpose() * src[1]).norm() / (dst[0].transpose() * dst[0]);
    dst[1] = src[1] - k * dst[0];
    //double k1 = (dst[0].transpose() * src[2]).norm() / (dst[0].norm() * dst[0].norm());
    //double k2 = (dst[1].transpose() * src[2]).norm() / (dst[1].norm() * dst[1].norm());
    double k1 = (dst[0].transpose() * src[2]).norm() / (dst[0].transpose() * dst[0]);
    double k2 = (dst[1].transpose() * src[2]).norm() / (dst[1].transpose() * dst[1]);
    dst[2] = src[2] - k1 * dst[0] - k2 * dst[1];
    
    for (int i = 0; i < 3; i++) {
        dst[i] = dst[i].normalized();
    }

    return dst;
}

controlRobotInfo ConvertMask2CameraPose(const cameraImgInfo& cami, const bboxImgInfo& bbi, int bboxIndex, int con2whichcam) {
    Eigen::Vector3d camPosition = cami._CameraPosition;
    // 中心点
    int goalPixelX = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width / 2;
    int goalPixelY = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height / 2;
    //Eigen::Vector3d cameraGoal = ConvertPixel2Camera(goalPixelX, goalPixelY, depthMean.at<double>(0, 0) / 2, 1);
    Eigen::Vector3d cameraGoal = ConvertPixel2Camera(goalPixelX, goalPixelY, 1, 1);
    cv::Mat pBase = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal[0], cameraGoal[1], cameraGoal[2], 1);
    Eigen::Vector3d pBaseEig{ pBase.at<double>(0,0) ,pBase.at<double>(1,0),pBase.at<double>(2,0) };

    // 左上点
    int goalPixelX0 = bbi._bboxRect[bboxIndex].x;
    int goalPixelY0 = bbi._bboxRect[bboxIndex].y;
    Eigen::Vector3d cameraGoal0 = ConvertPixel2Camera(goalPixelX0, goalPixelY0, 1, 1);
    cv::Mat pBase0 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal0[0], cameraGoal0[1], cameraGoal0[2], 1);
    Eigen::Vector3d pBaseEig0{ pBase0.at<double>(0,0) ,pBase0.at<double>(1,0),pBase0.at<double>(2,0) };

    // 右上点
    int goalPixelX1 = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width;
    int goalPixelY1 = bbi._bboxRect[bboxIndex].y;
    Eigen::Vector3d cameraGoal1 = ConvertPixel2Camera(goalPixelX1, goalPixelY1, 1, 1);
    cv::Mat pBase1 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal1[0], cameraGoal1[1], cameraGoal1[2], 1);
    Eigen::Vector3d pBaseEig1{ pBase1.at<double>(0,0) ,pBase1.at<double>(1,0),pBase1.at<double>(2,0) };

    // 左下点
    int goalPixelX2 = bbi._bboxRect[bboxIndex].x;
    int goalPixelY2 = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height;
    Eigen::Vector3d cameraGoal2 = ConvertPixel2Camera(goalPixelX2, goalPixelY2, 1, 1);
    cv::Mat pBase2 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal2[0], cameraGoal2[1], cameraGoal2[2], 1);
    Eigen::Vector3d pBaseEig2{ pBase2.at<double>(0,0) ,pBase2.at<double>(1,0),pBase2.at<double>(2,0) };

    //// 右下点
    //int goalPixelX3 = bbi._bboxRect[bboxIndex].x + bbi._bboxRect[bboxIndex].width;
    //int goalPixelY3 = bbi._bboxRect[bboxIndex].y + bbi._bboxRect[bboxIndex].height;
    //Eigen::Vector3d cameraGoal3 = ConvertPixel2Camera(goalPixelX3, goalPixelY3, 1, 1);
    //cv::Mat pBase3 = cami._Gripper2Base * H_Camera2Gripper_HD * (cv::Mat_<double>(4, 1) << cameraGoal3[0], cameraGoal3[1], cameraGoal3[2], 1);
    //Eigen::Vector3d pBaseEig3{ pBase3.at<double>(0,0) ,pBase3.at<double>(1,0),pBase3.at<double>(2,0) };
    //cout << "camera position:" << cameraGoal0.transpose() << "  " << cameraGoal1.transpose() << "  " << cameraGoal2.transpose() << endl;

    Eigen::Vector3d positionCam{ cami._CameraPosition };
    vector<Eigen::Vector3d> vDrawLinePoint; // 画线所需点
    vector<Eigen::Vector2i> vDrawLineIndex; // 直线点在容器中对应的索引
    int drawLenght = 1000.;
    // 边界的4个方向
    vDrawLinePoint.clear();
    vDrawLineIndex.clear();
    vDrawLinePoint.emplace_back(positionCam); // 0
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig - positionCam).normalized()); // 1
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig0 - positionCam).normalized()); // 2 左上点
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig1 - positionCam).normalized()); // 3 右上点
    vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig2 - positionCam).normalized()); // 4 左下点
    //vDrawLinePoint.emplace_back(positionCam + drawLenght * (pBaseEig3 - positionCam).normalized()); // 5
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 1));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 2));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(0, 5));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 3));
    vDrawLineIndex.emplace_back(Eigen::Vector2i(2, 4));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(3, 5));
    //vDrawLineIndex.emplace_back(Eigen::Vector2i(4, 5));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vDrawLinePoint, vDrawLineIndex));
    //open3d::visualization::DrawGeometries({ pcl_ptr, lineSetTemp_cloud }, "goal");

    controlRobotInfo cmdInfo;
    // position
    cmdInfo._campositionEigen = cami._CameraPosition;
    cmdInfo._campositionMat = (cv::Mat_<double>(4, 1) << cami._CameraPosition[0], cami._CameraPosition[1], cami._CameraPosition[2], 1);
    Eigen::Vector3d cameraZ = (pBaseEig - positionCam).normalized();
    Eigen::Vector3d cameraX = ((pBaseEig1 - positionCam).normalized() - (pBaseEig0 - positionCam).normalized()).normalized();
    Eigen::Vector3d cameraY = ((pBaseEig2 - positionCam).normalized() - (pBaseEig0 - positionCam).normalized()).normalized();
    //vector<Eigen::Vector3d> vPosture(3);
    //vPosture[0] = cameraZ; vPosture[1] = cameraY; vPosture[0] = cameraX;
    //vector<Eigen::Vector3d> vPosetureSchdimit = SchdimtOrthogonalityForPoseture(vPosture);
    //cameraZ = vPosetureSchdimit[0];
    //cameraY = vPosetureSchdimit[1];
    //cameraX = vPosetureSchdimit[2];

    // posture
    cv::Mat cameraPosture(3, 3, CV_64F, cv::Scalar(0));
    cameraPosture.at<double>(0, 0) = cameraX[0]; cameraPosture.at<double>(1, 0) = cameraX[1];  cameraPosture.at<double>(2, 0) = cameraX[2];
    cameraPosture.at<double>(0, 1) = cameraY[0]; cameraPosture.at<double>(1, 1) = cameraY[1];  cameraPosture.at<double>(2, 1) = cameraY[2];
    cameraPosture.at<double>(0, 2) = cameraZ[0]; cameraPosture.at<double>(1, 2) = cameraZ[1];  cameraPosture.at<double>(2, 2) = cameraZ[2];
    cmdInfo._camposture = cameraPosture.clone();
    cmdInfo._cameuler = RotationMatrixToEulerAngles(cameraPosture);
    // pose
    cv::Mat cameraPose(4, 4, CV_64F, cv::Scalar(0));
    cameraPosture.copyTo(cameraPose({ 0,0,3,3 }));
    cameraPose.at<double>(0, 3) = cmdInfo._campositionEigen[0];
    cameraPose.at<double>(1, 3) = cmdInfo._campositionEigen[1];
    cameraPose.at<double>(2, 3) = cmdInfo._campositionEigen[2];
    cameraPose.at<double>(3, 0) = 0.;
    cameraPose.at<double>(3, 1) = 0.;
    cameraPose.at<double>(3, 2) = 0.;
    cameraPose.at<double>(3, 3) = 1.;
    cmdInfo._campose = cameraPose.clone();
    //cout << "rx:" << cameraX.transpose() << "   ry:" << cameraY.transpose() << "    rz:" << cameraZ.transpose() << endl;
    //cout << "camera pose adjust:" << cameraPosture << endl;
    //// cmd string
    //cout << "image info:" << endl;
    //cout << "   string:" << cami._poseStr << endl;
    //cout << "   position:" << cami._CameraPosition.transpose() << endl;
    //cout << "   pose:" << cami._CameraPose << endl;
    //cout << "   G2B:" << cami._Gripper2Base << endl;

    //cout << "p_gripper:" << (cami._CameraPose * H_Camera2Gripper_HD.inv() * (cv::Mat_<double>(4, 1) << 0, 0, 0, 1)).t() << endl;
    //cv::Mat gripper = cameraPose * H_Camera2Gripper_HD.inv();
    cv::Mat gripper = cameraPose * H_Camera2Gripper.inv();
    string cmdStr;
    ConvertMat2String(gripper, cmdStr);
    cmdInfo._grippercmdStr = cmdStr;
    ////cout << "position:" << cami._CameraPosition.transpose() << endl;
    //cout << "真实坐标：" << cami._poseStr << endl;
    //cout << "调整目标：" << cmdInfo._grippercmdStr << endl;

    //cout << "z:" << cameraZ.transpose() << "    y:" << cameraY.transpose() << "    x:" << cameraX.transpose() 
    //    << "    x*z:" << cameraZ.transpose() * cameraX
    //    << "    y*z:" << cameraZ.transpose() * cameraY 
    //    << "    x*y:" << cameraY.transpose() * cameraX << endl; 
    //cout << "是不是旋转矩阵：" << isRotationMatrix(cameraPose) << endl;
    ////cout << "eluer:" << cmdInfo._euler << endl;
    ////Eigen::Matrix3d cameraPostureEigen;
    //////cameraPostureEigen << cameraX[0], cameraX[1], cameraX[2], 
    //////    cameraY[0], cameraY[1],cameraY[2], cameraZ[0], cameraZ[1], cameraZ[2];
    ////cameraPostureEigen << 
    ////    cameraX[0], cameraY[0], cameraZ[0],
    ////    cameraX[1], cameraY[1], cameraZ[1],
    ////    cameraX[2], cameraY[2], cameraZ[2];
    ////cout << "eluer:" << RotationMatrixToEulerAngles(cameraPostureEigen).transpose() << endl;

    return cmdInfo;
}



int OTSU(const vector<int>& vGray) {
    const int nGrayScale = 256;//灰度
    int nPixelCount[nGrayScale] = { 0 };//灰度直方图
    //统计图片中各个灰度值的个数
    for (int i = 0; i < vGray.size(); i++) {
        int val = vGray[i];
        nPixelCount[val]++;		//int nPixelCount[nGrayScale] = { 0 };//灰度直方图
    }
    //统计图片中各个灰度值所占的比例
    int nPixelSum = vGray.size();//总像素值
    float fPixelPct[nGrayScale] = { 0 };//各个灰度值占总体的比例
    for (int i = 0; i < nGrayScale; ++i) {
        fPixelPct[i] = 1.0 * nPixelCount[i] / nPixelSum;
    }
    double w0, w1;//背景/目标像素占比
    double u0, u1;//目标/背景平均灰度值
    double fTempVar = 0;//类间方差
    double fMaxVar = 0;//最大类间方差
    int fBestValue = 0;//最优阈值
    double fTemp0, fTemp1;
    for (int k = 0; k < nGrayScale; ++k) {
        w0 = w1 = u0 = u1 = fTempVar = 0;
        fTemp0 = fTemp1 = 0;
        //前景，背景区分 [0-k][k+1-255]
        for (int i = 0; i < nGrayScale; ++i) {
            //如果当前像素值小于阈值k则属于背景，反之属于目标
            if (i <= k) {
                //计算背景像素占比
                w0 += fPixelPct[i];
                //计算当前灰度值发生的概率:灰度值*灰度值发生的概率
                fTemp0 += (i * fPixelPct[i]);
            }
            else {
                //计算背景像素占比
                w1 += fPixelPct[i];
                fTemp1 += (i * fPixelPct[i]);
            }
        }
        //计算平均灰度值：p0/w0
        u0 = fTemp0 / w0;
        u1 = fTemp1 / w1;
        //计算类内方差
        fTempVar = (double)(w0 * w1 * pow((u0 - u1), 2));
        if (fTempVar > fMaxVar) {
            fMaxVar = fTempVar;
            fBestValue = k;
        }
    }

    return fBestValue;
}

//void MaskIntersection(const vector<cv::Mat>& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, vector<cv::Mat>& dst) {
void MaskIntersection(const cv::Mat &src, const vector<cv::Mat>&refImg, const vector<cv::Rect>&refRect, cv::Mat & dst) {
    dst = src;
    int area = 0;
    int idx = -1;
    for (int i = 0; i < refRect.size(); i++) {
        // 遍历每个掩膜
        if (refRect[i].width * refRect[i].height > area) {
            area = refRect[i].width * refRect[i].height;
            idx = i;
        }
    }
    if (idx != -1) dst = refImg[idx];

    return;
}

void DealMaskRegion(vector<cv::Mat>& src) {
    static int imageCnt = 0;
    cv::imwrite("D:/temp/0506/gary/" + to_string(imageCnt) + ".jpg", src[1]);
    vector<cv::Mat> vDup{ src };
    cv::Mat gary = vDup[1];
    int totalValidNum = 0;
    vector<int> vCnt(256);
    // 统计灰度分布
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            if ((int)gary.at<uchar>(row, col) == 0) continue;
            vCnt[(int)gary.at<uchar>(row, col)]++;
            totalValidNum++;
        }
    }
    int hisHeight = 400;
    cv::Mat hisImg(hisHeight, 512, CV_8U, Scalar(0));
    // 绘制折线图
    cv::Point lastPoint{0, hisHeight - vCnt[0] > hisHeight ? hisHeight : vCnt[0]};
    for (int i = 0; i < 256; i++) {
        cv::Point currPoint{ i * 2, hisHeight - (vCnt[i] > hisHeight ? hisHeight : vCnt[i]) };
        if (i != 0) cv::line(hisImg, lastPoint, currPoint, Scalar{ 128 });
        lastPoint = currPoint;
    }
    // 保存
    double ratio = 0.95;
    int subTotalValidNum = 0;
    int bestThresh = 0;
    for (int i = 256 - 1; i >= 0; i--) {
        subTotalValidNum += vCnt[i];
        bestThresh = i;
        if (subTotalValidNum >= totalValidNum * (1 - ratio)) break;
    }
    cv::imwrite("D:/temp/0506/hist/" + to_string(imageCnt) + ".jpg", hisImg);

    // 二值化
    cv::Mat binary = gary.clone();
    for (int row = 0; row < IMAGE_HEIGHT; row++) {
        for (int col = 0; col < IMAGE_WIDTH; col++) {
            if ((int)gary.at<uchar>(row, col) == 0 || (int)gary.at<uchar>(row, col) > bestThresh) 
                binary.at<uchar>(row, col) = 0;
            else 
                binary.at<uchar>(row, col) = 255;
        }
    }

    //vector<int> vPixel;
    //for (int row = 0; row < IMAGE_HEIGHT; row++) {
    //    for (int col = 0; col < IMAGE_WIDTH; col++) {
    //        //if ((int)gary.at<uchar>(row, col) == 0) continue;
    //        //cout << (int)gary.at<uchar>(row, col) << "  ";
    //        vPixel.emplace_back((int)gary.at<uchar>(row, col));
    //    }
    //}
    //int bestGray = OTSU(vPixel);
    //cout << "阈值为：" << bestGray << endl;

    //cv::threshold(gary, binary, bestThresh, 255., cv::THRESH_BINARY);
    //cv::imshow("binary", binary);
    //cv::waitKey(0);
    cv::imwrite("D:/temp/0506/binary/" + to_string(imageCnt) + ".jpg", binary);
    imageCnt++;
    src[0] = binary;
    src[3] = binary;
}

void ConvertPoint2VoxelPos(const Eigen::Vector3d p, Eigen::Vector3i& pos) {
    pos = Eigen::Vector3i(0, 0, 0);
    open3d::geometry::AxisAlignedBoundingBox axisBox = pcl_ptr->GetAxisAlignedBoundingBox(); // 点云包围框
    vector<Eigen::Vector3d> vBoundingBox = axisBox.GetBoxPoints();
    double x = axisBox.GetExtent()[0]; // x长度
    double y = axisBox.GetExtent()[1]; // y长度
    double z = axisBox.GetExtent()[2]; // z长度
    // 记录偏差
    double disX = abs(p[0] - vBoundingBox[0][0]); // x偏差 - col
    double disY = abs(p[1] - vBoundingBox[0][1]); // y偏差 - row
    double disZ = abs(p[2] - vBoundingBox[0][2]); // z偏差 - z
    int xVoxelNum = x / VOXEL_SIZE;
    int yVoxelNum = y / VOXEL_SIZE;
    int zVoxelNum = z / VOXEL_SIZE;
    // 第i个点对应图片上的坐标（nowRow、nowCol）
    int voxelX = round((disX * (double)xVoxelNum) / x);
    int voxelY = round((disY * (double)yVoxelNum) / y);
    int voxelZ = round((disZ * (double)zVoxelNum) / z);
    //cout << (disX * (double)xVoxelNum) / x << "四舍五入结果：" << voxelX << endl;
    //int voxelX = (disX * (double)xVoxelNum) / x - 1;
    //int voxelY = (disY * (double)yVoxelNum) / y - 1;
    //int voxelZ = (disZ * (double)zVoxelNum) / z - 1;
    voxelX = voxelX < 0 ? 0 : voxelX;
    voxelY = voxelY < 0 ? 0 : voxelY;
    voxelZ = voxelZ < 0 ? 0 : voxelZ;

    //pos[0] = xVoxelNum - voxelX;
    //pos[1] = yVoxelNum - voxelY;
    pos[0] = voxelX;
    pos[1] = voxelY;
    pos[2] = voxelZ;
    if (p[2] > vBoundingBox[2][2]) {
        // 大于最大上界
        cout << "相机位置大于点云上限！" << endl;
        //return;
    }
    else {
    }

    //std::cout << "当前3D点坐标为：" << p.transpose() << endl;
    //std::cout << "xVoxelNum：" << xVoxelNum << "   yVoxelNum：" << yVoxelNum << " zVoxelNum：" << zVoxelNum << endl;
    //std::cout << "x：" << x << "   y：" << y << " z：" << z << endl;
    //std::cout << "vBoundingBox[0]：" << vBoundingBox[0].transpose() << endl;
    //std::cout << "vBoundingBox[4]：" << vBoundingBox[4].transpose() << endl;
    //std::cout << "disX：" << disX << "   disY：" << disY << " disZ：" << disZ << endl;
    //std::cout << "voxelX：" << voxelX << "   voxelY：" << voxelY << " voxelZ：" << voxelZ << endl;
    //std::cout << "pos：" << pos.transpose() << endl;
    return;
}
