# AMR-AutoScanSystem

## AGV光激励红外三维重构目标检测
此项目通过AGV搭载机械臂以及光激励红外设备，激光导航AGV靠近试件，根据RGBD相机重构三维场景，确定检测试件及其三维模型，根据三维模型划分红外光激励检测设备的运动位姿，控制机械臂运动到指定位姿完成扫查试件；融合高清视觉与红外检测结果，并投影到三维模型中增强显示。<br>


## 代码环境:
+ Open3D_v0.10.0<br>
+ OpenCV3416<br>
+ depth camera：astra SDK<br>
+ mechanical arm：rokae SDK<br>

## 代码流程：
+ 标定红外系统、深度相机和机器人的变换关系<br>
+ 采集实验数据，包含红外图像、深度图像<br>
+ 三维重构+匹配<br>

## 实验结果
+ 原始数据<br>
<img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/1.jpg" width="180" height="105"><img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/2.jpg" width="180" height="105"><img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/1.jpg" width="180" height="105"/>
+ 2D指导3D分割结果<br>
<img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/1.jpg" width="180" height="105"><img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/2.jpg" width="180" height="105"><img src="https://github.com/XkdZy/AMR-AutoScanSystem/blob/main/result/1.jpg" width="180" height="105"/>
