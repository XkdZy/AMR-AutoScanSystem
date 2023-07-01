#include "header.h"

void RefreshWin()
{
	system("cls");
	cout << "			1：采集数据" << endl;
	cout << "			2：标定相机" << endl;
	cout << "			9：退出" << endl;
	cout << "选择想要的操作：";
}

void MakeDir()
{
	_mkdir("./imgs");
	_mkdir("./imgs/rgbd");
	_mkdir("./imgs/rgbd");
	_mkdir("./imgs/rgbd/color");
	_mkdir("./imgs/rgbd/depth");
}