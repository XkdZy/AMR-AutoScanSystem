#include "header.h"

void RefreshWin()
{
	system("cls");
	cout << "			1���ɼ�����" << endl;
	cout << "			2���궨���" << endl;
	cout << "			9���˳�" << endl;
	cout << "ѡ����Ҫ�Ĳ�����";
}

void MakeDir()
{
	_mkdir("./imgs");
	_mkdir("./imgs/rgbd");
	_mkdir("./imgs/rgbd");
	_mkdir("./imgs/rgbd/color");
	_mkdir("./imgs/rgbd/depth");
}