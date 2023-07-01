#pragma once
#include <iostream>
using namespace std;
#include <string>
#include <vector>
#include <tchar.h>

#include <io.h> // 统计文件夹文件个数ObtainFileNum
#include<cstdio> // remove删除文件夹
//#include "Config.h"

class IOFile
{
public:
	string _filePath;
	int _fileType; // 0:.txt	1:.csv	
	IOFile(string filePath, int fileType = 1)
	{
		_fileType = fileType;
		_filePath = filePath;
	}
	void ReadFile2VString(vector<string>& vStr);
	void ReadFile2FloatArr(vector<vector<float>>& vvf);
	void WriteVFloatArr2File(vector<double>& vf);
	void WriteVVFloatArr2File(vector<vector<float>>& vvf);
	void WriteString2File(string& str);
	void WriteString2File(string& str, bool trunc);
	//void WriteTrajectory2File(const vector<vertex>& v, bool trunc);
	string TCHAR2STRING(TCHAR* STR);
};


struct ThePathDirFile
{
	int _theFileNum;
	int _theDirNum;
};

ThePathDirFile ObtainFileNum(string path, vector<string>& fileName);
void ClearDirFiles(string dirPath);
void CreateDir(string dirPath);
string CreateDirUseDate(string root);
