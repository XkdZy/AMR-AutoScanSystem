#include "header.h"

char splitChar = ',';

void IOFile::ReadFile2VString(vector<string>& vStr) {

	fstream fs(_filePath, ios::in);

	char buff[1024];
	splitChar = (_fileType == 1) ? ',' : ' ';
	while (fs.getline(buff, 1024)) {
		string tempStr = string(buff); // 整个字符串
		vStr.emplace_back(tempStr);
	}
	vStr.swap(vStr);

	fs.close();

	return;
}

void IOFile::ReadFile2FloatArr(vector<vector<float>>& vvf) {
	fstream fs(_filePath, ios::in);
	
	vector<float> v;
	char buff[1024];
	string tempStr;
	int index;
	splitChar = (_fileType == 1) ? ',' : ' ';
	while (fs.getline(buff, 1024)) {
		tempStr = string(buff); // 整个字符串
		while (1)  {
			// 读取每一行信息
			index = tempStr.find(splitChar); // 查找字符','
			if (index == -1) { 
				//没找到
				break;
			}
			string substr = tempStr.substr(0, index); // 第一位
			tempStr = tempStr.substr(index + 1);
			v.push_back(stof(substr)); // stof：字符串转浮点数
			if (tempStr.length() == 1) {
				//只含有','
				break;
			}
		}
		vvf.push_back(v);
		v.clear();
	}
	
	fs.close();
}

void VFloat2VInt(vector<float>& vf, vector<int>& vI)
{
	for (vector<float>::iterator it = vf.begin(); it != vf.end(); it++) {
		vI.push_back(int((*it)*10)); // 乘10转换为整数保存
	}
}

void IOFile::WriteVFloatArr2File(vector<double>& vf)
{
	fstream fs(_filePath, ios::out | ios::app);
	string tempStr = "";
	Convert6D2String(vf, tempStr);
	fs << tempStr << endl;
	fs.close();
}

void IOFile::WriteVVFloatArr2File(vector<vector<float>>& vvf)
{
	splitChar = (_fileType == 1) ? ',' : ' ';
	fstream fs(_filePath, ios::out | ios::trunc);
	for (vector<vector<float>>::iterator it = vvf.begin(); it != vvf.end(); it++) {
		for (vector<float>::iterator it1 = (*it).begin(); it1 != (*it).end(); it1++) {
			char printChar[20];
			sprintf_s(printChar, "%lf", *it1);
			fs << printChar << splitChar;
		}
		fs << endl;
	}

	fs.close();
}

string IOFile::TCHAR2STRING(TCHAR* STR)
{
	int iLen = WideCharToMultiByte(CP_ACP, 0, STR, -1, NULL, 0, NULL, NULL);
	char* chRtn = new char[iLen * sizeof(char)];
	WideCharToMultiByte(CP_ACP, 0, STR, -1, chRtn, iLen, NULL, NULL);
	std::string str(chRtn);
	delete chRtn;
	return str;
}

void IOFile::WriteString2File(string& str) {
	fstream fs(_filePath, ios::out | ios::app);

	fs << str << endl;

	fs.close();
}

void IOFile::WriteString2File(string& str, bool trunc) {
	fstream fs;
	if (trunc) {
		fs = fstream(_filePath, ios::out | ios::trunc);
	}
	else {
		fs = fstream(_filePath, ios::out | ios::app);
	}

	fs << str << endl;

	fs.close();

	return;
}

//void IOFile::WriteTrajectory2File(const vector<vertex>& v, bool trunc) {
//	fstream fs;
//	if (trunc) {
//		fs = fstream(_filePath, ios::out | ios::trunc);
//	}
//	else {
//		fs = fstream(_filePath, ios::out | ios::app);
//	}
//	fs << "第i个点	体素编号	占用信息	空间位置	点云位置" << endl;
//	for (int i = 0; i < v.size(); i++) {
//		fs << i << " " <<  v[i].num << "	" << v[i].occupy << "	" << v[i].v.transpose() << "	" << v[i].vPcl.transpose() << endl;
//	}
//	fs.close();
//
//	return;
//}

ThePathDirFile ObtainFileNum(string path, vector<string>& fileName) {
	ThePathDirFile retStr = { 0, 0 };
	struct _finddata64i32_t filefind;
	string curr = path + "\\*.*";
	int   done = 0;
	intptr_t handle;
	int fileNum = 0;
	int dirNum = 0;
	if ((handle = _findfirst(curr.c_str(), &filefind)) == -1) { return retStr; }
	while (!(done = _findnext(handle, &filefind))) {
		//printf("%s\n", filefind.name);
		if (!strcmp(filefind.name, "..")) {
			continue;
		}
		//for (i = 0; i < layer; i++)cout << "     ";
		if ((_A_SUBDIR == filefind.attrib)) {
			//是目录
			//printf("----------%s\n", filefind.name);
			//cout << filefind.name << "(dir)" << endl;
			curr = path + "\\" + filefind.name;
			dirNum += 1;
		}
		else {
			//不是目录，是文件
			//cout << path + "\\" + filefind.name << endl;
			fileName.push_back(filefind.name);
			fileNum += 1;
		}
	}
	_findclose(handle);
	retStr._theDirNum = dirNum;
	retStr._theFileNum = fileNum;

	return retStr;
}

void ClearDirFiles(string dirPath) {
	vector<string> vFileNames;
	ThePathDirFile tpd = ObtainFileNum(dirPath, vFileNames);
	for (int i = 0; i < vFileNames.size(); i++) {
		if (remove((dirPath + vFileNames[i]).c_str())) {
			cout << "删除生成。。。" << endl;
		}
	}

	return;
}

void CreateDir(string dirPath) {
	if (_access(dirPath.c_str(), 0) != 0) {
		_mkdir(dirPath.c_str());
	}
	else {
		cout << "文件夹：" << dirPath << "存在!" << endl;
	}
}

// 按照日期创建文件夹
// 结构：root
//          22_08_28
//              01
//                  01.pcd、01.ply、01Iterated.pcd、01Iterated.ply
//              02
//                  02.pcd、02.ply、02Iterated.pcd、02Iterated.ply
// 返回root\\22_08_28\\01
string CreateDirUseDate(string root)
{
	time_t t;
	struct tm* tmp;
	char dirPath[128];
	time(&t);
	tmp = localtime(&t);
	// 22_08_28
	sprintf_s(dirPath, "%d%d_%02d_%02d", (tmp->tm_year / 10) % 10, tmp->tm_year % 10, tmp->tm_mon + 1, tmp->tm_mday);
	string dateStr = string(dirPath); // 获取日期字符串
	_mkdir((root + "\\" + dateStr).c_str()); // 创建日期文件夹，成功返回0，不成功则返回-1
	memset(dirPath, '\0', sizeof(dirPath)); // 清空
	// 01、02...
	vector<string> vFileName;
	ThePathDirFile tpdStruct = ObtainFileNum(root + "\\" + dateStr, vFileName); // 获取日期文件夹下文件夹数量
	//ThePathDirFile tpdStruct = ObtainFileNum(root + "\\" + dateStr); // 获取日期文件夹下文件夹数量
	int increaseCnt = 0;
	string numStr;
	while (1)
	{
		sprintf_s(dirPath, "%02d", tpdStruct._theDirNum + increaseCnt);
		numStr = string(dirPath); // 获取数量字符串
		int ret = _mkdir((root + "\\" + dateStr + "\\" + numStr).c_str()); // 创建数量文件夹，成功返回0，不成功则返回-1
		if (ret != 0) // 创建不成功
		{
			increaseCnt++;
		}
		else // 创建成功
			break;
	}
	return root + "\\" + dateStr + "\\" + numStr;
}