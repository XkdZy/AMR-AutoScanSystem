#include "header.h"

string QUERY = "query\r\n";
SerialPort sp;
string rokaeIP = "192.168.3.159";
XMateRobot* rokaeRobot = (XMateRobot*)malloc(sizeof(XMateRobot));
error_code ec; 
mutex mutWritePos; // ���̶߳�д���껥����
string sRealTimePos; // ʵʱ��е������

SerialPort::SerialPort(void) {
	m_hComm = INVALID_HANDLE_VALUE;
}

SerialPort::~SerialPort(void) { 
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
		std::cout << "serial close!" << std::endl;
	}
}
/*******************************************************************
* ����     ��  �򿪴���
* COMx     :   ���ں�, ��_T("COM2:")
* BaudRate:    ������
*******************************************************************/
bool SerialPort::Serial_open(LPCSTR COMx, int BaudRate) {
	DCB dcb = { 0 };
	m_hComm = CreateFileA(COMx,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		0,//FILE_FLAG_OVERLAPPED,	//ͬ����ʽ �� �ص���ʽ
		0
	);

	if (m_hComm == INVALID_HANDLE_VALUE) {
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(m_hComm, &dcb)) {
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.BaudRate = BaudRate;	//������
	dcb.ByteSize = 8;			//λ��
	dcb.Parity = NOPARITY;		//��ż����
	dcb.StopBits = ONESTOPBIT;	//ֹͣλ��

	if (!SetCommState(m_hComm, &dcb)) {
		DWORD dwError = GetLastError();
		return FALSE;
	}
	if (!PurgeComm(m_hComm, PURGE_RXCLEAR))    return -1;

	SetupComm(m_hComm, 1024, 1024);
	return TRUE;
}

/**
  serial write
  @param Buf:data buf
  @param size:bytes of Buf
  @return The len of writen
*/
int SerialPort::Serial_write_string(string& Buf, int size) {
	DWORD dw;
	char charBuf[1024];
	strcpy_s(charBuf, Buf.c_str());
	WriteFile(m_hComm, charBuf, size, &dw, NULL);
	return dw;
}
int SerialPort::Serial_write_char(unsigned char* Buf, int size)
{
	DWORD dw;
	WriteFile(m_hComm, Buf, size, &dw, NULL);
	return dw;
}
/**
  serial read
  @param OutBuf:���صõ�������
  @param maxSize:������������
  @return The len of read data
*/
int SerialPort::Serial_read_string(string &OutBuf, int maxSize) {
	DWORD readSize = 0;
	DWORD dwError  = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize) {
		PurgeComm(m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	char OutBufchar[100]="";
	//memset(OutBufchar, 0, sizeof(OutBuf));
	bReadStatus=ReadFile(m_hComm, OutBufchar, cs.cbInQue, &readSize, 0);

	OutBuf = OutBufchar;

	return readSize;
}

int SerialPort::Serial_read_char(unsigned char* OutBuf, int maxSize) {
	DWORD readSize = 0;
	DWORD dwError = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize) {
		PurgeComm(m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	bReadStatus = ReadFile(m_hComm, OutBuf, cs.cbInQue, &readSize, 0);
	OutBuf[readSize] = '\0';

	return readSize;
}

/*
	2022.05.19��ȡ��е�۵�ǰλ�á�������е������
*/
bool AnaysisPosData(string& PosData, vector<float>& vPos) {
	//// �����Ƿ����	len("A+1234+1234+1234+123+123+123BC") = 30
	//if (PosData.length() != 30)
	//{
	//	return false;
	//}
	//// ֡ͷ��֡β����
	//if ((PosData[0] == 'A') && (PosData[28] == 'B') && (PosData[29] == 'C'))
	//{
	//	return false;
	//}
	// ��������
	float temp;
	temp = (PosData[2] - '0') * 100 + (PosData[3] - '0') * 10 + (PosData[4] - '0') + (PosData[5] - '0') / 10.0; // x
	if (PosData[1] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[7] - '0') * 100 + (PosData[8] - '0') * 10 + (PosData[9] - '0') + (PosData[10] - '0') / 10.0; // y
	if (PosData[6] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[12] - '0') * 100 + (PosData[13] - '0') * 10 + (PosData[14] - '0') + (PosData[15] - '0') / 10.0; // z
	if (PosData[11] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[17] - '0') * 100 + (PosData[18] - '0') * 10 + (PosData[19] - '0'); // rx
	if (PosData[16] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[21] - '0') * 100 + (PosData[22] - '0') * 10 + (PosData[23] - '0'); // ry
	if (PosData[20] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[25] - '0') * 100 + (PosData[26] - '0') * 10 + (PosData[27] - '0'); // rz
	if (PosData[24] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);

	return true;
}

bool SerialPort::ReadRobotArmPos(SerialPort& sp, vector<float>& vPos) {
#if ROBOT_TYPE == 0
	
	int queryCnt = 0;
	string readBuff;

	// ���Ͳ�ѯָ��
	sp.Serial_write_string(QUERY, QUERY.size());
	std::cout << "��ѯ�ѷ��͡�����" << endl;
	//Sleep(100);
	Sleep(2);
	// ��ȡ����
	while (1)
	{
		if (queryCnt >= 1000) { // ��ֹ��һ�ν��ղ������ݣ�û1000��û���յ����ݾ��ط���ѯquery\r\nָ��
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()����size_of(QUERY)
			std::cout << "��ѯ�ѷ��͡�����" << endl;
			queryCnt = 0;
			Sleep(2);
		}
		std::cout << "��ѯ�С�����" << endl;
		// ��ȡ����
		int len = sp.Serial_read_string(readBuff, 100);
		if (len) {
			// ��ȡ������
			bool ret = AnaysisPosData(readBuff, vPos);
			break;
		}
		queryCnt++;
	}

#else if ROBOT_TYPE == 1

	Arr6 pose = rokaeRobot->flangePos(ec);
	vPos = vector<float>(6, 0.0);
	cout << "��ǰ����Ϊ��" << endl;
	for (int i = 0; i < 3; i++) {
		vPos[i] = pose[i] * 1000;
		//cout << vPos[i] << "	";
	}
	for (int i = 3; i < 6; i++) {
		vPos[i] = pose[i] * 180 / CV_PI;
		//cout << vPos[i] << "	";
	}
	return true;

#endif
}

void SerialPort::ReadRobotArmPosString(SerialPort& sp, string& strPos) {
#if ROBOT_TYPE == 0
	int queryCnt = 0;
	string readBuff;

	// ���Ͳ�ѯָ��
	sp.Serial_write_string(QUERY, QUERY.size());
	//cout << "��ѯ�ѷ��͡�����" << endl;
	//Sleep(100);
	Sleep(2);
	// ��ȡ����
	while (1)
	{
		if (queryCnt >= 1000) // ��ֹ��һ�ν��ղ������ݣ�û1000��û���յ����ݾ��ط���ѯquery\r\nָ��
		{
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()����size_of(QUERY)
			//cout << "��ѯ�ѷ��͡�����" << endl;
			queryCnt = 0;
			//Sleep(100);
			Sleep(2);
		}
		//cout << "��ѯ�С�����" << endl;
		// ��ȡ����
		int len = sp.Serial_read_string(readBuff, 100);
		if (len)
		{
			strPos = readBuff;
			break;
		}
		queryCnt++;
	}
#else if ROBOT_TYPE == 1
	Arr6 pose = rokaeRobot->flangePos(ec);
	strPos = "A";
	vector<int> vPosInfo(6, 0);
	for (int i = 0; i < 3; i++) {
		//cout << pose[i] << "	";
		char temp[9];
		vPosInfo[i] = pose[i] * 100000;
		vector<int> vBit(6);
		vBit[0] = vPosInfo[i] / 100000;
		vBit[1] = vPosInfo[i] / 10000 % 10;
		vBit[2] = vPosInfo[i] / 1000 % 10;
		vBit[3] = vPosInfo[i] / 100 % 10;
		vBit[4] = vPosInfo[i] / 10 % 10;
		vBit[5] = vPosInfo[i] % 10;
		if (vPosInfo[i] > 0) strPos += "+";
		else strPos += "-";
		strPos += to_string(abs(vBit[0]));
		strPos += to_string(abs(vBit[1]));
		strPos += to_string(abs(vBit[2]));
		strPos += to_string(abs(vBit[3]));
		strPos += to_string(abs(vBit[4]));
		strPos += to_string(abs(vBit[5]));
		//strPos.append(string(temp));
		//cout << vPosInfo[i] << "	";
	}
	for (int i = 3; i < 6; i++) {
		//cout << pose[i] << "	";
		char temp[9];
		// 08001
		vPosInfo[i] = pose[i] * 18000. / CV_PI;
		//cout << vPosInfo[i] << "	";
		vector<int> vBit(5);
		vBit[0] = vPosInfo[i] / 10000;
		vBit[1] = vPosInfo[i] / 1000 % 10;
		vBit[2] = vPosInfo[i] / 100 % 10;
		vBit[3] = vPosInfo[i] / 10 % 10;
		vBit[4] = vPosInfo[i] / 1 % 10;
		if (vPosInfo[i] > 0) strPos += "+";
		else strPos += "-";
		strPos += to_string(abs(vBit[0]));
		strPos += to_string(abs(vBit[1]));
		strPos += to_string(abs(vBit[2]));
		strPos += to_string(abs(vBit[3]));
		strPos += to_string(abs(vBit[4]));
	}
	strPos.append("BC");
#endif
	//cout << "��ǰ����Ϊ��" << strPos << endl;
	return;
}

//void MoveToOnePointMultiThread(string point, SerialPort* sp) {
//	thread readThread(MoveToOnePoint, point, &sp);
//	readThread.detach();
//	cout << "��ȡ�����������߳�������..." << endl;
//}

void MoveToOnePoint(string point, SerialPort* sp) {
#if ROBOT_TYPE == 0
	point += "\r\n";
	sp->Serial_write_string(point, 32);
	Sleep(2);
	char spdChar[5];
	sprintf_s(spdChar, "%04d", 400);
	string spdStr = "DL" + string(spdChar) + "E\r\n";
	sp->Serial_write_string(spdStr, spdStr.size());
	Sleep(2);
	while (!WhetherRobotArmMoveToPoint(point));
	//move2CurrTarget = true;
#else if ROBOT_TYPE == 1

	vector<double> v;
	AnalysisString26D(point, v);
	for (int i = 0; i < 3; i++) {
		v[i] = v[i] / 1000;
	}
	for (int i = 3; i < 6; i++) {
		v[i] = v[i] * CV_PI / 180.;
	}
	//cout << v[0] << "	" << v[1] << "	" << v[2] << "	"
	//	<< v[3] << "	" << v[4] << "	" << v[5] << "	" << endl;
	MoveRokae2OnePoint(v, 0);
	//MoveRokae2OnePoint(v, 1);
	//cout << "point:" << point << endl;
	//bool ret = WhetherRobotArmMoveToPoint(point);
	//if (!ret) {
	//	//// δ����
	//	//cout << "_---------------------------------------_" << endl;
	//	MoveRokae2OnePoint(v, 1);
	//}
	//move2CurrTarget = true;
	//vector<vector<double>> vv(4, vector<double>(6));
	//for (int i = 0; i < vv.size(); i++) {
	//	vv[i] = vDst;
	//}
	//MoveRokae2MultiPoint(vv);

#endif
	Sleep(500);

	return;
}

void WaitRokaeRobot(BaseRobot* robot) {
	bool running = true;
	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//error_code ec;
		auto st = robot->operationState(ec);
		if (st == OperationState::idle || st == OperationState::unknown) {
			running = false;
		}
	}
}

void StopRokaeRobot() {
	rokaeRobot->stop(ec); // ֹͣ�˶�
	rokaeRobot->setPowerState(false, ec); // �Զ�ģʽ���ϵ�
	//std::this_thread::sleep_for(std::chrono::seconds(2)); //�ȴ��л�����ģʽ
	std::this_thread::sleep_for(std::chrono::milliseconds(5)); //�ȴ��л�����ģʽ
}

void MoveRokae2OnePoint(const vector<double>& pos, INT8 moveMode) {
	rokaeRobot->setMotionControlMode(MotionControlMode::NrtCommand);
	rokaeRobot->setOperateMode(OperateMode::automatic, ec); // �л����Զ�ģʽ
	rokaeRobot->setPowerState(true, ec); // �Զ�ģʽ���ϵ�
	//std::this_thread::sleep_for(std::chrono::seconds(2)); //�ȴ��л�����ģʽ
	std::this_thread::sleep_for(std::chrono::milliseconds(5)); //�ȴ��л�����ģʽ
	rokaeRobot->moveReset(ec); // ���˶�ָ��ǰ�������û���
	if (moveMode == 0) {
		//cout << "pose:" << pos[0] << "	" << pos[1] << "	" << pos[2] << "	"
		//	<< pos[3] << "	" << pos[4] << "	" << pos[5] << "	" << endl;
		rokae::MoveLCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] }, 1000, 100);
		//rokae::MoveLCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] });
		rokaeRobot->append({ movel0 }, ec); // ����ָ������˿�ʼ�˶�
	}
	else {
		rokae::MoveJCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] }, 100, 100);
		rokaeRobot->append({ movel0 }, ec); // ����ָ������˿�ʼ�˶�
	}
	WaitRokaeRobot(rokaeRobot);
	//std::cout << "��һ��������Ϣ: " << rokaeRobot->lastErrorCode().message() << std::endl;

	return;
}

void MoveRokae2MultiPoint(const vector<vector<double>>& pos) {
	rokaeRobot->setMotionControlMode(MotionControlMode::NrtCommand);
	rokaeRobot->setOperateMode(OperateMode::automatic, ec); // �л����Զ�ģʽ
	rokaeRobot->setPowerState(true, ec); // �Զ�ģʽ���ϵ�
	//std::this_thread::sleep_for(std::chrono::seconds(2)); //�ȴ��л�����ģʽ
	std::this_thread::sleep_for(std::chrono::milliseconds(5)); //�ȴ��л�����ģʽ
	rokaeRobot->moveReset(ec); // ���˶�ָ��ǰ�������û���
	//vector<rokae::MoveLCommand> vCommand(pos.size());
	//for (int i = 0; i < pos.size(); i++) {
	//    vCommand[i] = rokae::MoveLCommand({ pos[i][0], pos[i][1], pos[i][2], pos[i][3], pos[i][4], pos[i][5] }, 2000, 100);
	//}
	for (int i = 0; i < pos.size(); i++) {
		rokae::MoveLCommand movel0({ pos[i][0], pos[i][1], pos[i][2], pos[i][3], pos[i][4], pos[i][5] }, 100, 100);
		rokaeRobot->append({ movel0 }, ec); // ����ָ������˿�ʼ�˶�
		WaitRokaeRobot(rokaeRobot);
	}
	//std::cout << "��һ��������Ϣ: " << rokaeRobot->lastErrorCode().message() << std::endl;

	return;
}

/*
* ��ȡ�˶�������
* ���룺
*       fHandle���ļ��������
*       ch����ά�����׵�ַ������˶�������
*       length����ά����Ŀ��
* �����
*       index���˶���ĸ���
* ʾ����
*		fstream fHandle1("../test.txt", ios::out | ios::in);
*       char buff[20][100] = { 0 };
*       int ret = ReadPointFile(&fHandle1, (char*)buff, 100);
*		fHandle1.close();
*/
int ReadPointFile(fstream* fHandle, char* ch, int length) {
	int index = 0;
	char tempbuff[100];
	while (1) {
		fHandle->getline(tempbuff, 100);
		if (tempbuff[0] == '\0') {
			return index;
		}
		else { // ��ȡ����
			for (int i = 0; i < length; i++){
				if (tempbuff[i] != '\0'){
					ch[i + index * length] = tempbuff[i];
				}
				else{
					break;
				}
			}
		}
		index++;
	}
}

void AnalysisString26D(const string& str, vector<double>& v) {
	v = vector<double>(6, 0.0);
#if ROBOT_TYPE == 0
	// 1+5+5+5+4+4+4+2
	// A+0794-4467+3576+179-027+179BC ---> 79.4 -446.7 357.6 179 -027 179
	for (int i = 0; i < 3; i++) {
		string temp = str.substr(1 + i * 5, 5);
		int val = atoi(temp.c_str());
		v[i] = val / 10.;
		//cout << "��ǰ�ִ�Ϊ��" << temp << "ת������ֵΪ��" << v[i] << endl;
	}
	for (int i = 3; i < 6; i++) {
		string temp = str.substr(16 + (i - 3) * 4, 4);
		int val = atoi(temp.c_str());
		v[i] = val;
		//cout << "��ǰ�ִ�Ϊ��" << temp << "ת������ֵΪ��" << v[i] << endl;
	}
#else
	// 1+7+7+7+6+6+6+2=42
	// A+079400-044670+035760+17900-02700+17900BC ---> 79.4 -446.7 357.6 179 -027 179
	//cout << str << endl;
	for (int i = 0; i < 3; i++) {
		string temp = str.substr(1 + i * 7, 7);
		int val = atoi(temp.c_str());
		v[i] = val / 100.;
		//cout << "��ǰ�ִ�Ϊ��" << temp << "ת������ֵΪ��" << v[i] << endl;
	}
	for (int i = 3; i < 6; i++) {
		string temp = str.substr(22 + (i - 3) * 6, 6);
		int val = atoi(temp.c_str());
		v[i] = val / 100.;
		//cout << "��ǰ�ִ�Ϊ��" << temp << "ת������ֵΪ��" << v[i] << endl;
	}
#endif 
}
void AnalysisString26D(const string& str, cv::Mat& pose) {
	vector<double> v;
	AnalysisString26D(str, v);
	pose = cv::Mat(1, 6, CV_64F, 0.0);
	pose.at<double>(0, 0) = v[0];
	pose.at<double>(0, 1) = v[1];
	pose.at<double>(0, 2) = v[2];
	pose.at<double>(0, 3) = v[3];
	pose.at<double>(0, 4) = v[4];
	pose.at<double>(0, 5) = v[5];
}

void ConvertMat2String(const cv::Mat& mat, string& str) {
	vector<double> v(6, 0.0);

	// λ��
	v[0] = mat.at<double>(0, 3);
	v[1] = mat.at<double>(1, 3);
	v[2] = mat.at<double>(2, 3);
	// ��̬
	cv::Mat rotate = mat({ 0,0,3,3 });
	Eigen::Vector3d euler = RotationMatrixToEulerAngles(rotate);
	v[3] = euler[0];
	v[4] = euler[1];
	v[5] = euler[2];
	Convert6D2String(v, str);

	return;
}

void Convert6D2String(const vector<double>& v, string& str) {
	vector<int> vI;
#if ROBOT_TYPE == 0
	for (int i = 0; i < v.size(); i++) {
		if (i < 3) {
			// ǰ��λ��10
			vI.push_back(int(v[i] * 10)); // ��10ת��Ϊ��������
		}
		else {
			vI.push_back(int(v[i])); // ��10ת��Ϊ��������
		}
	}
	char printChar[32];
	sprintf_s(printChar, "A");
	string tempStr = "A";
	for (int i = 0; i < vI.size(); i++) {
		if (vI[i] > 0) {
			// ����0��+��
			tempStr += "+";
			if (i < 3) {
				sprintf_s(printChar, "%04d", vI[i]);
				tempStr += string(printChar);
			}
			else {
				sprintf_s(printChar, "%03d", vI[i]);
				tempStr += string(printChar);
			}
		}
		else {
			tempStr += "-";
			if (i < 3) {
				sprintf_s(printChar, "%04d", -vI[i]);
				tempStr += string(printChar);
			}
			else {
				sprintf_s(printChar, "%03d", -vI[i]);
				tempStr += string(printChar);
			}
		}
	}
#else
	for (int i = 0; i < v.size(); i++) {
		vI.push_back(int(v[i] * 100)); // ��10ת��Ϊ��������
	}
	char printChar[44];
	sprintf_s(printChar, "A");
	string tempStr = "A";
	for (int i = 0; i < vI.size(); i++) {
		if (vI[i] > 0) {
			// ����0��+��
			tempStr += "+";
			if (i < 3) {
				sprintf_s(printChar, "%06d", vI[i]);
				tempStr += string(printChar);
			}
			else {
				sprintf_s(printChar, "%05d", vI[i]);
				tempStr += string(printChar);
			}
		}
		else {
			tempStr += "-";
			if (i < 3) {
				sprintf_s(printChar, "%06d", -vI[i]);
				tempStr += string(printChar);
			}
			else {
				sprintf_s(printChar, "%05d", -vI[i]);
				tempStr += string(printChar);
			}
		}
	}
#endif
	tempStr += "BC";
	str = tempStr;
}

bool WhetherRobotArmMoveToPoint(string dstPoint) {
#if ROBOT_TYPE == 0
	vector<double> vDstPoint;
	AnalysisString26D(dstPoint, vDstPoint); // ��6D�ַ���ת��Ϊ6D����
	string str = "";
	sp.ReadRobotArmPosString(sp, str); // ȫ���ַ���
	if (str.length() < 30) return false;
	//cout << "��ǰ������λ��Ϊ��" << str << endl;
	// ����ʵʱ����
	if (USE_MULTITHREAD) {
		// ���߳��ǣ�д������ʱ��������ֹ�����̶߳�ȡ
		mutWritePos.lock();
		sRealTimePos = str;
		mutWritePos.unlock();
		Sleep(2);
	}
	int cIndex = str.find('C'); // ��ȡA...BC��30���ַ�
	if (cIndex < 29) return false;
	str = str.substr(cIndex - 29, 30);
	vector<double> vnowPoint;
	//cout << "-------------" << str << endl;
	AnalysisString26D(str, vnowPoint); // ��6D�ַ���ת��Ϊ6D����
	double diff = sqrt(
		pow(vDstPoint[0] - vnowPoint[0], 2) + pow(vDstPoint[1] - vnowPoint[1], 2) +
		pow(vDstPoint[2] - vnowPoint[2], 2) + pow(vDstPoint[3] - vnowPoint[3], 2) +
		pow(vDstPoint[4] - vnowPoint[4], 2) + pow(vDstPoint[5] - vnowPoint[5], 2)
	); // �������
	//cout << "nowdiff:" << diff << endl;
	return (diff < 0.01) ? true : false; // ����С����ֵ������
#else if ROBOT_TYPE == 1
	vector<double> vDstPoint;
	AnalysisString26D(dstPoint, vDstPoint); // ��6D�ַ���ת��Ϊ6D����
	string str = "";
	sp.ReadRobotArmPosString(sp, str); // ȫ���ַ���
	//cout << str << endl;
	if (str == "" || str.length() < 42) {
		return false;
	}
	// ����ʵʱ����
	if (USE_MULTITHREAD) {
		// ���߳��ǣ�д������ʱ��������ֹ�����̶߳�ȡ
		mutWritePos.lock();
		sRealTimePos = str;
		mutWritePos.unlock();
		Sleep(1);
	}
	int cIndex = str.find('C'); // ��ȡA...BC��42���ַ�
	str = str.substr(cIndex - 41, 42);
	vector<double> vnowPoint;
	//cout << "-------------" << str << endl;
	AnalysisString26D(str, vnowPoint); // ��6D�ַ���ת��Ϊ6D����
	double diff = sqrt(
		pow(vDstPoint[0] - vnowPoint[0], 2) + pow(vDstPoint[1] - vnowPoint[1], 2) +
		pow(vDstPoint[2] - vnowPoint[2], 2) + pow(vDstPoint[3] - vnowPoint[3], 2) +
		pow(vDstPoint[4] - vnowPoint[4], 2) + pow(vDstPoint[5] - vnowPoint[5], 2)
	); // �������
	//cout << "nowdiff:" << diff << endl;
	return (diff < 2.) ? true : false; // ����С����ֵ������
#endif
}

/**
 * ���ܣ� ͨ����������ת��������Ӧ��ŷ����
 **/
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat & R) {
	assert(isRotationMatrix(R));
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6;
	double x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	x = x / CV_PI * 180.0;
	y = y / CV_PI * 180.0;
	z = z / CV_PI * 180.0;
	return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R) {
	Eigen::Vector3d n = R.col(0);
	Eigen::Vector3d o = R.col(1);
	Eigen::Vector3d a = R.col(2);

	Eigen::Vector3d ypr(3);
	double y = atan2(n(1), n(0));
	double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
	double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
	ypr(0) = y;
	ypr(1) = p;
	ypr(2) = r;

	return ypr / CV_PI * 180.0;
}

