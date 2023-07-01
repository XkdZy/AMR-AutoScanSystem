#include "Seer.h"
#include "header.h"

char buffersss[50048];

SOCKET m_SockClient10;//端口19210  其他API DO输出  抱闸打开
SOCKET m_SockClient04;//端口19204  查询
SOCKET m_SockClient06;//端口19206  导航

void delay_ms(int ms,int x)
{
    clock_t start = clock();
    while (clock() - start < ms);
}

void InitInternet() {
    WSADATA wsd; // 初始化网络环境
    if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) { //使用2.2版本的Socket
        cout << "INITIAL  FAILED!" << endl;
        system("pause");
    }
    return;
}

bool ConnectSeer(const u_short port, SOCKET& client) {
    sockaddr_in clientaddr;
    clientaddr.sin_family = AF_INET;//设置服务器地址	家族
    clientaddr.sin_port = htons(port);//设置服务器端口号	
    clientaddr.sin_addr.S_un.S_addr = inet_addr(SEERCONTROLIP);
    client = socket(AF_INET, SOCK_STREAM, 0);
    int result = connect(client, (sockaddr*)&clientaddr, sizeof(clientaddr));//连接超时 
    return result == 0;
}

char* RobotState2SeerFrame(const uint16_t m_number, const uint16_t m_type) {
    char* s_data = new char[16];
    uint8_t m_header = 0x5A;
    uint8_t m_version = 0x01;
    sprintf(
        s_data, 
        "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
        m_header,
        m_version,
        (m_number / 256) % 256, m_number % 256,
        0, 0, 0, 0,
        (m_type / 256) % 256, m_type % 256,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    );
    return s_data;
}

char* RobotNavigationJSON2SeerFrame(const uint16_t m_number, const uint16_t m_type, const char* json) {
        char* s_date = new char[1000];
        uint8_t m_header = 0x5A;
        uint8_t m_version = 0x01;
        uint8_t m_reserved[6] = { 0 };
        uint32_t m_length = 0;
        
        m_length = strlen(json);
        sprintf(
            s_date, 
            "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%s",
            m_header,
            m_version,
            (m_number / 256) % 256, m_number % 256,
            (m_length / 256 / 256 / 256) % 256, (m_length / 256 / 256) % 256, (m_length / 256) % 256, m_length % 256,
            (m_type / 256) % 256, m_type % 256,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            json
        );
        return s_date;
}

void AMRUnlock(const double id, const bool status, SOCKET& client) {
    cJSON* root = cJSON_CreateObject();//json数据段
    ////创建数据
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddBoolToObject(root, "status", status);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(6001, 6001, json_data);

    send(client, s_data, strlen(json_data)+16, 0);

    return;
}


void AMRLibration(const double dist, const double vx, const double vy, SOCKET& client, const bool mode) {
    cJSON* root = cJSON_CreateObject();//json数据段
    ////创建数据
    cJSON_AddNumberToObject(root, "dist", dist);
    cJSON_AddNumberToObject(root, "vx", vx);
    cJSON_AddNumberToObject(root, "vy", vy);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3005, 3055, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return;
}

void AMRRotation(const double angle, const double vw, SOCKET& client) {
    AMRLocalInfo amrliStart;
    RequestAMRLocal(amrliStart);
    //cout << amrliStart._x << "   " << amrliStart._y << "  " << amrliStart._angle << endl;
    AMRLocalInfo amrliGoal = amrliStart;
    amrliGoal._angle += vw > 0 ? angle : -angle;

    cJSON* root = cJSON_CreateObject();//json数据段
    ////创建数据
    cJSON_AddNumberToObject(root, "angle", angle);
    cJSON_AddNumberToObject(root, "vw", vw);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3056, 3056, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    AMRLocalInfo curr = amrliGoal;
    while (1) {
        if (WhetherAMRMove2Goal(m_SockClient04, amrliGoal, curr, 2)) break;
        RequestAMRLocal(curr);
        Sleep(100);
    }

    return;
}
bool WhetherAMRMove2Goal(const SOCKET& client, AMRLocalInfo& goal, AMRLocalInfo& last, int which) {
    AMRLocalInfo curr;
    RequestAMRLocal(curr);

    //cout << goal._angle << "    " << curr._angle << "    " << curr._angle << endl;
    if (which == 2) {
        //return abs(curr._angle - goal._angle) < 0.01 || abs(curr._angle - last._angle) < 1.e-8;
        return abs(curr._angle - last._angle) < 1.e-8;
    }
    else if (which == 1) {
        return abs(curr._y - goal._y) < 0.01 || abs(curr._y - last._y) < 1.e-8;
    }
    else {
        return abs(curr._x - goal._x) < 0.01 || abs(curr._x - last._x) < 1.e-8;
    }
}


void AMRRobotNavigation(const double x, const double y, const double theta, SOCKET& client) {
    // 定义对象 { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "coordinate", cJSON_CreateString("world"));
    cJSON_AddItemToObject(script_args, "theta", cJSON_CreateNumber(theta));
    cJSON* root = cJSON_CreateObject();//json数据段
    cJSON_AddItemToObject(root, "script_name", cJSON_CreateString("syspy/goPath.py"));
    cJSON_AddItemToObject(root, "script_args", script_args);
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("Script"));
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "source_id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "task_id", cJSON_CreateString("12344321"));
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return;
}


void AMRRobotNavigation(const double x, const double y, SOCKET& client) {
    // 定义对象 { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "coordinate", cJSON_CreateString("world"));
    cJSON* root = cJSON_CreateObject();//json数据段
    cJSON_AddItemToObject(root, "script_name", cJSON_CreateString("syspy/goPath.py"));
    cJSON_AddItemToObject(root, "script_args", script_args);
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("Script"));
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "source_id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "task_id", cJSON_CreateString("12344321"));
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return;
}


void AMRgofree(const double x, const double y, const double theta, SOCKET& client) {
    // 定义对象 { }
    cJSON* script_args = cJSON_CreateObject();
    cJSON_AddItemToObject(script_args, "x", cJSON_CreateNumber(x));
    cJSON_AddItemToObject(script_args, "y", cJSON_CreateNumber(y));
    cJSON_AddItemToObject(script_args, "theta", cJSON_CreateNumber(theta));
    cJSON* root = cJSON_CreateObject();//json数据段
    cJSON_AddItemToObject(root, "id", cJSON_CreateString("SELF_POSITION"));
    cJSON_AddItemToObject(root, "freeGo", script_args);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(3051, 3051, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return;
}

char* AMRstop_Navigation(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(3003, 3003);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}





////////////////////////////////////////////////////读
char* ReadAMRRespond(const SOCKET& client) {


    //char* buffer = new char[50048];//malloc
    memset(buffersss, '\0', sizeof(buffersss));
    while (1) {
        int num;
        //Sleep(1);
        num = recv(client, buffersss, 50048, 0); //接收客户端发送过来的数据	
        if (num > 0) {
            //cout << "数据帧长度：" << num << endl;
            //for (int i = 0; i < num; i++) {
            //    cout << (char)buffersss[i] << " ";
            //}
            //cout << "接收消息：" << string(buffersss) << endl;

            return buffersss;
        }
        //else
        //    cout << " 接受数据中 " << endl;
    }

    return nullptr;
}
int ReadAMRRespond(const SOCKET& client, char*& data) {
    //char* buffer = new char[50048];//malloc
    memset(buffersss, '\0', sizeof(buffersss));
    while (1) {
        int num;
        num = recv(client, buffersss, 50048, 0); //接收客户端发送过来的数据	
        if (num > 0) {
            //cout << "数据帧长度：" << num << endl;
            //for (int i = 0; i < num; i++) {
            //    cout << (char)buffersss[i] << " ";
            //}
            //cout << "接收消息：" << string(buffersss) << endl;
            data = buffersss;
            return num;
        }
        //else
        //    cout << " 接受数据中 " << endl;
    }

    return 0;
}

char* ReadAMRPostion(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1004, 1004);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

char* ReadAMRLaser_res(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1009, 1009);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}
int ReadAMRLaser_res(const SOCKET& client, char*& data, bool multi) {
    cJSON* root = cJSON_CreateObject();//json数据段
    ////创建数据
    cJSON_AddBoolToObject(root, "return_beams3D", multi);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1009, 1009, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return ReadAMRRespond(client, data);
}

char* ReadAMRnavigation(const SOCKET& client) {
    cJSON* root = cJSON_CreateObject();//json数据段
    ////创建数据
    cJSON_AddBoolToObject(root, "simple", true);
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1020, 1020, json_data);
    send(client, s_data, strlen(json_data) + 16, 0);

    return ReadAMRRespond(client);
}



int AMRnavigation_status(SOCKET& client) {
    //cout << "---------------------------------23-1" << endl;
    char* jsonStr = ReadAMRnavigation(client);
    jsonStr = jsonStr + 16;
    //cout << "读取导航状态:" << jsonStr << endl;
    // 将读取到的json字符串转换成json变量指针
    //cout << "---------------------------------23-12" << endl;
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        return 0;
    }
    //cout << "---------------------------------23-2" << endl;
    //free(jsonStr);//08162600008
    /*************** 解析 json***************/
    cJSON* item = NULL;
    int x=-1;
    item = cJSON_GetObjectItem(root, "task_status");
    if (item != NULL) {	// 合法性检查
        if (item->type == cJSON_Number) {		// 判断是不是数字
            x = item->valueint;			// 获取值
            //printf("x = %f\n", x);
        }
    }
    //cout << "---------------------------------23-3" << endl;
    cJSON_Delete(root);
    //cJSON_Delete(item);
    //cout << "---------------------------------23-4" << endl;
    //free(jsonStr);  
    return x;
}

char* ReadAMRInfo(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1000, 1000);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

int ReadAMRInfo(const SOCKET& client, char*& data) {
    char* s_data = RobotState2SeerFrame(1000, 1000);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client, data);
}

char* ReadAMblocked(const SOCKET& client) {
    char* s_data = RobotState2SeerFrame(1006, 1006);
    send(client, s_data, 16, 0);
    return ReadAMRRespond(client);
}

int AMRblocked_status(SOCKET& client) {
    char* jsonStr = ReadAMblocked(client);
    jsonStr = jsonStr + 16;
    //cout << "读取导航状态:" << jsonStr << endl;
    // 将读取到的json字符串转换成json变量指针
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //delete[] jsonStr;
        return 0;
    }
    //free(jsonStr);//08162600008
    
    /*************** 解析 json***************/
    cJSON* item = NULL;
    int x = 0;
    item = cJSON_GetObjectItem(root, "blocked");
    if (item != NULL) {	// 合法性检查
        if (item->type == cJSON_True) {		// 判断是不是数字
            x = 1;			// 获取值
            //printf("x = %f\n", x);
        }
    }
    cJSON_Delete(root);
    //delete[] jsonStr;
    return x;
}


float AMRnow_position(const string ss, SOCKET& client) {
    char* jsonStr = ReadAMRPostion(client);
    jsonStr = jsonStr + 16;
    //cout << "读取x，y数据:" << jsonStr << endl;
    // 将读取到的json字符串转换成json变量指针
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //free(jsonStr);
        //delete[] jsonStr;
        return 0;
    }
    //free(jsonStr);//08162600008
    /*************** 解析 json***************/
    cJSON* item = NULL;
    if (ss == "x") {
        float x;
        item = cJSON_GetObjectItem(root, "x");
        if (item != NULL) {	// 合法性检查
            if (item->type == cJSON_Number) {		// 判断是不是数字
                x = item->valuedouble;			// 获取值
                //printf("读取x = %f\n", x);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);  
        //delete[] jsonStr;
        return x;
    }
    if (ss == "y") {
        float y;
        item = cJSON_GetObjectItem(root, "y");
        if (item != NULL) {	// 合法性检查
            if (item->type == cJSON_Number) {		// 判断是不是数字
                y = item->valuedouble;			// 获取值
                //printf("读取y = %f\n", y);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);
        //delete[] jsonStr;
        return y;
    }
    
    if (ss == "angle") {
        float angle;
        item = cJSON_GetObjectItem(root, "angle");
        if (item != NULL) {	// 合法性检查
            if (item->type == cJSON_Number) {		// 判断是不是数字
                angle = item->valuedouble;			// 获取值
                //printf("读取angle = %f\n", angle);
            }
        }
        cJSON_Delete(root);
        //free(jsonStr);
        //delete[] jsonStr;
        return angle;
    }
    //delete[] jsonStr;
    return -1;

}

//char* QueryRobotLaserData(bool multi) {
//    cJSON* root = cJSON_CreateObject();//json数据段
//    ////创建数据
//    cJSON_AddNumberToObject(root, "return_beams3D", multi);
//    //创建数据
//    char* json_data = cJSON_PrintUnformatted(root);
//    cJSON_Delete(root);
//    printf("%s\n", json_data);
//    char* s_data = RobotNavigationJSON2SeerFrame(1009, 1009, json_data);
//    send(client, s_data, strlen(json_data) + 16, 0);
//}

void RequestAMRLocal(AMRLocalInfo& amrli) {
    cJSON* root = cJSON_CreateObject();//json数据段
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    //printf("%s\n", json_data);
    char* s_data = RobotNavigationJSON2SeerFrame(1004, 1004, json_data);
    send(m_SockClient04, s_data, strlen(json_data) + 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    string rStr1;
    for (int i = 0; i < num; i++) {
        //cout << r_data[i] << " ";
        rStr1.append(1, r_data[i]);
    }
    //cout << "read info:" << rStr1 << endl;
    //string rStr2 = r_data;
    //cout << "read info:" << rStr2 << endl;
    int idxSta = rStr1.find("angle"); // a:find index
    int idxEnd = rStr1.find(',');
    amrli._angle = stold(rStr1.substr(idxSta + 7, idxEnd - idxSta - 7));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._angle << endl;
    idxSta = rStr1.find("\"x\""); // "x:find index
    idxEnd = rStr1.find(',');
    amrli._x = stold(rStr1.substr(idxSta + 4, idxEnd - idxSta - 4));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._x << endl;
    idxSta = rStr1.find("\"y\""); // "y:find index
    idxEnd = rStr1.find(',');
    amrli._y = stold(rStr1.substr(idxSta + 4, idxEnd - idxSta - 4));
    //cout << "idxSta:" << idxSta << "    idxEnd:" << idxEnd << " val:" << amrli._y << endl;
    
}


void AMRStartCreateMap(int slam_tpye, bool real_time, int screen_width, int screen_height) {
    cJSON* root = cJSON_CreateObject();//json数据段
    //创建数据
    char* json_data = cJSON_PrintUnformatted(root);
    cJSON_AddNumberToObject(root, "slam_tpye", slam_tpye);
    cJSON_Delete(root);
    char* s_data = RobotNavigationJSON2SeerFrame(6100, 6100, json_data);
    send(m_SockClient10, s_data, strlen(json_data) + 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient10, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMREndCreateMap() {
    char* s_data = RobotState2SeerFrame(6101, 6101);
    send(m_SockClient10, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient10, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMRQueryScanMapStatus() {
    char* s_data = RobotState2SeerFrame(1025, 1025);
    send(m_SockClient04, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void AMRQueryToatlMap() {
    char* s_data = RobotState2SeerFrame(1300, 1300);
    send(m_SockClient04, s_data, 16, 0);

    char* r_data;
    int num = ReadAMRRespond(m_SockClient04, r_data);
    for (int i = 0; i < num; i++) {
        cout << r_data[i] << " ";
    }
    cout << endl;
}

void ReScanMap() {
    // 建立地图
    AMRStartCreateMap(1, 0, 0, 0);
    //AMRQueryToatlMap();
    //for (int i = 1; i <= 3; i++) {
    for (int i = 1; i <= 6; i++) {
        AMRRotation(2 * CV_PI / 6., 1., m_SockClient06);
        AMRLocalInfo amrli;
        RequestAMRLocal(amrli);
        cout << "rad:" << CV_PI / 6 << "    " << amrli._x << "   " << amrli._y << "  " << amrli._angle << endl;
        //AMRQueryScanMapStatus();
    }
    AMREndCreateMap();
    //AMRQueryToatlMap();
}