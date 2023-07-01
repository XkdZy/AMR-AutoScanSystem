#include "dijkstra.h"
//#include "header.h"

AgvMoveInfo agi;

int FR[X][Y] = { 0 };
int FR1[X][Y] = { 0 };
int FR2[X][Y] = { 0 };

char atom_window[] = "Drawing 1: Atom";
Mat atom_image = Mat::zeros(Y * 10, X * 10, CV_8UC3);//��

//���ܷ��ں�����  ����̫��
float edges[X * Y + 10][X * Y + 10];//����x*x�ĵ�ͼ   ��Ҫx*x����ű��   // ������еıߣ����� edges[i][j] �����i��j�ľ���

//·���滮dijkstra
int* dijkstra(float x, float y, float xx, float yy)
{
    //��ʼλ��
    int o_now = 0;
    Point2f point_now((x + 16) * 30, (y + 11) * 30);//Ŀǰλ�� x  y  תΪo_now
    circle(atom_image, point_now, 2, Scalar(255, 255, 255), -1);
    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            if (abs(m * 10 - point_now.x) <= 5 && abs(n * 10 - point_now.y) <= 5) {
                o_now = n * X + m;
                //cout << "m=" << m << " n=" << n << endl;
            }
        }
    }
    //cout << "o_now=" << o_now << endl;


    //�ҿ��ƶ���
    memset(edges, 100, sizeof(edges));//������ ��ʼ����Ϊ100=����
    Point2f point;
    for (int i = 0; i < X; i++) {
        for (int j = 0; j < Y; j++) {
            if ((i != 0) && (i != X - 1) && (j != 0) && (j != (Y - 1))) {

                if (FR[i][j] == 0 && FR[i + 1][j] == 0) {
                    edges[j * X + i][j * X + i + 1] = edges[j * X + i + 1][j * X + i] = 1;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1] = edges[j * X + i + 1][j * X + i] = 10000;
                //    point.x = (i + 0.5) * 10;
                //    point.y = j * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j] == 0) {
                    edges[j * X + i][j * X + i - 1] = edges[j * X + i - 1][j * X + i] = 1;
                    //point.x = (i - 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1] = edges[j * X + i - 1][j * X + i] = 10000;
                //    point.x = (i - 0.5) * 10;
                //    point.y = j * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i][j - 1] == 0) {
                    edges[j * X + i][j * X + i - X] = edges[j * X + i - X][j * X + i] = 1;
                    //point.x = i * 10;
                    //point.y = (j - 0.5) * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - X] = edges[j * X + i - X][j * X + i] = 10000;
                //    point.x = i * 10;
                //    point.y = (j - 0.5) * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i][j + 1] == 0) {
                    edges[j * X + i][j * X + i + X] = edges[j * X + i + X][j * X + i] = 1;
                    //point.x = i * 10;
                    //point.y = (j + 0.5) * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + X] = edges[j * X + i + X][j * X + i] = 10000;
                //    point.x = i * 10;
                //    point.y = (j + 0.5) * 10;
                //    circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}

                if (FR[i][j] == 0 && FR[i + 1][j + 1] == 0) {//&& FR[i][j + 1] == 0 && FR[i+1][j] == 0
                    edges[j * X + i][j * X + i + 1 + X] = edges[j * X + i + 1 + X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1 + X] = edges[j * X + i + 1 + X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i + 1][j - 1] == 0) {//&& FR[i+1][j] == 0 && FR[i][j - 1] == 0
                    edges[j * X + i][j * X + i + 1 - X] = edges[j * X + i + 1 - X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i + 1 - X] = edges[j * X + i + 1 - X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j - 1] == 0) {//&& FR[i][j - 1] == 0 && FR[i-1][j] == 0
                    edges[j * X + i][j * X + i - 1 - X] = edges[j * X + i - 1 - X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1 - X] = edges[j * X + i - 1 - X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}
                if (FR[i][j] == 0 && FR[i - 1][j + 1] == 0) {// && FR[i-1][j] == 0 && FR[i][j + 1] == 0
                    edges[j * X + i][j * X + i - 1 + X] = edges[j * X + i - 1 + X][j * X + i] = 1.14159;
                    //point.x = (i + 0.5) * 10;
                    //point.y = j * 10;
                    //circle(atom_image, point, 1, Scalar(150, 0, 150), -1);
                }
                //else {
                //    edges[j * X + i][j * X + i - 1 + X] = edges[j * X + i - 1 + X][j * X + i] = 10000;
                //    //point.x = (i + 0.5) * 10;
                //    //point.y = j * 10;
                //    //circle(atom_image, point, 1, Scalar(0, 255, 0), -1);
                //}


            }
        }
    }

    //�յ�λ��
    int end;
    Point2f point_end((xx + 16) * 30, (yy + 11) * 30);
    circle(atom_image, point_end, 3, Scalar(255, 255, 255), -1);
    for (int i = 0; i <= X; i++) {
        for (int j = 0; j <= Y; j++) {
            if (abs(i * 10 - point_end.x) <= 5 && abs(j * 10 - point_end.y) <= 5) {
                end = j * X + i;
                //cout << "m=" << i << " n=" << j << endl;
            }
        }
    }
    //cout << "end=" << end << endl;

    if (end == o_now)//�յ������һ��
        return nullptr;

    /////////////////////////·���滮////////////////////
    float dist[X * Y];  // ��¼��ǰ���е㵽���ľ���
    memset(dist, 100, sizeof(dist));  // ��ʼ��ÿ��dist��ֵΪ����100=��1684300900m�� 
    //memset�ǰ����ֽ������õģ�ÿ���ֽ�Ϊ0x3f, int�ĸ��ֽڣ������ 0x3f3f3f3f
    int visited[X * Y];  // ��ǵ�ǰ�ĵ��Ƿ��߳�
    memset(visited, 0, sizeof(visited)); // ��ʼ�����еĵ㶼û�б��߳���
    string path[X * Y];//ÿһ����·��˳��
    int m = o_now, n = end;///////�õ���ʼ���յ�
    int away[1000] = { 0 };//����·��

    for (int i = 0; i < X * Y; i++) {  // ÿ��ѭ�������޳���1���㣬�����Ҫforѭ������n�Ρ�
        int index = -1;  // index����ǰδ�����ʵľ���ԭ������ĵ�
        //dist[index + X] = dist[index + X+1];
        dist[m] = 0; //����ԭ��   ԭ�㵽ԭ��ľ���Ϊ0���������Ҫ����������forѭ�����е�dist����0x3f3f3f3f,�޷��ҳ�index��

        for (int j = 0; j < X * Y; j++) { // find the index of min distance 
            if (!visited[j] && (index == -1 || dist[j] < dist[index])) { // ��ǰ�ĵ�û�б��߳������ҵ�ǰ��ľ����index��ľ���С�������index��index == -1��ʾ��δ��ʼ�ҵ�dist����Сֵ�����dist[1]���롣
                index = j;
            }
        }

        visited[index] = 1;  //�ҵ���ǰ����ԭ����Сֵ�ĵ㣬��ѵ���б���߳���
        //�������·��
        if (dist[index] + edges[index][index + 1] < dist[index + 1]) { //index������������������е㡣
            dist[index + 1] = dist[index] + edges[index][index + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + 1] = path[index] + " " + to_string(index + 1);

            int nn = index + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - 1] < dist[index - 1]) { //index������������������е㡣
            dist[index - 1] = dist[index] + edges[index][index - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - 1] = path[index] + " " + to_string(index - 1);

            int nn = index - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X] < dist[index + X]) { //index������������������е㡣
            dist[index + X] = dist[index] + edges[index][index + X];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X] = path[index] + " " + to_string(index + X);

            int nn = index + X;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - X] < dist[index - X]) { //index������������������е㡣
            dist[index - X] = dist[index] + edges[index][index - X];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X] = path[index] + " " + to_string(index - X);

            int nn = index - X;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }

        if (dist[index] + edges[index][index - X - 1] < dist[index - X - 1]) { //index������������������е㡣
            dist[index - X - 1] = dist[index] + edges[index][index - X - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X - 1] = path[index] + " " + to_string(index - X - 1);

            int nn = index - X - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index - X + 1] < dist[index - X + 1]) { //index������������������е㡣
            dist[index - X + 1] = dist[index] + edges[index][index - X + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index - X + 1] = path[index] + " " + to_string(index - X + 1);

            int nn = index - X + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X + 1] < dist[index + X + 1]) { //index������������������е㡣
            dist[index + X + 1] = dist[index] + edges[index][index + X + 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X + 1] = path[index] + " " + to_string(index + X + 1);

            int nn = index + X + 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }
        }
        if (dist[index] + edges[index][index + X - 1] < dist[index + X - 1]) { //index������������������е㡣
            dist[index + X - 1] = dist[index] + edges[index][index + X - 1];
            //cout << "dis:" << dist[index + 1] << endl;
            path[index + X - 1] = path[index] + " " + to_string(index + X - 1);

            int nn = index + X - 1;
            int t = 0;
            int x = 0, y = 0;
            int x1, x2, x3;
            y = x;
            for (; path[nn][x] != '\0'; x++) {
                if (path[nn][x] == ' ') t++;
            }
            if (t > 2) {
                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//�ҵ�ÿһ��������
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x3 = stoi(paths, 0, 10);

                if (abs(x1 - x2) != abs(x3 - x2))
                    dist[nn] += 0.1;
            }

        }

        if (i > 5000 || dist[index] > 100) {//���߳���100��   ������ɨ������������   ����Ϊû�п���·��
            cout << "û��·�߿��Ե���" << endl;
            return nullptr;
        }

        if (n == index) {  //�����n���·�����򷵻�
            //cout << path[n] << endl;

            int x = 1, y = 1, p = 0;
            while (1) {//�ҵ��ܹ�����

                while (1) {//�ҵ�ÿһ��������
                    if (path[n][x] == ' ' || path[n][x] == '\0')
                        break;
                    x++;
                }
                string paths = path[n].substr(y, x - y);
                //cout << paths << endl;
                y = ++x;
                away[p] = stoi(paths, 0, 10);
                //cout << away[p] << endl;
                if (path[n][x - 1] == '\0')
                    break;
                p++;
            }
            //return dist[n];
            break;
        }
    }



    //��ͼ��ʾ·��//
    Point2f points;
    for (int i = 0; away[i] != 0; i++) {
        points.x = (int)(away[i] % X) * 10;
        int xxx = away[i] / X;
        points.y = (xxx) * 10;
        circle(atom_image, points, 2, Scalar(150, 150, 255), -1);
    }

    //aways[100]���ֱ�ߵ�
    int aways[100] = { 0 };
    int away_flags = away[1] - away[0];
    int away_flag = 0;
    aways[0] = end;
    for (int i = 2, j = 0; away[i] != 0; i++) {//��ֱ��
        if (away[i] - away[i - 1] == -1)  away_flag = -1;//8������
        if (away[i] - away[i - 1] == -(X + 1))  away_flag = -(X + 1);
        if (away[i] - away[i - 1] == -X)  away_flag = -X;
        if (away[i] - away[i - 1] == -(X - 1))  away_flag = -(X - 1);
        if (away[i] - away[i - 1] == 1)  away_flag = 1;
        if (away[i] - away[i - 1] == (X + 1))  away_flag = (X + 1);
        if (away[i] - away[i - 1] == X)  away_flag = X;
        if (away[i] - away[i - 1] == (X - 1))  away_flag = (X - 1);

        if (away_flag != away_flags) {
            away_flags = away_flag;
            aways[j] = away[i - 1];
            //cout << "xxxxxxxxxxx=" << aways[j] << endl;
            j++;
        }
        aways[j] = end;
    }

    //ֱ�ߵ�תΪ��ͼʵ�ʵ㣨x m��y m��
    for (int i = 0; aways[i] != 0; i++) {//��ֱ��
        //cout << "x=" << (aways[i] % X)/3-16 << "y=" << (aways[i] / X)/3-11 << endl;
        points.x = aways[i] % X * 10;
        points.y = aways[i] / X * 10;
        circle(atom_image, points, 5, Scalar(0, 255, 0), -1);
        float xx, yy;
        xx = ((int)(aways[i] % X)) / 3.0 - 16.0;
        yy = ((int)(aways[i] / X)) / 3.0 - 11.0;
        //printf("x=%f   y=%f\n", xx, yy);
    }
    //cout << "dist[n]=" << dist[n] << endl;
    return aways;
}

//int map_obstacles() {
int map_obstacles(string path) {
    // ��ͼ����
    // �򿪵�ͼ�ļ�
    FILE* file = NULL;
    //file = fopen("map.smap", "r");
    file = fopen(path.c_str(), "r");
    if (file == NULL) {
        printf("Open file fail��\n");
        return 0;
    }
    // ����ļ���С
    struct stat statbuf;
    //stat("map.smap", &statbuf);
    stat(path.c_str(), &statbuf);
    int fileSize = statbuf.st_size;
    printf("�ļ���С��%d\n", fileSize);

    // ��������ļ���С���ڴ�
    char* jsonStr = (char*)malloc(sizeof(char) * fileSize + 1);
    memset(jsonStr, 0, fileSize + 1);

    // ��ȡ�ļ��е�json�ַ���
    int size = fread(jsonStr, sizeof(char), fileSize, file);
    if (size == 0) {
        printf("��ȡ�ļ�ʧ�ܣ�\n");
        fclose(file);
        return 0;
    }
    //printf("%s\n", jsonStr);
    fclose(file);

    // ����ȡ����json�ַ���ת����json����ָ��
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        free(jsonStr);
        return 0;
    }
    free(jsonStr);

    /*************** ���� json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    item = cJSON_GetObjectItem(root, "normalPosList");
    if (item != NULL) {
        cJSON* obj = item->child;	// ��� [1][]
        while (obj) {
            if (obj->type == cJSON_Object) {

                cJSON* objValue = obj->child;	// [1][1]
                while (objValue) {
                    float v_double = objValue->valuedouble;

                    if (number_map % 2 == 0) {//ȡx
                        mapdata[number_map / 2][0] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][0]);
                        if (v_double > maxx)   maxx = v_double;
                        if (v_double < minx)   minx = v_double;
                    }
                    else {//ȡy
                        mapdata[number_map / 2][1] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][1]);
                        if (v_double > maxy)   maxy = v_double;
                        if (v_double < miny)   miny = v_double;
                    }
                    // ��ȡ��һ��Ԫ��
                    objValue = objValue->next;
                    number_map++;
                }
            }
            // ��ȡ��һ��Ԫ��
            obj = obj->next;
        }
    }
    cJSON_Delete(root);
    printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    printf("��ͼpoint��=%d\n", number_map / 2);

    //��ԭ��ͼ
    Point2f points;
    Point2f point;

    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    points.x = (int)(mapdata[0][0] * 30);
    points.y = (int)(mapdata[0][1] * 30);
    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    for (int pp = 0; pp < number_map / 2; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, Scalar(0, 255, 120), -1);//��ɫ
        //printf("x_max=%f   y_max=%f  %d\n", points.x, points.y, number_map);
    }

    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map / 2; pp++) {

                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//�����ϰ���
                    point.x = m * 10;
                    point.y = n * 10;
                    //circle(atom_image, point, 2, Scalar(0, 0, 255), -1);
                    FR1[m][n] = 1;
                }
            }
            if (!FR1[m][n]) {
                point.x = m * 10;
                point.y = n * 10;
                circle(atom_image, point, 1, Scalar(255, 0, 0), -1);
            }


        }
    }

    return 0;
}


int map_Laser_res(SOCKET& client) {

    float x = 0, y = 0, angle0 = 0;
    x = AMRnow_position("x", client);
    y = AMRnow_position("y", client);
    angle0 = AMRnow_position("angle", client);

    char* Laser_res = ReadAMRLaser_res(client);//��ȡ�ɹ����ܼ����״���������  ��Ӧ
    Laser_res = Laser_res + 16;
    cJSON* root = cJSON_Parse(Laser_res);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //free(jsonStr);
        return 0;
    }
    /*************** ���� json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    float dist_double;
    float angle_double;
    item = cJSON_GetObjectItem(root, "lasers");//
    if (item != NULL) {
        cJSON* array_items = cJSON_GetArrayItem(item, 0);//��ȡ��������
        cJSON* array_item = cJSON_GetObjectItem(array_items, "beams");//��ȡbeams����
        cJSON* obj = array_item->child;	// ��� [1][]
        //if(obj)
        //    cout << "�ҵ��˼����״�����" << endl;
        while (obj) {
            if (obj->type == cJSON_Object) {
                cJSON* angle = cJSON_GetObjectItem(obj, "angle");
                if (angle) {
                    angle_double = angle->valuedouble;
                    //printf("angle=%f\n", angle_double);
                }
                cJSON* dist = cJSON_GetObjectItem(obj, "dist");
                if (dist) {
                    dist_double = dist->valuedouble;
                }

                mapdata[number_map][0] = x + 0.553317 * cos(angle0) + dist_double * cos(angle0 + (angle_double)*PI / 180);//0.55�����״�����ڳ���λ��
                mapdata[number_map][1] = y + 0.553317 * sin(angle0) + dist_double * sin(angle0 + (angle_double)*PI / 180);
                //cout << sin(90.0 * PI / 180) << endl;;
                number_map++;

            }
            // ��ȡ��һ��Ԫ��
            obj = obj->next;
        }
    }
    cJSON_Delete(root);

    //printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    //printf("��ͼpoint��=%d\n", number_map / 2);

    //��ԭ�����״�����
    Point2f points;
    for (int pp = 0; pp < number_map - 1; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, Scalar(150, 120, 0), -1);//��ɫ
    }

    //�ҵ��ϰ���
    Point2f point;
    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map - 1; pp++) {
                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//�����ϰ���
                    point.x = m * 10;
                    point.y = n * 10;
                    circle(atom_image, point, 2, Scalar(50, 50, 255), -1);
                    FR2[m][n] = 1;
                }
            }
            if (!FR2[m][n]) {
                point.x = m * 10;
                point.y = n * 10;
                //circle(atom_image, point, 1, Scalar(255, 50, 30), -1);
            }
        }
    }

    return 0;
}
void Move2Goal(const Eigen::Vector2d turegoal, const Eigen::Vector2d goal) {
    //��ͼ���ݴ���
    string localMapPath = "./20230630114409271.smap";
    //string localMapPath = "C:/Users/Administrator/AppData/Local/RoboshopPro/appInfo/robots/All/2D0024001647393436343431/maps/bk_20230604151411285_2023-06-04-211506-397.smap";    map_obstacles(localMapPath);
    //��ͼ���ݴ���
    map_obstacles(localMapPath);

    //�ϰ�Ѱ��
    memset(FR2, 0, sizeof(FR2));
    map_Laser_res(m_SockClient04);
    for (int m = 0; m < X; m++)
        for (int n = 0; n < Y; n++)
            FR[m][n] = FR1[m][n] + FR2[m][n];

    AMRLocalInfo currAmrli;
    // ��ȡ��ǰagv���ԭʼλ��
    RequestAMRLocal(currAmrli);
    double yyy = turegoal[1] / 1000., yy = goal[1] / 1000.;
    double xxx = turegoal[0] / 1000., xx = goal[0] / 1000.;
    double x = currAmrli._x, y = currAmrli._y;
    float behind_thetas = asin((yyy - yy) / sqrt((xxx - xx) * (xxx - xx) + (yyy - yy) * (yyy - yy)));
    if ((xxx - xx) > 0)
        behind_thetas;
    else if ((yyy - yy) > 0)
        behind_thetas = 3.1415926 - behind_thetas;
    else
        behind_thetas = -(3.1415926 + behind_thetas);
    cout << "behind_thetas:" << behind_thetas << endl;
    //cout << atan2(yyy - yy, xxx - xx) << endl;
    cout << "x��y��xx��xy��" << x << "  " << y << " " << xx << "    " << yy << "    " << yyy << "   " << xxx << endl;
    int* aways;
    aways = dijkstra(x, y, xx, yy);////////////////·���滮
    if (aways == NULL) return;//��·�����Ե���         

    // ȡ��AGV�˶��켣
    vAGVMovePosition.clear();
    vAGVMovePosition.emplace_back(agi._oriAgvPos); // �����ʼλ��
    //cout << "AGV��ʼλ�ã�" << agi._oriAgvPos.transpose() << endl;
    //cout << "robot move pos:" << endl;
    for (int i = 0; aways[i] != 0; i++) {
        Eigen::Vector3d point{
            1000 * ((int)(aways[i] % X) / 3.0 - 16.0),
            1000 * ((int)(aways[i] / X) / 3.0 - 11.0),
            1.
        };
        vAGVMovePosition.emplace_back(point);
        //cout << point.transpose() << endl;
    }
    ConvertAGVMove2ArmTra(vAGVMovePosition, vArmMovePosition);

    //while (1) {
    //    AMRLocalInfo currAmrli;
    //    // ��ȡ��ǰagv���ԭʼλ��
    //    RequestAMRLocal(currAmrli);
    //    currAmrli._x *= 1000.;
    //    currAmrli._y *= 1000.;
    //    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

    //    //cout << "main thread!" << endl;
    //    Sleep(10);
    //}
    ////return;

    //cout << "agvλ�ã�x" << x << "    y:" << y << "  ����:" << sta << endl;
    //cout << "cos:" << asin(sinsta1) * 180 / CV_PI << "    sin:" << asin(cossta1) * 180 / CV_PI << endl;
    //cout << "xx:" << xx << "    yy:" << yy << "  " << behind_thetas << endl;
    //cout << "Position x:" << Position.at<double>(0) / 1000.0 << "   y:" << Position.at<double>(1) / 1000.0 << endl;
    //for (int i = 0; aways[i] != 0; i++) {
    //    cout << (int)(aways[i] % X) / 3.0 - 16.0 << "    " << (int)(aways[i] / Y) / 3.0 - 11.0 << endl;
    //}
    //cout << endl;
    //return ;

    int blocked = 0, next_blocked = 0;
    int navigation_status = -1;
    for (int i = 0; aways[i] != 0; i++) {//ֱ������
            // �ж�AGV�Ƿ��쳣
        if (agi._move2goal) {
            cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
            AMRstop_Navigation(m_SockClient06);
            return;
        }

        //cout << "-----------------------------------------------1" << endl;
        if (aways[i + 1] == 0)//��������һ���� ֱ�ӵ����յ�
            AMRRobotNavigation(xx, yy, behind_thetas, m_SockClient06);
        else
            AMRRobotNavigation(((int)(aways[i] % X)) / 3.0 - 16.0, ((int)(aways[i] / X)) / 3.0 - 11.0, m_SockClient06);



        Sleep(100);//������ʼ

        while (1) {
            AMRLocalInfo currAmrli;
            // ��ȡ��ǰagv���ԭʼλ��
            RequestAMRLocal(currAmrli);
            currAmrli._x *= 1000.;
            currAmrli._y *= 1000.;
            agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

            ////cout << "-----------------------------------------------2" << endl;
            //// �ж�AGV�Ƿ��쳣
            //if (agi._move2goal) {
            //    cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
            //    AMRstop_Navigation(m_SockClient06);
            //    return;
            //}

            //cout << "-----------------------------------------------2" << endl;
            navigation_status = AMRnavigation_status(m_SockClient04);
            if (navigation_status == 4 || navigation_status == 0)//�˶����||�޵���   ������һ���˶�
            {
                
                cout << "-----------------------------------------------3" << endl;
                break;
            }

            //cout << "-----------------------------------------------3" << endl;

            blocked = AMRblocked_status(m_SockClient04);//��ȡ�����״�״̬
            //cout << "-----------------------------------------------341" << endl;
            //cout << "daohqk" << blocked << endl;
            memset(FR2, 0, sizeof(FR2));
            map_Laser_res(m_SockClient04);
            //cout << "-----------------------------------------------3" << endl;
            if (blocked == 0)//
                for (int m = 0; m < X; m++)
                    for (int n = 0; n < Y; n++) {
                        if (FR2[m][n]) {
                            int t1 = aways[i - 1];
                            int t2 = aways[i];
                            int x1, y1, x2, y2;
                            x1 = t1 % X;
                            y1 = t1 / X;
                            x2 = t2 % X;
                            y2 = t2 / X;
                            for (; t1 != t2;) {
                                if (((x2 - x1) > 0) && ((y2 - y1) == 0)) {
                                    t1 += 1;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) == 0)) {
                                    t1 -= 1;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) == 0) && ((y2 - y1) > 0)) {
                                    t1 += X;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) == 0) && ((y2 - y1) < 0)) {
                                    t1 -= X;
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) < 0)) {
                                    t1 -= (X + 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) > 0) && ((y2 - y1) < 0)) {
                                    t1 -= (X - 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) < 0) && ((y2 - y1) > 0)) {
                                    t1 += (X - 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                                if (((x2 - x1) > 0) && ((y2 - y1) > 0)) {
                                    t1 += (X + 1);
                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
                                        blocked = 1;
                                    }
                                    break;
                                }
                            }
                        }
                    }
            //cout << "zusai" << blocked << endl;
            //cout << "-----------------------------------------------4" << endl;

            if (blocked == 1) {//���ⱻ����
                cout << "-----------------------------------------------5" << endl;
                //agi._move2goal = 1;
                //break;
                // ��������ֹͣ����
                AMRstop_Navigation(m_SockClient06);//ֹͣ��ǰ����
                memset(FR2, 0, sizeof(FR2));
                map_Laser_res(m_SockClient04);
                for (int m = 0; m < X; m++)
                    for (int n = 0; n < Y; n++)
                        FR[m][n] = FR1[m][n] + FR2[m][n];

                x = AMRnow_position("x", m_SockClient04);//���赲λ��Ϊ���
                y = AMRnow_position("y", m_SockClient04);

                int* aways;
                aways = dijkstra(x, y, xx, yy);
                //for (int i = 0; aways[i] != 0; i++) {//��ֱ��
                //    cout << "aways[i]:" << aways[i]<< endl;
                //} 
                i = -1;//iҪ�Լ�һ
                while (aways == NULL) {//��·�����Ե���
                    memset(FR2, 0, sizeof(FR2));
                    map_Laser_res(m_SockClient04);
                    for (int m = 0; m < X; m++)
                        for (int n = 0; n < Y; n++)
                            FR[m][n] = FR1[m][n] + FR2[m][n];
                    aways = dijkstra(x, y, xx, yy);
                }
                break;//���¿�ʼ
            }
            //cout << "-----------------------------------------------5" << endl;
        }

        //cout << "-----------------------------------------------6" << endl;
    }

    //cout << "-----------------------------------------------7" << endl;
}
//
//void Move2Goal(const Eigen::Vector3d& pose) {
//    //��ͼ���ݴ���
//    string localMapPath = "./20230604151411285.smap";
//    //string localMapPath = "C:/Users/Administrator/AppData/Local/RoboshopPro/appInfo/robots/All/2D0024001647393436343431/maps/bk_20230604151411285_2023-06-04-211506-397.smap";    map_obstacles(localMapPath);
//    //��ͼ���ݴ���
//    map_obstacles(localMapPath);
//
//    float xx = 0, yy = 0;  //end 
//    float x = 0, y = 0; //start
//
//    //�ϰ�Ѱ��
//    memset(FR2, 0, sizeof(FR2));
//    map_Laser_res(m_SockClient04);
//    for (int m = 0; m < X; m++)
//        for (int n = 0; n < Y; n++)
//            FR[m][n] = FR1[m][n] + FR2[m][n];
//
//    /*cout << "�������յ�x��y:" << endl;
//    cin >> xx >> yy;*/
//    x = AMRnow_position("x", m_SockClient04);
//    y = AMRnow_position("y", m_SockClient04);//���λ��
//    float sta = AMRnow_position("angle", m_SockClient04);
//
//    cv::Mat Position = (cv::Mat_<double>(1, 2) << pose[0], pose[1]);
//    Position.at<double>(1) = Position.at<double>(1) - 100; // ��е�۵��������agvƽ������ƫ��0.1m
//
//    float dist_c = (sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1))) - 2000) / 1000;
//    float dist_cs = sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1))) / 1000;
//    float cossta1 = (Position.at<double>(0)) / sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1)));
//    float sinsta1 = (Position.at<double>(1)) / sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1)));
//
//    float xxx = x + dist_cs * (cos(asin(sinsta1) + sta));
//    float yyy = y + dist_cs * (sin(asin(sinsta1) + sta));//����Ŀ��
//    xx = x + dist_c * (cos(asin(sinsta1) + sta));
//    yy = y + dist_c * (sin(asin(sinsta1) + sta));//����λ��
//    //cout << "x:" << x << "   y:" << y << "angle:" << sta << endl;
//    //cout << "xx:" << xx << "   yy:" << yy << endl;
//    //cout << "Position x:" << Position.at<double>(0) / 1000.0 << "   y:" << Position.at<double>(1) / 1000.0 << endl;
//
//    float behind_thetas = asin((yyy - yy) / sqrt((xxx - xx) * (xxx - xx) + (yyy - yy) * (yyy - yy)));
//    if ((xxx - xx) > 0)
//        behind_thetas;
//    else if ((yyy - yy) > 0)
//        behind_thetas = 3.1415926 - behind_thetas;
//    else
//        behind_thetas = -(3.1415926 + behind_thetas);
//    //cout << "behind_thetas:" << behind_thetas << endl;
//
//    int* aways;
//    aways = dijkstra(x, y, xx, yy);////////////////·���滮
//    if (aways == NULL) return ;//��·�����Ե���         
//
//    // ��ȡ��ǰ��е������
//    string currStrPose;
//    sp.ReadRobotArmPosString(sp, currStrPose);
//    // ����string
//    cv::Mat curr6DMatPose;
//    AnalysisString26D(currStrPose, curr6DMatPose);
//    // ȡ��AGV�˶��켣
//    agi._oriAgvPos = Eigen::Vector3d(x, y, sta); // ԭ���Ӧ��AGVλ��
//    agi._currAgvPos = Eigen::Vector3d(x, y, sta);
//    //agi._worldGoalPos = Eigen::Vector3d(xx, yy, pose[2] / 1000.); // AGV
//    agi._worldGoalPos = Eigen::Vector3d(pose[0] / 1000., pose[1] / 1000., pose[2] / 1000.); // ��е����������������
//    agi._oriArmPos = Eigen::Vector3d(curr6DMatPose.at<double>(0, 0), curr6DMatPose.at<double>(0, 1), curr6DMatPose.at<double>(0, 2));
//
//    vAGVMovePosition.emplace_back(agi._oriAgvPos); // �����ʼλ��
//    vAGVMovePosition[0][0] *= 1000.;
//    vAGVMovePosition[0][1] *= 1000.;
//    cout << "AGV��ʼλ�ã�" << agi._oriAgvPos.transpose() << endl;
//    cout << "robot move pos:" << endl;
//    for (int i = 0; aways[i] != 0; i++) {
//        Eigen::Vector3d point{
//            1000 * ((int)(aways[i] % X) / 3.0 - 16.0),
//            1000 * ((int)(aways[i] / X) / 3.0 - 11.0),
//            1.
//        };
//        vAGVMovePosition.emplace_back(point);
//        cout << point.transpose() << endl;
//    }
//    ConvertAGVMove2ArmTra(vAGVMovePosition, vArmMovePosition);
//
//    //while (1) {
//    //    //cout << "main thread!" << endl;
//    //    Sleep(100);
//    //}
//    ////return;
//
//    //cout << "agvλ�ã�x" << x << "    y:" << y << "  ����:" << sta << endl;
//    //cout << "cos:" << asin(sinsta1) * 180 / CV_PI << "    sin:" << asin(cossta1) * 180 / CV_PI << endl;
//    //cout << "xx:" << xx << "    yy:" << yy << "  " << behind_thetas << endl;
//    //cout << "Position x:" << Position.at<double>(0) / 1000.0 << "   y:" << Position.at<double>(1) / 1000.0 << endl;
//    //for (int i = 0; aways[i] != 0; i++) {
//    //    cout << (int)(aways[i] % X) / 3.0 - 16.0 << "    " << (int)(aways[i] / Y) / 3.0 - 11.0 << endl;
//    //}
//    //cout << endl;
//    //return ;
//
//    int blocked = 0, next_blocked = 0;
//    int navigation_status = -1;
//    for (int i = 0; aways[i] != 0; i++) {//ֱ������
//            // �ж�AGV�Ƿ��쳣
//        if (agi._move2goal) {
//            cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
//            AMRstop_Navigation(m_SockClient06);
//            return;
//        }
//
//        cout << "-----------------------------------------------1" << endl;
//        if (aways[i + 1] == 0)//��������һ���� ֱ�ӵ����յ�
//            AMRRobotNavigation(xx, yy, behind_thetas, m_SockClient06);
//        else
//            AMRRobotNavigation(((int)(aways[i] % X)) / 3.0 - 16.0, ((int)(aways[i] / X)) / 3.0 - 11.0, m_SockClient06);
//        
//        Sleep(100);//������ʼ
//        while (1) {
//            cout << "-----------------------------------------------2" << endl;
//            // �ж�AGV�Ƿ��쳣
//            if (agi._move2goal) {
//                cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD�����쳣" : "��������쳣") << endl;
//                AMRstop_Navigation(m_SockClient06);
//                return;
//            }
//
//            cout << "-----------------------------------------------2" << endl;
//            navigation_status = AMRnavigation_status(m_SockClient04);
//            if (navigation_status == 4 || navigation_status == 0)//�˶����||�޵��� ������һ���˶�
//                break;
//
//            cout << "-----------------------------------------------3" << endl;
//
//            blocked = AMRblocked_status(m_SockClient04);//��ȡ�����״�״̬
//            //cout << "daohqk" << blocked << endl;
//            memset(FR2, 0, sizeof(FR2));
//            map_Laser_res(m_SockClient04);
//            if (blocked == 0)//
//                for (int m = 0; m < X; m++)
//                    for (int n = 0; n < Y; n++) {
//                        if (FR2[m][n]) {
//                            int t1 = aways[i - 1];
//                            int t2 = aways[i];
//                            int x1, y1, x2, y2;
//                            x1 = t1 % X;
//                            y1 = t1 / X;
//                            x2 = t2 % X;
//                            y2 = t2 / X;
//                            for (; t1 != t2;) {
//                                if (((x2 - x1) > 0) && ((y2 - y1) == 0)) {
//                                    t1 += 1;
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) < 0) && ((y2 - y1) == 0)) {
//                                    t1 -= 1;
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) == 0) && ((y2 - y1) > 0)) {
//                                    t1 += X;
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) == 0) && ((y2 - y1) < 0)) {
//                                    t1 -= X;
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) < 0) && ((y2 - y1) < 0)) {
//                                    t1 -= (X + 1);
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) > 0) && ((y2 - y1) < 0)) {
//                                    t1 -= (X - 1);
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) < 0) && ((y2 - y1) > 0)) {
//                                    t1 += (X - 1);
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                                if (((x2 - x1) > 0) && ((y2 - y1) > 0)) {
//                                    t1 += (X + 1);
//                                    if ((m == (t1 % X)) && (n == (t1 / X))) {
//                                        blocked = 1;
//                                    }
//                                    break;
//                                }
//                            }
//                        }
//                    }
//            //cout << "zusai" << blocked << endl;
//            cout << "-----------------------------------------------4" << endl;
//
//            if (blocked == 1) {//���ⱻ����
//                //agi._move2goal = 1;
//                //break;
//                // ��������ֹͣ����
//                AMRstop_Navigation(m_SockClient06);//ֹͣ��ǰ����
//                memset(FR2, 0, sizeof(FR2));
//                map_Laser_res(m_SockClient04);
//                for (int m = 0; m < X; m++)
//                    for (int n = 0; n < Y; n++)
//                        FR[m][n] = FR1[m][n] + FR2[m][n];
//
//                x = AMRnow_position("x", m_SockClient04);//���赲λ��Ϊ���
//                y = AMRnow_position("y", m_SockClient04);
//
//                int* aways;
//                aways = dijkstra(x, y, xx, yy);
//                //for (int i = 0; aways[i] != 0; i++) {//��ֱ��
//                //    cout << "aways[i]:" << aways[i]<< endl;
//                //} 
//                i = -1;//iҪ�Լ�һ
//                while (aways == NULL) {//��·�����Ե���
//                    memset(FR2, 0, sizeof(FR2));
//                    map_Laser_res(m_SockClient04);
//                    for (int m = 0; m < X; m++)
//                        for (int n = 0; n < Y; n++)
//                            FR[m][n] = FR1[m][n] + FR2[m][n];
//                    aways = dijkstra(x, y, xx, yy);
//                }
//                break;//���¿�ʼ
//            }
//            cout << "-----------------------------------------------5" << endl;
//        }
//
//        cout << "-----------------------------------------------6" << endl;
//    }
//
//    cout << "-----------------------------------------------7" << endl;
//}

void RobotArmReset() {
    // ʹ��е��ĩ�˳�����ǰ��
    // ��ȡ��ǰ��е������
    string currStrPose;
    sp.ReadRobotArmPosString(sp, currStrPose);
    // ����string
    cv::Mat curr6DMatPose;
    AnalysisString26D(currStrPose, curr6DMatPose);
    // ת��Ϊ4*4����
    cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");        
    Eigen::Vector3d rx{ H_gripper2base.at<double>(0, 0), H_gripper2base.at<double>(1, 0), H_gripper2base.at<double>(2, 0) };
    Eigen::Vector3d ry{ H_gripper2base.at<double>(0, 1), H_gripper2base.at<double>(1, 1), H_gripper2base.at<double>(2, 1) }; 
    Eigen::Vector3d position{ H_gripper2base.at<double>(0, 3), H_gripper2base.at<double>(1, 3), H_gripper2base.at<double>(2, 3) };
    Eigen::Vector3d rz {1., 0., 0.};
    // ��x��y�������豸��ǰ����˹����������
    Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
    Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
    // ����λ��
    cv::Mat dstGripperMat = (cv::Mat_<double>(4, 4) <<
        dstRx[0], dstRy[0], rz[0], position[0],
        dstRx[1], dstRy[1], rz[1], position[1],
        dstRx[2], dstRy[2], rz[2], position[2],
        0, 0, 0, 1
        );
    // 4*4λ��ת��Ϊstring
    string cmdStr;
    ConvertMat2String(dstGripperMat, cmdStr);
    cout << "������ĩ��λ�ˣ�" << cmdStr << endl;
    // �´�����
    MoveToOnePoint(cmdStr, &sp);

    return;
}

void RobotArm2Goal() {
    //while (1) {
        Eigen::Vector3d goal = agi._worldGoalPos;
        //Eigen::Vector3d goal = Eigen::Vector3d{ 3250.22, 31.9879, 38.0136, };
        //if (goal.norm() < 0.2) continue;

        AMRLocalInfo currAmrli;
        // ��ȡ��ǰagv���ԭʼλ��
        RequestAMRLocal(currAmrli);
        currAmrli._x *= 1000.;
        currAmrli._y *= 1000.;
        agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
        // ��ȡ��ǰ��е������
        string currStrPose;
        sp.ReadRobotArmPosString(sp, currStrPose);

        // pose differ
        //cout << "curr - ori:" << (agi._currAgvPos - agi._oriAgvPos).transpose() << endl;
        double xDiff = agi._currAgvPos[0] - agi._oriAgvPos[0];
        double yDiff = agi._currAgvPos[1] - agi._oriAgvPos[1]; // agv����ϵ
        double rotateTheta = agi._currAgvPos[2] - agi._oriAgvPos[2];

        //if (goal.norm() < 1.0e-2) continue;  
        Eigen::Vector3d diff{ xDiff, yDiff, rotateTheta };
        Eigen::Vector3d goalRotatedEig = AGVMove2ArmPos(diff, goal, false, agi._oriAgvPos[2]);

        cv::Mat curr6DMatPose;
        AnalysisString26D(currStrPose, curr6DMatPose);
        cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
        // ������ת��Ļ�е�����꣬����AGV��ת�Ƕȣ�������е������
        // ��ĩ��ת�����������
        cv::Mat curr_camera2base = H_gripper2base * H_Camera2Gripper;

        string cmdStr1, cmdStr2;
        ConvertMat2String(H_gripper2base, cmdStr1);
        ConvertMat2String(curr_camera2base, cmdStr2);
        cout << "��ǰĩ��λ�ˣ�" << cmdStr1 << endl;
        cout << "��ǰ���λ�ˣ�" << cmdStr2 << endl;
        // rz���䣬ʹrx��ry�任��С��˹������������rx����ͶӰ
        Eigen::Vector3d rx{ curr_camera2base.at<double>(0, 0), curr_camera2base.at<double>(1, 0), curr_camera2base.at<double>(2, 0) };
        Eigen::Vector3d ry{ curr_camera2base.at<double>(0, 1), curr_camera2base.at<double>(1, 1), curr_camera2base.at<double>(2, 1) };
        Eigen::Vector3d position{ curr_camera2base.at<double>(0, 3), curr_camera2base.at<double>(1, 3), curr_camera2base.at<double>(2, 3) };
        Eigen::Vector3d rz = (goalRotatedEig - position).normalized();
        //Eigen::Vector3d rz {1., 0., 0.};

        // ��x��y�������豸��ǰ����˹����������
        Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
        Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
        //cout << "camera position info:" << position.transpose() << endl;

        cv::Mat dstCameraMat = (cv::Mat_<double>(4, 4) <<
            dstRx[0], dstRy[0], rz[0], position[0],
            dstRx[1], dstRy[1], rz[1], position[1],
            dstRx[2], dstRy[2], rz[2], position[2],
            0, 0, 0, 1
            );
        cv::Mat dstGripperMat = dstCameraMat * H_Camera2Gripper.inv();
        string dstStr1, dstStr2;
        ConvertMat2String(dstCameraMat, dstStr1);
        ConvertMat2String(dstGripperMat, dstStr2);
        cout << "���������λ�ˣ�" << dstStr1 << endl;
        cout << "������ĩ��λ�ˣ�" << dstStr2 << endl;
        cout << "����ת������" << isRotationMatrix(dstGripperMat) << endl;

        double distance = (goalRotatedEig - position).norm();
        cout << "��ǰ��ʼ�����Ŀ���ľ��룺" << distance << endl;

        MoveToOnePoint(dstStr2, &sp);
        cout << endl << endl << endl;
    //}
}

Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta) {
    cv::Mat diffXY = (cv::Mat_<double>(4, 1) << 0., -100., 0, 1); // ��е��x��yƫ��
    Eigen::Vector3d diffXYEig(0., -100, 0);
    robotPos += diffXYEig;
    double xDiff = agvMoveInfo[0];
    double yDiff = agvMoveInfo[1]; // agv����ϵ
    double rotateTheta = agvMoveInfo[2];
    cv::Mat agvTransposeCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(oriTheta), sin(oriTheta), 0, 0,
        -sin(oriTheta), cos(oriTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        );
    cv::Mat agvRotateCurr2Ori = (cv::Mat_<double>(4, 4) <<
        cos(rotateTheta), -sin(rotateTheta), 0, 0,
        sin(rotateTheta), cos(rotateTheta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
        ); // ˳ʱ��thetaΪ��������b1,b2,b3��=RI��H_curr2ori
    //// ƫ�Ĵ�������תƫ��
    //cv::Mat xDiffRotated = agvRotateCurr2Ori * diffXY;
    //cout << "ƫ����תƫ��Ϊ��" << xDiffRotated.t() << endl;
    //xDiff += xDiffRotated.at<double>(0, 0);
    //yDiff += xDiffRotated.at<double>(1, 0);
    // ������ת��ƽ�ƺ�Ļ�е������
    cv::Mat translation, rotate, dstPosition;
    if (curr2ori) {
        translation = agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        rotate = agvRotateCurr2Ori * (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1);
        dstPosition = translation + rotate - diffXY;
    }
    else {
        cv::Mat tmp = (cv::Mat_<double>(4, 1) << robotPos[0], robotPos[1], robotPos[2], 1) - 
            agvTransposeCurr2Ori * (cv::Mat_<double>(4, 1) << xDiff, yDiff, 0, 0);
        dstPosition = agvRotateCurr2Ori.inv() * tmp;
        dstPosition -= diffXY;
    }
    //cout << "ƽ�ƺ������Ϊ:" << translation.t() << endl;
    //cout << "��ת�������Ϊ:" << rotate.t() << endl;
    //cout << "��תƽ�ƺ������Ϊ:" << dstPosition.t() << endl;
    return { dstPosition.at<double>(0,0), dstPosition.at<double>(1,0), dstPosition.at<double>(2,0) };
}

void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos) {
    //cout << "--------------------------------:" << endl;
    //for (int i = 0; i < agvPos.size(); i++) {
    //    cout << agvPos[i].transpose() << endl;
    //}

    // ��е��ĩ���˶�����Ϊ��intervalDistance����ת�Ƕ�Ϊ��rotateThetaʱ����һ�ι켣��
    double intervalDistance = 50.; // mm
    double rotateTheta = 10. * CV_PI / 180.; 
    if (agvPos.size() <= 1) return;

    //for (int agvIdx = 0; agvIdx < agvPos.size(); agvIdx++) {
    //    cout << agvPos[agvIdx].transpose() << endl;
    //}

    Eigen::Vector3d lastPos = agvPos[0];
    for (int agvIdx = 1; agvIdx < agvPos.size(); agvIdx++) {
        Eigen::Vector3d currPos = agvPos[agvIdx];
        // ��������֮��нǣ��ж��Ƿ���Ҫ��תAGV����ʱ����תΪ��
        double angle = atan2((currPos[1] - lastPos[1]), (currPos[0] - lastPos[0]));
        //cout << "��ǰ�����꣺" << currPos.transpose() << endl
        //    << "  ǰһ�������꣺" << lastPos.transpose() << endl
        //    << "    �����н�Ϊ��" << angle * 180. / CV_PI
        //    << "    �нǲ" << (angle - lastPos[2]) * 180. / CV_PI << endl;
        double agvRotate = 0.;
        //// ��ת
        //if (abs(angle - lastPos[2]) * 180. / CV_PI > 2.) {
        //    cout << "�нǴ����趨�Ƕȣ�AGV��Ҫ��ת��" << endl;
        //    // �нǴ���x����Ҫ��ת��������ת�Ƕ��ϵĵ�
        //    for (int rTime = 0; rTime < floor(abs(angle - lastPos[2] / rotateTheta)); rTime++) {
        //        
        //    }
        //}
        // ƽ��
        double dist = (Eigen::Vector2d{ lastPos[0], lastPos[1] } - Eigen::Vector2d{ currPos[0], currPos[1] }).norm();
        int totalTTimes = abs(dist / intervalDistance); // floor(abs(dist / rotateTheta));
        //cout << "���룺" << dist << "    ����������" << totalTTimes
        //    << "   ��ǰAGV�Ƕȣ�" << angle << "    ��Ҫ��ת�Ƕȣ�" << angle - agi._oriAgvPos[2]
        //    << "    AGV��ʼ״̬" << agi._oriAgvPos.transpose() << endl;
        for (int tTime = 0; tTime < totalTTimes; tTime++) {
            double xPos = 0., yPos = 0., angleDif = 0.;
            xPos = (tTime + 1) * intervalDistance * cos(angle) + lastPos[0] - agi._oriAgvPos[0];
            yPos = (tTime + 1) * intervalDistance * sin(angle) + lastPos[1] - agi._oriAgvPos[1];
            xPos = (tTime + 1) * intervalDistance * cos(angle) + lastPos[0];
            yPos = (tTime + 1) * intervalDistance * sin(angle) + lastPos[1];
            //Eigen::Vector3d xyDif{ intervalDistance * cos(angle), intervalDistance * sin(angle), 0. };
            //Eigen::Vector3d goalPos{ xPos, yPos, 0. };
            angleDif = angle - agi._oriAgvPos[2];
            angleDif = angleDif > CV_PI ? angleDif - 2 * CV_PI: angleDif;
            angleDif = angleDif < -CV_PI ? angleDif + 2 * CV_PI : angleDif;
            Eigen::Vector3d xyDif{ xPos - agi._oriAgvPos[0], yPos - agi._oriAgvPos[1], angleDif };
            //Eigen::Vector3d goalPos{ 0., 0., agi._oriArmPos[2] };
            Eigen::Vector3d goalPos{ agi._oriArmPos };
            Eigen::Vector3d goalPosRotated = AGVMove2ArmPos(xyDif, goalPos, true, agi._oriAgvPos[2]);
            cout << "��ת��ƽ��Ϊ��" << xPos << "  " << yPos << "   ��������꣺" << agi._oriArmPos.transpose() << "  ת�����е�����Ĺ켣��" << goalPosRotated.transpose() << endl;
            vArmMovePosition.emplace_back(goalPosRotated);
        }
        vTest.push_back(vArmMovePosition.size());

        lastPos = currPos; // ������һ����λ
        lastPos[2] = angle;
        cout << endl << endl << endl;
    }
    //cout << atan2(-1, -1) * 180. / CV_PI << endl;
    return;
}

void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos) {
    currPos = vector<Eigen::Vector3d>(oriPos);
    for (int i = 0; i < oriPos.size(); i++) {
        Eigen::Vector3d diff = agi._currAgvPos - agi._oriAgvPos;
        Eigen::Vector3d posRT = AGVMove2ArmPos(diff, oriPos[i], false, agi._oriAgvPos[2]);
        currPos[i] = posRT;
    }
}

cv::Mat arm2agv(const double x, const double y, const double theta) {
    return (cv::Mat_<double>(4, 4) <<
        cos(theta), -sin(theta), 0, x,
        sin(theta), cos(theta), 0, y,
        0, 0, 1, 0,
        0, 0, 0, 1
        );
}

cv::Mat arm2agv(const Eigen::Vector3d agv) {
    return (cv::Mat_<double>(4, 4) <<
        cos(agv[2]), -sin(agv[2]), 0, agv[0],
        sin(agv[2]), cos(agv[2]), 0, agv[1],
        0, 0, 1, 0,
        0, 0, 0, 1
        );
}

Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint) {
    cv::Mat convertMat = (cv::Mat_<double>(4, 4) <<
        cos(agvInfo[2]), -sin(agvInfo[2]), 0, agvInfo[0],
        sin(agvInfo[2]), cos(agvInfo[2]), 0, agvInfo[1],
        0, 0, 1, 0,
        0, 0, 0, 1
        );
    //cv::Mat temp = convertMat * (cv::Mat_<double>(4, 1) << armPoint[0], armPoint[1] - 0., armPoint[2], 1);
    cv::Mat temp = convertMat * (cv::Mat_<double>(4, 1) << armPoint[0], armPoint[1] - 100., armPoint[2], 1);
    return Eigen::Vector3d{ temp.at<double>(0,0), temp.at<double>(1,0), temp.at<double>(2,0) };
}

cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint) {
    //cout << "���������ͨ������С��" << armPoint.channels() << "    " << armPoint.size() << endl;
    cv::Mat temp = (cv::Mat_<double>(4, 1) << 0., 0., 0., 1.);
    if (armPoint.size() == cv::Size{ 3, 1 } || armPoint.size() == cv::Size{ 4, 1 }) {
        temp.at<double>(0, 0) = armPoint.at<double>(0, 0);
        temp.at<double>(1, 0) = armPoint.at<double>(1, 0);
        temp.at<double>(2, 0) = armPoint.at<double>(2, 0);
    }
    else if (armPoint.size() == cv::Size{ 4, 4 }) {
        temp.at<double>(0, 0) = armPoint.at<double>(0, 3);
        temp.at<double>(1, 0) = armPoint.at<double>(1, 3);
        temp.at<double>(2, 0) = armPoint.at<double>(2, 3);
    }
    else {
        cout << "ͨ������Ϊ4*1��4*4��3*1!" << endl;
        return armPoint;
    }
    cv::Mat convertMat = (cv::Mat_<double>(4, 4) <<
        cos(agvInfo[2]), -sin(agvInfo[2]), 0, agvInfo[0],
        sin(agvInfo[2]), cos(agvInfo[2]), 0, agvInfo[1],
        0, 0, 1, 0,
        0, 0, 0, 1
        );
    temp.at<double>(1, 0) -= 100.;
    //temp.at<double>(1, 0) -= 100.;
    return convertMat * temp;
}

//vector<Eigen::Vector3d> CalcAgvPosForRGBD(double radius, double safedis, double thetas) {
//
//    radius += safedis;
//    Eigen::Vector2d goal{ agi._worldGoalPos[0], agi._worldGoalPos[1] };
//    Eigen::Vector2d centerDir = goal.normalized();
//    cout << "����������" << centerDir.transpose() << endl;
//    Eigen::Vector2d centerPos = goal - centerDir * radius;
//    //Eigen::Vector2d centerPos_behind = goal + centerDir * radius;
//
//    vector<Eigen::Vector3d> vRes;
//    AMRLocalInfo currAmrli;
//    // ��ȡ��ǰagv���ԭʼλ��
//    RequestAMRLocal(currAmrli);
//    currAmrli._x *= 1000;
//    currAmrli._y *= 1000;
//    cv::Mat pos = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos[0], centerPos[1] - 100., agi._worldGoalPos[2], 1.);
//    //vRes.emplace_back(Eigen::Vector3d{ pos.at<double>(0, 0), pos.at<double>(1, 0), pos.at<double>(2, 0) });
//
//    Eigen::Vector2d centerPos_l1, centerPos_r1;
//    double theta = 0;
//    while (1)
//    {
//        theta = thetas * CV_PI / 180. + theta;
//        if (theta > CV_PI);
//        break;
//
//        if (centerDir[0] != 0) {
//            double A = -centerDir[0], B = -centerDir[1];
//            double a = 1 + B * B / A / A;
//            double b = -2 * B * cos(theta) / A / A;
//            double c = cos(theta) * cos(theta) / A / A - 1;
//            double y1, y2 = 0.;
//            if ((b * b - 4 * a * c) > 0) {
//                y1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//                y2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//                cout << "�������������ȵ�ʵ����" << endl << "x1=" << y1 << ",x2=" << y2 << endl;
//            }
//            else if ((b * b - 4 * a * c) >= 0) {
//                y1 = -b / (2 * a);
//                y2 = y1;
//                cout << "������������ȵ�ʵ����" << endl << "x1=x2=" << y1 << endl;
//            }
//            else {
//                cout << "������ʵ����" << endl;
//            }
//            centerPos_l1 = Eigen::Vector2d((cos(theta) - B * y1) / A, y1);
//            centerPos_r1 = Eigen::Vector2d((cos(theta) - B * y2) / A, y2);
//        }
//        else {
//            centerPos_l1 = Eigen::Vector2d(sin(theta), cos(theta));
//            centerPos_r1 = Eigen::Vector2d(-sin(theta), cos(theta));
//        }
//        centerPos_l1 = goal + centerPos_l1 * radius;
//        centerPos_r1 = goal + centerPos_r1 * radius;
//        cout << "���ĵ�Ϊ��" << centerPos.transpose() << endl;
//        cout << "��1��Ϊ��" << centerPos_l1.transpose() << endl;
//        cout << "��1��Ϊ��" << centerPos_r1.transpose() << endl;
//
//        cv::Mat posl1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_l1[0], centerPos_l1[1] - 100., agi._worldGoalPos[2], 1.);
//        cv::Mat posr1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_r1[0], centerPos_r1[1] - 100., agi._worldGoalPos[2], 1.);
//
//        cout << "agv�����������ĵ�Ϊ��" << pos.t() << endl;
//        cout << "agv����������1��Ϊ��" << posl1.t() << endl;
//        cout << "agv����������1��Ϊ��" << posr1.t() << endl;
//
//        if (centerPos_l1 != centerPos_r1) {
//            vRes.emplace_back(Eigen::Vector3d{ posl1.at<double>(0, 0), posl1.at<double>(1, 0), posl1.at<double>(2, 0) });
//            vRes.emplace_back(Eigen::Vector3d{ pos.at<double>(0, 0), pos.at<double>(1, 0), pos.at<double>(2, 0) });
//            vRes.emplace_back(Eigen::Vector3d{ posr1.at<double>(0, 0), posr1.at<double>(1, 0), posr1.at<double>(2, 0) });
//        }
//        else
//            vRes.emplace_back(Eigen::Vector3d{ posr1.at<double>(0, 0), posr1.at<double>(1, 0), posr1.at<double>(2, 0) });
//
//    }
//
//    vector<Eigen::Vector3d> vRess = vRes;
//    if ((vRes.size() - 1) % 2 == 0)
//        for (int i = 1; i < vRes.size(); i++) {
//            if (i < ((vRes.size() + 1) / 2))
//                vRess[i] = vRes[i * 2 - 1];
//            else
//                vRess[i] = vRes[2 * (vRes.size() - i)];
//        }
//    else {
//        for (int i = 1; i < vRes.size(); i++) {
//            if (i < (vRes.size() / 2))
//                vRess[i] = vRes[i * 2 - 1];
//            else if (i == (vRes.size() / 2))
//                vRess[vRes.size() - 1] = vRes[vRes.size() - 1];
//            else
//                vRess[i] = vRes[2 * (vRes.size() - i)];
//        }
//    }
//
//
//    for (int i = 0; i < vRess.size(); i++)
//        cout << "vRess" << i << "����:" << vRess[i] << endl;
//
//    return vRes;
//}

vector<Eigen::Vector3d> CalcAgvPosForRGBD(double radius, double safedis, double theta) {
    theta = theta * CV_PI / 180.;
    radius += safedis;
    Eigen::Vector2d goal{ agi._worldGoalPos[0], agi._worldGoalPos[1] };
    Eigen::Vector2d centerDir = goal.normalized();
    cout << "����������" << centerDir.transpose() << endl;
    Eigen::Vector2d centerPos = goal - centerDir * radius;
    Eigen::Vector2d centerPos_l1, centerPos_r1;
    if (centerDir[0] != 0) {
        double A = -centerDir[0], B = -centerDir[1];
        double a = 1 + B * B / A / A;
        double b = -2 * B * cos(theta) / A / A;
        double c = cos(theta) * cos(theta) / A / A - 1;
        double y1, y2 = 0.;
        if ((b * b - 4 * a * c) > 0) {
            y1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            y2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            cout << "�������������ȵ�ʵ����" << endl << "x1=" << y1 << ",x2=" << y2 << endl;
        }
        else if ((b * b - 4 * a * c) >= 0) {
            y1 = -b / (2 * a);
            cout << "������������ȵ�ʵ����" << endl << "x1=x2=" << y1 << endl;
        }
        else {
            cout << "������ʵ����" << endl;
        }
        centerPos_l1 = Eigen::Vector2d((cos(theta) - B * y1) / A, y1);
        centerPos_r1 = Eigen::Vector2d((cos(theta) - B * y2) / A, y2);
    }
    else {
        centerPos_l1 = Eigen::Vector2d(sin(theta), cos(theta));
        centerPos_r1 = Eigen::Vector2d(-sin(theta), cos(theta));
    }
    centerPos_l1 = goal + centerPos_l1 * radius;
    centerPos_r1 = goal + centerPos_r1 * radius;
    cout << "���ĵ�Ϊ��" << centerPos.transpose() << endl;
    cout << "��1��Ϊ��" << centerPos_l1.transpose() << endl;
    cout << "��1��Ϊ��" << centerPos_r1.transpose() << endl;

    AMRLocalInfo currAmrli;
    // ��ȡ��ǰagv���ԭʼλ��
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000;
    currAmrli._y *= 1000;

    cv::Mat pos = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos[0], centerPos[1] - 100., agi._worldGoalPos [2], 1.);
    cv::Mat posl1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_l1[0], centerPos_l1[1] - 100., agi._worldGoalPos [2], 1.);
    cv::Mat posr1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_r1[0], centerPos_r1[1] - 100., agi._worldGoalPos [2], 1.);

    cout << "agv�����������ĵ�Ϊ��" << pos.t() << endl;
    cout << "agv����������1��Ϊ��" << posl1.t() << endl;
    cout << "agv����������1��Ϊ��" << posr1.t() << endl;
    vector<Eigen::Vector3d> vRes;
    //vRes.emplace_back(Eigen::Vector3d{ centerPos_l1[0], centerPos_l1[1], goal[2] });
    //vRes.emplace_back(Eigen::Vector3d{ centerPos[0], centerPos[1], goal[2] });
    //vRes.emplace_back(Eigen::Vector3d{ centerPos_r1[0], centerPos_r1[1], goal[2] });
    vRes.emplace_back(Eigen::Vector3d{ posl1.at<double>(0, 0), posl1.at<double>(1, 0), posl1.at<double>(2, 0) });
    vRes.emplace_back(Eigen::Vector3d{ pos.at<double>(0, 0), pos.at<double>(1, 0), pos.at<double>(2, 0) });
    vRes.emplace_back(Eigen::Vector3d{ posr1.at<double>(0, 0), posr1.at<double>(1, 0), posr1.at<double>(2, 0) });

    return vRes;
}