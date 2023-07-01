#include "dijkstra.h"
//#include "header.h"

AgvMoveInfo agi;

int FR[X][Y] = { 0 };
int FR1[X][Y] = { 0 };
int FR2[X][Y] = { 0 };

char atom_window[] = "Drawing 1: Atom";
Mat atom_image = Mat::zeros(Y * 10, X * 10, CV_8UC3);//宽长

//不能放在函数里  数组太大
float edges[X * Y + 10][X * Y + 10];//对于x*x的地图   需要x*x个序号编号   // 存放所有的边，例如 edges[i][j] 代表从i到j的距离

//路劲规划dijkstra
int* dijkstra(float x, float y, float xx, float yy)
{
    //初始位置
    int o_now = 0;
    Point2f point_now((x + 16) * 30, (y + 11) * 30);//目前位置 x  y  转为o_now
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


    //找可移动线
    memset(edges, 100, sizeof(edges));//网格线 开始设置为100=无穷
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

    //终点位置
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

    if (end == o_now)//终点与起点一致
        return nullptr;

    /////////////////////////路劲规划////////////////////
    float dist[X * Y];  // 记录当前所有点到起点的距离
    memset(dist, 100, sizeof(dist));  // 初始化每个dist的值为无穷100=》1684300900m， 
    //memset是按照字节来设置的，每个字节为0x3f, int四个字节，因此是 0x3f3f3f3f
    int visited[X * Y];  // 标记当前的点是否被踢出
    memset(visited, 0, sizeof(visited)); // 初始化所有的点都没有被踢出。
    string path[X * Y];//每一个点路径顺序
    int m = o_now, n = end;///////得到起始与终点
    int away[1000] = { 0 };//最优路径

    for (int i = 0; i < X * Y; i++) {  // 每次循环都会剔除掉1个点，因此需要for循环遍历n次。
        int index = -1;  // index代表当前未被访问的距离原点最近的点
        //dist[index + X] = dist[index + X+1];
        dist[m] = 0; //设置原点   原点到原点的距离为0，这个很重要，否则下面for循环所有的dist都是0x3f3f3f3f,无法找出index。

        for (int j = 0; j < X * Y; j++) { // find the index of min distance 
            if (!visited[j] && (index == -1 || dist[j] < dist[index])) { // 当前的点没有被踢出，并且当前点的距离比index点的距离小，则更新index。index == -1表示还未开始找到dist中最小值，则把dist[1]加入。
                index = j;
            }
        }

        visited[index] = 1;  //找到当前距离原点最小值的点，则把点进行标记踢出。
        //更新最短路径
        if (dist[index] + edges[index][index + 1] < dist[index + 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index - 1] < dist[index - 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index + X] < dist[index + X]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index - X] < dist[index - X]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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

        if (dist[index] + edges[index][index - X - 1] < dist[index - X - 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index - X + 1] < dist[index - X + 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index + X + 1] < dist[index + X + 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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
        if (dist[index] + edges[index][index + X - 1] < dist[index + X - 1]) { //index点更新与它相连的所有点。
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
                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                string paths = path[nn].substr(x + 1, y - x - 1);
                x1 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
                    x--;
                    if (path[nn][x] == ' ')
                        break;
                }
                paths = path[nn].substr(x + 1, y - x - 1);
                x2 = stoi(paths, 0, 10);
                y = x;

                while (1) {//找到每一个点数据
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

        if (i > 5000 || dist[index] > 100) {//行走超过100米   基本上扫查了所有数据   则认为没有可行路径
            cout << "没有路线可以到达" << endl;
            return nullptr;
        }

        if (n == index) {  //如果到n点的路径，则返回
            //cout << path[n] << endl;

            int x = 1, y = 1, p = 0;
            while (1) {//找到总共点数

                while (1) {//找到每一个点数据
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



    //地图显示路径//
    Point2f points;
    for (int i = 0; away[i] != 0; i++) {
        points.x = (int)(away[i] % X) * 10;
        int xxx = away[i] / X;
        points.y = (xxx) * 10;
        circle(atom_image, points, 2, Scalar(150, 150, 255), -1);
    }

    //aways[100]存放直线点
    int aways[100] = { 0 };
    int away_flags = away[1] - away[0];
    int away_flag = 0;
    aways[0] = end;
    for (int i = 2, j = 0; away[i] != 0; i++) {//找直线
        if (away[i] - away[i - 1] == -1)  away_flag = -1;//8个方向
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

    //直线点转为地图实际点（x m，y m）
    for (int i = 0; aways[i] != 0; i++) {//找直线
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
    // 地图数据
    // 打开地图文件
    FILE* file = NULL;
    //file = fopen("map.smap", "r");
    file = fopen(path.c_str(), "r");
    if (file == NULL) {
        printf("Open file fail！\n");
        return 0;
    }
    // 获得文件大小
    struct stat statbuf;
    //stat("map.smap", &statbuf);
    stat(path.c_str(), &statbuf);
    int fileSize = statbuf.st_size;
    printf("文件大小：%d\n", fileSize);

    // 分配符合文件大小的内存
    char* jsonStr = (char*)malloc(sizeof(char) * fileSize + 1);
    memset(jsonStr, 0, fileSize + 1);

    // 读取文件中的json字符串
    int size = fread(jsonStr, sizeof(char), fileSize, file);
    if (size == 0) {
        printf("读取文件失败！\n");
        fclose(file);
        return 0;
    }
    //printf("%s\n", jsonStr);
    fclose(file);

    // 将读取到的json字符串转换成json变量指针
    cJSON* root = cJSON_Parse(jsonStr);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        free(jsonStr);
        return 0;
    }
    free(jsonStr);

    /*************** 解析 json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    item = cJSON_GetObjectItem(root, "normalPosList");
    if (item != NULL) {
        cJSON* obj = item->child;	// 获得 [1][]
        while (obj) {
            if (obj->type == cJSON_Object) {

                cJSON* objValue = obj->child;	// [1][1]
                while (objValue) {
                    float v_double = objValue->valuedouble;

                    if (number_map % 2 == 0) {//取x
                        mapdata[number_map / 2][0] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][0]);
                        if (v_double > maxx)   maxx = v_double;
                        if (v_double < minx)   minx = v_double;
                    }
                    else {//取y
                        mapdata[number_map / 2][1] = v_double;
                        //printf("%s = %.2f\n", objValue->string, mapdata[number_map / 2][1]);
                        if (v_double > maxy)   maxy = v_double;
                        if (v_double < miny)   miny = v_double;
                    }
                    // 获取下一个元素
                    objValue = objValue->next;
                    number_map++;
                }
            }
            // 获取下一组元素
            obj = obj->next;
        }
    }
    cJSON_Delete(root);
    printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    printf("地图point数=%d\n", number_map / 2);

    //还原地图
    Point2f points;
    Point2f point;

    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    points.x = (int)(mapdata[0][0] * 30);
    points.y = (int)(mapdata[0][1] * 30);
    //circle(atom_image, points, 1, Scalar(0, 255, 120), -1);
    for (int pp = 0; pp < number_map / 2; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, Scalar(0, 255, 120), -1);//绿色
        //printf("x_max=%f   y_max=%f  %d\n", points.x, points.y, number_map);
    }

    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map / 2; pp++) {

                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//膨胀障碍物
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

    char* Laser_res = ReadAMRLaser_res(client);//读取仙工智能激光雷达所有数据  响应
    Laser_res = Laser_res + 16;
    cJSON* root = cJSON_Parse(Laser_res);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        //free(jsonStr);
        return 0;
    }
    /*************** 解析 json***************/
    cJSON* item = NULL;
    int number_map = 0;
    float mapdata[100000][2];
    float maxx = -1000, maxy = -1000, minx = 1000, miny = 1000;
    float dist_double;
    float angle_double;
    item = cJSON_GetObjectItem(root, "lasers");//
    if (item != NULL) {
        cJSON* array_items = cJSON_GetArrayItem(item, 0);//获取所有数据
        cJSON* array_item = cJSON_GetObjectItem(array_items, "beams");//获取beams数据
        cJSON* obj = array_item->child;	// 获得 [1][]
        //if(obj)
        //    cout << "找到了激光雷达数据" << endl;
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

                mapdata[number_map][0] = x + 0.553317 * cos(angle0) + dist_double * cos(angle0 + (angle_double)*PI / 180);//0.55激光雷达相对于车体位置
                mapdata[number_map][1] = y + 0.553317 * sin(angle0) + dist_double * sin(angle0 + (angle_double)*PI / 180);
                //cout << sin(90.0 * PI / 180) << endl;;
                number_map++;

            }
            // 获取下一组元素
            obj = obj->next;
        }
    }
    cJSON_Delete(root);

    //printf("x_max=%f   y_max=%f  x_mix=%f   y_mix=%f\n", maxx, maxy, minx, miny);
    //printf("地图point数=%d\n", number_map / 2);

    //还原激光雷达数据
    Point2f points;
    for (int pp = 0; pp < number_map - 1; pp++) {
        points.x = (int)(mapdata[pp][0] * 30) + 480;
        points.y = (int)(mapdata[pp][1] * 30) + 330;
        circle(atom_image, points, 1, Scalar(150, 120, 0), -1);//绿色
    }

    //找到障碍物
    Point2f point;
    for (int m = 0; m <= X; m++) {
        for (int n = 0; n <= Y; n++) {
            for (int pp = 0; pp < number_map - 1; pp++) {
                points.x = (int)(mapdata[pp][0] * 30) + 480;
                points.y = (int)(mapdata[pp][1] * 30) + 330;
                if (abs(m * 10 - points.x) < 20 && abs(n * 10 - points.y) < 20) {//膨胀障碍物
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
    //地图数据处理
    string localMapPath = "./20230630114409271.smap";
    //string localMapPath = "C:/Users/Administrator/AppData/Local/RoboshopPro/appInfo/robots/All/2D0024001647393436343431/maps/bk_20230604151411285_2023-06-04-211506-397.smap";    map_obstacles(localMapPath);
    //地图数据处理
    map_obstacles(localMapPath);

    //障碍寻找
    memset(FR2, 0, sizeof(FR2));
    map_Laser_res(m_SockClient04);
    for (int m = 0; m < X; m++)
        for (int n = 0; n < Y; n++)
            FR[m][n] = FR1[m][n] + FR2[m][n];

    AMRLocalInfo currAmrli;
    // 读取当前agv相对原始位置
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
    cout << "x、y、xx、xy：" << x << "  " << y << " " << xx << "    " << yy << "    " << yyy << "   " << xxx << endl;
    int* aways;
    aways = dijkstra(x, y, xx, yy);////////////////路劲规划
    if (aways == NULL) return;//无路径可以导航         

    // 取出AGV运动轨迹
    vAGVMovePosition.clear();
    vAGVMovePosition.emplace_back(agi._oriAgvPos); // 插入初始位置
    //cout << "AGV初始位置：" << agi._oriAgvPos.transpose() << endl;
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
    //    // 读取当前agv相对原始位置
    //    RequestAMRLocal(currAmrli);
    //    currAmrli._x *= 1000.;
    //    currAmrli._y *= 1000.;
    //    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

    //    //cout << "main thread!" << endl;
    //    Sleep(10);
    //}
    ////return;

    //cout << "agv位置：x" << x << "    y:" << y << "  朝向:" << sta << endl;
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
    for (int i = 0; aways[i] != 0; i++) {//直线行走
            // 判断AGV是否异常
        if (agi._move2goal) {
            cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD检测出异常" : "激光检测出异常") << endl;
            AMRstop_Navigation(m_SockClient06);
            return;
        }

        //cout << "-----------------------------------------------1" << endl;
        if (aways[i + 1] == 0)//如果是最后一个点 直接到达终点
            AMRRobotNavigation(xx, yy, behind_thetas, m_SockClient06);
        else
            AMRRobotNavigation(((int)(aways[i] % X)) / 3.0 - 16.0, ((int)(aways[i] / X)) / 3.0 - 11.0, m_SockClient06);



        Sleep(100);//导航开始

        while (1) {
            AMRLocalInfo currAmrli;
            // 读取当前agv相对原始位置
            RequestAMRLocal(currAmrli);
            currAmrli._x *= 1000.;
            currAmrli._y *= 1000.;
            agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

            ////cout << "-----------------------------------------------2" << endl;
            //// 判断AGV是否异常
            //if (agi._move2goal) {
            //    cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD检测出异常" : "激光检测出异常") << endl;
            //    AMRstop_Navigation(m_SockClient06);
            //    return;
            //}

            //cout << "-----------------------------------------------2" << endl;
            navigation_status = AMRnavigation_status(m_SockClient04);
            if (navigation_status == 4 || navigation_status == 0)//运动完成||无导航   进行下一步运动
            {
                
                cout << "-----------------------------------------------3" << endl;
                break;
            }

            //cout << "-----------------------------------------------3" << endl;

            blocked = AMRblocked_status(m_SockClient04);//获取激光雷达状态
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

            if (blocked == 1) {//激光被阻塞
                cout << "-----------------------------------------------5" << endl;
                //agi._move2goal = 1;
                //break;
                // 激光阻塞停止导航
                AMRstop_Navigation(m_SockClient06);//停止当前导航
                memset(FR2, 0, sizeof(FR2));
                map_Laser_res(m_SockClient04);
                for (int m = 0; m < X; m++)
                    for (int n = 0; n < Y; n++)
                        FR[m][n] = FR1[m][n] + FR2[m][n];

                x = AMRnow_position("x", m_SockClient04);//此阻挡位置为起点
                y = AMRnow_position("y", m_SockClient04);

                int* aways;
                aways = dijkstra(x, y, xx, yy);
                //for (int i = 0; aways[i] != 0; i++) {//找直线
                //    cout << "aways[i]:" << aways[i]<< endl;
                //} 
                i = -1;//i要自加一
                while (aways == NULL) {//无路径可以导航
                    memset(FR2, 0, sizeof(FR2));
                    map_Laser_res(m_SockClient04);
                    for (int m = 0; m < X; m++)
                        for (int n = 0; n < Y; n++)
                            FR[m][n] = FR1[m][n] + FR2[m][n];
                    aways = dijkstra(x, y, xx, yy);
                }
                break;//重新开始
            }
            //cout << "-----------------------------------------------5" << endl;
        }

        //cout << "-----------------------------------------------6" << endl;
    }

    //cout << "-----------------------------------------------7" << endl;
}
//
//void Move2Goal(const Eigen::Vector3d& pose) {
//    //地图数据处理
//    string localMapPath = "./20230604151411285.smap";
//    //string localMapPath = "C:/Users/Administrator/AppData/Local/RoboshopPro/appInfo/robots/All/2D0024001647393436343431/maps/bk_20230604151411285_2023-06-04-211506-397.smap";    map_obstacles(localMapPath);
//    //地图数据处理
//    map_obstacles(localMapPath);
//
//    float xx = 0, yy = 0;  //end 
//    float x = 0, y = 0; //start
//
//    //障碍寻找
//    memset(FR2, 0, sizeof(FR2));
//    map_Laser_res(m_SockClient04);
//    for (int m = 0; m < X; m++)
//        for (int n = 0; n < Y; n++)
//            FR[m][n] = FR1[m][n] + FR2[m][n];
//
//    /*cout << "请输入终点x，y:" << endl;
//    cin >> xx >> yy;*/
//    x = AMRnow_position("x", m_SockClient04);
//    y = AMRnow_position("y", m_SockClient04);//起点位置
//    float sta = AMRnow_position("angle", m_SockClient04);
//
//    cv::Mat Position = (cv::Mat_<double>(1, 2) << pose[0], pose[1]);
//    Position.at<double>(1) = Position.at<double>(1) - 100; // 机械臂底座相对与agv平面中心偏移0.1m
//
//    float dist_c = (sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1))) - 2000) / 1000;
//    float dist_cs = sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1))) / 1000;
//    float cossta1 = (Position.at<double>(0)) / sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1)));
//    float sinsta1 = (Position.at<double>(1)) / sqrt((Position.at<double>(0) * Position.at<double>(0) + Position.at<double>(1) * Position.at<double>(1)));
//
//    float xxx = x + dist_cs * (cos(asin(sinsta1) + sta));
//    float yyy = y + dist_cs * (sin(asin(sinsta1) + sta));//真正目标
//    xx = x + dist_c * (cos(asin(sinsta1) + sta));
//    yy = y + dist_c * (sin(asin(sinsta1) + sta));//到的位置
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
//    aways = dijkstra(x, y, xx, yy);////////////////路劲规划
//    if (aways == NULL) return ;//无路径可以导航         
//
//    // 读取当前机械臂坐标
//    string currStrPose;
//    sp.ReadRobotArmPosString(sp, currStrPose);
//    // 解析string
//    cv::Mat curr6DMatPose;
//    AnalysisString26D(currStrPose, curr6DMatPose);
//    // 取出AGV运动轨迹
//    agi._oriAgvPos = Eigen::Vector3d(x, y, sta); // 原点对应的AGV位置
//    agi._currAgvPos = Eigen::Vector3d(x, y, sta);
//    //agi._worldGoalPos = Eigen::Vector3d(xx, yy, pose[2] / 1000.); // AGV
//    agi._worldGoalPos = Eigen::Vector3d(pose[0] / 1000., pose[1] / 1000., pose[2] / 1000.); // 机械臂世界坐标下坐标
//    agi._oriArmPos = Eigen::Vector3d(curr6DMatPose.at<double>(0, 0), curr6DMatPose.at<double>(0, 1), curr6DMatPose.at<double>(0, 2));
//
//    vAGVMovePosition.emplace_back(agi._oriAgvPos); // 插入初始位置
//    vAGVMovePosition[0][0] *= 1000.;
//    vAGVMovePosition[0][1] *= 1000.;
//    cout << "AGV初始位置：" << agi._oriAgvPos.transpose() << endl;
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
//    //cout << "agv位置：x" << x << "    y:" << y << "  朝向:" << sta << endl;
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
//    for (int i = 0; aways[i] != 0; i++) {//直线行走
//            // 判断AGV是否异常
//        if (agi._move2goal) {
//            cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD检测出异常" : "激光检测出异常") << endl;
//            AMRstop_Navigation(m_SockClient06);
//            return;
//        }
//
//        cout << "-----------------------------------------------1" << endl;
//        if (aways[i + 1] == 0)//如果是最后一个点 直接到达终点
//            AMRRobotNavigation(xx, yy, behind_thetas, m_SockClient06);
//        else
//            AMRRobotNavigation(((int)(aways[i] % X)) / 3.0 - 16.0, ((int)(aways[i] / X)) / 3.0 - 11.0, m_SockClient06);
//        
//        Sleep(100);//导航开始
//        while (1) {
//            cout << "-----------------------------------------------2" << endl;
//            // 判断AGV是否异常
//            if (agi._move2goal) {
//                cout << agi._move2goal << "************************************" << (agi._move2goal == 2 ? "RGBD检测出异常" : "激光检测出异常") << endl;
//                AMRstop_Navigation(m_SockClient06);
//                return;
//            }
//
//            cout << "-----------------------------------------------2" << endl;
//            navigation_status = AMRnavigation_status(m_SockClient04);
//            if (navigation_status == 4 || navigation_status == 0)//运动完成||无导航 进行下一步运动
//                break;
//
//            cout << "-----------------------------------------------3" << endl;
//
//            blocked = AMRblocked_status(m_SockClient04);//获取激光雷达状态
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
//            if (blocked == 1) {//激光被阻塞
//                //agi._move2goal = 1;
//                //break;
//                // 激光阻塞停止导航
//                AMRstop_Navigation(m_SockClient06);//停止当前导航
//                memset(FR2, 0, sizeof(FR2));
//                map_Laser_res(m_SockClient04);
//                for (int m = 0; m < X; m++)
//                    for (int n = 0; n < Y; n++)
//                        FR[m][n] = FR1[m][n] + FR2[m][n];
//
//                x = AMRnow_position("x", m_SockClient04);//此阻挡位置为起点
//                y = AMRnow_position("y", m_SockClient04);
//
//                int* aways;
//                aways = dijkstra(x, y, xx, yy);
//                //for (int i = 0; aways[i] != 0; i++) {//找直线
//                //    cout << "aways[i]:" << aways[i]<< endl;
//                //} 
//                i = -1;//i要自加一
//                while (aways == NULL) {//无路径可以导航
//                    memset(FR2, 0, sizeof(FR2));
//                    map_Laser_res(m_SockClient04);
//                    for (int m = 0; m < X; m++)
//                        for (int n = 0; n < Y; n++)
//                            FR[m][n] = FR1[m][n] + FR2[m][n];
//                    aways = dijkstra(x, y, xx, yy);
//                }
//                break;//重新开始
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
    // 使机械臂末端朝向正前方
    // 读取当前机械臂坐标
    string currStrPose;
    sp.ReadRobotArmPosString(sp, currStrPose);
    // 解析string
    cv::Mat curr6DMatPose;
    AnalysisString26D(currStrPose, curr6DMatPose);
    // 转化为4*4矩阵
    cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");        
    Eigen::Vector3d rx{ H_gripper2base.at<double>(0, 0), H_gripper2base.at<double>(1, 0), H_gripper2base.at<double>(2, 0) };
    Eigen::Vector3d ry{ H_gripper2base.at<double>(0, 1), H_gripper2base.at<double>(1, 1), H_gripper2base.at<double>(2, 1) }; 
    Eigen::Vector3d position{ H_gripper2base.at<double>(0, 3), H_gripper2base.at<double>(1, 3), H_gripper2base.at<double>(2, 3) };
    Eigen::Vector3d rz {1., 0., 0.};
    // 将x、y方向（与设备当前朝向）斯密特正交化
    Eigen::Vector3d dstRx = (rx - rx.transpose() * rz * rz).normalized();
    Eigen::Vector3d dstRy = (ry - ry.transpose() * dstRx * dstRx - ry.transpose() * rz * rz).normalized();
    // 整合位姿
    cv::Mat dstGripperMat = (cv::Mat_<double>(4, 4) <<
        dstRx[0], dstRy[0], rz[0], position[0],
        dstRx[1], dstRy[1], rz[1], position[1],
        dstRx[2], dstRy[2], rz[2], position[2],
        0, 0, 0, 1
        );
    // 4*4位姿转换为string
    string cmdStr;
    ConvertMat2String(dstGripperMat, cmdStr);
    cout << "调整后末端位姿：" << cmdStr << endl;
    // 下达命令
    MoveToOnePoint(cmdStr, &sp);

    return;
}

void RobotArm2Goal() {
    //while (1) {
        Eigen::Vector3d goal = agi._worldGoalPos;
        //Eigen::Vector3d goal = Eigen::Vector3d{ 3250.22, 31.9879, 38.0136, };
        //if (goal.norm() < 0.2) continue;

        AMRLocalInfo currAmrli;
        // 读取当前agv相对原始位置
        RequestAMRLocal(currAmrli);
        currAmrli._x *= 1000.;
        currAmrli._y *= 1000.;
        agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
        // 读取当前机械臂坐标
        string currStrPose;
        sp.ReadRobotArmPosString(sp, currStrPose);

        // pose differ
        //cout << "curr - ori:" << (agi._currAgvPos - agi._oriAgvPos).transpose() << endl;
        double xDiff = agi._currAgvPos[0] - agi._oriAgvPos[0];
        double yDiff = agi._currAgvPos[1] - agi._oriAgvPos[1]; // agv坐标系
        double rotateTheta = agi._currAgvPos[2] - agi._oriAgvPos[2];

        //if (goal.norm() < 1.0e-2) continue;  
        Eigen::Vector3d diff{ xDiff, yDiff, rotateTheta };
        Eigen::Vector3d goalRotatedEig = AGVMove2ArmPos(diff, goal, false, agi._oriAgvPos[2]);

        cv::Mat curr6DMatPose;
        AnalysisString26D(currStrPose, curr6DMatPose);
        cv::Mat H_gripper2base = attitudeVectorToMatrix(curr6DMatPose, false, "xyz");
        // 计算旋转后的机械臂坐标，根据AGV旋转角度，修正机械臂坐标
        // 将末端转换到相机坐标
        cv::Mat curr_camera2base = H_gripper2base * H_Camera2Gripper;

        string cmdStr1, cmdStr2;
        ConvertMat2String(H_gripper2base, cmdStr1);
        ConvertMat2String(curr_camera2base, cmdStr2);
        cout << "当前末端位姿：" << cmdStr1 << endl;
        cout << "当前相机位姿：" << cmdStr2 << endl;
        // rz不变，使rx、ry变换最小，斯密特正交化，rx正交投影
        Eigen::Vector3d rx{ curr_camera2base.at<double>(0, 0), curr_camera2base.at<double>(1, 0), curr_camera2base.at<double>(2, 0) };
        Eigen::Vector3d ry{ curr_camera2base.at<double>(0, 1), curr_camera2base.at<double>(1, 1), curr_camera2base.at<double>(2, 1) };
        Eigen::Vector3d position{ curr_camera2base.at<double>(0, 3), curr_camera2base.at<double>(1, 3), curr_camera2base.at<double>(2, 3) };
        Eigen::Vector3d rz = (goalRotatedEig - position).normalized();
        //Eigen::Vector3d rz {1., 0., 0.};

        // 将x、y方向（与设备当前朝向）斯密特正交化
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
        cout << "调整后相机位姿：" << dstStr1 << endl;
        cout << "调整后末端位姿：" << dstStr2 << endl;
        cout << "是旋转矩阵吗：" << isRotationMatrix(dstGripperMat) << endl;

        double distance = (goalRotatedEig - position).norm();
        cout << "当前起始点相对目标点的距离：" << distance << endl;

        MoveToOnePoint(dstStr2, &sp);
        cout << endl << endl << endl;
    //}
}

Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta) {
    cv::Mat diffXY = (cv::Mat_<double>(4, 1) << 0., -100., 0, 1); // 机械臂x、y偏心
    Eigen::Vector3d diffXYEig(0., -100, 0);
    robotPos += diffXYEig;
    double xDiff = agvMoveInfo[0];
    double yDiff = agvMoveInfo[1]; // agv坐标系
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
        ); // 顺时针theta为：负；（b1,b2,b3）=RI，H_curr2ori
    //// 偏心带来的旋转偏移
    //cv::Mat xDiffRotated = agvRotateCurr2Ori * diffXY;
    //cout << "偏心旋转偏移为：" << xDiffRotated.t() << endl;
    //xDiff += xDiffRotated.at<double>(0, 0);
    //yDiff += xDiffRotated.at<double>(1, 0);
    // 计算旋转、平移后的机械臂坐标
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
    //cout << "平移后的坐标为:" << translation.t() << endl;
    //cout << "旋转后的坐标为:" << rotate.t() << endl;
    //cout << "旋转平移后的坐标为:" << dstPosition.t() << endl;
    return { dstPosition.at<double>(0,0), dstPosition.at<double>(1,0), dstPosition.at<double>(2,0) };
}

void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos) {
    //cout << "--------------------------------:" << endl;
    //for (int i = 0; i < agvPos.size(); i++) {
    //    cout << agvPos[i].transpose() << endl;
    //}

    // 机械臂末端运动距离为：intervalDistance或旋转角度为：rotateTheta时保留一次轨迹点
    double intervalDistance = 50.; // mm
    double rotateTheta = 10. * CV_PI / 180.; 
    if (agvPos.size() <= 1) return;

    //for (int agvIdx = 0; agvIdx < agvPos.size(); agvIdx++) {
    //    cout << agvPos[agvIdx].transpose() << endl;
    //}

    Eigen::Vector3d lastPos = agvPos[0];
    for (int agvIdx = 1; agvIdx < agvPos.size(); agvIdx++) {
        Eigen::Vector3d currPos = agvPos[agvIdx];
        // 计算两点之间夹角，判断是否需要旋转AGV，逆时针旋转为正
        double angle = atan2((currPos[1] - lastPos[1]), (currPos[0] - lastPos[0]));
        //cout << "当前点坐标：" << currPos.transpose() << endl
        //    << "  前一个点坐标：" << lastPos.transpose() << endl
        //    << "    两点间夹角为：" << angle * 180. / CV_PI
        //    << "    夹角差：" << (angle - lastPos[2]) * 180. / CV_PI << endl;
        double agvRotate = 0.;
        //// 旋转
        //if (abs(angle - lastPos[2]) * 180. / CV_PI > 2.) {
        //    cout << "夹角大于设定角度，AGV需要旋转！" << endl;
        //    // 夹角大于x°需要旋转，采样旋转角度上的点
        //    for (int rTime = 0; rTime < floor(abs(angle - lastPos[2] / rotateTheta)); rTime++) {
        //        
        //    }
        //}
        // 平移
        double dist = (Eigen::Vector2d{ lastPos[0], lastPos[1] } - Eigen::Vector2d{ currPos[0], currPos[1] }).norm();
        int totalTTimes = abs(dist / intervalDistance); // floor(abs(dist / rotateTheta));
        //cout << "距离：" << dist << "    插入点次数：" << totalTTimes
        //    << "   当前AGV角度：" << angle << "    需要旋转角度：" << angle - agi._oriAgvPos[2]
        //    << "    AGV初始状态" << agi._oriAgvPos.transpose() << endl;
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
            cout << "旋转、平移为：" << xPos << "  " << yPos << "   插入点坐标：" << agi._oriArmPos.transpose() << "  转换后机械臂中心轨迹：" << goalPosRotated.transpose() << endl;
            vArmMovePosition.emplace_back(goalPosRotated);
        }
        vTest.push_back(vArmMovePosition.size());

        lastPos = currPos; // 更新上一个点位
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
    //cout << "输入坐标点通道、大小：" << armPoint.channels() << "    " << armPoint.size() << endl;
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
        cout << "通道数不为4*1、4*4或3*1!" << endl;
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
//    cout << "方向向量：" << centerDir.transpose() << endl;
//    Eigen::Vector2d centerPos = goal - centerDir * radius;
//    //Eigen::Vector2d centerPos_behind = goal + centerDir * radius;
//
//    vector<Eigen::Vector3d> vRes;
//    AMRLocalInfo currAmrli;
//    // 读取当前agv相对原始位置
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
//                cout << "方程有两个不等的实数根" << endl << "x1=" << y1 << ",x2=" << y2 << endl;
//            }
//            else if ((b * b - 4 * a * c) >= 0) {
//                y1 = -b / (2 * a);
//                y2 = y1;
//                cout << "方程有两个相等的实数根" << endl << "x1=x2=" << y1 << endl;
//            }
//            else {
//                cout << "方程无实数根" << endl;
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
//        cout << "中心点为：" << centerPos.transpose() << endl;
//        cout << "左1点为：" << centerPos_l1.transpose() << endl;
//        cout << "右1点为：" << centerPos_r1.transpose() << endl;
//
//        cv::Mat posl1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_l1[0], centerPos_l1[1] - 100., agi._worldGoalPos[2], 1.);
//        cv::Mat posr1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_r1[0], centerPos_r1[1] - 100., agi._worldGoalPos[2], 1.);
//
//        cout << "agv世界坐标中心点为：" << pos.t() << endl;
//        cout << "agv世界坐标左1点为：" << posl1.t() << endl;
//        cout << "agv世界坐标右1点为：" << posr1.t() << endl;
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
//        cout << "vRess" << i << "个点:" << vRess[i] << endl;
//
//    return vRes;
//}

vector<Eigen::Vector3d> CalcAgvPosForRGBD(double radius, double safedis, double theta) {
    theta = theta * CV_PI / 180.;
    radius += safedis;
    Eigen::Vector2d goal{ agi._worldGoalPos[0], agi._worldGoalPos[1] };
    Eigen::Vector2d centerDir = goal.normalized();
    cout << "方向向量：" << centerDir.transpose() << endl;
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
            cout << "方程有两个不等的实数根" << endl << "x1=" << y1 << ",x2=" << y2 << endl;
        }
        else if ((b * b - 4 * a * c) >= 0) {
            y1 = -b / (2 * a);
            cout << "方程有两个相等的实数根" << endl << "x1=x2=" << y1 << endl;
        }
        else {
            cout << "方程无实数根" << endl;
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
    cout << "中心点为：" << centerPos.transpose() << endl;
    cout << "左1点为：" << centerPos_l1.transpose() << endl;
    cout << "右1点为：" << centerPos_r1.transpose() << endl;

    AMRLocalInfo currAmrli;
    // 读取当前agv相对原始位置
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000;
    currAmrli._y *= 1000;

    cv::Mat pos = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos[0], centerPos[1] - 100., agi._worldGoalPos [2], 1.);
    cv::Mat posl1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_l1[0], centerPos_l1[1] - 100., agi._worldGoalPos [2], 1.);
    cv::Mat posr1 = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << centerPos_r1[0], centerPos_r1[1] - 100., agi._worldGoalPos [2], 1.);

    cout << "agv世界坐标中心点为：" << pos.t() << endl;
    cout << "agv世界坐标左1点为：" << posl1.t() << endl;
    cout << "agv世界坐标右1点为：" << posr1.t() << endl;
    vector<Eigen::Vector3d> vRes;
    //vRes.emplace_back(Eigen::Vector3d{ centerPos_l1[0], centerPos_l1[1], goal[2] });
    //vRes.emplace_back(Eigen::Vector3d{ centerPos[0], centerPos[1], goal[2] });
    //vRes.emplace_back(Eigen::Vector3d{ centerPos_r1[0], centerPos_r1[1], goal[2] });
    vRes.emplace_back(Eigen::Vector3d{ posl1.at<double>(0, 0), posl1.at<double>(1, 0), posl1.at<double>(2, 0) });
    vRes.emplace_back(Eigen::Vector3d{ pos.at<double>(0, 0), pos.at<double>(1, 0), pos.at<double>(2, 0) });
    vRes.emplace_back(Eigen::Vector3d{ posr1.at<double>(0, 0), posr1.at<double>(1, 0), posr1.at<double>(2, 0) });

    return vRes;
}