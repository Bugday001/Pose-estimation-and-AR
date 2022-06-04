#include "stl_visualize.h"

using namespace std;
using namespace cv;
using namespace chrono;


bool ReadSTLFile::ReadFile(const char* cfilename)
{
    FILE* pFile;
    long lSize;
    char* buffer;
    size_t result;

    /* ��Ҫһ��byte��©�ض��������ļ���ֻ�ܲ��ö����Ʒ�ʽ�� */
    fopen_s(&pFile, cfilename, "rb");
    if (pFile == NULL)
    {
        fputs("File error", stderr);
        exit(1);
    }

    /* ��ȡ�ļ���С */
    fseek(pFile, 0, SEEK_END);
    lSize = ftell(pFile);
    rewind(pFile);

    /* �����ڴ�洢�����ļ� */
    buffer = (char*)malloc(sizeof(char) * lSize);
    if (buffer == NULL)
    {
        fputs("Memory error", stderr);
        exit(2);
    }

    /* ���ļ�������buffer�� */
    result = fread(buffer, 1, lSize, pFile);
    if (result != lSize)
    {
        fputs("Reading error", stderr);
        exit(3);
    }


    /* ������ʾ���ر��ļ����ͷ��ڴ� */
    fclose(pFile);

    ios::sync_with_stdio(false);
    //if (buffer[79] != '\0')//�жϸ�ʽ
    //{
    //    ReadASCII(buffer);
    //}
    if(1)
    {
        ReadBinary(buffer);
    }
    ios::sync_with_stdio(true);

    free(buffer);
    return true;
}

bool ReadSTLFile::ReadASCII(const char* buffer)
{
    unTriangles = 0;
    float x, y, z;
    int i;
    string name, useless;
    stringstream ss(buffer);
    ss >> name >> name;
    ss.get();
    Mat tmppoints = Mat::zeros(1, 3, CV_32F);
    do {
        ss >> useless;
        if (useless != "facet")
            break;
        getline(ss, useless);
        getline(ss, useless);
        for (i = 0; i < 3; i++)
        {
            ss >> useless >> x >> y >> z;
            pointList.push_back(Point3f(x, y, z));
            Mat row = (Mat_<float>(1,3)<<x,y,z);//����һ��1��3�е�mat,��double�����ʼ��
            tmppoints.push_back(row);//���һ����test
        }
        unTriangles++;
        getline(ss, useless);
        getline(ss, useless);
        getline(ss, useless);
    } while (1);
    Point_mat = tmppoints.t();
    return true;
}


bool ReadSTLFile::ReadBinary(const char* buffer)
{
    const char* p = buffer;
    char name[80];
    int i, j;
    memcpy(name, p, 80);
    p += 80;
    unTriangles = cpyint(p);
    Mat tmppoints = Mat::zeros(3, unTriangles, CV_32F);
    for (i = 0; i < unTriangles; i++)
    {
        p += 12;//����ͷ��������
        for (j = 0; j < 3; j++) {//��ȡ������
            float x = cpyfloat(p), y = cpyfloat(p), z = cpyfloat(p);
            pointList.push_back(Point3f(x, y, z));
            tmppoints.at<float>(0, i) = x;
            tmppoints.at<float>(1, i) = y;
            tmppoints.at<float>(2, i) = -z;
        }  
        p += 2;//����β����־
    }
    Point_mat = tmppoints;
    return true;
}

int ReadSTLFile::NumTri()
{
    return unTriangles;
}

//vector<Point3f>& ReadSTLFile::PointList()
//{
//    return pointList;
//}

int ReadSTLFile::cpyint(const char*& p)
{
    int cpy;
    memwriter = (char*)&cpy;
    memcpy(memwriter, p, 4);
    p += 4;
    return cpy;
}
float ReadSTLFile::cpyfloat(const char*& p)
{
    float cpy;
    memwriter = (char*)&cpy;
    memcpy(memwriter, p, 4);
    p += 4;
    return cpy;
}


/********************************************************************/
Mat canvas = Mat::zeros(Size(950, 900), CV_8UC1);
int step_max = 10;//쳲�����������
int self_r_max = 60; //��ת�ֱ��ʣ���360�ȷ�Ϊself_r_max��
Point2d offset{ 470, 470 };   //��ͼ��
double scalar = 7;         //��ͼ�ã�����ģ�ͱ���


/****************************************************************/
void read_stl(string path, vector<double>& triangles)
{
    ifstream stl(path, fstream::binary);
    stl.seekg(80);
    char _head[4];
    stl.read(_head, 4);
    stl.seekg(84);
    uint32_t triangle_size;
    memmove(&triangle_size, _head, 4);
    vector<double> points(triangle_size * 50);
    stl.read((char*)&points[0], triangle_size * 50);
    triangles.resize(9 * triangle_size);
    float point3f_3[9];

    for (int i = 0; i < triangle_size; i++)
    {
        memmove(point3f_3, &(points[i * 50 + 12]), 36);
        for (size_t j = 0; j < 3; j++)
            for (size_t k = 0; k < 3; k++)
                triangles[i * 9 + j * 3 + k] = point3f_3[j * 3 + k];
    }
}

void draw_stl_xy(Mat& canvas, vector<double>& triangles, int is_fill_triangle = 0)    //���һ�����������Ƿ�Ϳ��ÿ��������
{
    Point2d p[3];

    for (int i = 0; i < triangles.size() / 9; i++)
    {
        vector<Point> triangle;
        for (size_t j = 0; j < 3; j++)
        {
            p[j] = { triangles[i * 9 + j * 3 + 0], triangles[i * 9 + j * 3 + 1] };
            p[j] *= scalar;
            p[j] += offset;
            if (is_fill_triangle)triangle.emplace_back(p[j]);
        }
        if (is_fill_triangle)
            drawContours(canvas, vector<vector<Point>>{triangle}, -1, Scalar(255), FILLED);
        else
        {
            line(canvas, p[0], p[1], Scalar(255));
            line(canvas, p[1], p[2], Scalar(255));
            line(canvas, p[0], p[2], Scalar(255));
        }
    }
}

void rotate_stl(vector<double>& triangles_in, vector<double>& triangles_out, vector<double>& r_vector)
{
    Mat r_matrix; //��Ǿ���
    Rodrigues(r_vector, r_matrix);
    Mat in_matrix = Mat(triangles_in).reshape(1, triangles_in.size() / 3);
    Mat out_matrix = in_matrix * (r_matrix.t());
    triangles_out = out_matrix.reshape(1, 1);
}

void fibonacci_sphere(int index, int max, vector<double>& point)
{
    double phi = 3.1415926 * (3 - sqrt(5));
    double radius, theta;

    if (point.size() != 3)return;
    point[1] = 1 - (index / 1.0 / max) * 2;
    radius = sqrt(1 - point[1] * point[1]);
    theta = phi * index;
    point[0] = cos(theta) * radius;
    point[2] = sin(theta) * radius;
}

int stl_visual()
{
    vector<double> triangles_all;
    vector<double> triangles_rt;
    vector<double> triangles_rt_self;

    read_stl("img/1.stl", triangles_all);

    namedWindow("a", WINDOW_NORMAL);
    vector<double> r_vector(3); //��ǵ���
    vector<double> self_r_vector(3); //��ת��ǵ���
    vector<double> product_v(3); //yaw{0,1,0}����������Ĳ�ˣ���ʽ�������򣬾������򻯵�1
    Mat _canvas;        //�ջ���
    vector<double> yaw{ 0,0,1 };
    for (int k = 0; k < step_max; k++)
    {
        auto start = chrono::steady_clock::now();

        fibonacci_sphere(k, step_max, r_vector);

        product_v = {
            yaw[1] * r_vector[2] - yaw[2] * r_vector[1],
            yaw[2] * r_vector[0] - yaw[0] * r_vector[2],
            yaw[0] * r_vector[1] - yaw[1] * r_vector[0] };

        normalize(product_v, product_v, acos(yaw[0] * r_vector[0] + yaw[1] * r_vector[1] + yaw[2] * r_vector[2]), NORM_L2); //�Ƕȹ�ʽ�Ѿ���

        rotate_stl(triangles_all, triangles_rt, product_v);
        triangles_rt_self = triangles_rt;

        for (size_t n = 0; n < self_r_max; n++)
        {
            double self_scalar = 3.1415926 * 2 * n / self_r_max;
            self_r_vector[0] = r_vector[0] * self_scalar;
            self_r_vector[1] = r_vector[1] * self_scalar;
            self_r_vector[2] = r_vector[2] * self_scalar;
            rotate_stl(triangles_rt, triangles_rt_self, self_r_vector);

            _canvas = canvas.clone();
            //���һ������ѡ���Ƿ���Ϳ�����棬���ڴ�����ʵ�֣�û����ͼ�ο⣬���Ի�Ƚ���
            draw_stl_xy(_canvas, triangles_rt_self /*,true*/);
            //��һ����ת��
            line(_canvas, offset + 1500 * Point2d{ self_r_vector[0], self_r_vector[1] }, offset + Point2d{ 0, 0 }, Scalar(255), 2);
            imshow("a", _canvas);
            waitKey(1);
        }
        auto end = chrono::steady_clock::now();
        double diff_sec = chrono::duration_cast<chrono::duration<double>>(end - start).count();
        cout << diff_sec * 1000 << endl;
    }

    waitKey(0);

    return 0;
}
