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
            tmppoints.at<float>(2, i) = z;
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