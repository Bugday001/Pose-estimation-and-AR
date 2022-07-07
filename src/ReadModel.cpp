#include "ReadModel.h"

using namespace std;
using namespace cv;
using namespace chrono;

/**目前只能读取二进制方式读取，无法读取Ascii格式
*/
bool ReadModelFile::ReadSTL(const char* cfilename)
{
    FILE* pFile;
    long lSize;
    char* buffer;
    size_t result;

    /* 若要一个byte不漏地读入整个文件，只能采用二进制方式打开 */
    fopen_s(&pFile, cfilename, "rb");
    if (pFile == NULL)
    {
        fputs("File error", stderr);
        exit(1);
    }

    /* 获取文件大小 */
    fseek(pFile, 0, SEEK_END);
    lSize = ftell(pFile);
    rewind(pFile);

    /* 分配内存存储整个文件 */
    buffer = (char*)malloc(sizeof(char) * lSize);
    if (buffer == NULL)
    {
        fputs("Memory error", stderr);
        exit(2);
    }

    /* 将文件拷贝到buffer中 */
    result = fread(buffer, 1, lSize, pFile);
    if (result != lSize)
    {
        fputs("Reading error", stderr);
        exit(3);
    }


    /* 结束演示，关闭文件并释放内存 */
    fclose(pFile);

    ios::sync_with_stdio(false);
    ReadBinary(buffer);
    ios::sync_with_stdio(true);

    free(buffer);
    isstl = true;
    return true;
}


bool ReadModelFile::ReadBinary(const char* buffer)
{
    const char* p = buffer;
    char name[80];
    int i, j;
    memcpy(name, p, 80);
    p += 80;
    unTriangles = cpyint(p);
    Mat tmppoints = Mat::zeros(3, unTriangles, CV_64F);
    //缩放倍数
    double ratio = 800;
    for (i = 0; i < unTriangles; i++)
    {
        p += 12;//跳过头部法向量
        for (j = 0; j < 3; j++) {//读取三顶点
            float x = cpyfloat(p), y = cpyfloat(p), z = cpyfloat(p);
            pointList.push_back(Point3f(x, y, z));
            tmppoints.at<double>(0, i) = (double)x;
            tmppoints.at<double>(1, i) = (double)y;
            tmppoints.at<double>(2, i) = (double)z;
        }  
        p += 2;//跳过尾部标志
    }
    Point_mat = tmppoints / ratio;
    return true;
}


bool isFileExists_ifstream(string& name) {
    ifstream f(name.c_str());
    return f.good();
}

/**brief    读取ply文件
* @param filename   文件名
*/
bool ReadModelFile::ReadPly(string filename)
{
    isstl = false;
    ifstream ifs(filename);
    bool ret = isFileExists_ifstream(filename);
    if (ret) {
        string str1, str2, str3;
        int vector_length = 0;
        for (size_t i = 0; i < 12; ++i) {
            /*getline(ifs, str);*/
            ifs >> str1;
            if (str1 == "ply") {
                continue;
            }
            else if (str1 == "end_header") {
                break;
            }
            else {
                ifs >> str2 >> str3;
                if (str1 == "element" && str2 == "vertex")
                    vector_length = stoi(str3);
            }
        }
        Mat cloud(3, vector_length, CV_64F);
        Point3f data;
        float dummy1, dummy2;
        //缩放倍数
        double ratio = 1.5;
        for (size_t i = 0; i < vector_length; ++i) {
            ifs >> data.x >> data.y >> data.z >> dummy1 >> dummy2;
            cloud.at<double>(1, i) = (double)data.x;
            cloud.at<double>(2, i) = (double)data.y-0.2;
            cloud.at<double>(0, i) = (double)data.z;
        }
        Point_mat = cloud / ratio;
        return true;
    }
    else {
        cout << "ply error!!!!!!!!!!!!\n";
        return false;
    }

}
int ReadModelFile::NumTri()
{
    return unTriangles;
}

int ReadModelFile::cpyint(const char*& p)
{
    int cpy;
    memwriter = (char*)&cpy;
    memcpy(memwriter, p, 4);
    p += 4;
    return cpy;
}
float ReadModelFile::cpyfloat(const char*& p)
{
    float cpy;
    memwriter = (char*)&cpy;
    memcpy(memwriter, p, 4);
    p += 4;
    return cpy;
}