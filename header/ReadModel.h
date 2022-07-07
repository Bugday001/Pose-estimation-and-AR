#pragma once
#include "headfiles.h"
#include <fstream>
#include <stdio.h>
#include <chrono>

using namespace std;
using namespace cv;


class ReadModelFile
{
public:
    bool ReadSTL(const char* cfilename);
    bool ReadPly(string filename);
    int NumTri();
    vector<Point3f> pointList;
    Mat Point_mat;
    bool isstl = true;
private:
    unsigned int unTriangles;
    bool ReadBinary(const char* cfilename);
    char* memwriter;
    int cpyint(const char*& p);
    float cpyfloat(const char*& p);
};
