#pragma once
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <vector>
#include "stl_visualize.h"

using namespace std;
using namespace cv;


class ReadSTLFile
{
public:
    bool ReadFile(const char* cfilename);
    int NumTri();
    //vector<Point3f>& PointList();
    vector<Point3f> pointList;
    Mat Point_mat;
private:
    unsigned int unTriangles;
    //bool ReadASCII(const char* cfilename);
    bool ReadBinary(const char* cfilename);

    char* memwriter;
    int cpyint(const char*& p);
    float cpyfloat(const char*& p);
};
