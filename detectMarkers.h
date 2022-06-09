#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/** @brief 建立预定义的字典
* @returns vector<vector<int>>         返回字典
*/
vector<vector<int>> myGetPredefinedDictionary(void);

/** @brief 自适应二值化
* @param input_img      输入图像矩阵
* @param blockSize      取平均值的像素范围，建议值15
* @param delta          阈值 = 均值 - delta
* @returns Mat          返回二值化以后的图像矩阵
*/
Mat myAdaptiveThreshold(Mat input_img, int blockSize, int delta);

/**
* @brife 利用余弦定理取得三角形abc在a处的余弦值
* @param b 点b
* @param c 点c
* @param a 点a
* @returns double 夹角余弦值
*/
double getCOS(Point b, Point c, Point a);

/**
* @brife 获取点集到图像边缘的最小距离
* @param img			输入图像
* @param approxCurve	多边形点集
* @returns double		到图像边缘的最小距离
*/
double getMinBorderDistance(Mat img, vector<Point> approxCurve);

/**
* @brife 在图上找到可能的矩形
* @param img							输入图像矩阵(要求是灰度图)
* @param limitCosine					最小cos值
* @param minCurveLengthRate				最小周长率(与图像尺寸相关)		默认=0.03
* @param maxCurveLengthRate				最大周长率(与图像尺寸相关)		默认=4.0
* @param ApproxAccuracyRate				逼近精度						默认=0.05
* @param minBorderDistanceRate			轮廓到边界的最小距离率		默认=0.05
* @returns vector<vector<Point2f>>		找到的可能矩形 为nx4的向量
*/
vector<vector<Point2f>> findSquares(Mat img, double limitCosine, double minCurveLengthRate,
	double maxCurveLengthRate, double ApproxAccuracyRate, double minBorderDistanceRate);

/**
* @brife 在图中画出四边形框
* @param img		 在img上画框
* @param quads		 四边形点集
*/
void drawContours(Mat img, vector<vector<Point2f>> quads);

/**
* @brife 检查Marker有效性
* @param cell			存储Marker二值信息的数组
* @returns bool		    检查结果
*/
bool borderCheck(int* cell);

/**@brife 判断该Aruco marker是否包含在字典当中
* @param cell_without_border		存储Marker二值信息的二维数组(去除边界,6x6)
* @param dict						字典
* @param errorRate					可接受错误率
* @return int						如果在字典当中，则返回id，否则返回0
*/
int checkInDictionary(int* cell_without_border, vector<vector<int>> dict, float errorRate);

/**
* @brife 检测Markers
* @param img						输入图像
* @param dictionary					输入字典
* @param Markers					将找到的Marker角点集存储在这个二维向量中
* @param ids						将找到的Marker的id存储在这个一维向量中
*/
void myMarkerDetector(Mat img, vector<vector<int>> dictionary, vector<vector<Point2f>>& markerCorners, vector<int>& markerIds);

/**@brife 在图上画出aruco图示
* @param img				需要画图的图像矩阵
* @param markerCorners		marker角点集
* @param markerIds			marker编号
* @param cameraIntrinsics	相机内参
* @param cameraDistortion	相机畸变矩阵
*/
void myMarkerDrawer(Mat img, vector<vector<Point2f>> markerCorners, vector<int> markerIds, Mat cameraIntrinsics, Mat cameraDistortion);