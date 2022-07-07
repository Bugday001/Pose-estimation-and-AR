#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include "SomeCameraFuction.h"
#include "detectMarkers.h"
#include "PoseEstimation.h"


using namespace cv;

void myHarrisCorner_ave(Mat& srcImg);

/**@brife 棋盘格标定
* @param img			原图
* @param ArrayofCorners 角点s
* @param boardSize		棋盘格尺寸
* @param squareSize		方格大小
* @param cameraMatrix	相机内参
*/
void myCalibrationOnline(Mat& img, ArrayofArray& ArrayofCorners, cv::Size boardSize, const float squareSize, Mat& cameraMatrix);

/**@brife 计算内参
* @param imagePath		图路径
* @param boardSize		棋盘格尺寸
* @param squareSize		方格大小
*/
int mycalibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize);

/**@brife 计算内参
* @param ArrayofCorners	单应矩阵
* @param boardSize		棋盘格尺寸
* @param squareSize		方格大小
* @param cameraMatrix	相机内参
*/
void myCalibration(ArrayofArray ArrayofCorners, Size boardSize, const float squareSize, Mat& cameraMatrix);

