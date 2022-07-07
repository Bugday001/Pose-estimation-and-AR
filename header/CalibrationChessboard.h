#pragma once
#include "headfiles.h"
#include "detectMarkers.h"
#include "PoseEstimation.h"


using namespace cv;
const int ACTION_ESC = 27;
const int ACTION_SPACE = 32;

/**@brife 棋盘格标定
* @param img			原图
* @param ArrayofCorners 角点s
* @param boardSize		棋盘格尺寸
* @param squareSize		方格大小
* @param cameraMatrix	相机内参
* @return 是否完成标定
*/
bool myCalibrationOnline(Mat& img, ArrayofArray& ArrayofCorners, cv::Size boardSize, const float squareSize, Mat& cameraMatrix);

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

/**@brife 去畸变
* @param src            原始图像
* @param dst            去畸变后图像
* @param cameraMatrix   内参矩阵
* @param distCoeffs     畸变系数
*/
void Undistortion(cv::Mat src, cv::Mat& dst, cv::Mat cameraMatrix, cv::Mat distCoeffs);