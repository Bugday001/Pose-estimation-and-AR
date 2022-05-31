#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<string>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#elif _LINUX
#include <stdarg.h>
#include <sys/stat.h>
#endif

#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#elif _LINUX
#define ACCESS access
#define MKDIR(a) mkdir((a),0755)
#endif

using namespace cv;
using namespace std;

/**
* 创建路径 
* @param path      需要创建的路径名称
**/
int myMkdir(string &path);
/**
* 获取当前，星期，年月日，时间
**/
string getCurrentTimeDate();
/**
* 获取当前时间（1636797632）
**/
string getCurrentTime();

/**
* 生成棋盘格图片
**/
int generateCalibrationPicture();
/**
* 实时显示相机的画面
**/
int displayCameraRealTime();

/**
* 实时显示相机画面，按键保存图片
* 按空格键保存，按Esc退出
**/
int saveCameraImages(string savePath = "./");

/**
* 实时显示相机画面，按键保存能检测到角点的 棋盘格图片
* 按空格键保存，按Esc退出
* @param boardSize      格子尺寸Size 7*4
**/
int saveChessboardImages(cv::Size boardSize, string savePath = "./");

/**
* 执行标定并保存标定结果
* @param squareSize		格子尺寸 mm
* @param boardSize		格子尺寸Size 7*4
* @param image_size		图片尺寸Size 1920*1080
* @param image_points	图片角点集合
* @param cameraMatrix	相机参数
* @param distCoeffs		畸变参数
*/
void runCalibrationSave(float squareSize, const Size boardSize, const Size image_size,
	const vector<vector<Point2f>> &image_points,
	Mat &cameraMatrix, Mat &distCoeffs);

/**
* 实时检测角点，按键保存角点参数，达到数量执行标定并保存标定结果
* @param numBoards			需要几张标定图片，即获取几组角点参数
* @param boardSize			格子尺寸Size 7*4
* @param squareSize			格子尺寸 mm
* @param flipHorizontal		是否翻转
*/
int calibrateCameraRealTime(int numBoards, cv::Size boardSize, float squareSize = 1, int delay = 50, bool flipHorizontal = false);

/**
* 在指定图片中查找角点，并将结果输出到corners中
* @param img		待检测图片
* @param corners	检测到的角点列表
* @return			是否检测到角点（两个黑方格的交点）
*/
bool findCorners(Mat &img, vector<Point2f> &corners, cv::Size boardSize);

/**
* 离线相机标定
* @param imagePath    标定图片存放路径
* @param boardSize    格子尺寸Size 7*4
* @param squareSize   格子尺寸 mm
*/
int calibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize = 1);

/**
* 获取标定参数
* @param path			标定参数YAML存放路径
* @param image_size     图片尺寸Size 640*480
* @param cameraMatrix   内参
* @param distCoeffs		畸变参数
*/
void readCalibrationYaml(string path, Size &image_size, Mat &cameraMatrix, Mat &distCoeffs);

/**
* 去畸变 1、本地图片 2、实时相机图像
* @param path		标定参数存放路径
* @param imagePath	需要矫正的图片 存放路径
*/
int undistortRectifyImage(string paraPath, string imagePath = " ");


/*
* 打印相机设置参数
*/
void GetCameraParams();