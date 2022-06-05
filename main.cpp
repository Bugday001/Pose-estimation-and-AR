#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<string>
#include "PoseDetectAruco.h"
#include "SomeCameraFuction.h"
#include "HarrisCorner.h"
#include "stl_visualize.h"

using namespace cv;
using namespace std;

//生成棋盘格图片
void testGenerateCalibrationPicture()
{
	generateCalibrationPicture();
}
//在线实时标定
void testCalibrateCameraRealTime()
{
	int boardWidth = 6;
	int boardHeight = 4;
	float squareSize = 29;
	int numBoards = 15;
	cv::Size boardSize = cv::Size(boardWidth, boardHeight);
	std::cout << "boardWidth: " << boardWidth << std::endl;
	std::cout << "boardHeight: " << boardHeight << std::endl;
	std::cout << "squareSize: " << squareSize << std::endl;
	std::cout << "numBoards: " << numBoards << std::endl;
	calibrateCameraRealTime(numBoards, boardSize, squareSize);
}

//实时显示相机画面，按键保存能检测到角点的 棋盘格图片
void testSaveChessboardImages()
{
	string savePath("ChessboardImages");
	saveChessboardImages(cv::Size(6, 4), savePath);
}

//离线相机标定
void testCalibrateCameraOffLine()
{
	// 棋盘格的尺寸（宽6，高9）
	const Size patternSize(6, 4);
	// 黑方格的大小 20mm
	const float squareSize = 29;
	// 图片路径
	cv::String imagePath = "./ChessboardImages/*.png";
	calibrateCameraOffLine(imagePath, patternSize, squareSize);
}
//实时显示相机画面，按键保存图片
void testSaveCameraImages()
{
	string savePath("image");
	saveCameraImages(savePath);
}
//实时显示相机画面
void testDisplayCameraRealTime() 
{
	displayCameraRealTime();
}

//去畸变 1、本地图片
void testUndistortRectifyImageRT()
{
	string paraPath("calibration_in_params1636797037.yml");
	undistortRectifyImage(paraPath);
}
//去畸变 2、实时相机图像
void testUndistortRectifyImage()
{
	string paraPath("calibration_in_params1652946481.yml");
	//string imgPath("image/1637628270.png");
	undistortRectifyImage(paraPath, " ");
}
//角点检测
void CornerDetect() 
{
	Mat img = imread("./ChessboardImages/1652945137.png");
	myHarrisCorner_ave(img);
	waitKey(0);
}
int main(int argc, char **argv) {
	std::cout << cv::getVersionString() << std::endl;
	//GetCameraParams();
	//testSaveChessboardImages();
	//testCalibrateCameraOffLine();
	//testCalibrateCameraRealTime();
	//testUndistortRectifyImkage();
	//testDisplayCameraRealTime();
	//testSaveCameraImages();
	// Aruco Pose Opencv
	//detectPoseShow();
	//CornerDetect();
	//read stl
	ReadSTLFile STL;
	STL.ReadFile("img/huosai.STL");
	detectPoseShow(STL);
	system("pause");
	return 0;
}
