#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<string>
#include "PoseDetectAruco.h"
#include "SomeCameraFuction.h"
#include "HarrisCorner.h"

using namespace cv;
using namespace std;

//�������̸�ͼƬ
void testGenerateCalibrationPicture()
{
	generateCalibrationPicture();
}
//����ʵʱ�궨
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

//ʵʱ��ʾ������棬���������ܼ�⵽�ǵ�� ���̸�ͼƬ
void testSaveChessboardImages()
{
	string savePath("ChessboardImages");
	saveChessboardImages(cv::Size(6, 4), savePath);
}

//��������궨
void testCalibrateCameraOffLine()
{
	// ���̸�ĳߴ磨��6����9��
	const Size patternSize(6, 4);
	// �ڷ���Ĵ�С 20mm
	const float squareSize = 29;
	// ͼƬ·��
	cv::String imagePath = "./ChessboardImages/*.png";
	calibrateCameraOffLine(imagePath, patternSize, squareSize);
}
//ʵʱ��ʾ������棬��������ͼƬ
void testSaveCameraImages()
{
	string savePath("image");
	saveCameraImages(savePath);
}
//ʵʱ��ʾ�������
void testDisplayCameraRealTime() 
{
	displayCameraRealTime();
}

//ȥ���� 1������ͼƬ
void testUndistortRectifyImageRT()
{
	string paraPath("calibration_in_params1636797037.yml");
	undistortRectifyImage(paraPath);
}
//ȥ���� 2��ʵʱ���ͼ��
void testUndistortRectifyImage()
{
	string paraPath("calibration_in_params1652946481.yml");
	//string imgPath("image/1637628270.png");
	undistortRectifyImage(paraPath, " ");
}
//�ǵ���
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
	detectPoseShow();
	//CornerDetect();

	system("pause");
	return 0;
}
