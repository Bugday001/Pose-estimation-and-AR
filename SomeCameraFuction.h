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
* ����·�� 
* @param path      ��Ҫ������·������
**/
int myMkdir(string &path);
/**
* ��ȡ��ǰ�����ڣ������գ�ʱ��
**/
string getCurrentTimeDate();
/**
* ��ȡ��ǰʱ�䣨1636797632��
**/
string getCurrentTime();

/**
* �������̸�ͼƬ
**/
int generateCalibrationPicture();
/**
* ʵʱ��ʾ����Ļ���
**/
int displayCameraRealTime();

/**
* ʵʱ��ʾ������棬��������ͼƬ
* ���ո�����棬��Esc�˳�
**/
int saveCameraImages(string savePath = "./");

/**
* ʵʱ��ʾ������棬���������ܼ�⵽�ǵ�� ���̸�ͼƬ
* ���ո�����棬��Esc�˳�
* @param boardSize      ���ӳߴ�Size 7*4
**/
int saveChessboardImages(cv::Size boardSize, string savePath = "./");

/**
* ִ�б궨������궨���
* @param squareSize		���ӳߴ� mm
* @param boardSize		���ӳߴ�Size 7*4
* @param image_size		ͼƬ�ߴ�Size 1920*1080
* @param image_points	ͼƬ�ǵ㼯��
* @param cameraMatrix	�������
* @param distCoeffs		�������
*/
void runCalibrationSave(float squareSize, const Size boardSize, const Size image_size,
	const vector<vector<Point2f>> &image_points,
	Mat &cameraMatrix, Mat &distCoeffs);

/**
* ʵʱ���ǵ㣬��������ǵ�������ﵽ����ִ�б궨������궨���
* @param numBoards			��Ҫ���ű궨ͼƬ������ȡ����ǵ����
* @param boardSize			���ӳߴ�Size 7*4
* @param squareSize			���ӳߴ� mm
* @param flipHorizontal		�Ƿ�ת
*/
int calibrateCameraRealTime(int numBoards, cv::Size boardSize, float squareSize = 1, int delay = 50, bool flipHorizontal = false);

/**
* ��ָ��ͼƬ�в��ҽǵ㣬������������corners��
* @param img		�����ͼƬ
* @param corners	��⵽�Ľǵ��б�
* @return			�Ƿ��⵽�ǵ㣨�����ڷ���Ľ��㣩
*/
bool findCorners(Mat &img, vector<Point2f> &corners, cv::Size boardSize);

/**
* ��������궨
* @param imagePath    �궨ͼƬ���·��
* @param boardSize    ���ӳߴ�Size 7*4
* @param squareSize   ���ӳߴ� mm
*/
int calibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize = 1);

/**
* ��ȡ�궨����
* @param path			�궨����YAML���·��
* @param image_size     ͼƬ�ߴ�Size 640*480
* @param cameraMatrix   �ڲ�
* @param distCoeffs		�������
*/
void readCalibrationYaml(string path, Size &image_size, Mat &cameraMatrix, Mat &distCoeffs);

/**
* ȥ���� 1������ͼƬ 2��ʵʱ���ͼ��
* @param path		�궨�������·��
* @param imagePath	��Ҫ������ͼƬ ���·��
*/
int undistortRectifyImage(string paraPath, string imagePath = " ");


/*
* ��ӡ������ò���
*/
void GetCameraParams();