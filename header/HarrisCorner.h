#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include "SomeCameraFuction.h"
#include "detectMarkers.h"
#include "PoseEstimation.h"


using namespace cv;

void myHarrisCorner_ave(Mat& srcImg);

/**@brife ���̸�궨
* @param img			ԭͼ
* @param ArrayofCorners �ǵ�s
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
* @param cameraMatrix	����ڲ�
*/
void myCalibrationOnline(Mat& img, ArrayofArray& ArrayofCorners, cv::Size boardSize, const float squareSize, Mat& cameraMatrix);

/**@brife �����ڲ�
* @param imagePath		ͼ·��
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
*/
int mycalibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize);

/**@brife �����ڲ�
* @param ArrayofCorners	��Ӧ����
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
* @param cameraMatrix	����ڲ�
*/
void myCalibration(ArrayofArray ArrayofCorners, Size boardSize, const float squareSize, Mat& cameraMatrix);

