#pragma once
#include "headfiles.h"
#include "detectMarkers.h"
#include "PoseEstimation.h"


using namespace cv;
const int ACTION_ESC = 27;
const int ACTION_SPACE = 32;

/**@brife ���̸�궨
* @param img			ԭͼ
* @param ArrayofCorners �ǵ�s
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
* @param cameraMatrix	����ڲ�
* @return �Ƿ���ɱ궨
*/
bool myCalibrationOnline(Mat& img, ArrayofArray& ArrayofCorners, cv::Size boardSize, const float squareSize, Mat& cameraMatrix);

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

/**@brife ȥ����
* @param src            ԭʼͼ��
* @param dst            ȥ�����ͼ��
* @param cameraMatrix   �ڲξ���
* @param distCoeffs     ����ϵ��
*/
void Undistortion(cv::Mat src, cv::Mat& dst, cv::Mat cameraMatrix, cv::Mat distCoeffs);