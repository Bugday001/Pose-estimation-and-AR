#pragma once
#include "headfiles.h"
#include <opencv2/aruco.hpp>
#include "PoseEstimation.h"
#include "ReadModel.h"

/**@brife ʹ��ͼƬ�滻Aruco��
* @param test_image     ������ʾ��ͼ��Mat
* @param new_image		�滻��ͼ��Mat
* @param cameraMatrix   Aruco�ǵ�
*/
void Visualize2d(cv::Mat& test_image, cv::Mat new_image, ArrayofArray corners);

/**@brife ����Aruco��ֱ��ƽ����ʾͼ��
* @param test_image     ������ʾ��ͼ��Mat
* @param model			�滻��ͼ��Mat
* @param cameraMatrix   ����ڲ�
* @param distCoeffs     ����������
* @param tvec			��ת����
* @param tvec			ƽ������
* @param R				��ת����
*/
void Visualize2d_plus(cv::Mat& test_image, cv::Mat new_image, cv::Mat cameraMatrix,
				cv::Mat distCoeffs, std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, cv::Mat R);

/**@brife ��ʾ��ȡ��3Dģ��
* @param test_image     ������ʾ��ͼ��Mat
* @param model			STLģ�Ͷ�ȡ��������ǵ�
* @param cameraMatrix   ����ڲ�
* @param distCoeffs     ����������
* @param tvec			ƽ������
* @param R				��ת����
*/
void Visualize3d(cv::Mat& test_image, ReadModelFile model, cv::Mat cameraMatrix,
	cv::Mat distCoeffs, std::vector<cv::Vec3d> tvec, cv::Mat R);


/**@brife ͸�ӱ任
* @param src			ԭͼ
* @param dst			���ͼ
* @param H				��Ӧ����
* @param size			���ͼ��С
*/
void mywarpPerspective(Mat src, Mat& dst, Mat H, Size size);

/**@brife ���Ի��
* @param src1			ԭͼ
* @param alpha			���Ȩ��1
* @param src2			ԭͼ2
* @param beta			���Ȩ��2
* @param src3			���ͼ��
*/
void myaddweight(Mat& src1, double alpha, Mat& src2, double beta, Mat& src3);