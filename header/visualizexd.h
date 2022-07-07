#pragma once
#include "headfiles.h"
#include <opencv2/aruco.hpp>
#include "PoseEstimation.h"
#include "ReadModel.h"

/**@brife 使用图片替换Aruco码
* @param test_image     用于显示的图像Mat
* @param new_image		替换的图像Mat
* @param cameraMatrix   Aruco角点
*/
void Visualize2d(cv::Mat& test_image, cv::Mat new_image, ArrayofArray corners);

/**@brife 在与Aruco垂直的平面显示图像
* @param test_image     用于显示的图像Mat
* @param model			替换的图像Mat
* @param cameraMatrix   相机内参
* @param distCoeffs     相机畸变参数
* @param tvec			旋转向量
* @param tvec			平移向量
* @param R				旋转矩阵
*/
void Visualize2d_plus(cv::Mat& test_image, cv::Mat new_image, cv::Mat cameraMatrix,
				cv::Mat distCoeffs, std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, cv::Mat R);

/**@brife 显示读取的3D模型
* @param test_image     用于显示的图像Mat
* @param model			STL模型读取的三角面角点
* @param cameraMatrix   相机内参
* @param distCoeffs     相机畸变参数
* @param tvec			平移向量
* @param R				旋转矩阵
*/
void Visualize3d(cv::Mat& test_image, ReadModelFile model, cv::Mat cameraMatrix,
	cv::Mat distCoeffs, std::vector<cv::Vec3d> tvec, cv::Mat R);


/**@brife 透视变换
* @param src			原图
* @param dst			输出图
* @param H				单应矩阵
* @param size			输出图大小
*/
void mywarpPerspective(Mat src, Mat& dst, Mat H, Size size);

/**@brife 线性混合
* @param src1			原图
* @param alpha			混合权重1
* @param src2			原图2
* @param beta			混合权重2
* @param src3			输出图像
*/
void myaddweight(Mat& src1, double alpha, Mat& src2, double beta, Mat& src3);