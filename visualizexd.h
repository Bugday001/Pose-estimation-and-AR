#pragma once
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "PoseEstimation.h"
#include "PoseEstimation.h"


void Visualize2d(cv::Mat& test_image, cv::Mat new_image, ArrayofArray corners);
void Visualize2d_plus(cv::Mat& test_image, cv::Mat new_image, cv::Mat cameraMatrix,
				cv::Mat distCoeffs, std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, cv::Mat R);
/**@brife 显示STL读取的模型
* @param test_image     用于显示的图像Mat
* @param model			STL模型读取的三角面角点
* @param cameraMatrix   相机内参
* @param distCoeffs     相机畸变参数
* @param tvec			旋转向量
* @param R				旋转矩阵
*/
void Visualize3d(cv::Mat& test_image, cv::Mat model, cv::Mat cameraMatrix,
	cv::Mat distCoeffs, std::vector<cv::Vec3d> tvec, cv::Mat R);