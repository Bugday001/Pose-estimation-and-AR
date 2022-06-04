#pragma once
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "PoseEstimation.h"

void Visualize2d(cv::Mat& test_image, cv::Mat new_image, ArrayofArray corners);
void Visualize2d_plus(cv::Mat& test_image, cv::Mat new_image, cv::Mat cameraMatrix,
				cv::Mat distCoeffs, std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, cv::Mat R);

void Visualize3d(cv::Mat& test_image, cv::Mat model, cv::Mat cameraMatrix,
	cv::Mat distCoeffs, std::vector<cv::Vec3d> rvec, std::vector<cv::Vec3d> tvec, cv::Mat R);