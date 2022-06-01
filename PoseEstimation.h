#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<string>

typedef std::vector<std::vector<cv::Point2f>> ArrayofArray;
typedef std::vector<cv::Vec3d> Array3d;
typedef std::vector<cv::Point2f> ArrayPoint2f;

void estimatePose(ArrayofArray corners, float markerLength,
    cv::Mat cameraMatrix, cv::Mat distCoeffs, Array3d &rvecs, Array3d &tvecs);

cv::Mat HomographyMat(ArrayPoint2f src, ArrayPoint2f dst);