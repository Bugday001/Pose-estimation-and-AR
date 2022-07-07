#pragma once
#include "headfiles.h"
#include <highgui.h>


typedef std::vector<std::vector<cv::Point2f>> ArrayofArray;
typedef std::vector<cv::Vec3d> Array3d;
typedef std::vector<cv::Point2f> ArrayPoint2f;
/**@brife 得到旋转矩阵和位置矩阵，目前假设corners只有一组4个
 * @param corners           4个点的x，y
 * @param markerLength      Aruco实际尺寸，米
 * @param cameraMatrix      相机内参
 * @param distCoeffs        相机畸变参数
 * @param rvecs             旋转向量
 * @param tvecs             平移向量
 * @param R                 旋转矩阵
*/
void estimatePose(ArrayofArray corners, float markerLength, cv::Mat cameraMatrix, 
			cv::Mat distCoeffs, Array3d &rvecs, Array3d &tvecs, cv::Mat& R);
/**@brife 得到单应矩阵
* @param src		标记点的实际坐标
* @param dst		相机上的坐标
* @param ransac		是否使用ransac优化
*/
cv::Mat HomographyMat(ArrayPoint2f src, ArrayPoint2f dst, bool ransac);

