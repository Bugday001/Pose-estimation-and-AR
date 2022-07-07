#pragma once
#include "headfiles.h"
#include <highgui.h>


typedef std::vector<std::vector<cv::Point2f>> ArrayofArray;
typedef std::vector<cv::Vec3d> Array3d;
typedef std::vector<cv::Point2f> ArrayPoint2f;
/**@brife �õ���ת�����λ�þ���Ŀǰ����cornersֻ��һ��4��
 * @param corners           4�����x��y
 * @param markerLength      Arucoʵ�ʳߴ磬��
 * @param cameraMatrix      ����ڲ�
 * @param distCoeffs        ����������
 * @param rvecs             ��ת����
 * @param tvecs             ƽ������
 * @param R                 ��ת����
*/
void estimatePose(ArrayofArray corners, float markerLength, cv::Mat cameraMatrix, 
			cv::Mat distCoeffs, Array3d &rvecs, Array3d &tvecs, cv::Mat& R);
/**@brife �õ���Ӧ����
* @param src		��ǵ��ʵ������
* @param dst		����ϵ�����
* @param ransac		�Ƿ�ʹ��ransac�Ż�
*/
cv::Mat HomographyMat(ArrayPoint2f src, ArrayPoint2f dst, bool ransac);

