#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/** @brief ����Ԥ������ֵ�
* @returns vector<vector<int>>         �����ֵ�
*/
vector<vector<int>> myGetPredefinedDictionary(void);

/** @brief ����Ӧ��ֵ��
* @param input_img      ����ͼ�����
* @param blockSize      ȡƽ��ֵ�����ط�Χ������ֵ15
* @param delta          ��ֵ = ��ֵ - delta
* @returns Mat          ���ض�ֵ���Ժ��ͼ�����
*/
Mat myAdaptiveThreshold(Mat input_img, int blockSize, int delta);

/**
* @brife �������Ҷ���ȡ��������abc��a��������ֵ
* @param b ��b
* @param c ��c
* @param a ��a
* @returns double �н�����ֵ
*/
double getCOS(Point b, Point c, Point a);

/**
* @brife ��ȡ�㼯��ͼ���Ե����С����
* @param img			����ͼ��
* @param approxCurve	����ε㼯
* @returns double		��ͼ���Ե����С����
*/
double getMinBorderDistance(Mat img, vector<Point> approxCurve);

/**
* @brife ��ͼ���ҵ����ܵľ���
* @param img							����ͼ�����(Ҫ���ǻҶ�ͼ)
* @param limitCosine					��Сcosֵ
* @param minCurveLengthRate				��С�ܳ���(��ͼ��ߴ����)		Ĭ��=0.03
* @param maxCurveLengthRate				����ܳ���(��ͼ��ߴ����)		Ĭ��=4.0
* @param ApproxAccuracyRate				�ƽ�����						Ĭ��=0.05
* @param minBorderDistanceRate			�������߽����С������		Ĭ��=0.05
* @returns vector<vector<Point2f>>		�ҵ��Ŀ��ܾ��� Ϊnx4������
*/
vector<vector<Point2f>> findSquares(Mat img, double limitCosine, double minCurveLengthRate,
	double maxCurveLengthRate, double ApproxAccuracyRate, double minBorderDistanceRate);

/**
* @brife ��ͼ�л����ı��ο�
* @param img		 ��img�ϻ���
* @param quads		 �ı��ε㼯
*/
void drawContours(Mat img, vector<vector<Point2f>> quads);

/**
* @brife ���Marker��Ч��
* @param cell			�洢Marker��ֵ��Ϣ������
* @returns bool		    �����
*/
bool borderCheck(int* cell);

/**@brife �жϸ�Aruco marker�Ƿ�������ֵ䵱��
* @param cell_without_border		�洢Marker��ֵ��Ϣ�Ķ�ά����(ȥ���߽�,6x6)
* @param dict						�ֵ�
* @param errorRate					�ɽ��ܴ�����
* @return int						������ֵ䵱�У��򷵻�id�����򷵻�0
*/
int checkInDictionary(int* cell_without_border, vector<vector<int>> dict, float errorRate);

/**
* @brife ���Markers
* @param img						����ͼ��
* @param dictionary					�����ֵ�
* @param Markers					���ҵ���Marker�ǵ㼯�洢�������ά������
* @param ids						���ҵ���Marker��id�洢�����һά������
*/
void myMarkerDetector(Mat img, vector<vector<int>> dictionary, vector<vector<Point2f>>& markerCorners, vector<int>& markerIds);

/**@brife ��ͼ�ϻ���arucoͼʾ
* @param img				��Ҫ��ͼ��ͼ�����
* @param markerCorners		marker�ǵ㼯
* @param markerIds			marker���
* @param cameraIntrinsics	����ڲ�
* @param cameraDistortion	����������
*/
void myMarkerDrawer(Mat img, vector<vector<Point2f>> markerCorners, vector<int> markerIds, Mat cameraIntrinsics, Mat cameraDistortion);