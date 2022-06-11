#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "detectMarkers.h"

//#define DEBUG

using namespace std;
using namespace cv;

/** @brief 建立预定义的字典
* @returns vector<vector<int>>         返回字典
*/
vector<vector<int>> myGetPredefinedDictionary(void) {
	vector<vector<int>> dictionary;
	// 占位
	vector<int> dic0 = { 1,1,1,1,1,1,
						 1,1,1,1,1,1,
						 1,1,1,1,1,1,
						 1,1,1,1,1,1,
						 1,1,1,1,1,1,
						 1,1,1,1,1,1 };
	dictionary.push_back(dic0);
	// 录入Marker，0为黑色块，1为白色块，默认为6*6
	/*vector<int> dic1 = { 0,1,1,0,0,1,
						 0,0,0,0,0,0,
						 1,0,0,1,1,1,
						 1,0,1,0,0,0,
						 1,0,1,0,0,0,
						 0,0,1,0,1,1 };
	dictionary.push_back(dic1);*/
	// ↓这个就是doc里的那个
	vector<int> dic2 = { 0,0,0,1,1,1,
						 1,0,0,0,1,1,
						 1,1,0,1,1,1,
						 0,1,1,0,0,0,
						 0,0,1,0,1,0,
						 1,0,0,1,1,0 };
	dictionary.push_back(dic2);
	return dictionary;
}

/** @brief 自适应二值化
* @param input_img      输入图像矩阵
* @param blockSize      取平均值的像素范围，建议值15
* @param delta          阈值 = 均值 - delta
* @returns Mat          返回二值化以后的图像矩阵
*/
Mat myAdaptiveThreshold(Mat input_img, int blockSize, int delta) {
    Mat output_img;
    Size size = input_img.size();

    Mat means; // 均值
    boxFilter(input_img, means, input_img.type(), Size(blockSize, blockSize),
        Point(-1, -1), true, BORDER_REPLICATE); // 取均值
    // 遍历每个像素点
    uchar* p_img, * p_means;
    for (int i = 0; i < input_img.rows; i++)
    {
        // 获取首行地址
        p_img = input_img.ptr<uchar>(i);
        p_means = means.ptr<uchar>(i);
        for (int j = 0; j < input_img.cols; ++j)
        {
            // 二值化
            if (p_img[j] < (p_means[j]-delta))
                p_img[j] = 0;
            else
                p_img[j] = 255;
        }
    }
    output_img = input_img;
    return output_img;
}

/**
* @brife 利用余弦定理取得三角形abc在a处的余弦值
* @param b 点b
* @param c 点c
* @param a 点a
* @returns double 夹角余弦值
*/
double getCOS(Point b, Point c, Point a)
{
	double ab_sq = pow((a.x - b.x), 2) + pow((a.y - b.y), 2);
	double ac_sq = pow((a.x - c.x), 2) + pow((a.y - c.y), 2);
	double bc_sq = pow((c.x - b.x), 2) + pow((c.y - b.y), 2);
	double result = (ac_sq + ab_sq - bc_sq) / (2 * sqrt(ab_sq) * sqrt(ac_sq));
	return result;
}

/**
* @brife 获取点集到图像边缘的最小距离
* @param img			输入图像
* @param approxCurve	多边形点集
* @returns double		到图像边缘的最小距离
*/
double getMinBorderDistance(Mat img, vector<Point> approxCurve)
{
	double minDis = MAX(img.rows, img.cols);
	for (int i = 0; i < approxCurve.size(); i++) {
		if (fabs(approxCurve[i].x) < minDis || fabs(approxCurve[i].y) < minDis) {
			minDis = MIN(fabs(approxCurve[i].x), fabs(approxCurve[i].x));
		}
		if (fabs(img.cols - approxCurve[i].x) < minDis || fabs(img.rows - approxCurve[i].y) < minDis) {
			minDis = MIN(fabs(img.cols - approxCurve[i].x), fabs(img.rows - approxCurve[i].y));
		}
	}
	return minDis;
}

/**
* @brife 在图上找到可能的矩形
* @param img							输入图像矩阵(要求是灰度图)
* @param limitCosine					最小cos值
* @param minCurveLengthRate				最小周长率(与图像尺寸相关)		默认=0.03
* @param maxCurveLengthRate				最大周长率(与图像尺寸相关)		默认=4.0
* @param ApproxAccuracyRate				逼近精度						默认=0.05
* @param minBorderDistanceRate			轮廓到边界的最小距离率		默认=0.05
* @returns vector<vector<Point2f>>		找到的可能矩形 为nx4的向量
*/
vector<vector<Point2f>> findSquares(Mat img, double limitCosine, double minCurveLengthRate,
	double maxCurveLengthRate, double ApproxAccuracyRate, double minBorderDistanceRate)
{
	// 找到的可能矩形
	vector<vector<Point2f>> candidates;

	// 边缘点集，1维索引边缘点集，2维索引边缘上的点
	vector<vector<Point>> contours;

	// 轮廓提取
	findContours(img, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Point> approxCurve;	// 多边形逼近点集

	// 由图像尺寸得出周长阈值范围
	double maxCurveLength = MIN(img.rows, img.cols) * maxCurveLengthRate;
	double minCurveLength = MIN(img.rows, img.cols) * minCurveLengthRate;
	double minDisToBorder = MIN(img.rows, img.cols) * minBorderDistanceRate;

	for (int i = 0; i < contours.size(); i++){	// 每次取出一个边缘
		approxPolyDP(contours[i], approxCurve, arcLength(contours[i], true) * ApproxAccuracyRate, true);	// 多边形逼近
		if (approxCurve.size()==4 &&	// 条件1：四边形
			arcLength(approxCurve,true) < maxCurveLength && arcLength(approxCurve,true) > minCurveLength &&	// 条件2：在周长阈值的范围内
			getMinBorderDistance(img,approxCurve) > minDisToBorder && // 条件3：离边界足够远
			isContourConvex(approxCurve)){	// 条件3：是凸四边形
			
			double maxCOS = 0;

			// 取得4个cos值当中的最大值
			for (int j = 2; j < 5; j++)
			{
				double cosine = fabs(getCOS(approxCurve[j % 4], approxCurve[j - 2], approxCurve[j - 1]));
				maxCOS = MAX(maxCOS, cosine);
			}

			// 如果cos值小于阈值，则将这个四边形储存为候选
			if (maxCOS < limitCosine)
			{
				vector<Point2f> quad;

				for (int j = 0; j < 4; j++)
				{
					// 存一个抽一个
					quad.push_back(approxCurve[3-j]);
				}

				candidates.push_back(quad);
			}
		}
	}
#ifdef DEBUG
	for (int i=0;i<candidates.size();i++)
	cout << candidates[i] << endl;
#endif
	return candidates;
}

/**
* @brife 在图中画出四边形框
* @param img		 在img上画框
* @param quads		 四边形点集
*/
void drawContours(Mat img, vector<vector<Point2f>> quads)
{
	cvtColor(img, img, CV_GRAY2RGB);
	for (int i = 0; i < quads.size(); i++)
	{
		// 每个四边形两点之间划线
		for (int j = 0; j < 4; j++)
		{
			line(img, quads[i][j], quads[i][(j + 1) % 4], Scalar(0, 0, 255), 2); 
		}
	}
	imshow("Candidates", img);
}

/**
* @brife 检查Marker边框
* @param cell			存储Marker二值信息的数组
* @returns bool		    检查结果
*/
bool borderCheck(int* cell)
{
	for (int i = 0; i < 64; i++) {
		if ((i < 8 || i%8==0 || i%8==7 || i>(64-8)) && *(cell+i) != 0) return false;
	}



	return true;
}

/**@brife 判断该Aruco marker是否包含在字典当中
* @param cell_without_border		存储Marker二值信息的二维数组(去除边界,6x6)
* @param dict						字典
* @param errorRate					可接受错误率
* @return int						如果在字典当中，则返回id，否则返回0
*/
int checkInDictionary(int* cell_without_border, vector<vector<int>> dict, float errorRate)
{	
	int id = 0;
	int error_count = 0;	// 允许存在一些错误
	int flag = 1;
	for (int i = 0; i < dict.size(); i++) {
		flag = 1;
		error_count = 0;
		for (int j = 0; j < 6 * 6; j++) {	// 遍历检验是否相同
			if (*(cell_without_border + j) != dict[i][j]) {
				error_count++;
			}
			if (error_count > 36 * errorRate) {
				flag = 0;
				break;
			}
		}
		if (flag) {
			id = i;
			break;
		}
	}

	return id;
}

/**@brife 将矩阵顺时针旋转90度
* @param cell		存储Marker二值信息的二维数组
*/
void rotate_clockwise_90(int* cell)
{
	int temp[6][6];
	int n = 6;
	// 旋转，利用temp暂存
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			temp[i][j] = *(cell + n * (n - j - 1) + i);
		}
	}
	// 将temp搬回cell
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			*(cell + n * i + j) = temp[i][j];
		}
	}
}

/**
* @brife 检测Markers
* @param img						输入图像
* @param dictionary					输入字典
* @param Markers					将找到的Marker角点集存储在这个二维向量中
* @param ids						将找到的Marker的id存储在这个一维向量中
*/
void myMarkerDetector(Mat img, vector<vector<int>> dictionary, vector<vector<Point2f>> &markerCorners, vector<int> &markerIds)
{
	Mat img_copy;
	img.copyTo(img_copy);
	if(img_copy.channels() > 1) cvtColor(img_copy, img_copy, CV_RGB2GRAY);	// 如果不是灰度图，转化成灰度图
	
	img_copy = myAdaptiveThreshold(img_copy, 20, 5);

	vector<vector<Point2f>> candidates;
	candidates = findSquares(img_copy, 0.7, 0.1, 10.0, 0.03, 0.01);

	//drawContours(img_copy, candidates);
	//imshow("2", img_copy);
	Point2i calibrateSize = Point2i(64, 64);

	for (int i = 0; i < candidates.size(); i++) {

		Mat img_calibrated = Mat::zeros(calibrateSize.x, calibrateSize.y, CV_8UC3);

		// 利用4个角点形成校正框架
		vector<Point2f> calibrateFrame;
		calibrateFrame.push_back(Point2f(0, 0));
		calibrateFrame.push_back(Point2f(0, img_calibrated.rows));
		calibrateFrame.push_back(Point2f(img_calibrated.cols, img_calibrated.rows));
		calibrateFrame.push_back(Point2f(img_calibrated.cols, 0));

		// 校正候选图像
		Mat transformation = getPerspectiveTransform(candidates[i], calibrateFrame);
		warpPerspective(img_copy, img_calibrated, transformation, img_calibrated.size(), INTER_LINEAR);

		resize(img_calibrated, img_calibrated, Size(8, 8));

		// OTSU二值化
		threshold(img_calibrated, img_calibrated, 0, 255, CV_THRESH_OTSU);

		// 将Markers读取到二维数组(黑色为0，白色为1)
		int marker_cell[8][8];
		int cell_without_border[6][6];

		for (unsigned int i = 0; i < img_calibrated.cols * img_calibrated.rows; i++)
		{
			marker_cell[i / img_calibrated.cols][i % img_calibrated.cols] = (img_calibrated.data[i] == 255);
		}

		int id = 0;	// Marker id
		int rotate_times = 0;
		// 检查边界
		if (borderCheck(marker_cell[0])) {
			for (int m = 1; m < 7; m++) {
				for (int n = 1; n < 7; n++) {
					cell_without_border[m - 1][n - 1] = marker_cell[m][n];
				}
			}

			// 如果边界有效，旋转4个方向，获取Marker的ID，并检查是否在字典当中
			for (int j = 0; j < 4; j++) {
#ifdef DEBUG
				for (int i = 0; i < 6; i++) {
					for (int j = 0; j < 6; j++) {
						cout << cell_without_border[i][j] << " ";
					}
					cout << endl;
				}
				cout << "rotate_times: " << rotate_times << endl;
#endif // DEBUG
				if (id = checkInDictionary(cell_without_border[0], dictionary, 0.05)) {
					break;
				}
				rotate_clockwise_90(cell_without_border[0]);
				rotate_times++;	// 记录旋转次数，从而确定左上角点的位置
			}
			// 如果是Aruco Marker，存储到向量当中
			if (rotate_times < 4) {
				vector<Point2f> marker_corner;
				// markerCorners[i][0] 为右下角；按顺时针顺序存储，例如markerCorners[i][2]为左上角
				for (int j = 0; j < 4; j++) {
					marker_corner.push_back(candidates[i][(j + rotate_times + 2) % 4]);
				}

				markerCorners.push_back(marker_corner);
				markerIds.push_back(id);
			}
		}
		
#ifdef DEBUG
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 8; j++) {
				cout << marker_cell[i][j] << " ";
			}
			cout << endl;
		}
		cout << id <<" "<< rotate_times << endl;
		cout << endl;
		Mat debug;
		resize(img_calibrated, debug, Size(300, 300));
		imshow("1", debug);
		//waitKey(0);
#endif // DEBUG

	}
}

/**@brife 在图上画出aruco图示
* @param img				需要画图的图像矩阵
* @param markerCorners		marker角点集
* @param markerIds			marker编号		
* @param cameraIntrinsics	相机内参
* @param cameraDistortion	相机畸变矩阵
*/
void myMarkerDrawer(Mat img, vector<vector<Point2f>> markerCorners, vector<int> markerIds, Mat cameraIntrinsics, Mat cameraDistortion)
{
	if(img.channels()<3) cvtColor(img, img, CV_GRAY2RGB);	// 如果是灰度图，则要转为RGB图像
	for (unsigned int i = 0; i < markerCorners.size(); i++)
	{
		// 标定候选框
		for (unsigned int j = 0; j < 4; j++)
		{
			line(img, markerCorners[i][j], markerCorners[i][(j + 1) % 4], Scalar(0, 0, 255), 2);
		}
		string id_string = "id=" + to_string(markerIds[i]);
		// 标定右上角和id markerCorners[i][0] 为右上角
		circle(img, markerCorners[i][0], 5, Scalar(255, 0, 0), 2);
		putText(img, id_string, markerCorners[i][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
	}
#ifdef DEBUG
	imshow("myDetector", img);
#endif
}
