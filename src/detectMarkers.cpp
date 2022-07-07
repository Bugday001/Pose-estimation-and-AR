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
		/*vector<int> dic1 = { 0,1,1,0,0,1,
						 0,0,0,0,0,0,
						 1,0,0,1,1,1,
						 1,0,1,0,0,0,
						 1,0,1,0,0,0,
						 0,0,1,0,1,1 };
	dictionary.push_back(dic1);*/
	// 录入Marker，0为黑色块，1为白色块，6*6
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
#ifdef DEBUG
	imshow("adaptive threshold", output_img);
#endif
    return output_img;
}

/** @brief otsu(最大类间方差)二值化
* @param input_img      输入图像矩阵
* @returns Mat          返回二值化以后的图像矩阵
*/
Mat myOtsuThreshold(Mat input_img) {
	Mat output_img;
	input_img.copyTo(output_img);

	//求出图像的最大和最小像素值，确定阈值区间
	double mat_min, mat_max;
	Point min_position_mat, max_position_mat;
	minMaxLoc(output_img, &mat_min, &mat_max, &min_position_mat, &max_position_mat);
	vector <double> var(mat_max - mat_min + 1);    // 类间方差容器
	
	double thresh_value;   //二值化阈值
	int m = 0;    //m必须定义在第一层for函数外面，否则每次都会被初始化为0。
	for (thresh_value = mat_min; thresh_value < mat_max; thresh_value++)
	{
		double sum = output_img.rows * output_img.cols;     //图像像素点总数
		double sum_fg = 0, sum_bg = 0;						//目标和背景像素点总数
		double sum_vaule_fg = 0, sum_vaule_bg = 0;			//前景和背景像素点的总灰度
		for (int i = 0; i < output_img.rows; i++)
			for (int j = 0; j < output_img.cols; j++)
			{
				int vaule = output_img.at<uchar>(i, j);
				if (vaule < thresh_value)     //背景
				{
					sum_bg += 1;
					sum_vaule_bg += vaule;
				}
				else       //目标
				{
					sum_fg += 1;
					sum_vaule_fg += vaule;
				}
			}
		double proportion_fg = sum_fg / sum;													//目标像素点所占比例
		double proportion_bg = sum_bg / sum;													//背景像素点所占比例
		double mean_value_fg = sum_vaule_fg / sum_fg;											//目标像素点的平均灰度
		double mean_vaule_bg = sum_vaule_bg / sum_bg;											//背景像素点的平均灰度
		double aver_vaule_mat = proportion_fg * mean_value_fg + proportion_bg * mean_vaule_bg;  //图像总平均灰度

		//计算每个阈值下的类间方差并保存到var中
		var[m] = proportion_fg * (mean_value_fg - aver_vaule_mat) * (mean_value_fg - aver_vaule_mat) +
			proportion_bg * (mean_vaule_bg - aver_vaule_mat) * (mean_vaule_bg - aver_vaule_mat);
		m += 1;
	}

	//找到最大类间方差以及其对应的阈值
	double var_max = 0, var_maxnum = 0;
	for (int k = 0; k < mat_max - mat_min; k++)
		if (var[k] > var_max)
		{
			var_max = var[k];
			var_maxnum = k + mat_min;
		}

	thresh_value = var_maxnum;

	// 遍历每个像素点，做二值化
	uchar* p_img;
	for (int i = 0; i < output_img.rows; i++)
	{
		// 获取首行地址
		p_img = output_img.ptr<uchar>(i);
		for (int j = 0; j < output_img.cols; ++j)
		{
			// 二值化
			if (p_img[j] < thresh_value)
				p_img[j] = 0;
			else
				p_img[j] = 255;
		}
	}
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
* @brife 求两点距离
* @param a								a点
* @param b								b点
* @returns double						距离
*/
double getDistance(Point a, Point b) {
	return sqrt(fabs((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
}

/**
* @brife 求点c到直线(由a,b点确定)距离
* @param a								a点
* @param b								b点
* @param c								c点
* @returns double						距离
*/
double getDistanceToLine(Point a, Point b, Point c) {
	double tops = abs(a.x * c.y + b.x * a.y + c.x * b.y
		- a.x * b.y - b.x * c.y - c.x * a.y);
	double bottom = sqrt(pow(b.y - a.y, 2) + pow(b.x - a.x, 2));
	double height = 100 * tops / bottom;
	return height;
}

/**
* @brife 对轮廓进行4边形逼近(基于Douglas-Peucker算法)
* @param contours						检测到的轮廓
* @param ApproxAccuracyRate				逼近精度
* @returns vector<Point2f>          	返回可能的4边形顶点(不是4边型则返回(0,0))
*/
vector<Point> myApproxDP(vector<Point> contour, double ApproxAccuracyRate) {
	vector<Point> approxCurve;	// 多边形逼近点集

	// 如果不是4边形，则返回这个
	Point zero(0, 0);
	vector<Point> no_quadrilateral;
	no_quadrilateral.push_back(zero);

	// 筛选阈值
	double threshold_dis = arcLength(contour, true) * ApproxAccuracyRate;

	// 找到点集当中距离最远的2个点，是四边形的主对角线
	double max_dis = 0;
	int index[4];	// 顶点在轨迹当中的索引
	index[0] = 0, index[2] = 0;
	for (int i = 0; i < contour.size() - 1; i++) {
		for (int j = i + 1; j < contour.size(); j++) {
			double dis = getDistance(contour[i], contour[j]);
			if (dis > max_dis) {
				index[0] = i;
				index[2] = j;
				max_dis = dis;
			}
		}
	}

	// 找到距离该直线最远的1个点
	int max_dis_1 = 0, max_dis_2 = 0;
	index[1] = 0, index[3] = 0;

	for (int i = 0; i < contour.size(); i++) {
		double dis = 0;
		dis = getDistanceToLine(contour[index[0]], contour[index[2]], contour[i]);
		if (dis > max_dis_1) {
			max_dis_1 = dis;
			index[1] = i;
		}
	}
	
	// 找到与3条连线的距离之和最远的1个点
	for (int i = 0; i < contour.size(); i++) {
		double dis = 0;
		for(int j=0;j<3;j++) dis+= getDistanceToLine(contour[index[j]], contour[index[(j + 1) % 3]], contour[i]);
		if (dis > max_dis_2) {
			max_dis_2 = dis;
			index[3] = i;
		}
	}

	// 检查边数是不是多于4边
	for (int i = 0; i < contour.size(); i++) {
		double dis = 0;
		for (int j = 0; j < 4; j++) dis += getDistanceToLine(contour[index[j]], contour[index[(j + 1) % 4]], contour[i]);

		if (dis > threshold_dis) {
			return no_quadrilateral;
		}
	}


	//for (int i = index[0]; i <= index[2]; i++) {
	//	double dis = getDistanceToLine(contour[index[0]], contour[index[2]], contour[i]);
	//	if (dis > max_dis_1) {
	//		max_dis_1 = dis;
	//		index[1] = i;
	//	}
	//}
	//for (int i = index[2]; i < contour.size(); i++) {
	//	double dis = getDistanceToLine(contour[index[0]], contour[index[2]], contour[i]);
	//	if (dis > max_dis_1) {
	//		max_dis_2 = dis;
	//		index[3] = i;
	//	}
	//}
	//for (int i = 0; i <= index[0]; i++) {
	//	double dis = getDistanceToLine(contour[index[0]], contour[index[2]], contour[i]);
	//	if (dis > max_dis_1) {
	//		max_dis_2 = dis;
	//		index[3] = i;
	//	}
	//}
	//	
	// if (max_dis_1 <= threshold_dis || max_dis_2 < threshold_dis) return no_quadrilateral;

	// 将顶点按照左上->顺时针方向塞入approxCurve

	int right_down_index = 0;
	int maxDis = 0;
	for (int i = 0; i < 4; i++) {
		if (pow(contour[index[i]].x, 2) + pow(contour[index[i]].y, 2) > maxDis) {
			maxDis = pow(contour[index[i]].x, 2) + pow(contour[index[i]].y, 2);
			right_down_index = i;
		}
	}
	approxCurve.push_back(contour[index[(right_down_index+2)%4]]);	// 塞入左上角点
	int right_up_index;
	if (contour[index[(right_down_index + 1) % 4]].x > contour[index[(right_down_index+3)%4]].x) right_up_index = (right_down_index + 1)%4;
	else right_up_index = (right_down_index + 3) % 4;
	approxCurve.push_back(contour[index[right_up_index]]);	// 塞入右上角点
	approxCurve.push_back(contour[index[right_down_index]]);	// 塞入右下角点
	approxCurve.push_back(contour[index[(right_up_index+2)%4]]);	// 塞入左下角点

	return approxCurve;
}

/**
* @brife 在图上找到可能的矩形
* @param img							输入图像矩阵(要求是灰度图)
* @param limitCosine					最大cos值					建议=0.3
* @param minCurveLengthRate				最小周长率(与图像尺寸相关)		建议=0.03
* @param maxCurveLengthRate				最大周长率(与图像尺寸相关)		建议=4.0
* @param ApproxAccuracyRate				逼近精度						建议=75
* @param minBorderDistanceRate			轮廓到边界的最小距离率		建议=0.05
* @returns vector<vector<Point2f>>		找到的可能矩形 为nx4的向量
*/
vector<vector<Point2f>> myFindSquares(Mat img, double limitCosine, double minCurveLengthRate,
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

		approxCurve = myApproxDP(contours[i], ApproxAccuracyRate);	// 多边形逼近

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
	return candidates;
}

/**
* @brife 在图中画出四边形框
* @param img		 在img上画框
* @param quads		 四边形点集
*/
void myDrawContours(Mat img, vector<vector<Point2f>> quads)
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
	//imshow("Candidates", img);
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
	Mat img_copy;	 // 由于要转化为灰度图，所以创建副本，以免改变原图
	img.copyTo(img_copy);
	if(img_copy.channels() > 1) cvtColor(img_copy, img_copy, CV_RGB2GRAY);	// 如果不是灰度图，转化成灰度图
	
	img_copy = myAdaptiveThreshold(img_copy, 30, 5);

	vector<vector<Point2f>> candidates;
	candidates = myFindSquares(img_copy, 0.3, 0.1, 8.0, 75, 0.01);

	myDrawContours(img_copy, candidates);

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
		Mat transformation = HomographyMat(candidates[i], calibrateFrame, false);
		mywarpPerspective(img_copy, img_calibrated, transformation, img_calibrated.size());
		resize(img_calibrated, img_calibrated, Size(8, 8));

		// OTSU二值化
		//threshold(img_calibrated, img_calibrated, 0, 255, CV_THRESH_OTSU);
		img_calibrated = myOtsuThreshold(img_calibrated);

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
#endif
				if (id = checkInDictionary(cell_without_border[0], dictionary, 0.2)) {
					break;
				}
				rotate_clockwise_90(cell_without_border[0]);
				rotate_times++;	// 记录旋转次数，从而确定左上角点的位置
			}
			// 如果是Aruco Marker，存储到向量当中
			if (rotate_times < 4) {
				vector<Point2f> marker_corner;
				// markerCorners[i][0] 为右下角；按顺时针顺序存储，例如markerCorners[i][2]为左上角
				for (int j = 3; j >= 0; j--) {
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
		imshow("otsu", debug);
		waitKey(0);
#endif

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
		Point2f center;	// 中心点
		// 标定候选框
		for (unsigned int j = 0; j < 4; j++)
		{
			line(img, markerCorners[i][j], markerCorners[i][(j + 1) % 4], Scalar(0, 0, 255), 2);
			center.x += markerCorners[i][j].x;
			center.y += markerCorners[i][j].y;
		}

		center.x /= 4;
		center.y /= 4;
		
		// 标定右下角和id
		string id_string = "id=" + to_string(markerIds[i]);
		circle(img, markerCorners[i][0], 5, Scalar(255, 0, 0), 2);
		putText(img, id_string, markerCorners[i][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
	}
#ifdef DEBUG
	imshow("myDetector", img);
#endif
}
