#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "detectMarkers.h"

//#define DEBUG

using namespace std;
using namespace cv;

/** @brief ����Ԥ������ֵ�
* @returns vector<vector<int>>         �����ֵ�
*/
vector<vector<int>> myGetPredefinedDictionary(void) {
	vector<vector<int>> dictionary;
	// ռλ
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
	// ¼��Marker��0Ϊ��ɫ�飬1Ϊ��ɫ�飬6*6
	vector<int> dic2 = { 0,0,0,1,1,1,
						 1,0,0,0,1,1,
						 1,1,0,1,1,1,
						 0,1,1,0,0,0,
						 0,0,1,0,1,0,
						 1,0,0,1,1,0 };
	dictionary.push_back(dic2);
	return dictionary;
}

/** @brief ����Ӧ��ֵ��
* @param input_img      ����ͼ�����
* @param blockSize      ȡƽ��ֵ�����ط�Χ������ֵ15
* @param delta          ��ֵ = ��ֵ - delta
* @returns Mat          ���ض�ֵ���Ժ��ͼ�����
*/
Mat myAdaptiveThreshold(Mat input_img, int blockSize, int delta) {
    Mat output_img;
    Size size = input_img.size();

    Mat means; // ��ֵ
    boxFilter(input_img, means, input_img.type(), Size(blockSize, blockSize),
        Point(-1, -1), true, BORDER_REPLICATE); // ȡ��ֵ
    // ����ÿ�����ص�
    uchar* p_img, * p_means;
    for (int i = 0; i < input_img.rows; i++)
    {
        // ��ȡ���е�ַ
        p_img = input_img.ptr<uchar>(i);
        p_means = means.ptr<uchar>(i);
        for (int j = 0; j < input_img.cols; ++j)
        {
            // ��ֵ��
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

/** @brief otsu(�����䷽��)��ֵ��
* @param input_img      ����ͼ�����
* @returns Mat          ���ض�ֵ���Ժ��ͼ�����
*/
Mat myOtsuThreshold(Mat input_img) {
	Mat output_img;
	input_img.copyTo(output_img);

	//���ͼ���������С����ֵ��ȷ����ֵ����
	double mat_min, mat_max;
	Point min_position_mat, max_position_mat;
	minMaxLoc(output_img, &mat_min, &mat_max, &min_position_mat, &max_position_mat);
	vector <double> var(mat_max - mat_min + 1);    // ��䷽������
	
	double thresh_value;   //��ֵ����ֵ
	int m = 0;    //m���붨���ڵ�һ��for�������棬����ÿ�ζ��ᱻ��ʼ��Ϊ0��
	for (thresh_value = mat_min; thresh_value < mat_max; thresh_value++)
	{
		double sum = output_img.rows * output_img.cols;     //ͼ�����ص�����
		double sum_fg = 0, sum_bg = 0;						//Ŀ��ͱ������ص�����
		double sum_vaule_fg = 0, sum_vaule_bg = 0;			//ǰ���ͱ������ص���ܻҶ�
		for (int i = 0; i < output_img.rows; i++)
			for (int j = 0; j < output_img.cols; j++)
			{
				int vaule = output_img.at<uchar>(i, j);
				if (vaule < thresh_value)     //����
				{
					sum_bg += 1;
					sum_vaule_bg += vaule;
				}
				else       //Ŀ��
				{
					sum_fg += 1;
					sum_vaule_fg += vaule;
				}
			}
		double proportion_fg = sum_fg / sum;													//Ŀ�����ص���ռ����
		double proportion_bg = sum_bg / sum;													//�������ص���ռ����
		double mean_value_fg = sum_vaule_fg / sum_fg;											//Ŀ�����ص��ƽ���Ҷ�
		double mean_vaule_bg = sum_vaule_bg / sum_bg;											//�������ص��ƽ���Ҷ�
		double aver_vaule_mat = proportion_fg * mean_value_fg + proportion_bg * mean_vaule_bg;  //ͼ����ƽ���Ҷ�

		//����ÿ����ֵ�µ���䷽����浽var��
		var[m] = proportion_fg * (mean_value_fg - aver_vaule_mat) * (mean_value_fg - aver_vaule_mat) +
			proportion_bg * (mean_vaule_bg - aver_vaule_mat) * (mean_vaule_bg - aver_vaule_mat);
		m += 1;
	}

	//�ҵ������䷽���Լ����Ӧ����ֵ
	double var_max = 0, var_maxnum = 0;
	for (int k = 0; k < mat_max - mat_min; k++)
		if (var[k] > var_max)
		{
			var_max = var[k];
			var_maxnum = k + mat_min;
		}

	thresh_value = var_maxnum;

	// ����ÿ�����ص㣬����ֵ��
	uchar* p_img;
	for (int i = 0; i < output_img.rows; i++)
	{
		// ��ȡ���е�ַ
		p_img = output_img.ptr<uchar>(i);
		for (int j = 0; j < output_img.cols; ++j)
		{
			// ��ֵ��
			if (p_img[j] < thresh_value)
				p_img[j] = 0;
			else
				p_img[j] = 255;
		}
	}
	return output_img;
}

/**
* @brife �������Ҷ���ȡ��������abc��a��������ֵ
* @param b ��b
* @param c ��c
* @param a ��a
* @returns double �н�����ֵ
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
* @brife ��ȡ�㼯��ͼ���Ե����С����
* @param img			����ͼ��
* @param approxCurve	����ε㼯
* @returns double		��ͼ���Ե����С����
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
* @brife ���������
* @param a								a��
* @param b								b��
* @returns double						����
*/
double getDistance(Point a, Point b) {
	return sqrt(fabs((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
}

/**
* @brife ���c��ֱ��(��a,b��ȷ��)����
* @param a								a��
* @param b								b��
* @param c								c��
* @returns double						����
*/
double getDistanceToLine(Point a, Point b, Point c) {
	double tops = abs(a.x * c.y + b.x * a.y + c.x * b.y
		- a.x * b.y - b.x * c.y - c.x * a.y);
	double bottom = sqrt(pow(b.y - a.y, 2) + pow(b.x - a.x, 2));
	double height = 100 * tops / bottom;
	return height;
}

/**
* @brife ����������4���αƽ�(����Douglas-Peucker�㷨)
* @param contours						��⵽������
* @param ApproxAccuracyRate				�ƽ�����
* @returns vector<Point2f>          	���ؿ��ܵ�4���ζ���(����4�����򷵻�(0,0))
*/
vector<Point> myApproxDP(vector<Point> contour, double ApproxAccuracyRate) {
	vector<Point> approxCurve;	// ����αƽ��㼯

	// �������4���Σ��򷵻����
	Point zero(0, 0);
	vector<Point> no_quadrilateral;
	no_quadrilateral.push_back(zero);

	// ɸѡ��ֵ
	double threshold_dis = arcLength(contour, true) * ApproxAccuracyRate;

	// �ҵ��㼯���о�����Զ��2���㣬���ı��ε����Խ���
	double max_dis = 0;
	int index[4];	// �����ڹ켣���е�����
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

	// �ҵ������ֱ����Զ��1����
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
	
	// �ҵ���3�����ߵľ���֮����Զ��1����
	for (int i = 0; i < contour.size(); i++) {
		double dis = 0;
		for(int j=0;j<3;j++) dis+= getDistanceToLine(contour[index[j]], contour[index[(j + 1) % 3]], contour[i]);
		if (dis > max_dis_2) {
			max_dis_2 = dis;
			index[3] = i;
		}
	}

	// �������ǲ��Ƕ���4��
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

	// �����㰴������->˳ʱ�뷽������approxCurve

	int right_down_index = 0;
	int maxDis = 0;
	for (int i = 0; i < 4; i++) {
		if (pow(contour[index[i]].x, 2) + pow(contour[index[i]].y, 2) > maxDis) {
			maxDis = pow(contour[index[i]].x, 2) + pow(contour[index[i]].y, 2);
			right_down_index = i;
		}
	}
	approxCurve.push_back(contour[index[(right_down_index+2)%4]]);	// �������Ͻǵ�
	int right_up_index;
	if (contour[index[(right_down_index + 1) % 4]].x > contour[index[(right_down_index+3)%4]].x) right_up_index = (right_down_index + 1)%4;
	else right_up_index = (right_down_index + 3) % 4;
	approxCurve.push_back(contour[index[right_up_index]]);	// �������Ͻǵ�
	approxCurve.push_back(contour[index[right_down_index]]);	// �������½ǵ�
	approxCurve.push_back(contour[index[(right_up_index+2)%4]]);	// �������½ǵ�

	return approxCurve;
}

/**
* @brife ��ͼ���ҵ����ܵľ���
* @param img							����ͼ�����(Ҫ���ǻҶ�ͼ)
* @param limitCosine					���cosֵ					����=0.3
* @param minCurveLengthRate				��С�ܳ���(��ͼ��ߴ����)		����=0.03
* @param maxCurveLengthRate				����ܳ���(��ͼ��ߴ����)		����=4.0
* @param ApproxAccuracyRate				�ƽ�����						����=75
* @param minBorderDistanceRate			�������߽����С������		����=0.05
* @returns vector<vector<Point2f>>		�ҵ��Ŀ��ܾ��� Ϊnx4������
*/
vector<vector<Point2f>> myFindSquares(Mat img, double limitCosine, double minCurveLengthRate,
	double maxCurveLengthRate, double ApproxAccuracyRate, double minBorderDistanceRate)
{
	// �ҵ��Ŀ��ܾ���
	vector<vector<Point2f>> candidates;

	// ��Ե�㼯��1ά������Ե�㼯��2ά������Ե�ϵĵ�
	vector<vector<Point>> contours;

	// ������ȡ
	findContours(img, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	vector<Point> approxCurve;	// ����αƽ��㼯

	// ��ͼ��ߴ�ó��ܳ���ֵ��Χ
	double maxCurveLength = MIN(img.rows, img.cols) * maxCurveLengthRate;
	double minCurveLength = MIN(img.rows, img.cols) * minCurveLengthRate;
	double minDisToBorder = MIN(img.rows, img.cols) * minBorderDistanceRate;

	for (int i = 0; i < contours.size(); i++){	// ÿ��ȡ��һ����Ե

		approxCurve = myApproxDP(contours[i], ApproxAccuracyRate);	// ����αƽ�

		if (approxCurve.size()==4 &&	// ����1���ı���
			arcLength(approxCurve,true) < maxCurveLength && arcLength(approxCurve,true) > minCurveLength &&	// ����2�����ܳ���ֵ�ķ�Χ��
			getMinBorderDistance(img,approxCurve) > minDisToBorder && // ����3����߽��㹻Զ
			isContourConvex(approxCurve)){	// ����3����͹�ı���
			
			double maxCOS = 0;

			// ȡ��4��cosֵ���е����ֵ
			for (int j = 2; j < 5; j++)
			{
				double cosine = fabs(getCOS(approxCurve[j % 4], approxCurve[j - 2], approxCurve[j - 1]));
				maxCOS = MAX(maxCOS, cosine);
			}

			// ���cosֵС����ֵ��������ı��δ���Ϊ��ѡ
			if (maxCOS < limitCosine)
			{
				vector<Point2f> quad;

				for (int j = 0; j < 4; j++)
				{
					// ��һ����һ��
					quad.push_back(approxCurve[3-j]);
				}

				candidates.push_back(quad);
			}
		}
	}
	return candidates;
}

/**
* @brife ��ͼ�л����ı��ο�
* @param img		 ��img�ϻ���
* @param quads		 �ı��ε㼯
*/
void myDrawContours(Mat img, vector<vector<Point2f>> quads)
{
	cvtColor(img, img, CV_GRAY2RGB);
	for (int i = 0; i < quads.size(); i++)
	{
		// ÿ���ı�������֮�仮��
		for (int j = 0; j < 4; j++)
		{
			line(img, quads[i][j], quads[i][(j + 1) % 4], Scalar(0, 0, 255), 2); 
		}
	}
	//imshow("Candidates", img);
}

/**
* @brife ���Marker�߿�
* @param cell			�洢Marker��ֵ��Ϣ������
* @returns bool		    �����
*/
bool borderCheck(int* cell)
{
	for (int i = 0; i < 64; i++) {
		if ((i < 8 || i%8==0 || i%8==7 || i>(64-8)) && *(cell+i) != 0) return false;
	}
	return true;
}

/**@brife �жϸ�Aruco marker�Ƿ�������ֵ䵱��
* @param cell_without_border		�洢Marker��ֵ��Ϣ�Ķ�ά����(ȥ���߽�,6x6)
* @param dict						�ֵ�
* @param errorRate					�ɽ��ܴ�����
* @return int						������ֵ䵱�У��򷵻�id�����򷵻�0
*/
int checkInDictionary(int* cell_without_border, vector<vector<int>> dict, float errorRate)
{	
	int id = 0;
	int error_count = 0;	// �������һЩ����
	int flag = 1;
	for (int i = 0; i < dict.size(); i++) {
		flag = 1;
		error_count = 0;
		for (int j = 0; j < 6 * 6; j++) {	// ���������Ƿ���ͬ
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

/**@brife ������˳ʱ����ת90��
* @param cell		�洢Marker��ֵ��Ϣ�Ķ�ά����
*/
void rotate_clockwise_90(int* cell)
{
	int temp[6][6];
	int n = 6;
	// ��ת������temp�ݴ�
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			temp[i][j] = *(cell + n * (n - j - 1) + i);
		}
	}
	// ��temp���cell
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			*(cell + n * i + j) = temp[i][j];
		}
	}
}

/**
* @brife ���Markers
* @param img						����ͼ��
* @param dictionary					�����ֵ�
* @param Markers					���ҵ���Marker�ǵ㼯�洢�������ά������
* @param ids						���ҵ���Marker��id�洢�����һά������
*/
void myMarkerDetector(Mat img, vector<vector<int>> dictionary, vector<vector<Point2f>> &markerCorners, vector<int> &markerIds)
{
	Mat img_copy;	 // ����Ҫת��Ϊ�Ҷ�ͼ�����Դ�������������ı�ԭͼ
	img.copyTo(img_copy);
	if(img_copy.channels() > 1) cvtColor(img_copy, img_copy, CV_RGB2GRAY);	// ������ǻҶ�ͼ��ת���ɻҶ�ͼ
	
	img_copy = myAdaptiveThreshold(img_copy, 30, 5);

	vector<vector<Point2f>> candidates;
	candidates = myFindSquares(img_copy, 0.3, 0.1, 8.0, 75, 0.01);

	myDrawContours(img_copy, candidates);

	Point2i calibrateSize = Point2i(64, 64);

	for (int i = 0; i < candidates.size(); i++) {

		Mat img_calibrated = Mat::zeros(calibrateSize.x, calibrateSize.y, CV_8UC3);

		// ����4���ǵ��γ�У�����
		vector<Point2f> calibrateFrame;
		calibrateFrame.push_back(Point2f(0, 0));
		calibrateFrame.push_back(Point2f(0, img_calibrated.rows));
		calibrateFrame.push_back(Point2f(img_calibrated.cols, img_calibrated.rows));
		calibrateFrame.push_back(Point2f(img_calibrated.cols, 0));

		// У����ѡͼ��
		Mat transformation = HomographyMat(candidates[i], calibrateFrame, false);
		mywarpPerspective(img_copy, img_calibrated, transformation, img_calibrated.size());
		resize(img_calibrated, img_calibrated, Size(8, 8));

		// OTSU��ֵ��
		//threshold(img_calibrated, img_calibrated, 0, 255, CV_THRESH_OTSU);
		img_calibrated = myOtsuThreshold(img_calibrated);

		// ��Markers��ȡ����ά����(��ɫΪ0����ɫΪ1)
		int marker_cell[8][8];
		int cell_without_border[6][6];

		for (unsigned int i = 0; i < img_calibrated.cols * img_calibrated.rows; i++)
		{
			marker_cell[i / img_calibrated.cols][i % img_calibrated.cols] = (img_calibrated.data[i] == 255);
		}

		int id = 0;	// Marker id
		int rotate_times = 0;
		// ���߽�
		if (borderCheck(marker_cell[0])) {
			for (int m = 1; m < 7; m++) {
				for (int n = 1; n < 7; n++) {
					cell_without_border[m - 1][n - 1] = marker_cell[m][n];
				}
			}

			// ����߽���Ч����ת4�����򣬻�ȡMarker��ID��������Ƿ����ֵ䵱��
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
				rotate_times++;	// ��¼��ת�������Ӷ�ȷ�����Ͻǵ��λ��
			}
			// �����Aruco Marker���洢����������
			if (rotate_times < 4) {
				vector<Point2f> marker_corner;
				// markerCorners[i][0] Ϊ���½ǣ���˳ʱ��˳��洢������markerCorners[i][2]Ϊ���Ͻ�
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

/**@brife ��ͼ�ϻ���arucoͼʾ
* @param img				��Ҫ��ͼ��ͼ�����
* @param markerCorners		marker�ǵ㼯
* @param markerIds			marker���		
* @param cameraIntrinsics	����ڲ�
* @param cameraDistortion	����������
*/
void myMarkerDrawer(Mat img, vector<vector<Point2f>> markerCorners, vector<int> markerIds, Mat cameraIntrinsics, Mat cameraDistortion)
{
	if(img.channels()<3) cvtColor(img, img, CV_GRAY2RGB);	// ����ǻҶ�ͼ����ҪתΪRGBͼ��
	for (unsigned int i = 0; i < markerCorners.size(); i++)
	{
		Point2f center;	// ���ĵ�
		// �궨��ѡ��
		for (unsigned int j = 0; j < 4; j++)
		{
			line(img, markerCorners[i][j], markerCorners[i][(j + 1) % 4], Scalar(0, 0, 255), 2);
			center.x += markerCorners[i][j].x;
			center.y += markerCorners[i][j].y;
		}

		center.x /= 4;
		center.y /= 4;
		
		// �궨���½Ǻ�id
		string id_string = "id=" + to_string(markerIds[i]);
		circle(img, markerCorners[i][0], 5, Scalar(255, 0, 0), 2);
		putText(img, id_string, markerCorners[i][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
	}
#ifdef DEBUG
	imshow("myDetector", img);
#endif
}
