#include "visualizexd.h"

using namespace cv;
using namespace std;

/**@brife ʹ��ͼƬ�滻Aruco��
* @param test_image     ������ʾ��ͼ��Mat
* @param new_image		�滻��ͼ��Mat
* @param cameraMatrix   Aruco�ǵ�
*/
void Visualize2d(Mat& test_image, Mat new_image, ArrayofArray corners)
{
	int width = new_image.cols;
	int height = new_image.rows;
	//��ȡ�ĸ�����γɵ�ROI����
	vector<Point2f> roi_box_pt(4);
	//Ѱ�ҷ�����ĸ����㣬�Ӽ�⵽��ǵ�˳�����
	roi_box_pt[0] = corners[0][0];
	roi_box_pt[1] = corners[0][1];
	roi_box_pt[2] = corners[0][2];
	roi_box_pt[3] = corners[0][3];
	//��ȡ�滻ͼ����ĸ�����
	vector<Point2f> new_image_box(4);
	new_image_box[0] = Point(0, 0);
	new_image_box[1] = Point(width, 0);
	new_image_box[2] = Point(width, height);
	new_image_box[3] = Point(0, height);
	//������滻ͼ��Ŀ��ROIͼ���3x3��Ӧ�Ծ���
	Mat H = HomographyMat(new_image_box, roi_box_pt, false);
	Mat roi_new_image;
	//����͸�ӱ任
	mywarpPerspective(new_image, roi_new_image, H, test_image.size());
	//������Ĥ
	Mat mask = Mat(test_image.size(), CV_8UC3, Scalar(255, 255, 255));
	for (int i = 0; i < roi_new_image.rows; i++)
	{
		for (int j = 0; j < roi_new_image.cols; j++)
		{
			if (roi_new_image.at<Vec3b>(i, j) != Vec3b(0, 0, 0))
			{
				mask.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
	//��͸�ӱ任����滻ͼ���滻ԭͼ���е�ROI����
	Mat result;
	bitwise_and(test_image, mask, test_image);
	myaddweight(test_image, 1, roi_new_image, 0.7, test_image);
}


/**@brife ����Aruco��ֱ��ƽ����ʾͼ��
* @param test_image     ������ʾ��ͼ��Mat
* @param model			�滻��ͼ��Mat
* @param cameraMatrix   ����ڲ�
* @param distCoeffs     ����������
* @param tvec			��ת����
* @param tvec			ƽ������
* @param R				��ת����
*/
void Visualize2d_plus(Mat& test_image, Mat new_image, Mat cameraMatrix,
				Mat distCoeffs, vector<cv::Vec3d> rvec, vector<cv::Vec3d> tvec, Mat R)
{
	float ratio = 5000;
	float width = (float)new_image.cols/ ratio;
	float height = (float)new_image.rows/ ratio;

	vector<Point3f> objectPoints{ cv::Point3d(0,0,height),cv::Point3d(width,0,height),
									cv::Point3d(width,0,0),cv::Point3d(0,0,0) };
	Mat points = (cv::Mat_<double>(4, 4) << 0, width, width, 0,
											0, 0, 0, 0,
											height, height, 0, 0,
											1,1,1,1);

	Mat tmpM = Mat::zeros(3, 4, CV_32F), M, tmpMat, Zero1 = (Mat_<double>(1,4)<<0,0,0,1);
	//�ϲ�R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);

	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
									0, 1, 0, 0,
									0, 0, 1, 0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;

	//Matתpoint2f����һʹÿ�������һ��Ԫ��Ϊ1�����˱�����������ֵ
	vector<cv::Point2f> dst;
	cv::Point2f p;
	float norm_ratio;
	for (int i = 0; i < roi_box_pt.cols; i++) {
		norm_ratio = 1 / (float)roi_box_pt.at<double>(2, i);
		p.x = (float)roi_box_pt.at<double>(0, i) * norm_ratio;
		p.y = (float)roi_box_pt.at<double>(1, i) * norm_ratio;
		dst.push_back(p);
	}
	//��ȡ�滻ͼ����ĸ�����
	vector<Point2f> new_image_box(4);
	new_image_box[0] = Point(0, 0);
	new_image_box[1] = Point(width* ratio, 0);
	new_image_box[2] = Point(width* ratio, height* ratio);
	new_image_box[3] = Point(0, height* ratio);
	//������滻ͼ��Ŀ��ROIͼ���3x3��Ӧ�Ծ���

	Mat H = HomographyMat(new_image_box, dst, false);
	Mat roi_new_image;
	//����͸�ӱ任
	mywarpPerspective(new_image, roi_new_image, H, test_image.size());
	//������Ĥ
	Mat mask = Mat(test_image.size(), CV_8UC3, Scalar(255, 255, 255));
	for (int i = 0; i < roi_new_image.rows; i++)
	{
		for (int j = 0; j < roi_new_image.cols; j++)
		{
			if (roi_new_image.at<Vec3b>(i, j) != Vec3b(0, 0, 0))
			{
				mask.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
	//��͸�ӱ任����滻ͼ���滻ԭͼ���е�ROI����
	Mat result;
	bitwise_and(test_image, mask, test_image);
	myaddweight(test_image, 1, roi_new_image, 0.7, test_image);
}


/**@brife ��ʾ��ȡ��3Dģ��
* @param test_image     ������ʾ��ͼ��Mat
* @param model			STLģ�Ͷ�ȡ��������ǵ�
* @param cameraMatrix   ����ڲ�
* @param distCoeffs     ����������
* @param tvec			��ת����
* @param R				��ת����
*/
void Visualize3d(Mat& test_image, ReadModelFile model, Mat cameraMatrix,
	Mat distCoeffs, vector<cv::Vec3d> tvec, Mat R)
{
	//���ű���
	double ratio = 5;

	//�任3f=>2f
	Mat tmpM = Mat::zeros(3, 4, CV_64F), M, tmpMat, Zero1 = (Mat_<double>(1, 4) << 0, 0, 0, 1);
	//�ϲ�R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);
	Mat points, all_ones = Mat::ones(1, model.Point_mat.cols, CV_64F);
	vconcat(model.Point_mat, all_ones, points);
	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;
	//Matתpoint2i����һʹÿ�������һ��Ԫ��Ϊ1�����˱�����������ֵ
	vector<cv::Point2i> dst;
	vector<vector<cv::Point2i>> dsts;
	cv::Point2i p;
	double norm_ratio;
	int panel_count = 0;
	for (int i = 0; i < roi_box_pt.cols; i++) {
		norm_ratio = 1 / (double)roi_box_pt.at<double>(2, i);
		p.x = (int)(roi_box_pt.at<double>(0, i) * norm_ratio);
		p.y = (int)(roi_box_pt.at<double>(1, i) * norm_ratio);
		dst.push_back(p);
		panel_count++;
		if (panel_count % 3 == 0) {
			dsts.push_back(dst);
			dst.clear();
		}
	}
	//������ȥ
	if (model.isstl) {
		polylines(test_image, dsts, 0, Scalar(255, 255, 255));
	}
	else {
		for (int i = 0; i < dsts.size(); i++) {
			for (int j = 0; j < 3; j++) {
				circle(test_image, dsts[i][j], 1, Scalar(255, 255, 255));
			}
		}
	}

}


/**@brife ͸�ӱ任
* @param src			ԭͼ
* @param dst			���ͼ
* @param H				��Ӧ����
* @param size			���ͼ��С
*/
void mywarpPerspective(Mat src, Mat& dst, Mat H, Size size)
{
	//����ԭͼ���ĸ������3*4���󣨴˴��ҵ�˳��Ϊ���ϣ����ϣ����£����£�
	Mat tmp = (Mat_<double>(3, 4) << 0, src.cols, 0, src.cols,
		0, 0, src.rows, src.rows,
		1, 1, 1, 1);

	//���ԭͼ�ĸ�����任������꣬����任���ͼ��ߴ�
	Mat corner = H * tmp;
	//�ƶ�ͼ�����Ͻ�
	double minw = 0, minh = 0;
	//������ǰӳ����� map_x, map_y
	dst.create(size, src.type());
	Mat map_x(dst.size(), CV_32FC1);
	Mat map_y(dst.size(), CV_32FC1);
	Mat point_src(3, 1, CV_32FC1, 1);
	Mat point_dst(3, 1, CV_32FC1, 1);
	//H������ΪCV_64FC1��point_dst������CV_32FC1
	//������Ϊ����H��point_dstͬ���ͣ�ͬ���Ͳſ�����ˣ����򱨴�
	H.convertTo(H, point_dst.type());
	Mat H_inv = H.inv();

	//��������㼰��Ӧ���������Ѱ�������������ͼ���ж�Ӧ�����꣬��ȷ��ÿ������㶼��ֵ
	//���ͼ���������������dst.jpg����
	//�˴����ͼ��ĵ�һ����һ����(0, 0)����������ص��е����ϵ�  
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			point_dst.at<float>(0) = j + minw;
			point_dst.at<float>(1) = i + minh;
			point_src = H_inv * point_dst;
			//�������ת��Ϊ��������
			map_x.at<float>(i, j) = point_src.at<float>(0) / point_src.at<float>(2);
			map_y.at<float>(i, j) = point_src.at<float>(1) / point_src.at<float>(2);
		}
	}
	//���Բ�ֵ���Լ�ʵ�ֵ�ż���������⣬��ʱʹ��Opencv�⺯��
	remap(src, dst, map_x, map_y, CV_INTER_LINEAR);
}

/**@brife ���Ի��
* @param src1			ԭͼ
* @param alpha			���Ȩ��1
* @param src2			ԭͼ2
* @param beta			���Ȩ��2
* @param src3			���ͼ��
*/
void myaddweight(Mat& src1, double alpha, Mat& src2, double beta, Mat& src3)
{
	int row = src1.rows;
	int col = src1.cols;

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			src3.at<Vec3b>(i, j) = alpha * src1.at<Vec3b>(i, j) + beta * src2.at<Vec3b>(i, j);
		}
	}
}

