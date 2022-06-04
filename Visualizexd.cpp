#include "visualizexd.h"
using namespace cv;
using namespace std;

/*
*ƽ����ʾ��Ƭ
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
	Mat H = findHomography(new_image_box, roi_box_pt);
	Mat roi_new_image;
	//����͸�ӱ任
	warpPerspective(new_image, roi_new_image, H, test_image.size(), INTER_CUBIC);
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
	Mat kernel = getStructuringElement(MorphShapes::MORPH_RECT, Size(3, 3));
	dilate(mask, mask, kernel);
	//��͸�ӱ任����滻ͼ���滻ԭͼ���е�ROI����
	Mat result;
	bitwise_and(test_image, mask, test_image);
	addWeighted(test_image, 1, roi_new_image, 0.7, 1.0, test_image);
}


/*
������ʾ��Ƭ
*/
void Visualize2d_plus(Mat& test_image, Mat new_image, Mat cameraMatrix,
				Mat distCoeffs, vector<cv::Vec3d> rvec, vector<cv::Vec3d> tvec, Mat R)
{
	float ratio = 2000;
	float width = (float)new_image.cols/ ratio;
	float height = (float)new_image.rows/ ratio;

	vector<Point3f> objectPoints{ cv::Point3d(0,0,height),cv::Point3d(width,0,height),
									cv::Point3d(width,0,0),cv::Point3d(0,0,0) };
	Mat points = (cv::Mat_<double>(4, 4) << 0, width, width, 0,
		0, 0, 0, 0,
		height, height, 0, 0,
		1,1,1,1);
	//vector<Point2f> roi_box_pt(4);
	//projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, roi_box_pt);
	Mat tmpM = Mat::zeros(3, 4, CV_32F), M, tmpMat, Zero1 = (Mat_<double>(1,4)<<0,0,0,1);
	//�ϲ�R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);

	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
												0,1,0,0,
												0,0,1,0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;
	//cout << roi_box_pt << endl;
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
	Mat H = findHomography(new_image_box, dst);
	Mat roi_new_image;
	//����͸�ӱ任
	warpPerspective(new_image, roi_new_image, H, test_image.size(), INTER_CUBIC);
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
	Mat kernel = getStructuringElement(MorphShapes::MORPH_RECT, Size(3, 3));
	dilate(mask, mask, kernel);
	//��͸�ӱ任����滻ͼ���滻ԭͼ���е�ROI����
	Mat result;
	bitwise_and(test_image, mask, test_image);
	addWeighted(test_image, 1, roi_new_image, 0.7, 1.0, test_image);
}


/*
* ������ʾstlģ��
*/
void Visualize3d(Mat& test_image, Mat model, Mat cameraMatrix,
	Mat distCoeffs, vector<cv::Vec3d> rvec, vector<cv::Vec3d> tvec, Mat R)
{
	float ratio = 300;

	//vector<Point2f> roi_box_pt(4);
	//projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, roi_box_pt);
	Mat tmpM = Mat::zeros(3, 4, CV_32F), M, tmpMat, Zero1 = (Mat_<double>(1, 4) << 0, 0, 0, 1);
	//�ϲ�R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);
	Mat points, all_ones = Mat::ones(1, model.cols, CV_32F);
	vconcat(model/ratio, all_ones, points);
	points.convertTo(points, CV_64F);
	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;
	//cout << roi_box_pt << endl;
	//Matתpoint2f����һʹÿ�������һ��Ԫ��Ϊ1�����˱�����������ֵ
	vector<cv::Point2i> dst;
	vector<vector<cv::Point2i>> dsts;
	cv::Point2i p;
	float norm_ratio;
	int panel_count = 0;
	for (int i = 0; i < roi_box_pt.cols; i++) {
		norm_ratio = 1 / (float)roi_box_pt.at<double>(2, i);
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
	
	dsts.push_back(dst);
	//for (int i = 0; i < dst.size(); i++) {
	//	circle(test_image, dst[i], 5, cv::Scalar(0, 255, 0), 2);
	//}
	
	fillPoly(test_image, dsts, Scalar(255, 255, 255));
	cout << "ok" << endl;
}