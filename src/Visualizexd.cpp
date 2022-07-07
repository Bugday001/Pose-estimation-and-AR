#include "visualizexd.h"

using namespace cv;
using namespace std;

/**@brife 使用图片替换Aruco码
* @param test_image     用于显示的图像Mat
* @param new_image		替换的图像Mat
* @param cameraMatrix   Aruco角点
*/
void Visualize2d(Mat& test_image, Mat new_image, ArrayofArray corners)
{
	int width = new_image.cols;
	int height = new_image.rows;
	//提取四个标记形成的ROI区域
	vector<Point2f> roi_box_pt(4);
	//寻找方框的四个顶点，从检测到标记的顺序决定
	roi_box_pt[0] = corners[0][0];
	roi_box_pt[1] = corners[0][1];
	roi_box_pt[2] = corners[0][2];
	roi_box_pt[3] = corners[0][3];
	//提取替换图像的四个顶点
	vector<Point2f> new_image_box(4);
	new_image_box[0] = Point(0, 0);
	new_image_box[1] = Point(width, 0);
	new_image_box[2] = Point(width, height);
	new_image_box[3] = Point(0, height);
	//计算从替换图像到目标ROI图像的3x3单应性矩阵
	Mat H = HomographyMat(new_image_box, roi_box_pt, false);
	Mat roi_new_image;
	//进行透视变换
	mywarpPerspective(new_image, roi_new_image, H, test_image.size());
	//制作掩膜
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
	//用透视变换后的替换图像，替换原图像中的ROI区域
	Mat result;
	bitwise_and(test_image, mask, test_image);
	myaddweight(test_image, 1, roi_new_image, 0.7, test_image);
}


/**@brife 在与Aruco垂直的平面显示图像
* @param test_image     用于显示的图像Mat
* @param model			替换的图像Mat
* @param cameraMatrix   相机内参
* @param distCoeffs     相机畸变参数
* @param tvec			旋转向量
* @param tvec			平移向量
* @param R				旋转矩阵
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
	//合并R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);

	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
									0, 1, 0, 0,
									0, 0, 1, 0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;

	//Mat转point2f，归一使每组坐标后一个元素为1，按此比例调整坐标值
	vector<cv::Point2f> dst;
	cv::Point2f p;
	float norm_ratio;
	for (int i = 0; i < roi_box_pt.cols; i++) {
		norm_ratio = 1 / (float)roi_box_pt.at<double>(2, i);
		p.x = (float)roi_box_pt.at<double>(0, i) * norm_ratio;
		p.y = (float)roi_box_pt.at<double>(1, i) * norm_ratio;
		dst.push_back(p);
	}
	//提取替换图像的四个顶点
	vector<Point2f> new_image_box(4);
	new_image_box[0] = Point(0, 0);
	new_image_box[1] = Point(width* ratio, 0);
	new_image_box[2] = Point(width* ratio, height* ratio);
	new_image_box[3] = Point(0, height* ratio);
	//计算从替换图像到目标ROI图像的3x3单应性矩阵

	Mat H = HomographyMat(new_image_box, dst, false);
	Mat roi_new_image;
	//进行透视变换
	mywarpPerspective(new_image, roi_new_image, H, test_image.size());
	//制作掩膜
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
	//用透视变换后的替换图像，替换原图像中的ROI区域
	Mat result;
	bitwise_and(test_image, mask, test_image);
	myaddweight(test_image, 1, roi_new_image, 0.7, test_image);
}


/**@brife 显示读取的3D模型
* @param test_image     用于显示的图像Mat
* @param model			STL模型读取的三角面角点
* @param cameraMatrix   相机内参
* @param distCoeffs     相机畸变参数
* @param tvec			旋转向量
* @param R				旋转矩阵
*/
void Visualize3d(Mat& test_image, ReadModelFile model, Mat cameraMatrix,
	Mat distCoeffs, vector<cv::Vec3d> tvec, Mat R)
{
	//缩放倍数
	double ratio = 5;

	//变换3f=>2f
	Mat tmpM = Mat::zeros(3, 4, CV_64F), M, tmpMat, Zero1 = (Mat_<double>(1, 4) << 0, 0, 0, 1);
	//合并R,t
	hconcat(R, Mat(tvec[0]), tmpM);
	vconcat(tmpM, Zero1, M);
	Mat points, all_ones = Mat::ones(1, model.Point_mat.cols, CV_64F);
	vconcat(model.Point_mat, all_ones, points);
	Mat K1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat roi_box_pt = cameraMatrix * K1 * M * points;
	//Mat转point2i，归一使每组坐标后一个元素为1，按此比例调整坐标值
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
	//绘制上去
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


/**@brife 透视变换
* @param src			原图
* @param dst			输出图
* @param H				单应矩阵
* @param size			输出图大小
*/
void mywarpPerspective(Mat src, Mat& dst, Mat H, Size size)
{
	//创建原图的四个顶点的3*4矩阵（此处我的顺序为左上，右上，左下，右下）
	Mat tmp = (Mat_<double>(3, 4) << 0, src.cols, 0, src.cols,
		0, 0, src.rows, src.rows,
		1, 1, 1, 1);

	//获得原图四个顶点变换后的坐标，计算变换后的图像尺寸
	Mat corner = H * tmp;
	//移动图到左上角
	double minw = 0, minh = 0;
	//创建向前映射矩阵 map_x, map_y
	dst.create(size, src.type());
	Mat map_x(dst.size(), CV_32FC1);
	Mat map_y(dst.size(), CV_32FC1);
	Mat point_src(3, 1, CV_32FC1, 1);
	Mat point_dst(3, 1, CV_32FC1, 1);
	//H的类型为CV_64FC1，point_dst的类型CV_32FC1
	//本句是为了令H与point_dst同类型（同类型才可以相乘，否则报错）
	H.convertTo(H, point_dst.type());
	Mat H_inv = H.inv();

	//根据输出点及单应性逆矩阵来寻找输出点在输入图像中对应的坐标，以确保每个输出点都有值
	//输出图像的行数与列数由dst.jpg决定
	//此处输出图像的第一个点一定是(0, 0)，即输出像素点中的左上点  
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			point_dst.at<float>(0) = j + minw;
			point_dst.at<float>(1) = i + minh;
			point_src = H_inv * point_dst;
			//齐次坐标转换为像素坐标
			map_x.at<float>(i, j) = point_src.at<float>(0) / point_src.at<float>(2);
			map_y.at<float>(i, j) = point_src.at<float>(1) / point_src.at<float>(2);
		}
	}
	//线性插值，自己实现的偶尔出现问题，暂时使用Opencv库函数
	remap(src, dst, map_x, map_y, CV_INTER_LINEAR);
}

/**@brife 线性混合
* @param src1			原图
* @param alpha			混合权重1
* @param src2			原图2
* @param beta			混合权重2
* @param src3			输出图像
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

