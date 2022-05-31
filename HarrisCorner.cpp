#include "HarrisCorner.h"
#include<vector>
#include "SomeCameraFuction.h"

double myHarrisThrehold_ave = 10000000000;


void sobelGradient(Mat& img, Mat& dst, int para);
Mat computeImage(Mat& ix, Mat& iy, int wsize, int para);
Mat filterR(Mat& img, int wsize);
void mixP(Mat& point, Mat& img, int psize, vector<Point2f>& corners);

//使用全一的窗口函数
void myHarrisCorner_ave(Mat& srcImg) {
    Mat image, src_grayImg, Ix, Iy, I_xx, I_yy, I_xy, R, filter_R, result;
    cvtColor(srcImg, src_grayImg, COLOR_BGR2GRAY);
    image = srcImg.clone();

    int wsize = 3;//窗口大小
	//计算Ix, Iy
    sobelGradient(src_grayImg, Ix, 1);
    sobelGradient(src_grayImg, Iy, 2);
    //I_xx = computeImage(Ix, Iy, wsize, 1);
    //I_yy = computeImage(Ix, Iy, wsize, 2);
    //I_xy = computeImage(Ix, Iy, wsize, 4);
    //计算响应值
	std::vector<Point2f> my_corners;
    R = computeImage(Ix, Iy, wsize, 3);
    //局部非极大值抑制
    filter_R = filterR(R, 10);
    mixP(filter_R, image, 2, my_corners);
	imshow("binary", src_grayImg);
    imshow("Ave", image);
	//opencv检测角点
	//声明每张图片的角点
	const Size boardSize(6, 4);
	std::vector<Point2f> corners;
	bool found = findCorners(srcImg, corners, boardSize);
    imwrite("myHarris_Ave05.jpg", image);
}

void sobelGradient(Mat& img, Mat& dst, int para) {
    dst = Mat::zeros(img.size(), CV_64FC1);

    if (para == 1)
        Sobel(img, dst, CV_64FC1, 0, 1, 3);
    else if (para == 2)
        Sobel(img, dst, CV_64FC1, 1, 0, 3);
}

Mat computeImage(Mat& ix, Mat& iy, int wsize, int para) {

	Mat I_xx, I_yy, I_xy, r;
	I_xx = Mat::zeros(ix.size(), CV_64FC1);
	I_yy = Mat::zeros(ix.size(), CV_64FC1);
	r = Mat::zeros(ix.size(), CV_64FC1);
	I_xy = Mat::zeros(ix.size(), CV_64FC1);


	for (int i = wsize / 2; i < (ix.rows - wsize / 2); i++)
		for (int j = wsize / 2; j < (ix.cols - wsize / 2); j++) {
			//compute A B C, A = Ix * Ix, B = Iy * Iy, C = Ix * Iy
			double A = 0;
			double B = 0;
			double C = 0;
			for (int ii = i - wsize / 2; ii <= (i + wsize / 2); ii++)
				for (int jj = j - wsize / 2; jj <= (j + wsize / 2); jj++) {
					double xx = ix.at<double>(ii, jj);
					double yy = iy.at<double>(ii, jj);
					A += xx * xx;
					B += yy * yy;
					C += xx * yy;
				}
			double p = A + B;
			double q = A * B - C * C;  
			//double delta = p * p - 4 * q;  // A2+B2-AB+4C2

			I_xx.at<double>(i, j) = A;
			I_yy.at<double>(i, j) = B;
			I_xy.at<double>(i, j) = C;
			double rr = q - 0.06 * p * p;  // Harris operator：det(M)-alpha*trace(M)^2

			if (rr > myHarrisThrehold_ave) {
				r.at<double>(i, j) = rr;
			}

		}
	switch (para) {
	case 1: return I_xx; break;
	case 2: return I_yy; break;
	case 3: return r; break;
	case 4:return I_xy; break;
	}
}

Mat filterR(Mat& img, int wsize) {
	Mat result;
	result = Mat::zeros(img.size(), CV_64F);

	//find local maxima of R
	for (int i = wsize / 2; i < (img.rows - wsize / 2); i++)
		for (int j = wsize / 2; j < (img.cols - wsize / 2); j++) {
			double origin = img.at<double>(i, j);
			bool found = false;
			for (int ii = i - wsize / 2; ii <= (i + wsize / 2) && found == false; ii++)
				for (int jj = j - wsize / 2; jj <= (j + wsize / 2); jj++)
					if (origin < img.at<double>(ii, jj)) {
						origin = 0;
						found = true;
						break;
					}
			if (origin == 0)
				result.at<double>(i, j) = 0;
			else
				result.at<double>(i, j) = 255;
		}

	return result;
}

void mixP(Mat& point, Mat& img, int psize, vector<Point2f>& corners) {

	for (int i = psize; i < img.rows - psize; i++)
		for (int j = psize; j < img.cols - psize; j++) {
			if (point.at<double>(i, j) != 0) {
				for (int ii = i - psize; ii <= i + psize; ii++)
					for (int jj = j - psize; jj <= j + psize; jj++) {
						img.at<Vec3b>(ii, jj)[0] = 0;
						img.at<Vec3b>(ii, jj)[1] = 0;
						img.at<Vec3b>(ii, jj)[2] = 255;
					}
				Point2f temp_point(i, j);
				corners.push_back(temp_point);
			}
		}
}