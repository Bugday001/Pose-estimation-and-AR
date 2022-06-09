#include "PointsCloud.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <opencv2/opencv.hpp>  
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/traits.hpp>
#include "opencv2/features2d.hpp"
#include <iostream>  
#include <fstream>
#include <string>
#include <opencv2/viz.hpp>
#include <opencv2\features2d\features2d.hpp>
#include<cstdlib>


using namespace cv;
using namespace std;
bool isFileExists_ifstream(string& name) {
	ifstream f(name.c_str());
	return f.good();
}
void ReadK(string filename, Matx33d& K)
{
	string txt = ".txt";
	string txtfile = filename + txt;
	ifstream ifs(txtfile);
	float data[5];
	int vector_length = 0;
	bool ret = isFileExists_ifstream(txtfile);
	if (ret) {
		for (size_t i = 0; i < 5; ++i) {
			/*getline(ifs, str);*/
			ifs >> data[i];
		}
		Matx33d tmpK(data[0], data[2], data[3],
			0, data[1], data[4],
			0, 0, 1);
		K = tmpK;
	}
	else
	{
		cout << "erroe!" << endl;
	}
}


void PointsCloud()

{	//函数声明：
	Mat_<double> LinearLSTriangulation(
		Point3d u,//homogenous image point (u,v,1)  
		Matx34d P,//camera 1 matrix  
		Point3d u1,//homogenous image point in 2nd camera  
		Matx34d P1//camera 2 matrix  
	);

	bool CheckCoherentRotation(cv::Mat_<double> & R);

	//输出文件函数：
	string getfilename(string filename1, int i, string filename2);

	//点云显示框建立
	//pcl::visualization::PCLVisualizer viewer;
	//cout << "正在生成点云，请等待..." << endl;
	viz::Viz3d myWindow("Coordinate Frame");
	myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
	//相机标定部分



	ifstream fin; // 标定所用图像文件的路径 
	ofstream fout;  // 保存标定结果的文件 
	int k = 1;
	int ii = 0;


	//Matx33d K(1262.958489659621, 0, 562.3792964280647,
	//	0, 1266.222824039161, 727.8125879762329,
	//	0, 0, 1);
	Matx33d K(1431.908093836636, 0, 976.3946473077715,
	0, 1441.642851608545, 540.051435742684,
	0, 0, 1);
	cout << "K=" << K << endl;
	//sift匹配部分
	string data_txt = "img/mydata.txt";
	fin.open(data_txt);
	string filename1, filename2;
	string jpg = ".png";
	while (getline(fin, filename1))
	{
		if (filename1 == "")
			break;
		while (getline(fin, filename2))
		{
			if (k <= ii) {
				continue;
			}
			else if (k >= ii + 10) {
				break;
			}
			//读入图片
			if (filename2 == "")
				break;
			Mat img_1 = imread(filename1 + jpg);
			Mat img_2 = imread(filename2 + jpg);
			cout << "输入图片为：" << filename1 << endl;
			cout << "输入图片为：" << filename2 << endl;

			//Create SIFT class pointer
			Ptr<Feature2D> f2d = SIFT::create(); //SIFT::create(); //ORB::create(2000); //SIFT::create();// //xfeatures2d::SURF::create();//SURF::create();// ORB::create(5000);//
			//Detect the keypoints
			vector<KeyPoint> keypoints_1, keypoints_2;
			f2d->detect(img_1, keypoints_1);
			f2d->detect(img_2, keypoints_2);
			//draw
			//Mat keyPointimg1, keyPointimg2;
			//drawKeypoints(img_1, keypoints_1, keyPointimg1);
			//drawKeypoints(img_2, keypoints_2, keyPointimg2);
			//namedWindow("keypoints1", 0);
			//imshow("keypoints1", keyPointimg1);
			//namedWindow("keypoints2", 0);
			//imshow("keypoints2", keyPointimg2);
			//Calculate descriptors (feature vectors)
			Mat descriptors_1, descriptors_2;
			f2d->compute(img_1, keypoints_1, descriptors_1);
			f2d->compute(img_2, keypoints_2, descriptors_2);
			//Matching descriptor vector using BFMatcher
			BFMatcher matcher;
			vector<DMatch> pre_matches;
			matcher.match(descriptors_1, descriptors_2, pre_matches);
			// 匹配对筛选：选择 hamming距离小于最小距离的两倍的特征点
			auto min_max = minmax_element(pre_matches.begin(), pre_matches.end(), [](const DMatch& m1, const DMatch& m2) { return m1.distance < m2.distance; });
			double min_dist = min_max.first->distance;
			double max_dist = min_max.second->distance;

			//printf("-- Max dist : &f \n", max_dist);
			//printf("-- Min dist : &f \n", min_dist);

			//当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
			std::vector<DMatch> matches;
			for (int i = 0; i < descriptors_1.rows; ++i) {
				if (pre_matches[i].distance <= max(2 * min_dist, 30.0)||1) {
					matches.push_back(pre_matches[i]);
				}
			}

			//此处应该有掌声，谢谢大佬
			int ptCount = (int)matches.size();
			cout << "第" << k << "次匹配点云数量为：" << ptCount << endl;
			Mat p1(ptCount, 2, CV_32F);
			Mat p2(ptCount, 2, CV_32F);
			// 把Keypoint转换为Mat
			//cout << "关键点数目：" << ptCount << endl;
			Point2f pt;
			for (int i = 0; i < ptCount; i++)
			{
				pt = keypoints_1[matches[i].queryIdx].pt;
				p1.at<float>(i, 0) = pt.x;
				p1.at<float>(i, 1) = pt.y;

				pt = keypoints_2[matches[i].trainIdx].pt;
				p2.at<float>(i, 0) = pt.x;
				p2.at<float>(i, 1) = pt.y;
			}
			Mat F;
			F = findFundamentalMat(p1, p2, FM_RANSAC);
			if (F.dims != 0) {
				cout << "基础矩阵为：" << F << endl;
				//绘制匹配出的关键点
				Mat img_matches;
				drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
				namedWindow("【match图】",0);
				resizeWindow("【match图】", 1280, 480);
				imshow("【match图】", img_matches);
				waitKey(100);
				
				
				//重建部分
				//ReadK(filename1, K);
				Mat_<double> E = Mat(K.t()) * F * Mat(K);
				SVD svd(E);
				Matx33d W(0, -1, 0,//HZ 9.13  
					1, 0, 0,
					0, 0, 1);
				Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19  
				Mat_<double> t = svd.u.col(2); //u3 
				if (!CheckCoherentRotation(R))
				{
					cout << "resulting rotation is not coherent\n";
				}
				Matx34d P(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0);
				Matx34d P1(R(0, 0), R(0, 1), R(0, 2), t(0),
					R(1, 0), R(1, 1), R(1, 2), t(1),
					R(2, 0), R(2, 1), R(2, 2), t(2));

				Mat p_1(ptCount, 2, CV_32F);
				Mat p_2(ptCount, 2, CV_32F);
				// 把Keypoint转换为Mat
				for (int i = 0; i < ptCount; i++)
				{
					pt = keypoints_1[matches[i].queryIdx].pt;
					p_1.at<float>(i, 0) = pt.x;
					p_1.at<float>(i, 1) = pt.y;

					pt = keypoints_2[matches[i].trainIdx].pt;
					p_2.at<float>(i, 0) = pt.x;
					p_2.at<float>(i, 1) = pt.y;
				}


				Mat Kinv;
				invert(K, Kinv, DECOMP_LU);
				cout << "这个矩阵为：" << Kinv << endl;
				//vector<Point3d> u_1, u_2;
				vector<Point3d> pointcloud;

				Mat cloud(1, ptCount, CV_64FC3);
				Point3d* data = cloud.ptr<cv::Point3d>();

				Mat_<double> X_1, X;
				vector<double> reproj_error;
				for (size_t i = 0; i < ptCount; i++)
				{
					Point2f kp = keypoints_1[i].pt;
					Point2f kp1 = keypoints_1[i].pt;

					Point3d	u_1(kp.x, kp.y, 1);
					Point3d	u_2(kp1.x, kp1.y, 1);
					//cout << "abc"<<u_1 << u_2<<endl;
					Mat_<double> um_1 = Kinv * Mat_<double>(u_1);
					//	cout <<"ijk"<< um_1 << endl;
					u_1 = Point3d(um_1);
					Mat_<double> um_2 = Kinv * Mat_<double>(u_2);
					u_2 = Point3d(um_2);
					//	cout << "xyz" << u_1 << u_2 << endl;
					X_1 = LinearLSTriangulation(u_1, P, u_2, P1);
					X.push_back(X_1);
					pointcloud.push_back(Point3d(X_1(0), X_1(1), X_1(2)));
					data[i].x = X_1(0);
					data[i].y = X_1(1);
					data[i].z = X_1(2);
					//Mat_<double> xPt_img = K * Mat(P1) * X_1;
					//Point2f xPt_img1(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
					//reproj_error.push_back(norm(xPt_img1 - kp1));

				}
				//cloud *= 5.0f;

				//记录点云结果
				//string filename3 = getfilename("pointclouds/pointcloud", k, ".txt");
				//fout.open(filename3);
				////cout <<"点云结果为："<< pointcloud<<endl;
				//fout << "点云结果为：" << endl;
				//fout << pointcloud;
				//fout.close();
				////Scalar me = mean(reproj_error);
				////cout<< me[0];

				//可视化
				Vec3d cam_pos(3.0f, 3.0f, 3.0f), cam_focal_point(3.0f, 3.0f, 2.0f), cam_y_dir(-1.0f, 0.0f, 0.0f);
				Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
				Affine3d transform = viz::makeTransformToGlobal(Vec3f(0.0f, -1.0f, 0.0f), Vec3f(-1.0f, 0.0f, 0.0f), Vec3f(0.0f, 0.0f, -1.0f), cam_pos);
				Mat bunny_cloud = cloud;
				viz::WCloud cloud_widget(bunny_cloud, viz::Color::green());
				Affine3d cloud_pose = Affine3d().translate(Vec3f(0.0f, 0.0f, 3.0f));
				Affine3d cloud_pose_global = transform * cloud_pose;
				string name = to_string(rand());
				myWindow.showWidget(name, cloud_widget, cloud_pose_global);
				myWindow.setViewerPose(cam_pose);
			}
			
			k++;
		}

		ii++;
		fin.close();
		fin.open(data_txt);
		for (int j = 0; j < ii; j++)
		{
			getline(fin, filename1);
		}
	}


	cout << "点云结果如图所示" << endl;
	myWindow.spin();
	system("pause");
}





//函数定义
bool CheckCoherentRotation(cv::Mat_<double>& R) {
	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
		cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
		return false;
	}
	return true;
}

Mat_<double> LinearLSTriangulation(
	Point3d u,//homogenous image point (u,v,1)  
	Matx34d P,//camera 1 matrix  
	Point3d u1,//homogenous image point in 2nd camera  
	Matx34d P1//camera 2 matrix  
)
{
	//build A matrix  
	Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1), u.x * P(2, 2) - P(0, 2),
		u.y * P(2, 0) - P(1, 0), u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
		u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1), u1.x * P1(2, 2) - P1(0, 2),
		u1.y * P1(2, 0) - P1(1, 0), u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2)
	);
	//build B vector  
	Matx41d B(-(u.x * P(2, 3) - P(0, 3)),
		-(u.y * P(2, 3) - P(1, 3)),
		-(u1.x * P1(2, 3) - P1(0, 3)),
		-(u1.y * P1(2, 3) - P1(1, 3)));
	//solve for X  
	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);
	return X;
}

string getfilename(string filename1, int i, string filename2)
{
	//第i个文件名为：

	string filename;
	char s[10];
	_itoa(i, s, 10);
	string str = s;
	filename = filename1 + str + filename2;

	return filename;

}