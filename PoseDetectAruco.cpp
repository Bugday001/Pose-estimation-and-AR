#include "PoseDetectAruco.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

int detectPoseShow()
{
    // step 1: 加载当前搭载相机的内参矩阵和畸变系数
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<double> camera = { 601.0847934393354, 0, 328.0792083850147,
                                    0, 603.8012877668342, 240.9372217623055,
                                    0, 0, 1 };
    cameraMatrix = cv::Mat(camera);
    cameraMatrix = cameraMatrix.reshape(1, 3);
    std::vector<double> dist = { -0.1819841154584121,
                                0.5775998031142531,
                                -0.003283415071597713,
                                 0.001489140905045077,
                                -0.2056383075620613 };
    distCoeffs = cv::Mat(dist);
    distCoeffs = distCoeffs.reshape(1, 1);

    // step 2: 对标记图像都进行aruco标记的检测以及姿态估计
        //1.从摄像头读入视频
    VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cout << "无法开启摄像头！" << std::endl;
        return -1;
    }
    //2.循环显示每一帧
    while (1)
    {
        Mat cam;
        capture >> cam;//获取当前帧图像

        cv::Mat test_image;
        cv::resize(cam, test_image, cv::Size(800, 600));
        cv::imshow("test_image", test_image);
        auto dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
        std::vector<std::vector<cv::Point2f>> corners, rejectedImgPoints;
        std::vector<int> ids;
        auto parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(test_image, dictionary, corners, ids, parameters,
            rejectedImgPoints);
        cv::aruco::drawDetectedMarkers(test_image, corners, ids,
            cv::Scalar(0, 255, 0));

        std::vector<cv::Vec3d> rvecs;
        std::vector<cv::Vec3d> tvecs;
        // 姿态检算
        cv::aruco::estimatePoseSingleMarkers(corners, 0.053, cameraMatrix,
            distCoeffs, rvecs, tvecs);

        // step 3: 绘制坐标轴并进行可视化显示
        for (int i = 0; i < rvecs.size(); i++) {
            cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs[i],
                tvecs[i], 0.02);
        }
        namedWindow("pose", WINDOW_AUTOSIZE);
        cv::imshow("pose", test_image);

        
        //imshow("实时相机画面", cam);//显示当前帧图像
                              //imwrite(to_string(i) + ".png", cam);
        waitKey(20);//延时20ms
    }


    
}
