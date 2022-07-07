#include "PoseDetectAruco.h"


using namespace cv;
using namespace std;

#define TRACK_NUM 100

int detectPoseShow(ReadModelFile Model)
{
    // step 1: 加载当前搭载相机的内参矩阵和畸变系数
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<double> camera = { 601.0847934393354, 1, 328.0792083850147,
                                    0, 603.8012877668342, 240.9372217623055,
                                    0, 0, 1 };
    cameraMatrix = cv::Mat(camera);
    cameraMatrix = cameraMatrix.reshape(1, 3);
    std::vector<double> dist = { 6.0457800985820168e-02, 2.0220867596709421e-02,
                                -1.1646429144627779e-03, -7.0050063043331011e-04,
                                -2.3355091747708559e-01 };
    distCoeffs = cv::Mat(dist);
    distCoeffs = distCoeffs.reshape(1, 1);

    // step 2: 对标记图像都进行aruco标记的检测以及姿态估计
        //1.从摄像头读入视频
    VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cout << "无法开启摄像头！" << std::endl;
        return -1;
    }
    //读取替换图像
    Mat Prypiat = imread("img/Prypiat.jpg");
    Mat grass = imread("./img/grass.jpg");
    Mat win_xp = imread("./img/win_xp.jpg");
    //2.循环显示每一帧
    vector<vector<Mat>> rotationsVector;
    vector<vector<Vec3d>> translationsVector;
    while (1)
    {
        Mat cam, test_image, undist_image;
        capture >> cam;//获取当前帧图像

        resize(cam, test_image, cv::Size(800, 600));

        //反畸变，默认不使用
        if (false) {
            undist_image = Mat::zeros(test_image.size(), test_image.type());
            Undistortion(test_image, undist_image, cameraMatrix, distCoeffs);
        }
        else {
            undist_image = test_image;
        }


        //检测Aruco
        vector<vector<Point2f>> corners;
        vector<int> ids;
        auto myDictionary = myGetPredefinedDictionary();
        myMarkerDetector(undist_image, myDictionary, corners, ids);
        myMarkerDrawer(undist_image, corners, ids, cameraMatrix, distCoeffs);

        // 检测marker,opencv函数
        //auto dictionary = cv::aruco::getPredefinedDictionary(
        //    cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
        //std::vector<std::vector<cv::Point2f>> corners, rejectedImgPoints;
        //std::vector<int> ids;
        //auto parameters = cv::aruco::DetectorParameters::create();
        //cv::aruco::detectMarkers(undist_image, dictionary, corners, ids, parameters,
        //    rejectedImgPoints);
        //cv::aruco::drawDetectedMarkers(undist_image, corners, ids,
        //    cv::Scalar(0, 255, 0));

        // 姿态检算,目前只能容纳一个R，待改进
        std::vector<cv::Vec3d> rvecs2;
        std::vector<cv::Vec3d> tvecs2;
        cv::Mat R(3, 3, CV_64F);
        estimatePose(corners, 0.099, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);

        // step 3: 绘制坐标轴并进行可视化显示
        //for (int i = 0; i < rvecs2.size(); i++) {
        //    cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs2[i],
        //        tvecs2[i], 0.2);
        //}

        //或者进行图像/3D模型显示
        for (int i = 0; i < rvecs2.size(); i++) {
            Visualize2d(undist_image, grass, corners);
            Visualize2d_plus(undist_image, win_xp, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);
            Visualize3d(undist_image, Model, cameraMatrix, distCoeffs, tvecs2, R);
            break;
        }
        
        namedWindow("pose", WINDOW_NORMAL);
        cv::imshow("pose", undist_image);


        vector<Mat> rotation;
        rotation.push_back(R);

        // 如果检测到marker 将其位姿记录下来
        if (corners.size() > 0) saveTrack(rotationsVector, translationsVector, rotation, tvecs2, "");

        // 收集到100个轨迹以后就退出
        if (translationsVector.size() >= TRACK_NUM) break;

        waitKey(1);//延时  ms
    }  
    // 显示3D轨迹
    drawTracks(rotationsVector, translationsVector);
}
