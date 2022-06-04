#include "PoseDetectAruco.h"


using namespace cv;
using namespace std;

int detectPoseShow(ReadSTLFile STL)
{
    // step 1: ���ص�ǰ����������ڲξ���ͻ���ϵ��
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

    // step 2: �Ա��ͼ�񶼽���aruco��ǵļ���Լ���̬����
        //1.������ͷ������Ƶ
    VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cout << "�޷���������ͷ��" << std::endl;
        return -1;
    }
    //��ȡ�滻ͼ��
    Mat new_image = imread("img/Prypiat.jpg");
    //2.ѭ����ʾÿһ֡
    while (1)
    {
        Mat cam;
        capture >> cam;//��ȡ��ǰ֡ͼ��

        cv::Mat test_image;
        cv::resize(cam, test_image, cv::Size(800, 600));
        //cv::imshow("test_image", test_image);
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
        // ��̬����,opencv����
        //cv::aruco::estimatePoseSingleMarkers(corners, 0.053, cameraMatrix,
        //    distCoeffs, rvecs, tvecs);
        //for (int i = 0; i < rvecs.size(); i++) {
        //    cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs[i],
        //        tvecs[i], 0.02);
        //}
        //

        // ��̬����,�Լ�ʵ��
        std::vector<cv::Vec3d> rvecs2;
        std::vector<cv::Vec3d> tvecs2;
        cv::Mat R(3, 3, CV_64F);
        estimatePose(corners, 0.0053, cameraMatrix,
            distCoeffs, rvecs2, tvecs2, R);
        // step 3: ���������Ტ���п��ӻ���ʾ
        //for (int i = 0; i < rvecs2.size(); i++) {
        //    cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs2[i],
        //        tvecs2[i], 0.2);
        //}
        //���߽���ͼ���滻
        for (int i = 0; i < rvecs2.size(); i++) {
            //Visualize2d(test_image, new_image, corners);
            //Visualize2d_plus(test_image, new_image, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);
            Visualize3d(test_image, STL.Point_mat, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);
            break;
        }
        
        namedWindow("pose", WINDOW_AUTOSIZE);
        cv::imshow("pose", test_image);
        waitKey(20);//��ʱ20ms
    }


    
}
