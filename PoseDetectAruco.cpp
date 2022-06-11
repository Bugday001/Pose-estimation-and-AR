#include "PoseDetectAruco.h"
#include "detectMarkers.h"

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
    std::vector<double> dist = { 6.0457800985820168e-02, 2.0220867596709421e-02,
                                -1.1646429144627779e-03, -7.0050063043331011e-04,
                                -2.3355091747708559e-01 };
    distCoeffs = cv::Mat(dist);
    distCoeffs = distCoeffs.reshape(1, 1);

    // step 2: �Ա��ͼ�񶼽���aruco��ǵļ���Լ���̬����
        //1.������ͷ������Ƶ
    VideoCapture capture(1);
    if (!capture.isOpened()) {
        std::cout << "�޷���������ͷ��" << std::endl;
        return -1;
    }
    //��ȡ�滻ͼ��
    Mat new_image = imread("img/Prypiat.jpg");
    //2.ѭ����ʾÿһ֡
    while (1)
    {
        Mat cam, test_image;
        capture >> cam;//��ȡ��ǰ֡ͼ��

        resize(cam, test_image, cv::Size(800, 600));


        //������
        cv::Mat undist_image = cv::Mat::zeros(test_image.size(), test_image.type());
        Undistortion(test_image, undist_image, cameraMatrix, distCoeffs);

        //���Aruco
        vector<vector<Point2f>> corners;
        vector<int> ids;
        auto myDictionary = myGetPredefinedDictionary();
        myMarkerDetector(undist_image, myDictionary, corners, ids);
        myMarkerDrawer(undist_image, corners, ids, cameraMatrix, distCoeffs);

        // ���marker,opencv����
        //auto dictionary = cv::aruco::getPredefinedDictionary(
        //    cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
        //std::vector<std::vector<cv::Point2f>> corners, rejectedImgPoints;
        //std::vector<int> ids;
        //auto parameters = cv::aruco::DetectorParameters::create();
        //cv::aruco::detectMarkers(undist_image, dictionary, corners, ids, parameters,
        //    rejectedImgPoints);
        //cv::aruco::drawDetectedMarkers(undist_image, corners, ids,
        //    cv::Scalar(0, 255, 0));


        // ��̬����,opencv����
        //std::vector<cv::Vec3d> rvecs;
        //std::vector<cv::Vec3d> tvecs;
        //cv::aruco::estimatePoseSingleMarkers(corners, 0.053, cameraMatrix,
        //    distCoeffs, rvecs, tvecs);
        //for (int i = 0; i < rvecs.size(); i++) {
        //    cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs[i],
        //        tvecs[i], 0.02);
        //}
        //

        // ��̬����,�Լ�ʵ�֣�Ŀǰֻ������һ��R�����Ľ�
        std::vector<cv::Vec3d> rvecs2;
        std::vector<cv::Vec3d> tvecs2;
        cv::Mat R(3, 3, CV_64F);
        estimatePose(corners, 0.099, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);

        // step 3: ���������Ტ���п��ӻ���ʾ
        //for (int i = 0; i < rvecs2.size(); i++) {
        //    cv::aruco::drawAxis(test_image, cameraMatrix, distCoeffs, rvecs2[i],
        //        tvecs2[i], 0.2);
        //}

        //���߽���ͼ��/3Dģ����ʾ
        for (int i = 0; i < rvecs2.size(); i++) {
            //Visualize2d(test_image, new_image, corners);
            //Visualize2d_plus(test_image, new_image, cameraMatrix, distCoeffs, rvecs2, tvecs2, R);
            Visualize3d(undist_image, STL.Point_mat, cameraMatrix, distCoeffs, tvecs2, R);
            break;
        }
        
        namedWindow("pose", WINDOW_NORMAL);
        cv::imshow("pose", undist_image);
        waitKey(1);//��ʱ  ms
    }  
}
