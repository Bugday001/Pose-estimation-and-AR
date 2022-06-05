#include "PoseEstimation.h"


/*
 *   得到旋转矩阵和位置矩阵，目前假设corners只有一组4个
 *   @param corners 4个点的x，y
 *   @param cameraMatrix 相机内参
*/
void estimatePose(ArrayofArray corners, float markerLength, cv::Mat cameraMatrix, 
                cv::Mat distCoeffs, Array3d &rvecs, Array3d &tvecs, cv::Mat& R)
{
    //假设一个顶点为(0,0)
    float half_length = markerLength / 2;
    std::vector<cv::Point2f> uv{ cv::Point2f(0,0),cv::Point2f(0,markerLength),
                                    cv::Point2f(markerLength,markerLength),cv::Point2f(markerLength,0) };
    //PNP法使用
    /*std::vector<cv::Point3d> uv3d{ cv::Point3d(-half_length,half_length,0),cv::Point3d(half_length,half_length,0),
                                    cv::Point3d(half_length,-half_length,0),cv::Point3d(-half_length,-half_length,0) };*/
    for (int i = 0; i < corners.size(); i++) {
        /*
        *PNP
        */
        //cv::Mat rvec, tvec;
        //solvePnP(uv3d, corners[i], cameraMatrix, distCoeffs, rvec, tvec);
        //rvecs.push_back(rvec);
        //tvecs.push_back(tvec);
        /*
        * 理论方法
        */
        //计算单应矩阵，Homography
        cv::Mat H = HomographyMat(uv, corners[i]);
        H.convertTo(H, CV_64F);  //类型转换，否则solve报错
        cv::Mat M;
        cv::solve(cameraMatrix, H, M);
        // Normalization to ensure that ||c1|| = 1
        double norm = sqrt(M.at<double>(0, 0) * M.at<double>(0, 0) +
            M.at<double>(1, 0) * M.at<double>(1, 0) +
            M.at<double>(2, 0) * M.at<double>(2, 0));
        M /= norm;
        cv::Mat c1 = M.col(0);
        cv::Mat c2 = M.col(1);
        cv::Mat c3 = c1.cross(c2);
        cv::Mat tvec;
        //归一，使画出来的轴在合适的大小
        normalize(M.col(2), tvec, 1.0, 0.0, cv::NORM_INF);
        tvecs.push_back(tvec);
        //cv::Mat R(3, 3, CV_64F);
        for (int i = 0; i < 3; i++)
        {
            R.at<double>(i, 0) = c1.at<double>(i, 0);
            R.at<double>(i, 1) = c2.at<double>(i, 0);
            R.at<double>(i, 2) = c3.at<double>(i, 0);
        }
        cv::Mat W, U, Vt;
        SVDecomp(R, W, U, Vt);
        R = U * Vt;
        //求lambda
        float lambda = cv::norm(R.col(0));
        R *= lambda;
        cv::Mat rvec;
        Rodrigues(R, rvec);
        rvecs.push_back(rvec);
    }
}


/*
*得到单应矩阵
*/
cv::Mat HomographyMat(ArrayPoint2f src, ArrayPoint2f dst)
{
    const int N = src.size();

    cv::Mat A(2 * N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float u1 = src[i].x;
        const float v1 = src[i].y;
        const float u2 = dst[i].x;
        const float v2 = dst[i].y;

        A.at<float>(2 * i, 0) = 0.0;
        A.at<float>(2 * i, 1) = 0.0;
        A.at<float>(2 * i, 2) = 0.0;
        A.at<float>(2 * i, 3) = -u1;
        A.at<float>(2 * i, 4) = -v1;
        A.at<float>(2 * i, 5) = -1;
        A.at<float>(2 * i, 6) = v2 * u1;
        A.at<float>(2 * i, 7) = v2 * v1;
        A.at<float>(2 * i, 8) = v2;

        A.at<float>(2 * i + 1, 0) = u1;
        A.at<float>(2 * i + 1, 1) = v1;
        A.at<float>(2 * i + 1, 2) = 1;
        A.at<float>(2 * i + 1, 3) = 0.0;
        A.at<float>(2 * i + 1, 4) = 0.0;
        A.at<float>(2 * i + 1, 5) = 0.0;
        A.at<float>(2 * i + 1, 6) = -u2 * u1;
        A.at<float>(2 * i + 1, 7) = -u2 * v1;
        A.at<float>(2 * i + 1, 8) = -u2;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}