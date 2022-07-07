#include "PoseEstimation.h"

static cv::Mat BasicHomographyMat(ArrayPoint2f src, ArrayPoint2f dst);
static cv::Mat VectorCreateMat(ArrayPoint2f src);
/*
 * 得到旋转矩阵和位置矩阵，目前假设corners只有一组4个
 * @param corners           4个点的x，y
 * @param markerLength      Aruco实际尺寸，米
 * @param cameraMatrix      相机内参
 * @param distCoeffs        相机畸变参数
 * @param rvecs             旋转向量
 * @param tvecs             平移向量
 * @param R                 旋转矩阵
*/
void estimatePose(ArrayofArray corners, float markerLength, cv::Mat cameraMatrix, 
                cv::Mat distCoeffs, Array3d &rvecs, Array3d &tvecs, cv::Mat& R)
{
    //假设一个顶点为(0,0)
    float half_length = markerLength / 2;
    std::vector<cv::Point2f> uv{ cv::Point2f(0,0),cv::Point2f(0,markerLength),
                                    cv::Point2f(markerLength,markerLength),cv::Point2f(markerLength,0) };
    
    for (int i = 0; i < corners.size(); i++) {
        //计算单应矩阵，Homography
        cv::Mat H = HomographyMat(uv, corners[i], false);
        cv::Mat M;
        //相当于乘逆矩阵
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
        //normalize(M.col(2), tvec, 1.0, 0.0, cv::NORM_INF);
        tvec = M.col(2);
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

/**@brife 得到单应矩阵
* @param src		标记点的实际坐标
* @param dst		相机上的坐标
* @param ransac		是否使用ransac优化
*/
cv::Mat HomographyMat(ArrayPoint2f srcs, ArrayPoint2f dsts, bool ransac=false)
{
    //采样点数
    int randnum = 21;
    //Ransca参数
    int pretotal = 0;
    double sigma = 800;
    int iters = 1000;
    double P = 0.99;
    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    if (srcs.size() > randnum && ransac) {
        for (int iter = 0; iter < iters; iter++) {
            int total_inlier = 0;
            //生成四个随机数，取点
            ArrayPoint2f src, dst;
            std::vector<int> randindex;
            while (src.size() < randnum)
            {
                int tempindex = rand() % (srcs.size() - 0) + 0;
                auto iter = std::find(randindex.begin(), randindex.end(), tempindex);
                if (iter == randindex.end())
                {
                    randindex.push_back(tempindex);
                    src.push_back(srcs[tempindex]);
                    dst.push_back(dsts[tempindex]);
                }
            }
            //计算当前选取randnum个角点得到的H，记作M
            cv::Mat M = BasicHomographyMat(src, dst);
            M /= M.at<double>(2, 2);
            //计算欧拉距离
            cv::Mat dstsMat = VectorCreateMat(dsts);
            cv::Mat srcsMat = VectorCreateMat(srcs);
            for (int i = 0; i < srcs.size(); i++) {
                cv::Mat M_dot_srcMat = M * srcsMat.col(i);
                cv::Mat M_1_dot_dstMat = M.inv() * dstsMat.col(i);
                double distance = norm(M_dot_srcMat, dstsMat.col(i), cv::NORM_L2) + norm(srcsMat.col(i), M_1_dot_dstMat, cv::NORM_L2);
                //判断内点
                if (distance < sigma) {
                    total_inlier++;
                }
            }
            //判断当前H的性能是否更优
            if (total_inlier > pretotal) {
                iters = log(1 - P) / log(1 - pow(total_inlier / srcs.size(), 2));
                pretotal = total_inlier;
                H = M;
            }

            if (total_inlier > srcs.size() / 2)
                break;
        }
        return H;
    }

    H = BasicHomographyMat(srcs, dsts);
    return H / H.at<double>(2, 2);
}

/**@brife 得到3d坐标的Mat，shape = 3 * src.size()
* @param src      point2f的vector
*/
static cv::Mat VectorCreateMat(ArrayPoint2f src)
{
    cv::Mat Points(3, src.size(), CV_64F);
    for (int i = 0; i < src.size(); i++) {
        Points.at<double>(0, i) = src[i].x;
        Points.at<double>(1, i) = src[i].y;
        Points.at<double>(2, i) = 0;
    }
    return Points;
}

/**@brife 得到4点单应矩阵
* @param src      标记点的实际坐标
* @param dst      相机上的坐标
*/
static cv::Mat BasicHomographyMat(ArrayPoint2f src, ArrayPoint2f dst)
{
    const int N = src.size();

    cv::Mat A(2 * N, 9, CV_64F);

    for (int i = 0; i < N; i++)
    {
        const double u1 = (double)src[i].x;
        const double v1 = (double)src[i].y;
        const double u2 = (double)dst[i].x;
        const double v2 = (double)dst[i].y;

        A.at<double>(2 * i, 0) = 0.0;
        A.at<double>(2 * i, 1) = 0.0;
        A.at<double>(2 * i, 2) = 0.0;
        A.at<double>(2 * i, 3) = -u1;
        A.at<double>(2 * i, 4) = -v1;
        A.at<double>(2 * i, 5) = -1;
        A.at<double>(2 * i, 6) = v2 * u1;
        A.at<double>(2 * i, 7) = v2 * v1;
        A.at<double>(2 * i, 8) = v2;

        A.at<double>(2 * i + 1, 0) = u1;
        A.at<double>(2 * i + 1, 1) = v1;
        A.at<double>(2 * i + 1, 2) = 1;
        A.at<double>(2 * i + 1, 3) = 0.0;
        A.at<double>(2 * i + 1, 4) = 0.0;
        A.at<double>(2 * i + 1, 5) = 0.0;
        A.at<double>(2 * i + 1, 6) = -u2 * u1;
        A.at<double>(2 * i + 1, 7) = -u2 * v1;
        A.at<double>(2 * i + 1, 8) = -u2;
    }

    cv::Mat u, w, vt;

    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

