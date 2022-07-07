#include "CalibrationChessboard.h"


static void Caculate_distortion(Mat M, Mat cameraMatrix, ArrayofArray ArrayofCorners, vector<Point2f> hat_uv, Mat& k);
static void Caculate_intrinsics_param(Mat H, Mat& cameraMatrix);
static void Caculate_extrinsics_param(Mat H, Mat cameraMatrix, Mat& R);
static Mat create_v(Mat H, int i, int j);


/**
* ��ָ��ͼƬ�в��ҽǵ㣬������������corners��
* @param img     �����ͼƬ
* @param corners ��⵽�Ľ����б�
* @return �Ƿ��⵽�ǵ㣨�����ڷ���Ľ��㣩
*/
bool findCorners(Mat& img, vector<Point2f>& corners, cv::Size boardSize) {
	Mat gray;
	// ��ͼƬת�ɻҶ�ͼ
	cvtColor(img, gray, COLOR_RGB2GRAY);
	// ���ҵ�ǰͼƬ���еĽǵ�
	//�ڲα궨(���̸�)
	bool patternWasFound = findChessboardCorners(gray, boardSize, corners);
	if (patternWasFound) {
		drawChessboardCorners(img, boardSize, corners, patternWasFound);
		// ������ǵ�֮����ʾԭͼ
		namedWindow("src", WINDOW_NORMAL);
		imshow("src", img);
		waitKey(1);
	}
	else {
		namedWindow("no Corners", WINDOW_NORMAL);
		imshow("no Corners", gray);
		waitKey(500);
		cout << "�ǵ���ʧ�ܣ�" << endl;
		return patternWasFound;
	}
	return patternWasFound;
}
/**@brife ���̸�궨
* @param img			ԭͼ
* @param ArrayofCorners �ǵ�s
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
* @param cameraMatrix	����ڲ�
* @return �Ƿ���ɱ궨
*/
bool myCalibrationOnline(Mat& img, ArrayofArray& ArrayofCorners, Size boardSize, const float squareSize, Mat& cameraMatrix)
{
	Mat gray;

	int img_num = 10;//��¼�궨������

	/*
	��¼�ǵ�
	*/
	if (ArrayofCorners.size() < img_num) {
		vector<Point2f> corners;
		//cvtColor(img, gray, COLOR_RGB2GRAY);
		bool patternWasFound = findChessboardCorners(img, boardSize, corners);
		int action = waitKey(30) & 255;
		if (patternWasFound) {// �ҵ��ǵ�
			drawChessboardCorners(img, boardSize, corners, patternWasFound);
			if (action == ACTION_SPACE) { // �û������˿ո�
				ArrayofCorners.push_back(corners);
				cout << "��¼�ǵ�:" << ArrayofCorners.size() << endl;
			}
		}
	}
	/*
	�궨
	*/
	else {
		myCalibration(ArrayofCorners, boardSize, squareSize, cameraMatrix);
		return true;
	}
	return false;
}

/**@brife �����ڲ�
* @param imagePath		ͼ·��
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
*/
int mycalibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize)
{
	// �������ͼƬ������б�
	vector<vector<Point3f>> objectPoints;
	// �������ͼƬ�Ľǵ��б�
	vector<vector<Point2f>> cornerPoints;
	// ͼƬ���سߴ�
	Size imgSize;

	std::vector<String> filenames;
	cv::glob(imagePath, filenames);//��ȡ·���������ļ���
	cout << "filenames.size:" << filenames.size() << endl;
	for (auto& imgName : filenames) {
		// ��ȡͼƬ
		Mat img = imread(imgName, IMREAD_COLOR);
		if (img.empty())
		{
			cout << "load file error" << endl;
			continue;
		}
		// ��ȡͼƬ���سߴ�
		imgSize = img.size();
		std::cout << "name: " << imgName << " imgSize: " << imgSize << std::endl;
		//����ÿ��ͼƬ�Ľǵ�
		vector<Point2f> corners;
		bool found = findCorners(img, corners, boardSize);
		if (found) {
			vector<Point3f> objPoints;
			// �ҵ��ǵ㣬֤������ͼ����Ч��
			objectPoints.push_back(objPoints);
			cornerPoints.push_back(corners);
		}
	}
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	myCalibration(cornerPoints, boardSize, squareSize, cameraMatrix);
	cv::destroyAllWindows();
	return 0;
}

/**@brife �궨
* @param ArrayofCorners	��Ӧ����
* @param boardSize		���̸�ߴ�
* @param squareSize		�����С
* @param cameraMatrix	����ڲ�
*/
void myCalibration(ArrayofArray ArrayofCorners, Size boardSize, const float squareSize, Mat& cameraMatrix)
{
	cout << "��ʼ�궨��" << endl;
	//�����궨��ʵ������
	vector<Point2f> uv;
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			uv.emplace_back(j * squareSize, i * squareSize);//��widthΪx,��heightΪy
		}
	}
	//�õ�ÿ��ͼ��H
	Mat H(3, 3 * ArrayofCorners.size(), CV_64F);
	for (int i = 0; i < ArrayofCorners.size(); i++) {
		 Mat W = HomographyMat(uv, ArrayofCorners[i], false);
		 W.copyTo(H.colRange(3 * i, 3 * i + 3));
	}
	//�����ڲ�
	Caculate_intrinsics_param(H, cameraMatrix);
	cout << "�ڲ�:" << endl << cameraMatrix << endl;
	//�������
	Mat M(3, 3 * ArrayofCorners.size(), CV_64F);
	Caculate_extrinsics_param(H, cameraMatrix, M);
	//�������
	Mat k(2, 1, CV_64F);
	Caculate_distortion(M, cameraMatrix, ArrayofCorners, uv, k);
	cout << "����ϵ��" << k << endl;
}

/**@brife �����ڲ�
* @param V				��Ӧ����
* @param cameraMatrix	����ڲ�
*/
static void Caculate_intrinsics_param(Mat H, Mat& cameraMatrix)
{
	Mat V(2 * H.cols / 3, 6, CV_64F);
	for (int i = 0; i < H.cols / 3; i++) {
		V.row(i * 2) = create_v(H.colRange(3 * i, 3 * i + 3), 0, 1).t();
		V.row(2 * i + 1) = create_v(H.colRange(3 * i, 3 * i + 3), 0, 0).t() - create_v(H.colRange(3 * i, 3 * i + 3), 1, 1).t();
	}
	//SVD�ֽ⣬��B
	Mat u, W, vt, B;
	SVDecomp(V, W, u, vt, SVD::MODIFY_A | SVD::FULL_UV);
	//B.shape = 2 * 3
	B = vt.row(5).reshape(0, 2);

	double B11 = B.at<double>(0, 0), B12 = B.at<double>(0, 1), B22 = B.at<double>(0, 2),
		B13 = B.at<double>(1, 0), B23 = B.at<double>(1, 1), B33 = B.at<double>(1, 2);

	double	w = B11 * B22 * B33 - B12 * B12 * B33 - B11 * B23 * B23
		+ 2 * B12 * B13 * B23 - B22 * B13 * B13;
	double	d = B11 * B22 - B12 * B12;
	//alpha
	cameraMatrix.at<double>(0, 0) = sqrt(w / (d * B11));
	//beta
	cameraMatrix.at<double>(1, 1) = sqrt(w / (d * d) * B11);
	//gamma
	cameraMatrix.at<double>(0, 1) = sqrt(w / (d * d * B11)) * B12;
	//uc
	cameraMatrix.at<double>(0, 2) = (B12 * B23 - B22 * B13) / d;
	//vc
	cameraMatrix.at<double>(1, 2) = (B12 * B13 - B11 * B23) / d;

}

/**@brife �������
* @param V				��Ӧ����
* @param cameraMatrix	����ڲ�
* @param M				r1, r2, t
*/
static void Caculate_extrinsics_param(Mat H, Mat cameraMatrix, Mat& M)
{
	for (int i = 0; i < H.cols/3; i++) {
		cv::Mat tempM;
		//�൱�ڳ������
		cv::solve(cameraMatrix, H.colRange(i * 3, i * 3 + 3), tempM);
		// Normalization to ensure that ||c1|| = 1
		//��һ
		//double norm = sqrt(tempM.at<double>(0, 0) * tempM.at<double>(0, 0) +
		//	tempM.at<double>(1, 0) * tempM.at<double>(1, 0) +
		//	tempM.at<double>(2, 0) * tempM.at<double>(2, 0));
		//tempM /= norm;
		//����
		Mat s = cameraMatrix.inv() * H.col(i * 3);
		double scalar_factor = 1 / norm(s);
		tempM *= scalar_factor;

		cv::Mat c1 = tempM.col(0);
		cv::Mat c2 = tempM.col(1);
		cv::Mat c3 = c1.cross(c2);
		cv::Mat tvec = tempM.col(2);
		Mat tempR(3, 3, CV_64F);
		tempM.colRange(0, 2).copyTo(tempR.colRange(0, 2));
		c3.copyTo(tempR.col(2));
		cv::Mat W, U, Vt;
		SVDecomp(tempR, W, U, Vt);
		tempR = U * Vt;
		//��lambda
		float lambda = norm(tempR.col(0));
		tempR *= lambda;
		tempR.colRange(0, 2).copyTo(M.colRange(i * 3, i * 3 + 2));
		tvec.copyTo(M.col(i * 3 + 2));
	}
}

/**@brife ����������
* @param M				
* @param cameraMatrix	����ڲ�
* @param ArrayofCorners	�ǵ�����
* @param hat_uv			�������������
* @param K				����ϵ��
*/
static void Caculate_distortion(Mat M, Mat cameraMatrix, ArrayofArray ArrayofCorners, vector<Point2f> hat_uv, Mat& k)
{
	Mat H = cameraMatrix * M;
	//����Mat UV1
	Mat UV1(3, hat_uv.size(), CV_64F);
	for (int i = 0; i < hat_uv.size(); i++) {
		UV1.at<double>(0, i) = hat_uv[i].x;
		UV1.at<double>(1, i) = hat_uv[i].y;
		UV1.at<double>(2, i) = 1;
	}
	//�޻�������
	Mat uv_undist(3, UV1.cols * H.cols / 3, CV_64F);
	for (int i = 0; i < H.cols / 3; i++) {
		Mat h = H.colRange(3 * i, 3 * i + 3);
		Mat uv = h * UV1;
		uv.copyTo(uv_undist.colRange(i * UV1.cols, (i + 1) * UV1.cols));
	}
	//����D������������d
	Mat D(2 * uv_undist.cols, 2, CV_64F);
	Mat d(2 * uv_undist.cols, 1, CV_64F);
	int RowofArray = ArrayofCorners.size(), ColofArray = ArrayofCorners[0].size();
	//�������ĵ�
	double u0 = cameraMatrix.at<double>(0, 2), v0 = cameraMatrix.at<double>(1, 2);
	double u = 0, v = 0;
	for (int i = 0; i < uv_undist.cols; i++) {
		u = uv_undist.at<double>(0, i) / uv_undist.at<double>(2, i);
		v = uv_undist.at<double>(1, i) / uv_undist.at<double>(2, i);
		double x1 = (u - u0) / cameraMatrix.at<double>(0, 0),
			y1 = (v - v0) / cameraMatrix.at<double>(1, 1);
		double r2 = x1 * x1 + y1 * y1;
		D.at<double>(2 * i, 0) = (u - u0) * r2; 
		D.at<double>(2 * i, 1) = (u - u0) * r2 * r2;
		D.at<double>(2 * i + 1, 0) = (v - v0) * r2;
		D.at<double>(2 * i + 1, 1) = (v - v0) * r2 * r2;
		d.at<double>(2 * i, 0) = ArrayofCorners[i / ColofArray][i % ColofArray].x - u;
		d.at<double>(2 * i + 1, 0) = ArrayofCorners[i / ColofArray][i % ColofArray].y - v;
	}
	//���K����С����
	k = (D.t() * D).inv() * D.t() * d;
}

/**@brife ����v���B����
* @param H		��Ӧ����
* @param i		ѡ��H�ĵ�i��
* @param j		ѡ��H�ĵ�h��
*/
static Mat create_v(Mat H, int i, int j)
{
	Mat V(6, 1, CV_64F);

	V.at<double>(0, 0) = H.at<double>(0, i) * H.at<double>(0, j);
	V.at<double>(1, 0) = H.at<double>(0, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(0, j);
	V.at<double>(2, 0) = H.at<double>(1, i) * H.at<double>(1, j);
	V.at<double>(3, 0) = H.at<double>(0, i) * H.at<double>(2, j) + H.at<double>(2, i) * H.at<double>(0, j);
	V.at<double>(4, 0) = H.at<double>(1, i) * H.at<double>(2, j) + H.at<double>(2, i) * H.at<double>(1, j);
	V.at<double>(5, 0) = H.at<double>(2, i) * H.at<double>(2, j);
	return  V;
}


/**@brife ȥ����
* @param src            ԭʼͼ��
* @param dst            ȥ�����ͼ��
* @param cameraMatrix   �ڲξ���
* @param distCoeffs     ����ϵ��
*/
void Undistortion(cv::Mat src, cv::Mat& dst, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
	// �������
	double k1 = distCoeffs.at<double>(0, 0), k2 = distCoeffs.at<double>(0, 1),
		p1 = distCoeffs.at<double>(0, 2), p2 = distCoeffs.at<double>(0, 3);
	// �ڲ�
	double fx = cameraMatrix.at<double>(0, 0), fy = cameraMatrix.at<double>(1, 1),
		cx = cameraMatrix.at<double>(0, 2), cy = cameraMatrix.at<double>(1, 2);

	int rows = src.rows, cols = src.cols;

	// ����ȥ�����ͼ�������
	for (int v = 0; v < rows; v++)
		for (int u = 0; u < cols; u++) {

			double u_distorted = 0, v_distorted = 0;

			double x1, y1, x2, y2;
			x1 = (u - cx) / fx;
			y1 = (v - cy) / fy;
			double r2;
			r2 = x1 * x1 + y1 * y1;
			//����ʽ����
			x2 = x1 * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * x1 * y1 + p2 * (r2 + 2 * x1 * x1);
			y2 = y1 * (1 + k1 * r2 + k2 * r2 * r2) + p1 * (r2 + 2 * y1 * y1) + 2 * p2 * x1 * y1;

			u_distorted = fx * x2 + cx;
			v_distorted = fy * y2 + cy;

			// ��ֵ (����ڲ�ֵ)
			if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
				dst.at<cv::Vec3b>(v, u) = src.at<cv::Vec3b>((int)v_distorted, (int)u_distorted);
			}
		}

	// ��ͼȥ�����ͼ��
	//cv::imshow("image undistorted", dst);
}