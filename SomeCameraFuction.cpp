#include "SomeCameraFuction.h"

using namespace cv;
using namespace std;
const int ACTION_ESC = 27;
const int ACTION_SPACE = 32;

int myMkdir(string &path)
{
	size_t len = path.size();
	if (path.back() != '\\' && path.back() != '/')
	{
		path += "/";
	}
	string pathDir = "";
	for (size_t i = 0; i < path.size(); i++)
	{
		pathDir += path[i];
		if (path[i] == '\\' || path[i] == '/')
		{
			pathDir[i] = '/';
			path[i] = '/';
			if (0 != ACCESS(pathDir.c_str(), 0))
			{
				cout << pathDir << " bu cun zai " << endl;
				int ret = MKDIR(pathDir.c_str());
				if (ret != 0)
				{
					cout << "mkdir failed" << endl;
				}
			}
		}
	}
	return 1;
}

string getCurrentTimeDate()
{
	// ��ȡ��ǰʱ��
	time_t tm;
	time(&tm);
	//cout << "tm" << tm << endl;
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", t2);
	return buf;
}

string getCurrentTime()
{
	// ��ȡ��ǰʱ��
	time_t tm;
	time(&tm);
	return to_string(tm);
}
int generateCalibrationPicture()
{
	//Mat frame = imread("3A4.bmp"); // cols*rows = 630*891  
	Mat frame(1600, 2580, CV_8UC3, Scalar(0, 0, 0));

	int nc = frame.channels();

	int nWidthOfROI = 320;

	for (int j = 10; j<frame.rows - 10; j++)
	{
		uchar* data = frame.ptr<uchar>(j);
		for (int i = 10; i<(frame.cols - 10)*nc; i += nc)
		{
			if ((i / nc / nWidthOfROI + j / nWidthOfROI) % 2)
			{
				// bgr  
				data[i / nc*nc + 0] = 255;
				data[i / nc*nc + 1] = 255;
				data[i / nc*nc + 2] = 255;
			}
		}
	}
	imshow("test", frame);
	//imwrite("3.bmp", frame);
	waitKey(0);
	return 0;
}
int displayCameraRealTime()
{
	//1.������ͷ������Ƶ
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cout << "�޷���������ͷ��" << std::endl;
		return -1;
	}
	//2.ѭ����ʾÿһ֡
	while (1)
	{
		Mat cam;
		capture >> cam;//��ȡ��ǰ֡ͼ��
		namedWindow("ʵʱ�������", WINDOW_AUTOSIZE);
		imshow("ʵʱ�������", cam);//��ʾ��ǰ֡ͼ��
							  //imwrite(to_string(i) + ".png", cam);
		waitKey(20);//��ʱ20ms
	}
}

/**
* ʵʱ��ʾ������棬��������ͼƬ
* ���ո�����棬��Esc�˳�
**/
int saveCameraImages(string savePath)
{
	//1.������ͷ������Ƶ
	VideoCapture capture(1);
	if (!capture.isOpened()) {
		std::cout << "�޷���������ͷ��" << std::endl;
		return -1;
	}
	if (savePath != "./")
	{
		myMkdir(savePath);
	}
	//2.ѭ����ʾÿһ֡ 
	while (1) {
		Mat image0, image;
		capture >> image0;
		// ��ͼ���Ƶ�image
		image0.copyTo(image);
		int action = waitKey(30) & 255;
		// �ж϶���
		if (action == ACTION_SPACE) { // �û������˿ո�
									  // ����ͼƬ
			string imgFileName = savePath + getCurrentTime() + ".png";
			imwrite(imgFileName, image);
			cout << imgFileName << " saved" << endl;
		}
		else if (action == ACTION_ESC) { // �û�������ESC
			break;
		}
		cv::imshow("ʵʱ�������", image);
	}
	cv::destroyAllWindows();
	return 1;
}

/**
* ʵʱ��ʾ������棬���������ܼ�⵽�ǵ�� ���̸�ͼƬ
* ���ո�����棬��Esc�˳�
* @param boardSize      ���ӳߴ�Size 7*4
**/
int saveChessboardImages(cv::Size boardSize, string savePath)
{
	//1.������ͷ������Ƶ
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cout << "�޷���������ͷ��" << std::endl;
		return -1;
	}
	if (savePath != "./")
	{
		myMkdir(savePath);
	}
	//2.ѭ����ʾÿһ֡
	while (1) {
		Mat image0, image;
		capture >> image0;
		// ��ͼ���Ƶ�image
		image0.copyTo(image);
		// ���ұ궨��(���Գ�Բ�����)
		vector<Point2f> corners;
		//bool found = findCirclesGrid(image, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID);
		bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_FAST_CHECK);

		// ����ȥ
		drawChessboardCorners(image, boardSize, corners, found);

		int action = waitKey(30) & 255;

		// �ж϶���
		if (action == ACTION_SPACE) { // �û������˿ո�
			if (found) {
				// ����ͼƬ
				string imgFileName = savePath + getCurrentTime() + ".png";
				imwrite(imgFileName, image0);
				cout << imgFileName << " saved" << endl;
			}
			else {
				printf("%s\n", "δ��⵽�ǵ�");
			}
		}
		else if (action == ACTION_ESC) { // �û�������ESC
			break;
		}
		cv::imshow("Calibration", image);
	}
	cv::destroyAllWindows();
	return 1;
}

static void print_help() {
	printf("ͨ�����̸�궨���\n");
	printf("������CalibrationChessboard <boardWidth> n<boardHeight> <squareSize> [number_of_boards] "
		"[--delay=<delay>] [-s=<scale_factor>]\n");
	printf("���ӣ�CalibrationACircle2 6 9 30 500 1.0\n");
	//opencv�Ĳ��������� �Ժ��о�
	//bool flipHorizontal = false;
	//// ��������
	//cv::CommandLineParser parser(argc, argv,
	//	"{@arg1||}{@arg2||}{@arg3|20|}{@arg4|15|}"
	//	"{help h||}{delay d|500|}{scale s|1.0|}{f||}");
	//if (parser.has("help")) {
	//	print_help();
	//	return 0;
	//}
	//int boardWidth = parser.get<int>(7);
	//int boardHeight = parser.get<int>(4);
	//float squareSize = parser.get<float>(10);
	//int numBoards = parser.get<int>(3);
	//int delay = parser.get<int>("delay");

	//if (boardWidth < 1 || boardHeight < 1) {
	//	printf("Command-line parameter error: both image of width and height must be specified\n");
	//	print_help();
	//	return -1;
	//}
	//if (parser.has("f")) {
	//	flipHorizontal = true;
	//}
}

/**
* ִ�б궨������궨���
* @param squareSize   ���ӳߴ� mm
* @param boardSize      ���ӳߴ�Size 7*4
* @param image_size    ͼƬ�ߴ�Size 1920*1080
* @param image_points  ͼƬ�ǵ㼯��
* @param cameraMatrix  �������
* @param distCoeffs    �������
*/
void
runCalibrationSave(float squareSize, const Size boardSize, const Size image_size,
	const vector<vector<Point2f>> &image_points, Mat &cameraMatrix, Mat &distCoeffs)
{
	vector<vector<Point3f>> object_points;
	vector<Point3f> objPoints;
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			// ע��ǶԳ�Բ����Ķ���������㷽ʽ
			/*objPoints.push_back(Point3f(float((2 * j + i % 2) * squareSize),
			float(i * squareSize), 0));*/
			//���̸� ����uv�ռ��нǵ��Ӧ���������ϵ����ֵ����ZΪ0
			objPoints.emplace_back(j * squareSize, i * squareSize, 0);//��widthΪx,��heightΪy
		}
	}
	object_points.resize(image_points.size(), objPoints);
	vector<Mat> rvecs, tvecs;

	// ִ�б궨
	double rms = calibrateCamera(object_points, image_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs);

	// ������ֵ(RMS)
	printf("RMS error reported by calibrateCamera: %g\n", rms);
	// ���궨������
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);


	if (ok) {
		cout << "�궨������" << endl;
		cout << cameraMatrix << endl;
		cout << "���������" << endl;
		cout << distCoeffs << endl;

		// ʱ�䡢ͼƬ������ͼƬ�ߴ硢�궨���ߡ��궨���ߴ硢RMS
		// �궨�������������

		// д������
		// String inCailFilePath = "./calibration_in_params.xml";
		String inCailFilePath = "./calibration_in_params" + getCurrentTime() + ".yml";
		cout << "inCailFilePath" << inCailFilePath << endl;
		FileStorage inCailFs(inCailFilePath, FileStorage::WRITE);
		inCailFs << "calibration_time" << getCurrentTimeDate();
		inCailFs << "frame_count" << (int)image_points.size();
		inCailFs << "image_width" << image_size.width;
		inCailFs << "image_height" << image_size.height;
		inCailFs << "boardWidth" << boardSize.width;
		inCailFs << "boardHeight" << boardSize.height;
		inCailFs << "squareSize" << squareSize;
		inCailFs << "rms" << rms;

		inCailFs << "cameraMatrix" << cameraMatrix;
		inCailFs << "distCoeffs" << distCoeffs;
		//inCailFs << "rvecs" << rvecs;
		//inCailFs << "tvecs" << tvecs;
		inCailFs.release();
		std::cout << "�궨����ѱ��棺" << inCailFilePath << std::endl;
	}
	else {
		std::cout << "�궨������������±궨��" << std::endl;
	}
}
/**
* ʵʱ���ǵ㣬��������ǵ�������ﵽ����ִ�б궨������궨���
* @param numBoards    ��Ҫ���ű궨ͼƬ������ȡ����ǵ����
* @param boardSize      ���ӳߴ�Size 7*4
* @param squareSize   ���ӳߴ� mm
* @param flipHorizontal    �Ƿ�ת
*/
int calibrateCameraRealTime(int numBoards, cv::Size boardSize, float squareSize, int delay, bool flipHorizontal)
{
	cv::VideoCapture capture(1);
	if (!capture.isOpened()) {
		std::cout << "�޷���������ͷ��" << std::endl;
		return -1;
	}
	vector<vector<Point2f>> image_points;	//�ǵ����������
	vector<vector<Point3f>> object_points;	//�ǵ����ά����
	cv::Size image_size;
	while (image_points.size() < numBoards) {
		Mat image0, image;
		capture >> image0;
		image_size = image0.size();
		// ��ͼ���Ƶ�image
		image0.copyTo(image);
		// ˮƽ��ת
		if (flipHorizontal) {
			flip(image, image, 1);
		}
		// ���ұ궨��(���Գ�Բ�����)
		vector<Point2f> corners;
		//bool found = findCirclesGrid(image, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID);
		bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		// ����ȥ
		drawChessboardCorners(image, boardSize, corners, found);

		int action = waitKey(delay) & 255;

		// �ж϶���
		if (action == ACTION_SPACE) { // �û������˿ո�
			if (found) {
				// ����
				bitwise_not(image, image);
				// ����ǵ�
				printf("%s: %d/%d \n", "save�ǵ�", (int)image_points.size() + 1, numBoards);
				image_points.push_back(corners);
				// ����ͼƬ
			}
			else {
				printf("%s\n", "δ��⵽�ǵ�");
			}

		}
		else if (action == ACTION_ESC) { // �û�������ESC
			break;
		}

		cv::imshow("Calibration", image);

	}

	if (image_points.size() < numBoards) {
		printf("�ǵ�δ�ﵽĿ��������궨����ֹ��");
		return -1;
	}

	printf("�ǵ��ռ����, ִ�б궨��... ͼƬ�ߴ� %d:%d\n", image_size.width, image_size.height);

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	runCalibrationSave(squareSize, boardSize, image_size, image_points, cameraMatrix, distCoeffs);

	cv::destroyWindow("Calibration");
	return 1;
}


/**
* ��ָ��ͼƬ�в��ҽǵ㣬������������corners��
* @param img     �����ͼƬ
* @param corners ��⵽�Ľ����б�
* @return �Ƿ��⵽�ǵ㣨�����ڷ���Ľ��㣩
*/
bool findCorners(Mat &img, vector<Point2f> &corners, cv::Size boardSize) {
	Mat gray;
	// ��ͼƬת�ɻҶ�ͼ
	cvtColor(img, gray, COLOR_RGB2GRAY);
	// ���ҵ�ǰͼƬ���еĽǵ�
	//�ڲα궨(���̸�)
	bool patternWasFound = findChessboardCorners(gray, boardSize, corners);
	//�ڲα궨(Բ����)
	//bool found = findCirclesGrid(image, boardSize, corners);
	//bool patternWasFound = 1;
	if (patternWasFound) { // �ҵ��ǵ�
						   // ��߽ǵ�ľ�ȷ��
						   // ԭ��https://docs.opencv.org/4.1.0/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e
		//cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
		//	TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		//ר��������ȡ����ͼ���ڽǵ�ľ�ȷλ��
		//find4QuadCornerSubpix(gray, corners, Size(5, 5));
		// �����еĽ�����ԭͼ�л��Ƴ���
		drawChessboardCorners(img, boardSize, corners, patternWasFound);
		// ������ǵ�֮����ʾԭͼ
		namedWindow("src", WINDOW_NORMAL);
		imshow("src", img);
		waitKey(500);
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

int calibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize)
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
	runCalibrationSave(squareSize, boardSize, imgSize, cornerPoints, cameraMatrix, distCoeffs);

	cv::destroyAllWindows();
	return 0;
}

void readCalibrationYaml(string path, Size &image_size, Mat &intrinsic_matrix, Mat &distortion_coeffs)
{
	// ��ȡ������󡢻���ϵ��
	cv::FileStorage fs(path, FileStorage::READ);
	int image_width{ 0 }, image_height{ 0 };
	fs["image_width"] >> image_width;
	fs["image_height"] >> image_height;
	image_size = Size(image_width, image_height);

	fs["cameraMatrix"] >> intrinsic_matrix;
	fs["distCoeffs"] >> distortion_coeffs;
	fs.release();
	std::cout << intrinsic_matrix << std::endl;
	std::cout << distortion_coeffs << std::endl;
	std::cout << image_size << std::endl;
}

int undistortRectifyImage(string paraPath, string imagePath)
{
	Size image_size;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	readCalibrationYaml(paraPath, image_size, cameraMatrix, distCoeffs);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	if (imagePath == " ")
	{
		cout << "����ʵʱ���" << endl;
		cv::VideoCapture capture(0);
		if (!capture.isOpened())
		{
			cout << "\nCouldn't open the camera\n";
			return -1;
		}
		while (true)
		{
			Mat image, image0;
			capture >> image0;

			if (image0.empty()) {
				break;
			}
			// ִ��ӳ��ת��
			remap(image0, image, mapx, mapy,
				cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

			imshow("original", image0);
			imshow("undistorted", image);

			if ((waitKey(30) & 255) == 27) {
				break;
			}
		}
	}
	else
	{
		const Mat &image0 = imread(imagePath, IMREAD_COLOR);
		Mat image;
		undistort(image0, image, cameraMatrix, distCoeffs);

		imshow("original", image0);
		imshow("undistorted", image);
		waitKey();
	}
	cv::destroyAllWindows();
	return 1;
}

void GetCameraParams()
{
	double brightness = 0;        //����
	double contrast = 0;        //�Աȶ�
	double saturation = 0;        //���Ͷ�
	double hue = 0;                //ɫ��
	double gain = 0;            //����
	double exposure = 0;        //�ع�
	double white_balance = 0;    //��ƽ��
	double width = 0;
	double height = 0;

	VideoCapture capture(0);
	brightness = capture.get(cv::CAP_PROP_BRIGHTNESS);
	contrast = capture.get(cv::CAP_PROP_CONTRAST);
	saturation = capture.get(cv::CAP_PROP_SATURATION);
	hue = capture.get(cv::CAP_PROP_HUE);
	gain = capture.get(cv::CAP_PROP_GAIN);
	exposure = capture.get(cv::CAP_PROP_EXPOSURE);
	white_balance = capture.get(cv::CAP_PROP_WHITE_BALANCE_BLUE_U);
	width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	std::cout << "---------------------------------------------" << endl;
	std::cout << "����ͷ���ȣ�" << brightness << endl;
	std::cout << "����ͷ�Աȶȣ�" << contrast << endl;
	std::cout << "����ͷ���Ͷȣ�" << saturation << endl;
	std::cout << "����ͷɫ����" << hue << endl;
	std::cout << "����ͷ���棺" << gain << endl;
	std::cout << "����ͷ�ع�ȣ�" << exposure << endl;
	std::cout << "����ͷ��ƽ�⣺" << white_balance << endl;
	std::cout << "����ͷ����������" << width << endl;
	std::cout << "����ͷ�������ߣ���" << height << endl;
	std::cout << "---------------------------------------------" << endl;
}