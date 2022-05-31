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
	// 获取当前时间
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
	// 获取当前时间
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
		namedWindow("实时相机画面", WINDOW_AUTOSIZE);
		imshow("实时相机画面", cam);//显示当前帧图像
							  //imwrite(to_string(i) + ".png", cam);
		waitKey(20);//延时20ms
	}
}

/**
* 实时显示相机画面，按键保存图片
* 按空格键保存，按Esc退出
**/
int saveCameraImages(string savePath)
{
	//1.从摄像头读入视频
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cout << "无法开启摄像头！" << std::endl;
		return -1;
	}
	if (savePath != "./")
	{
		myMkdir(savePath);
	}
	//2.循环显示每一帧 
	while (1) {
		Mat image0, image;
		capture >> image0;
		// 将图像复制到image
		image0.copyTo(image);
		int action = waitKey(30) & 255;
		// 判断动作
		if (action == ACTION_SPACE) { // 用户按下了空格
									  // 保存图片
			string imgFileName = savePath + getCurrentTime() + ".png";
			imwrite(imgFileName, image);
			cout << imgFileName << " saved" << endl;
		}
		else if (action == ACTION_ESC) { // 用户按下了ESC
			break;
		}
		cv::imshow("实时相机画面", image);
	}
	cv::destroyAllWindows();
	return 1;
}

/**
* 实时显示相机画面，按键保存能检测到角点的 棋盘格图片
* 按空格键保存，按Esc退出
* @param boardSize      格子尺寸Size 7*4
**/
int saveChessboardImages(cv::Size boardSize, string savePath)
{
	//1.从摄像头读入视频
	VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cout << "无法开启摄像头！" << std::endl;
		return -1;
	}
	if (savePath != "./")
	{
		myMkdir(savePath);
	}
	//2.循环显示每一帧
	while (1) {
		Mat image0, image;
		capture >> image0;
		// 将图像复制到image
		image0.copyTo(image);
		// 查找标定板(不对称圆网格板)
		vector<Point2f> corners;
		//bool found = findCirclesGrid(image, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID);
		bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_FAST_CHECK);

		// 画上去
		drawChessboardCorners(image, boardSize, corners, found);

		int action = waitKey(30) & 255;

		// 判断动作
		if (action == ACTION_SPACE) { // 用户按下了空格
			if (found) {
				// 保存图片
				string imgFileName = savePath + getCurrentTime() + ".png";
				imwrite(imgFileName, image0);
				cout << imgFileName << " saved" << endl;
			}
			else {
				printf("%s\n", "未检测到角点");
			}
		}
		else if (action == ACTION_ESC) { // 用户按下了ESC
			break;
		}
		cv::imshow("Calibration", image);
	}
	cv::destroyAllWindows();
	return 1;
}

static void print_help() {
	printf("通过棋盘格标定相机\n");
	printf("参数：CalibrationChessboard <boardWidth> n<boardHeight> <squareSize> [number_of_boards] "
		"[--delay=<delay>] [-s=<scale_factor>]\n");
	printf("例子：CalibrationACircle2 6 9 30 500 1.0\n");
	//opencv的参数解析器 以后研究
	//bool flipHorizontal = false;
	//// 解析参数
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
* 执行标定并保存标定结果
* @param squareSize   格子尺寸 mm
* @param boardSize      格子尺寸Size 7*4
* @param image_size    图片尺寸Size 1920*1080
* @param image_points  图片角点集合
* @param cameraMatrix  相机参数
* @param distCoeffs    畸变参数
*/
void
runCalibrationSave(float squareSize, const Size boardSize, const Size image_size,
	const vector<vector<Point2f>> &image_points, Mat &cameraMatrix, Mat &distCoeffs)
{
	vector<vector<Point3f>> object_points;
	vector<Point3f> objPoints;
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			// 注意非对称圆网格的对象坐标计算方式
			/*objPoints.push_back(Point3f(float((2 * j + i % 2) * squareSize),
			float(i * squareSize), 0));*/
			//棋盘格 计算uv空间中角点对应的相机坐标系坐标值，设Z为0
			objPoints.emplace_back(j * squareSize, i * squareSize, 0);//宽width为x,高height为y
		}
	}
	object_points.resize(image_points.size(), objPoints);
	vector<Mat> rvecs, tvecs;

	// 执行标定
	double rms = calibrateCamera(object_points, image_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs);

	// 均方根值(RMS)
	printf("RMS error reported by calibrateCamera: %g\n", rms);
	// 检查标定结果误差
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);


	if (ok) {
		cout << "标定参数：" << endl;
		cout << cameraMatrix << endl;
		cout << "畸变参数：" << endl;
		cout << distCoeffs << endl;

		// 时间、图片个数、图片尺寸、标定板宽高、标定板块尺寸、RMS
		// 标定参数、畸变参数

		// 写出数据
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
		std::cout << "标定结果已保存：" << inCailFilePath << std::endl;
	}
	else {
		std::cout << "标定结果有误，请重新标定！" << std::endl;
	}
}
/**
* 实时检测角点，按键保存角点参数，达到数量执行标定并保存标定结果
* @param numBoards    需要几张标定图片，即获取几组角点参数
* @param boardSize      格子尺寸Size 7*4
* @param squareSize   格子尺寸 mm
* @param flipHorizontal    是否翻转
*/
int calibrateCameraRealTime(int numBoards, cv::Size boardSize, float squareSize, int delay, bool flipHorizontal)
{
	cv::VideoCapture capture(0);
	if (!capture.isOpened()) {
		std::cout << "无法开启摄像头！" << std::endl;
		return -1;
	}
	vector<vector<Point2f>> image_points;	//角点的像素坐标
	vector<vector<Point3f>> object_points;	//角点的三维坐标
	cv::Size image_size;
	while (image_points.size() < numBoards) {
		Mat image0, image;
		capture >> image0;
		image_size = image0.size();
		// 将图像复制到image
		image0.copyTo(image);
		// 水平翻转
		if (flipHorizontal) {
			flip(image, image, 1);
		}
		// 查找标定板(不对称圆网格板)
		vector<Point2f> corners;
		//bool found = findCirclesGrid(image, boardSize, corners, CALIB_CB_ASYMMETRIC_GRID);
		bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		// 画上去
		drawChessboardCorners(image, boardSize, corners, found);

		int action = waitKey(delay) & 255;

		// 判断动作
		if (action == ACTION_SPACE) { // 用户按下了空格
			if (found) {
				// 闪屏
				bitwise_not(image, image);
				// 保存角点
				printf("%s: %d/%d \n", "save角点", (int)image_points.size() + 1, numBoards);
				image_points.push_back(corners);
				// 保存图片
			}
			else {
				printf("%s\n", "未检测到角点");
			}

		}
		else if (action == ACTION_ESC) { // 用户按下了ESC
			break;
		}

		cv::imshow("Calibration", image);

	}

	if (image_points.size() < numBoards) {
		printf("角点未达到目标个数，标定已终止！");
		return -1;
	}

	printf("角点收集完毕, 执行标定中... 图片尺寸 %d:%d\n", image_size.width, image_size.height);

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	runCalibrationSave(squareSize, boardSize, image_size, image_points, cameraMatrix, distCoeffs);

	cv::destroyWindow("Calibration");
	return 1;
}


/**
* 在指定图片中查找角点，并将结果输出到corners中
* @param img     待检测图片
* @param corners 检测到的焦点列表
* @return 是否检测到角点（两个黑方格的交点）
*/
bool findCorners(Mat &img, vector<Point2f> &corners, cv::Size boardSize) {
	Mat gray;
	// 将图片转成灰度图
	cvtColor(img, gray, COLOR_RGB2GRAY);
	// 查找当前图片所有的角点
	//内参标定(棋盘格)
	bool patternWasFound = findChessboardCorners(gray, boardSize, corners);
	//内参标定(圆网格)
	//bool found = findCirclesGrid(image, boardSize, corners);
	//bool patternWasFound = 1;
	if (patternWasFound) { // 找到角点
						   // 提高角点的精确度
						   // 原理：https://docs.opencv.org/4.1.0/dd/d1a/group__imgproc__feature.html#ga354e0d7c86d0d9da75de9b9701a9a87e
		//cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
		//	TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		//专门用来获取棋盘图上内角点的精确位置
		//find4QuadCornerSubpix(gray, corners, Size(5, 5));
		// 将所有的焦点在原图中绘制出来
		drawChessboardCorners(img, boardSize, corners, patternWasFound);
		// 绘制完角点之后，显示原图
		namedWindow("src", WINDOW_NORMAL);
		imshow("src", img);
		waitKey(500);
	}
	else {
		namedWindow("no Corners", WINDOW_NORMAL);
		imshow("no Corners", gray);
		waitKey(500);
		cout << "角点检测失败！" << endl;
		return patternWasFound;
	}
	return patternWasFound;
}

int calibrateCameraOffLine(string imagePath, const Size boardSize, float squareSize)
{
	// 保存多张图片对象点列表
	vector<vector<Point3f>> objectPoints;
	// 保存多张图片的角点列表
	vector<vector<Point2f>> cornerPoints;
	// 图片像素尺寸
	Size imgSize;

	std::vector<String> filenames;
	cv::glob(imagePath, filenames);//获取路径下所有文件名
	cout << "filenames.size:" << filenames.size() << endl;
	for (auto& imgName : filenames) {
		// 读取图片
		Mat img = imread(imgName, IMREAD_COLOR);
		if (img.empty())
		{
			cout << "load file error" << endl;
			continue;
		}
		// 获取图片像素尺寸
		imgSize = img.size();
		std::cout << "name: " << imgName << " imgSize: " << imgSize << std::endl;
		//声明每张图片的角点
		vector<Point2f> corners;
		bool found = findCorners(img, corners, boardSize);
		if (found) {
			vector<Point3f> objPoints;
			// 找到角点，证明这张图是有效的
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
	// 读取相机矩阵、畸变系数
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
		cout << "矫正实时相机" << endl;
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
			// 执行映射转换
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
	double brightness = 0;        //亮度
	double contrast = 0;        //对比度
	double saturation = 0;        //饱和度
	double hue = 0;                //色调
	double gain = 0;            //增益
	double exposure = 0;        //曝光
	double white_balance = 0;    //白平衡
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
	std::cout << "摄像头亮度：" << brightness << endl;
	std::cout << "摄像头对比度：" << contrast << endl;
	std::cout << "摄像头饱和度：" << saturation << endl;
	std::cout << "摄像头色调：" << hue << endl;
	std::cout << "摄像头增益：" << gain << endl;
	std::cout << "摄像头曝光度：" << exposure << endl;
	std::cout << "摄像头白平衡：" << white_balance << endl;
	std::cout << "摄像头质量（宽）：" << width << endl;
	std::cout << "摄像头质量（高）：" << height << endl;
	std::cout << "---------------------------------------------" << endl;
}