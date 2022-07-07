#include "main.h"


using namespace cv;
using namespace std;

//离线相机标定
void EntranceCalibrateCameraOffLine()
{
	// 棋盘格的尺寸（宽6，高9）
	const Size patternSize(11, 8);
	// 黑方格的大小 20mm
	const float squareSize = 20;
	// 图片路径
	cv::String imagePath = "./Img/Calibration/*.png";
	mycalibrateCameraOffLine(imagePath, patternSize, squareSize);
}

//在线标定，微卡
void EntranceCalibrateCameraOnline()
{
	VideoCapture capture(1);
	if (!capture.isOpened()) {
		std::cout << "无法开启摄像头！" << std::endl;
	}
	// 棋盘格的尺寸（宽6，高9）
	const Size boardSize(6, 9);
	//方格大小
	const float squareSize = 19.5;
	//内参
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	//角点
	vector<vector<Point2f>> ArrayofCorners;
	cout << "按下空格保存此图片信息" << endl;
	bool calibrate_finish = false;
	while (!calibrate_finish)
	{
		Mat cam;
		capture >> cam;//获取当前帧图像
		calibrate_finish = myCalibrationOnline(cam, ArrayofCorners, boardSize, squareSize, cameraMatrix);
		namedWindow("实时相机画面", WINDOW_NORMAL);
		imshow("实时相机画面", cam);//显示当前帧图像
		waitKey(1);//延时20ms
	}
	
}

int main() {
	std::cout << cv::getVersionString() << std::endl;
	/*
		使用图片标定
	*/
	//EntranceCalibrateCameraOffLine();

	/*
		可视化3D模型
	*/
	ReadModelFile Model;
	//STL
	Model.ReadSTL("models/huosai.STL");
	//ply
	//Model.ReadPly("models/bunny.ply");
	detectPoseShow(Model);
	//system("pause");
	return 0;
}
