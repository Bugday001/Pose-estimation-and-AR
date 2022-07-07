#include "main.h"


using namespace cv;
using namespace std;

//��������궨
void EntranceCalibrateCameraOffLine()
{
	// ���̸�ĳߴ磨��6����9��
	const Size patternSize(11, 8);
	// �ڷ���Ĵ�С 20mm
	const float squareSize = 20;
	// ͼƬ·��
	cv::String imagePath = "./Img/Calibration/*.png";
	mycalibrateCameraOffLine(imagePath, patternSize, squareSize);
}

//���߱궨��΢��
void EntranceCalibrateCameraOnline()
{
	VideoCapture capture(1);
	if (!capture.isOpened()) {
		std::cout << "�޷���������ͷ��" << std::endl;
	}
	// ���̸�ĳߴ磨��6����9��
	const Size boardSize(6, 9);
	//�����С
	const float squareSize = 19.5;
	//�ڲ�
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	//�ǵ�
	vector<vector<Point2f>> ArrayofCorners;
	cout << "���¿ո񱣴��ͼƬ��Ϣ" << endl;
	bool calibrate_finish = false;
	while (!calibrate_finish)
	{
		Mat cam;
		capture >> cam;//��ȡ��ǰ֡ͼ��
		calibrate_finish = myCalibrationOnline(cam, ArrayofCorners, boardSize, squareSize, cameraMatrix);
		namedWindow("ʵʱ�������", WINDOW_NORMAL);
		imshow("ʵʱ�������", cam);//��ʾ��ǰ֡ͼ��
		waitKey(1);//��ʱ20ms
	}
	
}

int main() {
	std::cout << cv::getVersionString() << std::endl;
	/*
		ʹ��ͼƬ�궨
	*/
	//EntranceCalibrateCameraOffLine();

	/*
		���ӻ�3Dģ��
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
