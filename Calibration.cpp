#include "opencv2/opencv.hpp" 

using namespace cv;
using namespace std;
int displayCameraRealTime();
int main(int argc, char** argv)
{
    // Read source image.
	displayCameraRealTime();
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

