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

