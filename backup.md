# 标定
```cpp
	void creat_v()
	{
			//1
	V.at<double>(0, 0) = H.at<double>(0, i) * H.at<double>(0, j);
	V.at<double>(1, 0) = H.at<double>(0, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(0, j);
	V.at<double>(2, 0) = H.at<double>(0, i) * H.at<double>(2, j) + H.at<double>(2, i) * H.at<double>(0, j);
	V.at<double>(3, 0) = H.at<double>(1, i) * H.at<double>(1, j);
	V.at<double>(4, 0) = H.at<double>(1, i) * H.at<double>(2, j) + H.at<double>(2, i) * H.at<double>(1, j);
	V.at<double>(5, 0) = H.at<double>(2, i) * H.at<double>(2, j);

	//2
	}

	//法一
	//使B_33==1
	double lambda = B.at<double>(1, 2) -
		B.at<double>(0, 2) * B.at<double>(0, 2) / (B.at<double>(0, 0) * B.at<double>(0, 0)) -
		B.at<double>(1, 2) * B.at<double>(1, 2) / (B.at<double>(1, 1) * B.at<double>(1, 1));
	//fx
	cameraMatrix.at<double>(0, 0) = sqrt(lambda / B.at<double>(0, 0));
	//fy
	cameraMatrix.at<double>(1, 1) = sqrt(lambda / B.at<double>(1, 1));
	//Cx
	cameraMatrix.at<double>(0, 2) = -B.at<double>(0, 2) / B.at<double>(0, 0);
	//Cy
	cameraMatrix.at<double>(1, 2) = -B.at<double>(1, 1) / B.at<double>(1, 0);
	cout << "内参:" << endl << cameraMatrix << endl;
	cout << "lambda:" << lambda << endl;

	//法二
	//Cv v0
	cameraMatrix.at<double>(1, 2) = (B.at<double>(0, 1) * B.at<double>(0, 2) -
		B.at<double>(0, 0) * B.at<double>(1, 1)) /
		(B.at<double>(0, 0) * B.at<double>(1, 0) -
			B.at<double>(0, 1) * B.at<double>(0, 1));
	//lambda
	lambda = B.at<double>(1, 2) - (B.at<double>(0, 2) * B.at<double>(0, 2) +
		cameraMatrix.at<double>(1, 2) * (B.at<double>(0, 1) * B.at<double>(0, 2) -
			B.at<double>(0, 0) * B.at<double>(1, 1))) / B.at<double>(0, 0);
	cout << "lambda" << lambda << endl;
	//fx alpha
	cameraMatrix.at<double>(0, 0) = sqrt(lambda / B.at<double>(0, 0));
	//fy beta
	cameraMatrix.at<double>(1, 1) = sqrt(lambda * B.at<double>(0, 0) /
		(B.at<double>(0, 0) * B.at<double>(1, 0) -
			B.at<double>(0, 1) * B.at<double>(0, 1)));
	//s gamma
	cameraMatrix.at<double>(0, 1) = -B.at<double>(0, 1) * cameraMatrix.at<double>(0, 0) *
		cameraMatrix.at<double>(0, 0) * cameraMatrix.at<double>(1, 1) / lambda;

	//Cu u0
	cameraMatrix.at<double>(0, 2) = cameraMatrix.at<double>(0, 1) * cameraMatrix.at<double>(1, 2) / cameraMatrix.at<double>(0, 0) -
		B.at<double>(0, 1) * cameraMatrix.at<double>(0, 0) * cameraMatrix.at<double>(0, 0) / lambda;
```