#include "ColorDetection.h"
int main()
{
	int iLowH = 100;
	int iHighH = 150;
	int iLowS = 100;
	int iHighS = 150;
	int iLowV = 100;
	int iHighV = 150;
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	createTrackbar("lowH", "Control",&iLowH, 255);
	createTrackbar("HighH","Control", &iHighH, 255);
	createTrackbar("lowS","Control", &iLowS, 255);
	createTrackbar("HightS","Control",&iHighS, 255);
	createTrackbar("LowV","Control",&iLowV, 255);
	createTrackbar("HighV","Control", &iHighV, 255);
	ColorDetection *detection = new ColorDetection("40.jpg");
	detection->initColorDetection("40.jpg");
	Mat img = detection->gethsvImg();
	while(true){
		vector<Mat> hsvSplit;
		split(img, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, img);
		Mat imgThreshold;
		inRange(img, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThreshold);
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(imgThreshold, imgThreshold, MORPH_OPEN, element);

		morphologyEx(imgThreshold, imgThreshold, MORPH_CLOSE, element);
		//imshow("HSVImg", detection->gethsvImg());
		imshow("ImgThreshold", imgThreshold);
		//imshow("RawImg", detection->getRawImg());
		char c = waitKey(30);
		if(c ==27)
			break;
	}
	printf("print any key to end...");
	getchar();
	return 0;
}