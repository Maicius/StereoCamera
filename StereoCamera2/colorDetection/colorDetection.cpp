#include "ColorDetection.h"
void ColorDetection::initColorDetection(std:: string filename)
{
	imgSize.width = 1280;
	imgSize.height = 720;
	rawImg = imread(filename);
	if(rawImg.empty()){
		printf("Image is empty\n");
		exit(1);
	}
	leftImg = rawImg(Rect(0, 0, 1280, 720));
	rightImg = rawImg(Rect(1280,0, 1280, 720));
	imshow("right_img", rightImg);
	imshow("left_img", leftImg);
	waitKey();
	cvtColor(leftImg, hsvImg, COLOR_BGR2HSV);
}