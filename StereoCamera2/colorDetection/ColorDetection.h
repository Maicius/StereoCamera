#include "stdafx.h"
using namespace cv;
class ColorDetection{
public:
	ColorDetection(std::string){}
	~ColorDetection(){}
	void initColorDetection(std:: string);

	Mat getRawImg(){ return rawImg;}
	Mat gethsvImg(){ return hsvImg;}
private:
	Mat rawImg, hsvImg, leftImg, rightImg;
	Rect imgSize;

};