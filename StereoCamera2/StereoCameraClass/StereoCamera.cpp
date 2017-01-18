#include "StereoCamera.h"

void StereoCamera::saveCloudPoints(std::string cloud_point_filename, std::string distance_filename, const cv::Mat& mat)
{
	const double max_z = 1.0e4;
	std::ofstream fp(cloud_point_filename);
	std::ofstream fpDistance(distance_filename);
	if (!fp.is_open())
	{
		std::cout<<"打开点云文件失败"<<std::endl;    
		fp.close();  
		return ;
	}  
	//遍历写入
	for(int y = 0; y < mat.rows; y++)
	{
		for(int x = 0; x < mat.cols; x++)
		{
			cv::Vec3f point = mat.at<cv::Vec3f>(y, x);   //三通道浮点型
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)   
				continue;   
			fp<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;
			fpDistance<<sqrt(point[0]*point[0] + point[1]*point[1] + point[2]+point[2])<<std::endl; 
		}
	}
	fp.close();
	fpDistance.close();
}

void StereoCamera::initPictureFileList(std::string pic_filename, int first, int last)
{
	picFileList.clear();
	for(int cur = first; cur <= last; cur++){
		std::string str_file = std::string(pic_filename) + "\\" + std::to_string(cur) + ".jpg";
		std::cout<<str_file<<std::endl;
		picFileList.push_back(str_file);
	}
}

void StereoCamera::saveParallaxPic(std::string filename, const cv::Mat& mat)
{
	std::ofstream fp(filename, std::ios::out);
	fp<<mat.rows<<std::endl;
	fp<<mat.cols<<std::endl;
	for(int y = 0; y < mat.rows; y++)
	{
		for(int x = 0; x < mat.cols; x++)
		{
			double disp = mat.at<short>(y, x); // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
			fp<<disp<<std::endl;       // 若视差矩阵是 CV_32F 格式，则用 float 类型读取
		}
	}
	fp.close();
}

cv::Mat StereoCamera::mergeImg(cv::Mat img1, cv::Mat disp8){
	cv::Mat color_mat = cv::Mat::zeros(img1.size(), CV_8UC3);

	cv::Mat red = img1.clone();
	cv::Mat green = disp8.clone();
	cv::Mat blue = cv::Mat::zeros(img1.size(), CV_8UC1);

	std::vector<cv::Mat> vec;
	vec.push_back(red);
	vec.push_back(blue);
	vec.push_back(green);
	cv::merge(vec, color_mat);
	return color_mat;
}

void StereoCamera::fromGrayToColor(cv::Mat gray_mat, cv::Mat& color_mat)
{
	using namespace cv;
	color_mat = Mat::zeros(gray_mat.size(), CV_8UC3);
	int rows = color_mat.rows, cols = color_mat.cols;

	Mat red = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
	Mat green = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
	Mat blue = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
	Mat mask = Mat(gray_mat.rows, gray_mat.cols, CV_8U);

	subtract(gray_mat, Scalar(255), blue);         // blue(I) = 255 - gray(I)
	red = gray_mat.clone();                        // red(I) = gray(I)
	green = gray_mat.clone();                      // green(I) = gray(I),if gray(I) < 128

	compare(green, 128, mask, CMP_GE);             // green(I) = 255 - gray(I), if gray(I) >= 128
	subtract(green, Scalar(255), green, mask);
	convertScaleAbs(green, green, 2.0, 2.0);

	vector<Mat> vec;
	vec.push_back(red);
	vec.push_back(green);
	vec.push_back(blue);
	cv::merge(vec, color_mat);
}

void StereoCamera::tailImage(cv::Mat rawImg)
{
	cv::Rect leftRect(0, 0, imgSize.width, imgSize.height);
	cv::Rect rightRect(imgSize.width, 0, imgSize.width, imgSize.height);

	this->leftImage = rawImg(leftRect);       //切分得到的左原始图像
	this->rightImage = rawImg(rightRect);     //切分得到的右原始图像
}

void StereoCamera::findchessboardCorners(cv::Mat leftImg, cv::Mat rightImg, uint i){
	//寻找角点， 图像缩放
	using namespace cv;	
	vector<Point2f> leftPts, rightPts;      // 存储左右相机的角点位置
	Mat leftSimg, rightSimg, leftCimg, rightCimg;
	resize(leftImg, leftSimg, Size(), imgScale, imgScale);      //图像以0.5的比例缩放
	resize(rightImg, rightSimg, Size(), imgScale, imgScale);
	cvtColor(leftSimg, leftCimg, CV_GRAY2BGR);     //转为BGR图像，cimg和simg是800*600的图像
	cvtColor(rightSimg, rightCimg, CV_GRAY2BGR);
	//寻找棋盘角点
	bool leftFound = findChessboardCorners(leftCimg, chessboardSize, leftPts, CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
	bool rightFound = findChessboardCorners(rightCimg, chessboardSize, rightPts, CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);

	if(leftFound)
		cornerSubPix(leftSimg, leftPts, Size(11, 11), Size(-1,-1 ), 
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 300, 0.01));
	if(rightFound)
		cornerSubPix(rightSimg, rightPts, Size(11, 11), Size(-1, -1), 
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 300, 0.01));   //亚像素

	//放大为原来的尺度
	for(uint j = 0;j < leftPts.size();j++)
		leftPts[j] *= 1./imgScale;
	for(uint j = 0;j < rightPts.size();j++)
		rightPts[j] *= 1./imgScale;
	//显示
	string leftWindowName = "Left Corner Pic", rightWindowName = "Right Corner Pic";
	Mat leftPtsTmp = Mat(leftPts) * imgScale;      //再次乘以 imgScale
	Mat rightPtsTmp = Mat(rightPts) * imgScale;

	drawChessboardCorners(leftCimg, chessboardSize, leftPtsTmp, leftFound);      //绘制角点坐标并显示
	imshow(leftWindowName, leftCimg);
	imwrite("输出/DrawChessBoard/"+std::to_string(i)+"_left.jpg", leftCimg);
	waitKey(200);
	drawChessboardCorners(rightCimg, chessboardSize, rightPtsTmp, rightFound);   //绘制角点坐标并显示
	imshow(rightWindowName, rightCimg);
	imwrite("输出/DrawChessBoard/"+std::to_string(i)+"_right.jpg", rightCimg);
	waitKey(200);
	cv::destroyAllWindows();

	//保存角点坐标
	if(leftFound && rightFound)   
	{
		imagePoints[0].push_back(leftPts);
		imagePoints[1].push_back(rightPts);  //保存角点坐标
		std::cout<<"图片 "<<i<<" 处理成功！"<<std::endl;
		idx.push_back(i);
	}
}

void StereoCamera::preBlurImage(cv::Mat leftImg, cv::Mat rightImg)
{
	using namespace cv;
	Mat leftMask, rightMask;
	resize(leftImg, leftMask, Size(200, 200));		//resize对原图像img重新调整大小生成mask图像大小为200*200
	resize(rightImg, rightMask, Size(200, 200));
	GaussianBlur(leftMask, leftMask, Size(13, 13), 7);   
	GaussianBlur(rightMask, rightMask, Size(13, 13), 7); 
	resize(leftMask, leftMask, imgSize);
	resize(rightMask, rightMask, imgSize);
	medianBlur(leftMask, leftMask, 9);    //中值滤波
	medianBlur(rightMask, rightMask, 9);

	for (int v = 0; v < imgSize.height; v++) {
		for (int u = 0; u < imgSize.width; u++) {
			int leftX = ((int)leftImg.at<uchar>(v, u) - (int)leftMask.at<uchar>(v, u)) * 2 + 128;
			int rightX = ((int)rightImg.at<uchar>(v, u) - (int)rightMask.at<uchar>(v, u)) * 2 + 128;
			leftImg.at<uchar>(v, u)  = max(min(leftX, 255), 0);
			rightImg.at<uchar>(v, u) = max(min(rightX, 255), 0);
		}
	}
	this->leftImage = leftImg;
	this->rightImage = rightImg;
}

void StereoCamera::writeParameterMatrix(std::string intrinsic_filename, std::string extrinsic_filename, 
										cv::Mat R1, cv::Mat R2, cv::Mat P1, cv::Mat P2, cv::Mat Q)
{
	cv::FileStorage fs(intrinsic_filename, CV_STORAGE_WRITE);
	if(fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		std::cout << "Error: can not save the intrinsic parameters\n";
	fs.open(extrinsic_filename, CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else{
		std::cout << "Error: can not save the intrinsic parameters\n";
		exit(4);
	}
}

void StereoCamera::stereoCalibrateCamera(std::string intrinsic_filename, std::string extrinsic_filename)
{
	using namespace cv;
	for(uint i = 0; i < picFileList.size();++i)
	{
		cv::Mat rawImg = cv::imread(picFileList[i]);					   //原始图像
		if(rawImg.empty()){
			std::cout<<"the Image is empty..."<<picFileList[i]<<std::endl;
			continue;
		}
		//截取左右图片
		tailImage(rawImg);
		Mat leftRawImg = getLeftImage();      
		Mat rightRawImg = getRightImage();    
		Mat leftImg, rightImg, leftSimg, rightSimg, leftCimg, rightCimg, leftMask, rightMask;
		// BGT -> GRAY	
		if(leftRawImg.type() == CV_8UC3)
			cvtColor(leftRawImg, leftImg, CV_BGR2GRAY);  //转为灰度图
		else
			leftImg = leftRawImg.clone();
		if(rightRawImg.type() == CV_8UC3)
			cvtColor(rightRawImg, rightImg, CV_BGR2GRAY); 
		else
			rightImg = rightRawImg.clone();
		imgSize = leftImg.size();
		//图像预处理	
		preBlurImage(leftImg, rightImg);
		findchessboardCorners(this->getLeftImage(),this->getRightImage(),i);
	}
	cv::destroyAllWindows();
	this->imagePoints[0].resize(this->idx.size());
	this->imagePoints[1].resize(this->idx.size());
	std::cout<<"成功标定的标定板个数为"<<idx.size()<<"  序号分别为: ";
	for(unsigned int i = 0;i < idx.size();++i)
		std::cout<<idx[i]<<"  ";

	//生成物点坐标
	vector<vector<Point3f>> objPts(idx.size());  //idx.size代表成功检测的图像的个数
	for (int y = 0; y < chessboardSize.height; y++) {
		for (int x = 0; x < chessboardSize.width; x++) {
			objPts[0].push_back(Point3f((float)x, (float)y, 0) * chessboardLength);
		}
	}
	for (uint i = 1; i < objPts.size(); i++) {
		objPts[i] = objPts[0];
	}
	// 双目立体标定	
	vector<Mat> rvecs[2], tvecs[2]; 
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	cv::calibrateCamera(objPts, imagePoints[0], imgSize, cameraMatrix[0],    
		distCoeffs[0], rvecs[0], tvecs[0], CV_CALIB_FIX_K3);

	cv::calibrateCamera(objPts, imagePoints[1], imgSize, cameraMatrix[1],    
		distCoeffs[1], rvecs[1], tvecs[1], CV_CALIB_FIX_K3);

	std::cout<<"Left Camera Matrix: "<<std::endl<<cameraMatrix[0]<<std::endl;
	std::cout<<"Right Camera Matrix："<<std::endl<<cameraMatrix[1]<<std::endl;
	std::cout<<"Left Camera DistCoeffs: "<<std::endl<<distCoeffs[0]<<std::endl;
	std::cout<<"Right Camera DistCoeffs: "<<std::endl<<distCoeffs[1]<<std::endl;

	double rms = stereoCalibrate(objPts, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imgSize, R, T, E, F,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
	//CV_CALIB_USE_INTRINSIC_GUESS);
	std::cout<<"立体标定完成！ "<<std::endl<<"done with RMS error=" << rms << std::endl;  //反向投影误差

	std::cout<<"Left Camera Matrix: "<<std::endl<<cameraMatrix[0]<<std::endl;
	std::cout<<"Right Camera Matrix："<<std::endl<<cameraMatrix[1]<<std::endl;
	std::cout<<"Left Camera DistCoeffs: "<<std::endl<<distCoeffs[0]<<std::endl;
	std::cout<<"Right Camera DistCoeffs: "<<std::endl<<distCoeffs[1]<<std::endl;
	accuracyCheck();
	stereoCameraRectify(cameraMatrix, distCoeffs, intrinsic_filename, extrinsic_filename);
	std::cout<<"双目标定完成..."<<std::endl;
}

void StereoCamera::accuracyCheck()
{
	std::cout<<" 极线计算...  误差计算... "<<std::endl;
	double err = 0;
	int npoints = 0;
	std::vector<cv::Vec3f> lines[2];
	for(unsigned int i = 0; i < idx.size(); i++)
	{
		int npt = (int)imagePoints[0][i].size();  //角点个数
		cv::Mat imgpt[2];
		for(int k = 0; k < 2; k++ )
		{
			imgpt[k] = cv::Mat(imagePoints[k][i]);  //
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]); // 畸变
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);  // 计算极线
		}
		for(int j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;   // 累计误差
		}
		npoints += npt;
	}
	std::cout << "  平均误差 average reprojection err = " <<  err/npoints << std::endl;  // 平均误差
}

void StereoCamera::stereoCameraRectify(cv::Mat cameraMatrix[2], cv::Mat distCoeffs[2], 
									   std::string intrinsic_filename, std::string extrinsic_filename)
{
	using namespace cv;
	// 立体矫正  BOUGUET'S METHOD
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imgSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imgSize, &validRoi[0], &validRoi[1]);
	writeParameterMatrix(intrinsic_filename, extrinsic_filename, 
						R1, R2, P1, P2, Q);
	std::cout<<"双目校正完成"<<std::endl;
}

void StereoCamera::BMstereoMatch(cv::Mat left_img, cv::Mat right_img, cv::Mat disp, cv::Mat disp8)
{
	cv::StereoBM bm;

	int unitDisparity = 15;//40
	int numberOfDisparities = unitDisparity * 16;
	bm.state->roi1 = roi1;
	bm.state->roi2 = roi2;
	bm.state->preFilterCap = 13;
	bm.state->SADWindowSize = 19;                                     // 窗口大小
	bm.state->minDisparity = 0;                                       // 确定匹配搜索从哪里开始  默认值是0
	bm.state->numberOfDisparities = numberOfDisparities;                // 在该数值确定的视差范围内进行搜索
	bm.state->textureThreshold = 1000;//10                                  // 保证有足够的纹理以克服噪声
	bm.state->uniquenessRatio = 1;     //10                               // !!使用匹配功能模式
	bm.state->speckleWindowSize = 200;   //13                             // 检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查
	bm.state->speckleRange = 32;    //32                                  // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型
	bm.state->disp12MaxDiff = -1;

	bm(left_img, right_img, disp);

	// 将16位符号整形的视差矩阵转换为8位无符号整形矩阵
	disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
	this->disp = disp;
	this->disp8 = disp8;
}

void StereoCamera::readingParameterMatrix(std::string intrinsic_filename, std::string extrinsic_filename)
{
	using namespace cv;
	time_t = getTickCount();
	// reading intrinsic parameters
	FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
	if(!fs.isOpened())
	{
		std::cout<<"Failed to open file "<<intrinsic_filename<<std::endl;
		exit(0);
	}
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs.release();
	M1 *= imgScale;
	M2 *= imgScale;

	// 读取双目相机的立体矫正参数
	fs.open(extrinsic_filename, CV_STORAGE_READ);
	if(!fs.isOpened())
	{
		std::cout<<"Failed to open file  "<<extrinsic_filename<<std::endl;
		exit(1);
	}
	// reading extrinsic matrix
	fs["R"] >> R;
	fs["T"] >> T;
	//fs["R1"] >> R1;
	//fs["R2"] >> R2;
	//fs["P1"] >> P1;
	//fs["P2"] >> P2;
	//fs["Q"] >> Q;
	fs.release();
	time_t = getTickCount()-time_t;
	std::cout<<"读参数耗时："<<getTimeSpended()<<std::endl;
	std::cout<<"Reading parameters finished...\n";
}

void StereoCamera::stereoMatch(int pic_num, bool no_display, std::string point_cloud_filename, std::string distance_filename)
{
	using namespace cv;
	int color_mode = 0;
	Mat rawImg = imread(picFileList[pic_num], color_mode);    //待处理图像  grayScale
	if(rawImg.empty()){
		std::cout<<"In Function stereoMatch, the Image is empty..."<<std::endl;
		exit(-1);
	}
	//截取
	tailImage(rawImg);
	Mat img1 = getLeftImage();       //切分得到的左原始图像
	Mat img2 = getRightImage();      //切分得到的右原始图像
	//图像根据比例缩放
	if(imgScale != 1.f){
		time_t = getTickCount();
		Mat temp1, temp2;
		int method = imgScale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), imgScale, imgScale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), imgScale, imgScale, method);
		img2 = temp2;
		time_t = getTickCount() - time_t;
		printf("缩放图像耗时：%fms\n", getTimeSpended());
	}

	imwrite("输出/原始左图像.jpg", img1);
	imwrite("输出/原始右图像.jpg", img2);

	Size img_size = img1.size();
	time_t = getTickCount();
	//Mat R1, P1, R2, P2, Q;
	//Alpha取值为-1时，OpenCV自动进行缩放和平移
	cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
	//cv::FileStorage fs("test.yml",CV_STORAGE_WRITE);
	//fs <<"R1"<<_R1;
	//fs <<"this R1"<<R1;
	//fs <<"P1"<<_P1;
	//fs <<"this P1"<< P1;
	//fs.release();
	time_t = getTickCount()-time_t;
	std::cout<<"立体校正耗时："<<getTimeSpended()<<std::endl;
	 //获取两相机的矫正映射
	time_t = getTickCount();
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	time_t = getTickCount()-time_t;
	printf("获取矫正映射耗时：%fms\n",getTimeSpended());
	// 矫正原始图像
	time_t = getTickCount();
	Mat img1r, img2r;
	remap(img1, img1r, map11, map12, INTER_LINEAR);
	remap(img2, img2r, map21, map22, INTER_LINEAR);
	img1 = img1r;
	img2 = img2r;
	time_t = getTickCount() - time_t;
	printf("矫正映射耗时：%fms\n",getTimeSpended());
	int64 t = getTickCount();
	// 使用BM算法匹配
	Mat disp_8, disp;
	BMstereoMatch(img1, img2, disp, disp_8);
	t = getTickCount() - t;
	printf("BM立体匹配耗时: %fms\n", t*1000/getTickFrequency());
	// 视差图转为彩色图
	Mat vdispRGB = this->disp8;
	fromGrayToColor(disp8, vdispRGB);
	// 将左侧矫正图像与视差图融合
	Mat merge_mat = mergeImg(img1, disp8);
	saveParallaxPic("输出/视差数据.txt", disp);

	//显示
	if(!no_display){
		time_t = getTickCount();
		imshow("左侧矫正图像", img1);
		imwrite("输出/left_undistortRectify.jpg", img1);
		imshow("右侧矫正图像", img2);
		imwrite("输出/right_undistortRectify.jpg", img2);
		imshow("视差图", disp8);
		imwrite("输出/视差图.jpg", disp8);
		imshow("视差图_彩色.jpg", vdispRGB);
		imwrite("输出/视差图_彩色.jpg", vdispRGB);
		imshow("左矫正图像与视差图合并图像", merge_mat);
		imwrite("输出/左矫正图像与视差图合并图像.jpg", merge_mat);
		time_t = getTickCount() - time_t;
		printf("显示耗时：%fms",time_t*1000/getTickFrequency());
		cv::waitKey();
		std::cout<<std::endl;
	}
	cv::destroyAllWindows();
	// 视差图转为深度图
	std::cout<<"计算深度映射... "<<std::endl;
	Mat xyz;
	reprojectImageTo3D(disp, xyz, Q, true);    //获得深度图  disp: 720*1280 
	cv::destroyAllWindows(); 
	std::cout<<"保存点云坐标... "<<std::endl;
	saveCloudPoints(point_cloud_filename, distance_filename, xyz);
	std::cout<<std::endl<<"结束"<<std::endl<<"Press any key to end... ";
	getchar(); 
}
