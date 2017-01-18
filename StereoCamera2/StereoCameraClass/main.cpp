#include "StereoCamera.h"
int main()
{
	std::string intrinsic_filename = "intrinsics.yml";
	std::string extrinsic_filename = "extrinsics.yml";
	std::string distance_filename = "输出/视差数据.txt";
	std::string point_cloud_filename = "输出/point_test3D.txt";
	std::string pic_filename = "calib_pic2";
	std::string test_pic_filename = "test_pic";

	StereoCamera *stereoCamera  = new StereoCamera();

	/* 立体标定 运行一次即可 */	
	stereoCamera->initPictureFileList(pic_filename, 1, 13);
	stereoCamera->stereoCalibrateCamera(intrinsic_filename, extrinsic_filename); 

	stereoCamera->initPictureFileList(test_pic_filename, 1, 2);
	
	/*读取相机内外参数*/
	stereoCamera->readingParameterMatrix(intrinsic_filename, extrinsic_filename);

	/* 立体匹配 */
	stereoCamera->stereoMatch(0, true, point_cloud_filename, distance_filename);

	return 0;
}