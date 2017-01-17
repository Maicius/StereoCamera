#include "StereoCamera.h"
int main()
{
	std::string intrinsic_filename = "intrinsics.yml";
	std::string extrinsic_filename = "extrinsics.yml";
	std::string distance_filename = "输出/视差数据.txt";
	std::string point_cloud_filename = "输出/point_test3D.txt";

	/* 立体标定 运行一次即可 */
	StereoCamera *stereoCamera  = new StereoCamera();
	stereoCamera->initPictureFileList("I:\\StereoCamera\\StereoCamera2\\StereoCamera2\\calib_pic2", 1, 12);
	stereoCamera->stereoCalibrateCamera(intrinsic_filename, extrinsic_filename); 

	/* 立体匹配 */

	stereoCamera->initPictureFileList("I:\\StereoCamera\\StereoCamera2\\StereoCamera2\\test_pic", 1, 2);
	stereoCamera->stereoMatch(0, intrinsic_filename, extrinsic_filename, false, point_cloud_filename, distance_filename);

	return 0;
}