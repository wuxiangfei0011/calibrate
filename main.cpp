/*
TODO：
	校正后左右图像每对角点误差都 < 3pixel
	disparity尝试也加入上面功能（对于非标定板的场景）
	找出远距离标定不了的原因（人眼能找出角点，机器应该没问题）
	16-10为何tif格式就能标定，而有些png能、tif却不能
	对每一张标定板的all corner pixel err设定一个最大值
	*/

#include <iostream>
#include "CaliCamExter.h"
#include "CaliCamInter.h"

using namespace std;
using namespace cv;

char filename[100];
Size imageSize;
int numDisparities, numDisparitiesHalf;
string cameraParamter = "..\\CameraParamter.yml";

/******************************需要改参数的部分******************************/
int blockLenght = 150; //mm 砖块宽度
Size numCorner = Size(4, 5);
//int blockLenght = 200; 
//Size numCorner = Size(4,3); 
//int blockLenght = 100; 
//Size numCorner = Size(7, 8);

string imagePlace = "4-15"; // R、L图像文件夹，10-20对图像最佳
int numImage = 15; //每个相机使用的标定图像个数
const char* imagePlaceL = "..\\..\\4-15\\L\\%d.png"; //左相机图像文件夹
const char* imagePlaceR = "..\\..\\4-15\\R\\%d.png"; //右相机图像文件夹
/****************************************************************************/

CCaliCamSingal camLeft(numCorner, numImage, blockLenght, imagePlaceL);
CCaliCamSingal camRight(numCorner, numImage, blockLenght, imagePlaceR);
CCaliCamExter stereoCamCalibrate;

void getNumDisparities(Mat rectifyImageL, Mat rectifyImageR);
void storeParamYML();

int main(int argc, char** argv) {

	//读取图像尺寸
	snprintf(filename, 100, imagePlaceL, 1);
	Mat src = imread(filename);
	imageSize = src.size();

	//计算左相机内参
	camLeft.FindCorners();
	camLeft.CalculateInter();

	//计算右相机内参
	camRight.FindCorners();
	camRight.CalculateInter();

	//将左、右相机内参传入立体标定程序中
	stereoCamCalibrate.intrinsicL = camLeft.intrinsic;
	stereoCamCalibrate.distCoeffsL = camLeft.distCoeffs;
	stereoCamCalibrate.allImageSubCornerL = camLeft.allImageSubCorner;

	stereoCamCalibrate.intrinsicR = camRight.intrinsic;
	stereoCamCalibrate.distCoeffsR = camRight.distCoeffs;
	stereoCamCalibrate.allImageSubCornerR = camRight.allImageSubCorner;

	stereoCamCalibrate.bgrSrc = camLeft.bgrSrc;
	stereoCamCalibrate.allImage3DCorner = camLeft.allImage3DCorner;

	//立体标定，计算R、T
	stereoCamCalibrate.CalculateExter();

	//为disparity中sgbm算法提供参数numDisparitise
	getNumDisparities(stereoCamCalibrate.rectifyImageL, stereoCamCalibrate.rectifyImageR);

	//保存参数到Yml文件，供disparity工程使用
	storeParamYML();

	system("pause");
	return 0;
}

void getNumDisparities(Mat rectifyImageL, Mat rectifyImageR) {
	//此函数只适用于双目相机平行的情况下

	vector<Point2f> cornersL, cornersR;

	if (true == findChessboardCorners(rectifyImageL, numCorner, cornersL, CALIB_CB_ADAPTIVE_THRESH)
		&& true == findChessboardCorners(rectifyImageR, numCorner, cornersR, CALIB_CB_ADAPTIVE_THRESH)) {

		numDisparities = (int(cornersL[0].x - cornersR[0].x + 0.5) / 16 + 4) * 16; //如果视差效果不好，可以调整倒数第二个数字
		//当disparity工程中的sgbm使用的是resize图像，则sgbm中的numDisparitise = numDisparitiseHalf
		numDisparitiesHalf = (int((cornersL[0].x - cornersR[0].x) / 2.0 + 0.5) / 16 + 4) * 16;
		cout << " numDisparitise " << numDisparities << endl;
		cout << " numDisparitiseHalf " << numDisparitiesHalf << endl;
	}
}

void storeParamYML() {

	cameraParamter = "..\\" + imagePlace + " " + cameraParamter.substr(3);
	FileStorage fs(cameraParamter, FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "intrinsicL" << camLeft.intrinsic;
		fs << "distCoeffsL" << camLeft.distCoeffs;
		fs << "intrinsicR" << camRight.intrinsic;
		fs << "distCoeffsR" << camRight.distCoeffs;
		fs << "R" << stereoCamCalibrate.R;
		fs << "T" << stereoCamCalibrate.T;
		fs << "numDisparities" << numDisparities;
		fs << "numDisparitiesHalf" << numDisparitiesHalf;

		fs.release();
	}
	else cout << " can't open file '' " << endl;
}