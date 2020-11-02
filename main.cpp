/*
TODO��
	У��������ͼ��ÿ�Խǵ��� < 3pixel
	disparity����Ҳ�������湦�ܣ����ڷǱ궨��ĳ�����
	�ҳ�Զ����궨���˵�ԭ���������ҳ��ǵ㣬����Ӧ��û���⣩
	16-10Ϊ��tif��ʽ���ܱ궨������Щpng�ܡ�tifȴ����
	��ÿһ�ű궨���all corner pixel err�趨һ�����ֵ
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

/******************************��Ҫ�Ĳ����Ĳ���******************************/
int blockLenght = 150; //mm ש����
Size numCorner = Size(4, 5);
//int blockLenght = 200; 
//Size numCorner = Size(4,3); 
//int blockLenght = 100; 
//Size numCorner = Size(7, 8);

string imagePlace = "4-15"; // R��Lͼ���ļ��У�10-20��ͼ�����
int numImage = 15; //ÿ�����ʹ�õı궨ͼ�����
const char* imagePlaceL = "..\\..\\4-15\\L\\%d.png"; //�����ͼ���ļ���
const char* imagePlaceR = "..\\..\\4-15\\R\\%d.png"; //�����ͼ���ļ���
/****************************************************************************/

CCaliCamSingal camLeft(numCorner, numImage, blockLenght, imagePlaceL);
CCaliCamSingal camRight(numCorner, numImage, blockLenght, imagePlaceR);
CCaliCamExter stereoCamCalibrate;

void getNumDisparities(Mat rectifyImageL, Mat rectifyImageR);
void storeParamYML();

int main(int argc, char** argv) {

	//��ȡͼ��ߴ�
	snprintf(filename, 100, imagePlaceL, 1);
	Mat src = imread(filename);
	imageSize = src.size();

	//����������ڲ�
	camLeft.FindCorners();
	camLeft.CalculateInter();

	//����������ڲ�
	camRight.FindCorners();
	camRight.CalculateInter();

	//����������ڲδ�������궨������
	stereoCamCalibrate.intrinsicL = camLeft.intrinsic;
	stereoCamCalibrate.distCoeffsL = camLeft.distCoeffs;
	stereoCamCalibrate.allImageSubCornerL = camLeft.allImageSubCorner;

	stereoCamCalibrate.intrinsicR = camRight.intrinsic;
	stereoCamCalibrate.distCoeffsR = camRight.distCoeffs;
	stereoCamCalibrate.allImageSubCornerR = camRight.allImageSubCorner;

	stereoCamCalibrate.bgrSrc = camLeft.bgrSrc;
	stereoCamCalibrate.allImage3DCorner = camLeft.allImage3DCorner;

	//����궨������R��T
	stereoCamCalibrate.CalculateExter();

	//Ϊdisparity��sgbm�㷨�ṩ����numDisparitise
	getNumDisparities(stereoCamCalibrate.rectifyImageL, stereoCamCalibrate.rectifyImageR);

	//���������Yml�ļ�����disparity����ʹ��
	storeParamYML();

	system("pause");
	return 0;
}

void getNumDisparities(Mat rectifyImageL, Mat rectifyImageR) {
	//�˺���ֻ������˫Ŀ���ƽ�е������

	vector<Point2f> cornersL, cornersR;

	if (true == findChessboardCorners(rectifyImageL, numCorner, cornersL, CALIB_CB_ADAPTIVE_THRESH)
		&& true == findChessboardCorners(rectifyImageR, numCorner, cornersR, CALIB_CB_ADAPTIVE_THRESH)) {

		numDisparities = (int(cornersL[0].x - cornersR[0].x + 0.5) / 16 + 4) * 16; //����Ӳ�Ч�����ã����Ե��������ڶ�������
		//��disparity�����е�sgbmʹ�õ���resizeͼ����sgbm�е�numDisparitise = numDisparitiseHalf
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