#pragma once
#ifndef _CALICAMINTER_H
#define _CALICAMINTER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream>

using namespace std;
using namespace cv;

extern ofstream fout;
extern Size imageSize;

class CCaliCamSingal {
public:
	CCaliCamSingal(Size sizeCorner, int numImages, int blockLength, const char* imageAddress);
	~CCaliCamSingal();

	void FindCorners();
	void CalculateInter();
public:
	const char* imageAddress;

	static int numCam;

	int numImages;
	int blockLength;

	Size sizeCorner;

	Mat bgrSrc;
	Mat intrinsic; // �ڲξ���
	Mat distCoeffs; // 5������ϵ��

	vector<Mat> R_Vector, T_Vector; /* ÿ��ͼ�����ת��ƽ������ */
	vector<vector<Point2f>> allImageSubCorner;// ����ͼ��������ؽǵ�2D����
	vector<vector<Point3f>> allImage3DCorner; // ����ͼ���3D�ǵ����ͬһ��Vector�У��ͱ�allImageSubCorner����z����z=0

};

#endif // !_CALICAMINTER_H