#pragma once
#ifndef _CALIVAMEXTER_H
#define _CALIVAMEXTER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "CaliCamInter.h"

using namespace std;
using namespace cv;

extern const char* imagePlaceL;
extern const char* imagePlaceR;
extern Size imageSize;


class CCaliCamExter {
public:
	CCaliCamExter();
	~CCaliCamExter();

	void CalculateExter();
	//void ValidCalibrate();

public:
	Mat R, T, E, F;
	Mat RL, Rr, PL, Pr, Q;
	Mat bgrSrc;
	Mat intrinsicL, intrinsicR; // �ڲξ���
	Mat distCoeffsL, distCoeffsR; // 5������ϵ��
	Mat mapLx, mapLy, mapRx, mapRy;
	Mat rectifyImageL, rectifyImageR;

	Rect validPixRoiL, validPixRoiR;

	vector<vector<Point2f>> allImageSubCornerL, allImageSubCornerR;
	vector<vector<Point3f>> allImage3DCorner; // ����ͼ���3D�ǵ����ͬһ��Vector��

};

void getNumDisparities();

#endif // !_CALIVAMEXTER_H

