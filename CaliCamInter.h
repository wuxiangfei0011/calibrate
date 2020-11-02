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
	Mat intrinsic; // 内参矩阵
	Mat distCoeffs; // 5个畸变系数

	vector<Mat> R_Vector, T_Vector; /* 每幅图像的旋转、平移向量 */
	vector<vector<Point2f>> allImageSubCorner;// 所有图像的亚像素角点2D坐标
	vector<vector<Point3f>> allImage3DCorner; // 所有图像的3D角点放在同一个Vector中，就比allImageSubCorner多轴z，且z=0

};

#endif // !_CALICAMINTER_H