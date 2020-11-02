#include "CaliCamExter.h"

CCaliCamExter::CCaliCamExter() {


}

CCaliCamExter::~CCaliCamExter() {

}

void CCaliCamExter::CalculateExter() {

	//读取出图像
	char filenameR[100], filenameL[100];
	snprintf(filenameL, 100, imagePlaceL, 1);
	snprintf(filenameR, 100, imagePlaceR, 1); //任何图片计算出来的R\T都相等
	Mat bgrSrcL = imread(filenameL), bgrSrcR = imread(filenameR);

	cout << endl << endl << " /**** Begin to calculate stereo cameras' R and T . ****/ " << endl;

	double reProjectErr = stereoCalibrate(allImage3DCorner,
		allImageSubCornerL, allImageSubCornerR,
		intrinsicL, distCoeffsL,
		intrinsicR, distCoeffsR,
		imageSize, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-6));
	cout << " recompulate re-projection error with stereoCalibrate: " << reProjectErr << endl << endl
		<< " camera -> camera: " << endl
		<< " R = " << endl << R << endl
		<< " T = " << endl << T << endl;

	stereoRectify(intrinsicL, distCoeffsL, intrinsicR, distCoeffsR, imageSize,
		R, T, RL, Rr, PL, Pr, Q, 
		CALIB_ZERO_DISPARITY, -1, imageSize, &validPixRoiL, &validPixRoiR);

	initUndistortRectifyMap(intrinsicL, distCoeffsL, RL, PL, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(intrinsicR, distCoeffsR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
	
	remap(bgrSrcL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(bgrSrcR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	imwrite("..\\rectifyImageL(calibrate).png", rectifyImageL);
	imwrite("..\\rectifyImageR(calibrate).png", rectifyImageR);

	/***************合并左右图并绘制水平线，高乘1/2，宽不变********************************/
	Mat canvas;
	double zoom = 0.5;
	int w, h;
	
	w = cvRound(imageSize.width * zoom);
	h = cvRound(imageSize.height * zoom);
	canvas.create(h, w * 2, CV_8UC3);

	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                               
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);   
	Rect vroiL(cvRound(validPixRoiL.x * zoom), cvRound(validPixRoiL.y * zoom),
		cvRound(validPixRoiL.width * zoom), cvRound(validPixRoiL.height * zoom));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);              

	canvasPart = canvas(Rect(w, 0, w, h));                              
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validPixRoiR.x * zoom), cvRound(validPixRoiR.y * zoom),
		cvRound(validPixRoiR.width * zoom), cvRound(validPixRoiR.height * zoom));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	/************************************************************************************/

	imwrite("..\\lineVerify(calibrate).png", canvas);
	cout << endl << " The horizontal-validation image have been stored in folder " << endl << endl;
}