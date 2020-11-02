#include "CaliCamInter.h"

int CCaliCamSingal::numCam = 0; /* 相机个数初始化为0 */

CCaliCamSingal::CCaliCamSingal(Size sizeCorner, int numImage,
	int blockLength, const char* imageAddress)
	: sizeCorner(sizeCorner), numImages(numImage),
		blockLength(blockLength), imageAddress(imageAddress) {

}

CCaliCamSingal::~CCaliCamSingal() {

}

void CCaliCamSingal::FindCorners() {
	
	char filename[100];
	vector<Point2f> corners;
	Mat graySrc;

	numCam++; // calculate the num of camera
	cout << endl << endl << " /**** calibrate " << numCam << "th cameras' intrinsic parameter. ****/ " << endl;
	
	for (int i = 0; i < numImages; i++) {

		//读取图像
		snprintf(filename, 100, imageAddress, i+1);
		bgrSrc = imread(filename);
		//转换为灰度图，一些API只能使用8bit灰度图，同时速度也快
		cvtColor(bgrSrc, graySrc, COLOR_BGR2GRAY);

		//是否成功读取到图像
		if (false == static_cast<bool>(graySrc.data)) cerr << " can't find image, check the images' path. " << endl;

		//计算角点的亚像素坐标
		cout << " detecting corners of image: " << filename << " ... " << endl;
		if (true == findChessboardCorners(graySrc, sizeCorner, corners, CALIB_CB_ADAPTIVE_THRESH)) { 
			cornerSubPix(graySrc, corners, Size(5, 5), Size(-1, -1), 
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));
			//drawChessboardCorners(bgrSrc, sizeCorner, corners, true); //程序未写判定所画图正确性也没保存图像，所以没啥用
			allImageSubCorner.push_back(corners);
		}
		else cerr << " can't find all corners of " << i + 1 << "th. press any key to exit. " << endl;

	}
	cout << " successed to find all images' corners. " << endl;
}

void CCaliCamSingal::CalculateInter() {

	//计算所有角点在world坐标系下的坐标
	vector<Point3f> subCornerTo3D; // 3D亚像素角点
	for (int row = 0; row < sizeCorner.height; row++)
		for (int col = 0; col < sizeCorner.width; col++) {
			subCornerTo3D.push_back(Point3f(row * blockLength, col * blockLength, 0));
		}
	for (int num = 0; num < numImages; num++) { //所有图的角点world坐标都相同
		allImage3DCorner.push_back(subCornerTo3D);
	}

	//标定相机内参、畸变系数、(camera系->world系)R、T
	calibrateCamera(allImage3DCorner, allImageSubCorner, imageSize,
		intrinsic, distCoeffs, R_Vector, T_Vector);
	cout << endl << " intrinsic paramater: " << endl << intrinsic << endl
		<< " undistort coefficient: " << endl << distCoeffs << endl << endl;
	
	//输出所有图片的 camera系->world系 的R、T
	//for (int numImage = 0; numImage < numImages; numImage++) {
	//	cout << " ---- " << numImage + 1 << "th images' cameral -> world coordinate: ---- " << endl
	//		<< " R = " << endl << R_Vector[numImage] << endl
	//		<< " T = " << endl << T_Vector[numImage] << endl << endl;
	//}

	//计算所有图像的重投影误差: 每张图片上所有角点误差之和、所有图片误差的平均值
	cout << " ----- calculate images' calibrate average error... ----- " << endl;
	double meanErrOfAllImage = 0.0;
	for (int numImage = 0; numImage < numImages; numImage++) {
		vector<Point2f> projectCorner;
		//Mat rmatrix;
		//Rodrigues(R_Matrix[numImage], rmatrix); //?
		projectPoints(allImage3DCorner[numImage], R_Vector[numImage], T_Vector[numImage],
			intrinsic, distCoeffs, projectCorner); // 对空间的三维点进行重新投影计算，对标定结果进行评价

		double sumErrPerImage = norm(projectCorner, allImageSubCorner[numImage], NORM_L2);
		meanErrOfAllImage += sumErrPerImage / double(numImages);
		cout << numImage + 1 << "th images' average projection pixel err: " << sumErrPerImage << endl;
		
		assert(sumErrPerImage > 3 && "Error: someone images' reprojection is greater than 3pixel ");

	}
	cout << "All images' average projection pixel err: " << meanErrOfAllImage << endl;

}
