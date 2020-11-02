#include "CaliCamInter.h"

int CCaliCamSingal::numCam = 0; /* ���������ʼ��Ϊ0 */

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

		//��ȡͼ��
		snprintf(filename, 100, imageAddress, i+1);
		bgrSrc = imread(filename);
		//ת��Ϊ�Ҷ�ͼ��һЩAPIֻ��ʹ��8bit�Ҷ�ͼ��ͬʱ�ٶ�Ҳ��
		cvtColor(bgrSrc, graySrc, COLOR_BGR2GRAY);

		//�Ƿ�ɹ���ȡ��ͼ��
		if (false == static_cast<bool>(graySrc.data)) cerr << " can't find image, check the images' path. " << endl;

		//����ǵ������������
		cout << " detecting corners of image: " << filename << " ... " << endl;
		if (true == findChessboardCorners(graySrc, sizeCorner, corners, CALIB_CB_ADAPTIVE_THRESH)) { 
			cornerSubPix(graySrc, corners, Size(5, 5), Size(-1, -1), 
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));
			//drawChessboardCorners(bgrSrc, sizeCorner, corners, true); //����δд�ж�����ͼ��ȷ��Ҳû����ͼ������ûɶ��
			allImageSubCorner.push_back(corners);
		}
		else cerr << " can't find all corners of " << i + 1 << "th. press any key to exit. " << endl;

	}
	cout << " successed to find all images' corners. " << endl;
}

void CCaliCamSingal::CalculateInter() {

	//�������нǵ���world����ϵ�µ�����
	vector<Point3f> subCornerTo3D; // 3D�����ؽǵ�
	for (int row = 0; row < sizeCorner.height; row++)
		for (int col = 0; col < sizeCorner.width; col++) {
			subCornerTo3D.push_back(Point3f(row * blockLength, col * blockLength, 0));
		}
	for (int num = 0; num < numImages; num++) { //����ͼ�Ľǵ�world���궼��ͬ
		allImage3DCorner.push_back(subCornerTo3D);
	}

	//�궨����ڲΡ�����ϵ����(cameraϵ->worldϵ)R��T
	calibrateCamera(allImage3DCorner, allImageSubCorner, imageSize,
		intrinsic, distCoeffs, R_Vector, T_Vector);
	cout << endl << " intrinsic paramater: " << endl << intrinsic << endl
		<< " undistort coefficient: " << endl << distCoeffs << endl << endl;
	
	//�������ͼƬ�� cameraϵ->worldϵ ��R��T
	//for (int numImage = 0; numImage < numImages; numImage++) {
	//	cout << " ---- " << numImage + 1 << "th images' cameral -> world coordinate: ---- " << endl
	//		<< " R = " << endl << R_Vector[numImage] << endl
	//		<< " T = " << endl << T_Vector[numImage] << endl << endl;
	//}

	//��������ͼ�����ͶӰ���: ÿ��ͼƬ�����нǵ����֮�͡�����ͼƬ����ƽ��ֵ
	cout << " ----- calculate images' calibrate average error... ----- " << endl;
	double meanErrOfAllImage = 0.0;
	for (int numImage = 0; numImage < numImages; numImage++) {
		vector<Point2f> projectCorner;
		//Mat rmatrix;
		//Rodrigues(R_Matrix[numImage], rmatrix); //?
		projectPoints(allImage3DCorner[numImage], R_Vector[numImage], T_Vector[numImage],
			intrinsic, distCoeffs, projectCorner); // �Կռ����ά���������ͶӰ���㣬�Ա궨�����������

		double sumErrPerImage = norm(projectCorner, allImageSubCorner[numImage], NORM_L2);
		meanErrOfAllImage += sumErrPerImage / double(numImages);
		cout << numImage + 1 << "th images' average projection pixel err: " << sumErrPerImage << endl;
		
		assert(sumErrPerImage > 3 && "Error: someone images' reprojection is greater than 3pixel ");

	}
	cout << "All images' average projection pixel err: " << meanErrOfAllImage << endl;

}
