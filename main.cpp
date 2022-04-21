#include "marker_identifier.h"

#include <opencv2/opencv.hpp>

const cv::Mat cameraMatrixl = (cv::Mat_<float>(3, 3) <<
	3.6893312873969357e+03, 0., 1.2276919951156651e+03,
	0.,	3.7213916883471502e+03, 1.0259843160683670e+03,
	0., 0., 1.);
const cv::Mat cameraMatrixr = (cv::Mat_<float>(3, 3) <<
	3.6893312873969357e+03, 0., 1.2146499586267064e+03,
	0.,	3.7213916883471502e+03, 1.0229887294201624e+03,
	0., 0., 1.);
const cv::Mat distCoeffsl = (cv::Mat_<float>(1, 14) <<
	2.6414536323521552e-02, -7.6172971269805689e-01, 0., 0., 0.,
	0., 0., -4.8543545578704244e+00, 0., 0., 0., 0., 0., 0.);
const cv::Mat distCoeffsr = (cv::Mat_<float>(1, 14) <<
	1.5009905154834927e-01, -4.3632973373348820e+00, 0., 0., 0.,
	0., 0., -3.1428340823877907e+01, 0., 0., 0., 0., 0., 0.);
const cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
	7.9686560163007181e-01, -4.9783956901361809e-03, -6.0413610098643855e-01, 1.8392840810519419e+02,
	3.2219028565208920e-04, 9.9996940822133040e-01, -7.8152936542482329e-03, -2.1824514289170679e-01,
	6.0415652701279621e-01, 6.0330918967586413e-03, 7.9684282808468676e-01, 6.4846659173011275e+01);

std::vector<cv::Point3f> uv2xyz(const std::vector<cv::Point2f>& lPts, const std::vector<cv::Point2f>& rPts,
	const cv::Mat& cameraMatrixl, const cv::Mat& distCoeffsl,
	const cv::Mat& cameraMatrixr, const cv::Mat& distCoeffsr, const cv::Mat& T2) {

	std::vector<cv::Point3f> pts3D;

	const cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 1., 0.);

	// undistortion
	std::vector<cv::Point2f> lPts_ud, rPts_ud;
	cv::undistortPoints(lPts, lPts_ud, cameraMatrixl, distCoeffsl);
	cv::undistortPoints(rPts, rPts_ud, cameraMatrixr, distCoeffsr);
	//lPts_ud = lPts;
	//rPts_ud = rPts;

	// projection matrix: from world (left camera) coordinate, to image coordinate.
	cv::Mat proMl(3, 4, CV_32F), proMr(3, 4, CV_32F);
	proMl = cameraMatrixl * T1;
	proMr = cameraMatrixr * T2;
	cv::Mat pts4D;
	// calculate 3D position.
	cv::triangulatePoints(proMl, proMr, lPts_ud, rPts_ud, pts4D);

	for (int i = 0; i < pts4D.cols; i++)
	{
		cv::Point3f pointTmp(pts4D.at<float>(0, i) / pts4D.at<float>(3, i),
			pts4D.at<float>(1, i) / pts4D.at<float>(3, i), pts4D.at<float>(2, i) / pts4D.at<float>(3, i));
		pts3D.push_back(pointTmp);
	}

	return pts3D;
}

int main() {
	cv::Mat leftSrc = cv::imread("left.png",0);  // CV_8UC1
	cv::Mat rightSrc = cv::imread("right.png", 0);  // CV_8UC1
	
	std::vector<cv::Point2f> leftPoints, rightPoints;
	float leftPR, rightPR;
	bool leftSuccess = find8Points(leftSrc, leftPoints, leftPR);
	bool rightSuccess = find8Points(rightSrc, rightPoints, rightPR);
	if (!leftSuccess || !rightSuccess || fabs(leftPR - rightPR) > leftPR * 0.1) {
		std::cout << "can't find 8 points!" << std::endl;
		return 0;
	}

	PatternContainer leftPC = distinguish8Points(leftPoints, leftPR);
	PatternContainer rightPC = distinguish8Points(rightPoints, rightPR);

	bool success = crossCheck(leftPC, rightPC, leftPR, rightPR);
	if (!success || leftPC.getId()!=rightPC.getId()) {
		std::cout << "8 points found wrong!" << std::endl;
		return 0;
	}
	std::cout << "current marker: " << leftPC.getId() << std::endl;
	//std::cout << pc.p1 << std::endl;
	//std::cout << pc.p2 << std::endl;
	//std::cout << pc.p3 << std::endl;
	//std::cout << pc.p4 << std::endl;
	//std::cout << pc.p5 << std::endl;
	//std::cout << pc.p6 << std::endl;
	//std::cout << pc.p7 << std::endl;
	//std::cout << pc.p8 << std::endl;

	std::vector<cv::Point2f> lPts, rPts;
	lPts.emplace_back(leftPC.p1);
	lPts.emplace_back(leftPC.p3);
	lPts.emplace_back(leftPC.p4);
	rPts.emplace_back(rightPC.p1);
	rPts.emplace_back(rightPC.p3);
	rPts.emplace_back(rightPC.p4);
	std::vector<cv::Point3f> pts3D = uv2xyz(lPts, rPts, cameraMatrixl, distCoeffsl, cameraMatrixr, distCoeffsr, T2);
	
	//std::cout << pts3D[0] << std::endl;
	//std::cout << pts3D[1] << std::endl;
	//std::cout << pts3D[2] << std::endl;

	// calculate Tcam_marker.
	cv::Point3f xCoor = (pts3D[1] - pts3D[0]) / norm(pts3D[1] - pts3D[0]);
	cv::Point3f yCoor = (pts3D[2] - pts3D[0]) / norm(pts3D[2] - pts3D[0]);
	cv::Point3f zCoor = xCoor.cross(yCoor);
	zCoor /= norm(zCoor);
	yCoor = zCoor.cross(xCoor);
	yCoor /= norm(yCoor);
	cv::Mat Tcam_marker = (cv::Mat_<float>(4, 4) <<
		xCoor.x, yCoor.x, zCoor.x, pts3D[0].x,
		xCoor.y, yCoor.y, zCoor.y, pts3D[0].y,
		xCoor.z, yCoor.z, zCoor.z, pts3D[0].z,
		0.0, 0.0, 0.0, 1.0);

	return 0;
}