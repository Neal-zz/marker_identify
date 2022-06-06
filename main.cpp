#include "marker_identifier.h"

#include <opencv2/opencv.hpp>



void showResult(const cv::Mat& leftSrc, const cv::Mat& rightSrc, const std::vector<cv::Mat>& Tcam_marker) {
	/*camera parameters*/
	const cv::Mat cameraMatrixl = (cv::Mat_<float>(3, 3) <<
		1096.6, 0., 1004.6,
		0., 1101.2, 547.8316,
		0., 0., 1.);
	const cv::Mat cameraMatrixr = (cv::Mat_<float>(3, 3) <<
		1095.2, 0., 996.3653,
		0., 1100.7, 572.7941,
		0., 0., 1.);
	const cv::Mat distCoeffsl = (cv::Mat_<float>(1, 4) <<
		0.0757, -0.0860, -2.2134e-4, 2.4925e-4);
	const cv::Mat distCoeffsr = (cv::Mat_<float>(1, 4) <<
		0.0775, -0.0932, -9.2589e-4, 1.5443e-4);
	const cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		0.999991605633247, -0.000345422533180, 0.004082811079910, -4.049079591541234,
		0.000333318919503, 0.999995549306528, 0.002964838213577, -0.004234103893078,
		-0.004083817030495, -0.002963452447460, 0.999987270113001, -0.065436975906188);
	const cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1., 0., 0., 0.,
		0., 1., 0., 0.,
		0., 0., 1., 0.);

	cv::Mat proMl(3, 4, CV_32F), proMr(3, 4, CV_32F);
	proMl = cameraMatrixl * T1;
	proMr = cameraMatrixr * T2;
	cv::Mat leftOut, rightOut;
	cv::cvtColor(leftSrc, leftOut, cv::COLOR_GRAY2BGR);
	cv::cvtColor(rightSrc, rightOut, cv::COLOR_GRAY2BGR);
	for (int i = 0; i < Tcam_marker.size(); i++) {
		// 3D points
		float l0 = 5;  // length from point0 to pointx
		cv::Point3f point0(Tcam_marker[i].at<float>(0, 3), Tcam_marker[i].at<float>(1, 3), Tcam_marker[i].at<float>(2, 3));
		cv::Point3f pointx = point0 + l0 * cv::Point3f(Tcam_marker[i].at<float>(0, 0), Tcam_marker[i].at<float>(1, 0), Tcam_marker[i].at<float>(2, 0));
		cv::Point3f pointy = point0 + l0 * cv::Point3f(Tcam_marker[i].at<float>(0, 1), Tcam_marker[i].at<float>(1, 1), Tcam_marker[i].at<float>(2, 1));
		cv::Point3f pointz = point0 + l0 * cv::Point3f(Tcam_marker[i].at<float>(0, 2), Tcam_marker[i].at<float>(1, 2), Tcam_marker[i].at<float>(2, 2));

		// project to image plane
		cv::Mat point0_l(3, 1, CV_32F), point0_r(3, 1, CV_32F);
		point0_l = proMl * (cv::Mat_<float>(4, 1) << point0.x, point0.y, point0.z, 1.0);
		point0_r = proMr * (cv::Mat_<float>(4, 1) << point0.x, point0.y, point0.z, 1.0);
		cv::Point2f point0_l_2d(point0_l.at<float>(0, 0) / point0_l.at<float>(2, 0), point0_l.at<float>(1, 0) / point0_l.at<float>(2, 0));
		cv::Point2f point0_r_2d(point0_r.at<float>(0, 0) / point0_r.at<float>(2, 0), point0_r.at<float>(1, 0) / point0_r.at<float>(2, 0));
		cv::Mat pointx_l(3, 1, CV_32F), pointx_r(3, 1, CV_32F);
		pointx_l = proMl * (cv::Mat_<float>(4, 1) << pointx.x, pointx.y, pointx.z, 1.0);
		pointx_r = proMr * (cv::Mat_<float>(4, 1) << pointx.x, pointx.y, pointx.z, 1.0);
		cv::Point2f pointx_l_2d(pointx_l.at<float>(0, 0) / pointx_l.at<float>(2, 0), pointx_l.at<float>(1, 0) / pointx_l.at<float>(2, 0));
		cv::Point2f pointx_r_2d(pointx_r.at<float>(0, 0) / pointx_r.at<float>(2, 0), pointx_r.at<float>(1, 0) / pointx_r.at<float>(2, 0));
		cv::Mat pointy_l(3, 1, CV_32F), pointy_r(3, 1, CV_32F);
		pointy_l = proMl * (cv::Mat_<float>(4, 1) << pointy.x, pointy.y, pointy.z, 1.0);
		pointy_r = proMr * (cv::Mat_<float>(4, 1) << pointy.x, pointy.y, pointy.z, 1.0);
		cv::Point2f pointy_l_2d(pointy_l.at<float>(0, 0) / pointy_l.at<float>(2, 0), pointy_l.at<float>(1, 0) / pointy_l.at<float>(2, 0));
		cv::Point2f pointy_r_2d(pointy_r.at<float>(0, 0) / pointy_r.at<float>(2, 0), pointy_r.at<float>(1, 0) / pointy_r.at<float>(2, 0));
		cv::Mat pointz_l(3, 1, CV_32F), pointz_r(3, 1, CV_32F);
		pointz_l = proMl * (cv::Mat_<float>(4, 1) << pointz.x, pointz.y, pointz.z, 1.0);
		pointz_r = proMr * (cv::Mat_<float>(4, 1) << pointz.x, pointz.y, pointz.z, 1.0);
		cv::Point2f pointz_l_2d(pointz_l.at<float>(0, 0) / pointz_l.at<float>(2, 0), pointz_l.at<float>(1, 0) / pointz_l.at<float>(2, 0));
		cv::Point2f pointz_r_2d(pointz_r.at<float>(0, 0) / pointz_r.at<float>(2, 0), pointz_r.at<float>(1, 0) / pointz_r.at<float>(2, 0));

		cv::line(leftOut, point0_l_2d, pointx_l_2d, cv::Scalar(0, 0, 255), 2);
		cv::line(leftOut, point0_l_2d, pointy_l_2d, cv::Scalar(0, 255, 0), 2);
		cv::line(leftOut, point0_l_2d, pointz_l_2d, cv::Scalar(255, 0, 0), 2);
		cv::line(rightOut, point0_r_2d, pointx_r_2d, cv::Scalar(0, 0, 255), 2);
		cv::line(rightOut, point0_r_2d, pointy_r_2d, cv::Scalar(0, 255, 0), 2);
		cv::line(rightOut, point0_r_2d, pointz_r_2d, cv::Scalar(255, 0, 0), 2);
	}


	cv::imshow("left", leftOut);
	cv::imshow("right", rightOut);
	cv::waitKey(0);
}

int main() {
	cv::Mat leftSrc = cv::imread("test0.jpg",0);  // CV_8UC1
	cv::Mat rightSrc = cv::imread("test1.jpg", 0);  // CV_8UC1
	
	//cv::Mat image;
	//leftSrc = leftSrc;
	//cv::medianBlur(leftSrc, leftSrc, 5);
	//cv::boxFilter(leftSrc, image, -1, cv::Size(21, 21));
	//image = leftSrc - image;
	//float thresh = 0.04;
	//cv::Mat out;
	//out = image > thresh;
	//cv::imshow("ss", out);
	//cv::waitKey(0);

	std::vector<int> markerId;
	std::vector<cv::Mat> Tcam_marker;
	bool success = markerIdentify(leftSrc, rightSrc, markerId, Tcam_marker);

	if (success) {
		for (int i = 0; i < markerId.size(); i++) {
			std::cout << "current marker: " << markerId[i] << std::endl;
			std::cout << "Tcam_marker: " << Tcam_marker[i] << std::endl;
		}
	}
	else {
		std::cout << "fail..." << std::endl;
	}


	showResult(leftSrc, rightSrc, Tcam_marker);

	return 0;
}