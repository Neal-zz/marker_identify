#include "marker_identifier.h"

#include <opencv2/opencv.hpp>



int main() {
	cv::Mat leftSrc = cv::imread("../img/id123.png",0);  // CV_8UC1
	
	std::vector<cv::Point2f> points;
	float pR;
	bool success = find8Points(leftSrc, points, pR);

	//if (!success) {
	//	std::cout << "can't find 8 points!" << std::endl;
	//	return 0;
	//}

	PatternContainer pc = distinguish8Points(points, pR);

	//std::cout << pc.p1 << std::endl;
	//std::cout << pc.p2 << std::endl;
	//std::cout << pc.p3 << std::endl;
	//std::cout << pc.p4 << std::endl;
	//std::cout << pc.p5 << std::endl;
	//std::cout << pc.p6 << std::endl;
	//std::cout << pc.p7 << std::endl;
	//std::cout << pc.p8 << std::endl;

	std::cout << "current marker: " << pc.getId() << std::endl;

	return 0;
}