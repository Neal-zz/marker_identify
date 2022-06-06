#include "marker_identifier.h"

void findSquares(const cv::Mat& image, std::vector<std::vector<cv::Point>>& squares)
{
	auto angle = [](const cv::Point& pt1, const cv::Point& pt2, const cv::Point& pt0)
	{
		double dx1 = static_cast<double>(pt1.x) - pt0.x;
		double dy1 = static_cast<double>(pt1.y) - pt0.y;
		double dx2 = static_cast<double>(pt2.x) - pt0.x;
		double dy2 = static_cast<double>(pt2.y) - pt0.y;
		return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
	};


	//int thresh = 100;
	int N = 1;  // 5;  // try 5 different thresholds.
	squares.clear();

	cv::Mat dst, gray_one, gray;
	gray_one = cv::Mat(image.size(), CV_8U);
	dst = image.clone();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// search the rectangle in each channel.
	for (int c = 0; c < image.channels(); c++)
	{
		int ch[] = { c, 0 };

		// convert to one channel image
		cv::mixChannels(&dst, 1, &gray_one, 1, ch, 1);

		// method 1
		// sobel edge detection
		cv::medianBlur(gray_one, gray_one, 5);
		cv::Mat img_x, img_y, img_sobel;
		cv::Sobel(gray_one, img_x, CV_16S, 1, 0); // be careful to use CV_16S here, maintaining useful information.
		cv::Sobel(gray_one, img_y, CV_16S, 0, 1);
		cv::convertScaleAbs(img_x, img_x); // absolute value.
		cv::convertScaleAbs(img_y, img_y);
		cv::addWeighted(img_x, 0.5, img_y, 0.5, 0.0, img_sobel);
		gray = img_sobel > 20;
		//
		//cv::Canny(gray, gray, 5, 100, 5);


		// method 2
		//cv::Mat temp;
		//cv::boxFilter(gray_one, temp, -1, cv::Size(21, 21));
		//temp = gray_one - temp;
		//float thresh = 10;  // 0.04;
		//gray = temp > thresh;
		//cv::dilate(gray, gray, cv::Mat(), cv::Point(-1, -1));  // while area grow.
		//cv::Canny(gray, gray, 5, 100, 5);

		// findContours(gray, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		cv::findContours(gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

		std::vector<cv::Point> approx;

		// verify contours
		for (size_t i = 0; i < contours.size(); i++)
		{
			// poly approximation
			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.05, true);

			// calculate contour area size, and then get 4 points
			if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx)))
			{
				double maxCosine = 0;

				for (int j = 2; j < 5; j++)
				{
					// find the max angle
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				if (maxCosine < 0.7)
				{
					// unique test.
					bool unique = true;
					for (int j = 0; j < squares.size(); j++) {
						std::vector<cv::Point> approx_j = squares[j];
						if ((fabs(cv::contourArea(cv::Mat(approx))) - fabs(cv::contourArea(cv::Mat(approx_j)))) < fabs(cv::contourArea(cv::Mat(approx))) * 0.001 &&
							(pow((approx[0].x + approx[1].x + approx[2].x + approx[3].x) - (approx_j[0].x + approx_j[1].x + approx_j[2].x + approx_j[3].x), 2) +
								pow((approx[0].y + approx[1].y + approx[2].y + approx[3].y) - (approx_j[0].y + approx_j[1].y + approx_j[2].y + approx_j[3].y), 2)) < fabs(cv::contourArea(cv::Mat(approx))) * 0.01)
							unique = false;
						break;
					}
					if (unique == 1) {
						squares.push_back(approx);
					}
				}
			}
		}
	}

	// show result
	//cv::Mat out;
	//cv::cvtColor(image, out, cv::COLOR_GRAY2BGR);
	//for (size_t i = 0; i < squares.size(); i++)
	//{
	//	const cv::Point* p = &squares[i][0];

	//	int n = (int)squares[i].size();
	//	if (p->x > 3 && p->y > 3)
	//	{
	//		cv::polylines(out, &p, &n, 1, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
	//	}
	//}
	//cv::imwrite("result.jpg", out);
}

bool find8Points(const cv::Mat& image, std::vector<std::vector<cv::Point2f>>& result, std::vector<float>& pR,
	std::vector<int>& minx, std::vector<int>& miny, std::vector<int>& maxx, std::vector<int>& maxy) {
	// pR: points radius
	// parameters:
	const float areaPortion = 2;
	std::vector < std::vector<cv::Point>> contoursRect;
	findSquares(image, contoursRect);

	// try to find 8 points in the rectangle. 
	for (const auto& cR : contoursRect) {
		std::vector<std::vector<cv::Point>> contoursUpdated2;
		cv::Mat roi;
		int temp_minx = std::min({ cR[0].x, cR[1].x, cR[2].x, cR[3].x });
		int temp_miny = std::min({ cR[0].y, cR[1].y, cR[2].y, cR[3].y });
		int temp_maxx = std::max({ cR[0].x, cR[1].x, cR[2].x, cR[3].x });
		int temp_maxy = std::max({ cR[0].y, cR[1].y, cR[2].y, cR[3].y });
		cv::Mat temp = image(cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx, temp_maxy - temp_miny));
		cv::medianBlur(temp, temp, 5);
		
		// method 1
		// sobel edge detection
		//cv::Mat img_x, img_y, img_sobel;
		//cv::Sobel(temp, img_x, CV_16S, 1, 0); // be careful to use CV_16S here, maintaining useful information.
		//cv::Sobel(temp, img_y, CV_16S, 0, 1);
		//cv::convertScaleAbs(img_x, img_x); // absolute value.
		//cv::convertScaleAbs(img_y, img_y);
		//cv::addWeighted(img_x, 0.5, img_y, 0.5, 0.0, img_sobel);
		//roi = img_sobel > 20;
		
		// method 2
		cv::Mat temp2;
		cv::boxFilter(temp, temp2, -1, cv::Size(21, 21));
		temp2 = temp - temp2;
		roi = temp2 > 10;  // 0.04;

		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::morphologyEx(roi, roi, cv::MORPH_OPEN, element);  // eliminate while noise in black region.
		cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, element);  // eliminate black noise in white region.
		cv::Canny(roi, roi, 5, 50.0 * 2, 5);

		std::vector<std::vector<cv::Point>> contoursInRect;
		cv::findContours(roi, contoursInRect, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

		// 1. overlap contour erasing.
		// if two contours' area is similar, and distance is small, then erase one of them.
		// 2. too little and too big contour erasing.
		// 3. verify poins are in the rect.
		std::vector<std::vector<cv::Point>> contoursUpdated0;
		for (int i = 0; i < contoursInRect.size(); i++) {
			auto getCross = [](const cv::Point& p1, const cv::Point& p2, const cv::Point& p) {
				return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
			};
			auto sign = [](const double x) {
				if (x > 0) {
					return 1;
				}
				else if (x < 0) {
					return -1;
				}
				else {
					return 0;
				}
			};

			if (contoursInRect[i].size() < 10) {  // too little points
				continue;
			}
			cv::RotatedRect boxi = cv::fitEllipse(contoursInRect[i]);
			double areai = cv::contourArea(contoursInRect[i]);
			if (areai < 0.001 * roi.cols * roi.rows || areai > 0.1 * roi.cols * roi.rows) {
				continue;
			}
			bool unique = 1;
			for (const auto& cUj : contoursUpdated0) {
				cv::RotatedRect boxj = cv::fitEllipse(cUj);
				if ((pow(boxi.center.x - boxj.center.x, 2) + pow(boxi.center.y - boxj.center.y, 2)) * CV_PI < areai &&
					abs(cv::contourArea(cUj) - areai) < areai * 0.4) {
					unique = 0;
					break;
				}
			}
			
			if (unique == 1 && sign(getCross(cR[0], cR[1], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) * sign(getCross(cR[2], cR[3], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) >= 0 &&
				sign(getCross(cR[1], cR[2], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) * sign(getCross(cR[3], cR[0], cv::Point(temp_minx + boxi.center.x, temp_miny + boxi.center.y))) >= 0) {
				contoursUpdated0.emplace_back(contoursInRect[i]);
			}
		}

		// 1. contour area voting.
		// 2. concentric area eliminate.
		int candidate = -1;
		for (int i = 0; i < contoursUpdated0.size(); i++) {
			double area_i = cv::contourArea(contoursUpdated0[i]);
			int pixel_i = contoursUpdated0[i].size();
			cv::RotatedRect box_i = cv::fitEllipse(contoursUpdated0[i]);
			
			int voteNum = 0;
			for (int j = 0; j < contoursUpdated0.size(); j++) {
				double area_j = cv::contourArea(contoursUpdated0[j]);
				int pixel_j = contoursUpdated0[j].size();
				cv::RotatedRect box_j = cv::fitEllipse(contoursUpdated0[j]);
				
				// concentric area. the smaller one is erased.
				if ((pow(box_i.center.x - box_j.center.x, 2) + pow(box_i.center.y - box_j.center.y, 2)) * CV_PI < area_i &&
					area_i < area_j) {
					voteNum = 0;
					break;
				}

				// vote
				if (area_j / area_i < areaPortion && area_i / area_j < areaPortion &&
					pixel_i / pixel_j < areaPortion && pixel_j / pixel_i < areaPortion) {
					voteNum++;
				}
			}

			if (voteNum >= 8) {
				candidate = i;
				break;
			}
		}
		if (candidate == -1) {
			continue;
		}

		// update contours1.
		std::vector<std::vector<cv::Point>> contoursUpdated1;
		float areaCan = cv::contourArea(contoursUpdated0[candidate]);
		int pixelCan = contoursUpdated0[candidate].size();
		for (int i = 0; i < contoursUpdated0.size(); i++) {
			float area_i = cv::contourArea(contoursUpdated0[i]);
			int pixel_i = contoursUpdated0[i].size();
			if (areaCan / area_i < areaPortion && area_i / areaCan < areaPortion &&
				pixel_i / pixelCan < areaPortion && pixelCan / pixel_i < areaPortion) {
				contoursUpdated1.emplace_back(contoursUpdated0.at(i));
			}
		}

		// distance voting. points which are too close to the edge are erased.
		candidate = -1;
		//float ppDistance = sqrt(areaCan / CV_PI) * 8;  // r*8
		float temp_pR = sqrt(areaCan / CV_PI);
		for (int i = 0; i < contoursUpdated1.size(); i++) {
			auto getCross = [](const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p) {
				return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
			};

			cv::RotatedRect boxi = cv::fitEllipse(contoursUpdated1[i]);
			bool validate = true;
			for (int j = 0; j < 4; j++) {  // point j (0-3)
				cv::Point2f corner0 = cv::Point2f(cR[(j + 1) % 4].x - temp_minx, cR[(j + 1) % 4].y - temp_miny);
				cv::Point2f corner1 = cv::Point2f(cR[(j) % 4].x - temp_minx, cR[(j) % 4].y - temp_miny);
				float dis = fabs(getCross(corner0, corner1, boxi.center)) / sqrt(pow(corner0.x - corner1.x, 2) + pow(corner0.y - corner1.y, 2));
				if (dis < temp_pR) {
					validate = false;
					break;
				}
			}
			if (validate == true) {
				contoursUpdated2.emplace_back(contoursUpdated1.at(i));
			}
		}

		// update output
		if (contoursUpdated2.size() == 8) {
			pR.emplace_back(temp_pR);
			minx.emplace_back(temp_minx);
			maxx.emplace_back(temp_maxx);
			miny.emplace_back(temp_miny);
			maxy.emplace_back(temp_maxy);
			roi = cv::Scalar::all(0);
			std::vector<cv::Point2f> temp_result;
			for (int i = 0; i < 8; i++)
			{
				cv::RotatedRect box = cv::fitEllipse(contoursUpdated2[i]);

				// draw the ellipse
				//cv::drawContours(out, contoursUpdated2, i, cv::Scalar::all(100));
				//cv::ellipse(roi, box, cv::Scalar::all(255));

				temp_result.emplace_back(cv::Point2f(box.center.x, box.center.y));
			}
			result.emplace_back(temp_result);
		}
	}

	if (result.size() == 0) {
		return false;
	}
	//cv::imshow("left Gaussian", image);
	//cv::imshow("left Binary", imgBin);
	//cv::imshow("left Edge", imgEdge);

	//cv::waitKey(0);

	return true;
}

PatternContainer distinguish8Points(const std::vector<cv::Point2f>& pointsIn, const float pR) {
	PatternContainer pointsOut;

	if (pointsIn.size() != 8) {
		std::cout << "distinguish8Points: input size wrong!" << std::endl;
		return pointsOut;
	}

	// find X coordinate.
	bool findX = 0;
	for (const auto& a : pointsIn) {
		for (const auto& b : pointsIn) {
			if (b == a) {
				continue;
			}
			for (const auto& c : pointsIn) {
				if (c == a || c == b) {
					continue;
				}

				// criteria1: {Vec_ab}=={Vec_bc}
				if (fabs(a.y + c.y - 2 * b.y) > pR || fabs(a.x + c.x - 2 * b.x) > pR) {
					continue;
				}

				// criteria2: the remaining points are to the left of the {Vec_ac}
				// criteria3: max a angle theta1 == min c angle theta2
				bool cri2 = 1;
				float theta_a = 0, theta_c = 4;
				for (const auto& d : pointsIn) {
					if (d == a || d == b || d == c) {
						continue;
					}
					if (((c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x)) >= 0) {  // on the right side
						cri2 = 0;
						break;
					}

					float temp_a, temp_c;
					temp_a = acos(((c.x - a.x) * (d.x - a.x) + (c.y - a.y) * (d.y - a.y)) /
						sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
						sqrt((d.x - a.x) * (d.x - a.x) + (d.y - a.y) * (d.y - a.y)));
					temp_c = acos(((c.x - a.x) * (d.x - c.x) + (c.y - a.y) * (d.y - c.y)) /
						sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
						sqrt((d.x - c.x) * (d.x - c.x) + (d.y - c.y) * (d.y - c.y)));
					if (temp_a > theta_a) {  // max a
						theta_a = temp_a;
					}
					if (temp_c < theta_c) {  // min c
						theta_c = temp_c;
					}
				}
				if (!cri2) {
					continue;
				}
				if (fabs(theta_a - theta_c) > 5.0 / 180.0 * CV_PI) {
					continue;
				}
				float theta = (theta_a + theta_c) / 2;

				// criteria4: only one point can be found, at c's theta direction.
				int cCount = 0;
				cv::Point2f p8;
				for (const auto& d : pointsIn) {
					if (d == a || d == b || d == c) {
						continue;
					}
					float temp_c = acos(((c.x - a.x) * (d.x - c.x) + (c.y - a.y) * (d.y - c.y)) /
						sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
						sqrt((d.x - c.x) * (d.x - c.x) + (d.y - c.y) * (d.y - c.y)));
					if (fabs(theta - temp_c) < (5.0 / 180.0 * CV_PI)) {
						cCount++;
						p8 = d;
					}
				}
				if (cCount != 1) {
					continue;
				}

				// criteria 5: distance between a and the first points at a's theta direction == 
				// distance between b and the first points at b's theta direction.
				float dis_a = pR * 100, dis_b = pR * 200;
				cv::Point2f p4, p5;
				for (const auto& d : pointsIn) {
					if (d == a || d == b || d == c) {
						continue;
					}
					float temp_a = acos(((c.x - a.x) * (d.x - a.x) + (c.y - a.y) * (d.y - a.y)) /
						sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
						sqrt((d.x - a.x) * (d.x - a.x) + (d.y - a.y) * (d.y - a.y)));
					float temp_b = acos(((c.x - a.x) * (d.x - b.x) + (c.y - a.y) * (d.y - b.y)) /
						sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
						sqrt((d.x - b.x) * (d.x - b.x) + (d.y - b.y) * (d.y - b.y)));
					if (fabs(theta - temp_a) < (5.0 / 180.0 * CV_PI)) {
						float temp_dis = sqrt((d.x - a.x) * (d.x - a.x) + (d.y - a.y) * (d.y - a.y));
						if (temp_dis < dis_a) {
							dis_a = temp_dis;
							p4 = d;
						}
					}
					if (fabs(theta - temp_b) < (5.0 / 180.0 * CV_PI)) {
						float temp_dis = sqrt((d.x - b.x) * (d.x - b.x) + (d.y - b.y) * (d.y - b.y));
						if (temp_dis < dis_b) {
							dis_b = temp_dis;
							p5 = d;
						}
					}
				}
				if (fabs(dis_a - dis_b) > pR) {
					continue;
				}

				// all criterias are met
				findX = 1;
				pointsOut.p1 = a;
				pointsOut.p2 = b;
				pointsOut.p3 = c;
				pointsOut.p4 = p4;
				pointsOut.p5 = p5;
				pointsOut.p8 = p8;
				std::vector<cv::Point2f> points2;
				for (const auto& d : pointsIn) {
					if (d == a || d == b || d == c ||
						norm(d - p4) < pR || norm(d - p5) < pR || norm(d - p8) < pR) {
						continue;
					}
					points2.emplace_back(d);  // only p6 and p7 are emplaced back.
				}
				float temp0 = acos(((c.x - a.x) * (points2[0].x - b.x) + (c.y - a.y) * (points2[0].y - b.y)) /
					sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
					sqrt((points2[0].x - b.x) * (points2[0].x - b.x) + (points2[0].y - b.y) * (points2[0].y - b.y)));
				float temp1 = acos(((c.x - a.x) * (points2[1].x - b.x) + (c.y - a.y) * (points2[1].y - b.y)) /
					sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y)) /
					sqrt((points2[1].x - b.x) * (points2[1].x - b.x) + (points2[1].y - b.y) * (points2[1].y - b.y)));
				if (temp0 < temp1) {
					pointsOut.p6 = points2[1];
					pointsOut.p7 = points2[0];
				}
				else {
					pointsOut.p6 = points2[0];
					pointsOut.p7 = points2[1];
				}
				return pointsOut;
			}
		}
	}

	// didn't find X coordinate
	std::cout << "distinguish8Points: can't find X coordinate..." << std::endl;
	return pointsOut;
}

std::vector<int> crossCheck(std::vector<PatternContainer>& leftPoints, std::vector<PatternContainer>& rightPoints) {
	// right pattern id order that matches left pattern id
	std::vector<int> rightMatchLeft(leftPoints.size(), -1);
	for (int i = 0; i < leftPoints.size(); i++) {
		int temp_leftid = leftPoints[i].getId();
		for (int j = 0; j < rightPoints.size(); j++) {
			int temp_rightid = rightPoints[j].getId();
			if (temp_rightid == temp_leftid) {
				rightMatchLeft[i] = j;
			}
		}
	}

	//if (fabs(leftPR - rightPR) > leftPR * 0.1) {
	//	std::cout << "croosCheck fail: condition0..." << std::endl;
	//	return false;
	//}
	//float PR = leftPR * 0.5 + rightPR * 0.5;

	//if (fabs(leftPoints.p1.y - rightPoints.p1.y) > PR ||
	//	fabs(leftPoints.p2.y - rightPoints.p2.y) > PR || 
	//	fabs(leftPoints.p3.y - rightPoints.p3.y) > PR || 
	//	fabs(leftPoints.p4.y - rightPoints.p4.y) > PR || 
	//	fabs(leftPoints.p5.y - rightPoints.p5.y) > PR || 
	//	fabs(leftPoints.p6.y - rightPoints.p6.y) > PR || 
	//	fabs(leftPoints.p7.y - rightPoints.p7.y) > PR || 
	//	fabs(leftPoints.p8.y - rightPoints.p8.y) > PR) {
	//	std::cout << "croosCheck fail: condition1..." << std::endl;
	//	return false;
	//}

	return rightMatchLeft;
}

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
	//proMl = cameraMatrixl * T1;
	//proMr = cameraMatrixr * T2;
	proMl = T1;
	proMr = T2;
	cv::Mat pts4D(4, 3, CV_64F);
	cv::Mat camlpnts(1, 3, CV_64FC2);
	cv::Mat camrpnts(1, 3, CV_64FC2);
	camlpnts.at<cv::Vec2d>(0, 0) = cv::Vec2d(lPts_ud[0].x, lPts_ud[0].y);
	camlpnts.at<cv::Vec2d>(0, 1) = cv::Vec2d(lPts_ud[1].x, lPts_ud[1].y);
	camlpnts.at<cv::Vec2d>(0, 2) = cv::Vec2d(lPts_ud[2].x, lPts_ud[2].y);
	camrpnts.at<cv::Vec2d>(0, 0) = cv::Vec2d(rPts_ud[0].x, rPts_ud[0].y);
	camrpnts.at<cv::Vec2d>(0, 1) = cv::Vec2d(rPts_ud[1].x, rPts_ud[1].y);
	camrpnts.at<cv::Vec2d>(0, 2) = cv::Vec2d(rPts_ud[2].x, rPts_ud[2].y);
	// calculate 3D position.
	cv::triangulatePoints(proMl, proMr, camlpnts, camrpnts, pts4D);

	//pts4D.at<cv::Vec4d>(0, 1)[0];
	for (int i = 0; i < 3; i++)
	{
		cv::Point3f pointTmp(pts4D.at<double>(0, i) / pts4D.at<double>(3, i),
			pts4D.at<double>(1, i) / pts4D.at<double>(3, i), pts4D.at<double>(2, i) / pts4D.at<double>(3, i));
		pts3D.push_back(pointTmp);
	}

	return pts3D;
}

bool markerIdentify(const cv::Mat& leftSrc, const cv::Mat& rightSrc, std::vector<int>& markerId, std::vector<cv::Mat>& Tcam_marker) {
	/*camera parameters*/
	bool markerIdentify_success = true;

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


	/* find 8 points.*/
	std::vector<std::vector<cv::Point2f>> leftPoints, rightPoints;
	std::vector<float> leftPR, rightPR;
	cv::Mat leftCopy, rightCopy;
	leftCopy = leftSrc.clone();
	rightCopy = rightSrc.clone();
	std::vector<int> roi_l_minx, roi_l_miny, roi_l_maxx, roi_l_maxy;
	std::vector<int> roi_r_minx, roi_r_miny, roi_r_maxx, roi_r_maxy;
	bool leftSuccess = find8Points(leftCopy, leftPoints, leftPR, roi_l_minx, roi_l_miny, roi_l_maxx, roi_l_maxy);
	bool rightSuccess = find8Points(rightCopy, rightPoints, rightPR, roi_r_minx, roi_r_miny, roi_r_maxx, roi_r_maxy);
	if (!leftSuccess || !rightSuccess) {
		std::cout << "can't find 8 points!" << std::endl;
		std::cout << "left: " << std::to_string(leftSuccess) << "; right: " << std::to_string(rightSuccess) << std::endl;
		markerIdentify_success = false;
		return markerIdentify_success;
	}

	/* distinguish 8 points.*/
	std::vector<PatternContainer> leftPC, rightPC;
	for (int i = 0; i < leftPoints.size(); i++) {
		PatternContainer temp_leftPC = distinguish8Points(leftPoints[i], leftPR[i]);
		leftPC.emplace_back(temp_leftPC);
	}
	for (int i = 0; i < rightPoints.size(); i++) {
		PatternContainer temp_rightPC = distinguish8Points(rightPoints[i], rightPR[i]);
		rightPC.emplace_back(temp_rightPC);
	}

	/* check and match.*/
	std::vector<int> rightMatchLeft;
	rightMatchLeft = crossCheck(leftPC, rightPC);
	markerIdentify_success = false;
	for (int i = 0; i < rightMatchLeft.size(); i++) {
		if (rightMatchLeft[i] != -1) {
			markerIdentify_success = true;
			break;
		}
	}
	if (markerIdentify_success == false) {
		std::cout << "crossCheck fail..." << std::endl;
		return false;
	}

	/* calculate T*/
	for (int i = 0; i < rightMatchLeft.size(); i++) {
		int j = rightMatchLeft[i];
		if (j == -1) {
			continue;
		}

		std::vector<cv::Point2f> lPts, rPts;
		lPts.emplace_back(leftPC[i].p1 + cv::Point2f(roi_l_minx[i], roi_l_miny[i]));
		lPts.emplace_back(leftPC[i].p3 + cv::Point2f(roi_l_minx[i], roi_l_miny[i]));
		lPts.emplace_back(leftPC[i].p4 + cv::Point2f(roi_l_minx[i], roi_l_miny[i]));
		rPts.emplace_back(rightPC[j].p1 + cv::Point2f(roi_r_minx[j], roi_r_miny[j]));
		rPts.emplace_back(rightPC[j].p3 + cv::Point2f(roi_r_minx[j], roi_r_miny[j]));
		rPts.emplace_back(rightPC[j].p4 + cv::Point2f(roi_r_minx[j], roi_r_miny[j]));
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
		cv::Mat temp_Tcam_marker = (cv::Mat_<float>(4, 4) <<
			xCoor.x, yCoor.x, zCoor.x, pts3D[0].x,
			xCoor.y, yCoor.y, zCoor.y, pts3D[0].y,
			xCoor.z, yCoor.z, zCoor.z, pts3D[0].z,
			0.0, 0.0, 0.0, 1.0);

		// update results.
		markerId.emplace_back(leftPC[i].getId());
		Tcam_marker.emplace_back(temp_Tcam_marker);

	}


	return markerIdentify_success;
}