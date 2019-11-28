// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"
#include "opencv2/opencv_modules.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>
#include <cstdint>

#include "GRANSAC.hpp"
#include "CircleModel.hpp"

// GRANSAC::VPFloat Slope(int x0, int y0, int x1, int y1)
// {
// 	return (GRANSAC::VPFloat)(y1 - y0) / (x1 - x0);
// }
// 
// void DrawFullLine(cv::Mat& img, cv::Point a, cv::Point b, cv::Scalar color, int LineWidth)
// {
// 	GRANSAC::VPFloat slope = Slope(a.x, a.y, b.x, b.y);
// 
// 	cv::Point p(0, 0), q(img.cols, img.rows);
// 
// 	p.y = -(a.x - p.x) * slope + a.y;
// 	q.y = -(b.x - q.x) * slope + b.y;
// 
// 	cv::line(img, p, q, color, LineWidth, cv::LINE_AA, 0);
//}

int my_RANSAC(cv::Mat img)
{
	std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
	int range{ img.cols * img.rows };

	for (int idx = 0; idx < range; ++idx)
	{
		if (img.data[idx] > 0) 
		{
			cv::Point Pt{ idx % img.cols, idx / img.cols }; 
			std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(Pt.x, Pt.y);
			CandPoints.push_back(CandPt);

		}
	}


	GRANSAC::RANSAC<Circle2DModel, 3> Estimator;
	Estimator.Initialize(7, 500); // Threshold, iterations
	int64_t start = cv::getTickCount();
	Estimator.Estimate(CandPoints);
	int64_t end = cv::getTickCount();
	std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;
	
	
	cvtColor(img, img, CV_GRAY2BGR);
	
	auto BestInliers = Estimator.GetBestInliers();
	if (BestInliers.size() > 0)
	{
		for (auto& Inlier : BestInliers)
		{
			auto RPt = std::dynamic_pointer_cast<Point2D>(Inlier);
			cv::Point Pt(floor(RPt->m_Point2D[0]), floor(RPt->m_Point2D[1]));
			cv::circle(img, Pt, floor(img.cols / 100), cv::Scalar(255, 100, 100), -1);
		}
	}

	auto BestLine = Estimator.GetBestModel();
	if (BestLine)
	{
		auto BestLinePt1 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[0]);
		auto BestLinePt2 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[1]);
		auto BestLinePt3 = std::dynamic_pointer_cast<Point2D>(BestLine->GetModelParams()[2]);
		if (BestLinePt1 && BestLinePt2 && BestLinePt3)
		{
			cv::Point Pt1(BestLinePt1->m_Point2D[0], BestLinePt1->m_Point2D[1]);
			cv::Point Pt2(BestLinePt2->m_Point2D[0], BestLinePt2->m_Point2D[1]);
			cv::Point Pt3(BestLinePt3->m_Point2D[0], BestLinePt3->m_Point2D[1]);
		}
	}

	while (true)
	{
		
		cv::imshow("RANSAC Example", img);

		char Key = cv::waitKey(1);
		if (Key == 27)
			return 0;
		if (Key == ' ')
			cv::imwrite("LineFitting.png", img);
	}

	return 0;
}
