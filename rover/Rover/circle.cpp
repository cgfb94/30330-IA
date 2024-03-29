#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;

int circle(const char* source)
{
	Mat src, src_gray, dst;
	/// Read the image
	src = imread(source, 1);
	resize(src, src, Size(), 0.4, 0.4);


	if (!src.data)
	{
		return -1;
	}

	vector<Vec3f> circles;
	int kernel_size = 5;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	/// Apply the Hough Transform to find the circles
	Mat abs_dst, src2;


	/// Reduce the noise so we avoid false circle detection
	for (int i(0); i < 1; ++i) 
	{
		medianBlur(src, src, 7);
		blur(src, src, Size(3, 3));
	}
	/// Convert it to gray
	//blur(src, src, Size(5,5));
	cvtColor(src, src_gray, CV_BGR2GRAY);
	
	threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	
	//threshold()
	for (int i(0); i < 50; ++i) medianBlur(src_gray, src_gray, 7);
	//for (int i(0); i < 5; ++i) blur(src_gray, src_gray, Size(3, 3));
	
	//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
	Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	//Canny(src_gray, abs_dst, 20, 20*5);

	convertScaleAbs(dst, abs_dst);
	//threshold(abs_dst, abs_dst, 100, 255, THRESH_TOZERO | THRESH_OTSU);
	
	threshold(abs_dst, abs_dst, 40, 255, THRESH_BINARY | THRESH_OTSU);


	
	
	//for (int i(0); i < 2; ++i) medianBlur(abs_dst, abs_dst, 5);

	//distanceTransform
	
	HoughCircles(abs_dst, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 150, 15, 0, 0);
	//HoughCircles();
	

	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	/// Show your results
	namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	imshow("Hough Circle Transform Demo", src);
	imshow("Processed Image", abs_dst);

	// distance transform

	waitKey(0);
	return 0;
}
int circle(cv::Mat source, float dim = 1.0)
{
	Mat src, src_gray, dst;
	source.copyTo(src);
	resize(src, src, Size(), dim, dim);


	if (!src.data)
	{
		return -1;
	}

	vector<Vec3f> circles;


	//for (int i(0); i < 2; ++i) medianBlur(abs_dst, abs_dst, 5);

	HoughCircles(src, circles, CV_HOUGH_GRADIENT, 1, src.rows / 8, 50, 20, 150, 250);
	//HoughCircles();


	/// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}

	/// Show your results
	namedWindow("Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE);
	imshow("Hough Circle Transform Demo", src);
	//imshow("Processed Image", src);

	waitKey(0);
	return 0;
}