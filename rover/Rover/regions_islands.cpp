// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv; using namespace std;


Mat watershed_regions(Mat pic_RGB, int bin_threshold, int bin_type) {

	Mat pic;
	cvtColor(pic_RGB, pic, COLOR_BGR2GRAY);
	threshold(pic, pic, bin_threshold, 255, bin_type);
	//imshow("Binary Image", pic);

	// Perform the distance transform algorithm
	Mat dist;
	distanceTransform(pic, dist, 2, 3);

	// Normalize the distance image for range = {0.0, 1.0}
	// so we can visualize and threshold it
	normalize(dist, dist, 0, 1.0, NORM_MINMAX);
	imshow("Distance Transform Image", dist);

	// Threshold to obtain the peaks
	// This will be the markers for the foreground objects
	threshold(dist, dist, 0.4, 1.0, THRESH_BINARY);
	// Dilate a bit the dist image
	Mat kernel1 = Mat::ones(3, 3, CV_8U);
	dilate(dist, dist, kernel1);
	//imshow("Peaks", dist);

	// Create the CV_8U version of the distance image
	// It is needed for findContours()
	Mat dist_8u;
	dist.convertTo(dist_8u, CV_8U);
	// Find total markers
	vector<vector<Point> > contours;
	findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	// Create the marker image for the watershed algorithm
	Mat markers = Mat::zeros(dist.size(), CV_32S);
	// Draw the foreground markers
	for (size_t i = 0; i < contours.size(); i++)
	{
		drawContours(markers, contours, static_cast<int>(i), Scalar(static_cast<int>(i) + 1), -1);
	}
	// Draw the background marker
	circle(markers, Point(5, 5), 3, Scalar(255), -1);
	//imshow("Markers", markers * 10000);

	// Perform the watershed algorithm
	watershed(pic_RGB, markers);

	Mat mark;
	markers.convertTo(mark, CV_8U);
	bitwise_not(mark, mark);
	//    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
	// image looks like at that point
	// Generate random colors
	vector<Vec3b> colors;
	for (size_t i = 0; i < contours.size(); i++)
	{
		int b = theRNG().uniform(0, 256);
		int g = theRNG().uniform(0, 256);
		int r = theRNG().uniform(0, 256);
		colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}
	// Create the result image
	Mat dst = Mat::zeros(markers.size(), CV_8UC3);
	// Fill labeled objects with random colors
	for (int i = 0; i < markers.rows; i++)
	{
		for (int j = 0; j < markers.cols; j++)
		{
			int index = markers.at<int>(i, j);
			if (index > 0 && index <= static_cast<int>(contours.size()))
			{
				dst.at<Vec3b>(i, j) = colors[index - 1];
			}
		}
	}
	// Visualize the final image
	imshow("Final Result", dst);

	return dst;
}

Mat dist_transf_slopes(Mat pic_RGB, int bin_threshold, int bin_type) {

	Mat pic;
	cvtColor(pic_RGB, pic, COLOR_BGR2GRAY);
	threshold(pic, pic, bin_threshold, 255, bin_type);
	//imshow("Binary Image", pic);

	// Perform the distance transform algorithm
	Mat dist;
	distanceTransform(pic, dist, 2, 3);

	// Normalize the distance image for range = {0.0, 1.0}
	// so we can visualize and threshold it
	//normalize(dist, dist, 0, 255, NORM_MINMAX);
	//imshow("Distance Transform Image", dist);
	return dist;
}

