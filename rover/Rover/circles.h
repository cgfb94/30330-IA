// Disable depreciation warning
#pragma warning(disable : 4996)

//TODO: efficiency improvements, calculate r and stop if it is out of bounds
//		calc the expected size of the circle from the distance input

#include "mars.h"
#include "rover.h"

#include <random>
#include <tuple>

#include <time.h>

vector <int>
my_RANSAC(cv::Mat img, float r1, float r2);

cv::Mat
webcam_capture_main(bool save = false, std::string name = "")
{
	VideoCapture cap(0);

	// 	if (!cap.isOpened())
	// 		// check if we succeeded
	// 		std::cout << "Camera cannot be opened!\n";
	// 		return cv::Mat();

	cv::Mat Image;
	cap >> Image;

	if (save)
	{
		imwrite(name, Image);
	}

	return Image;
}

cv::Mat
preprocess_main(cv::Mat image, float dx = 1.0) {
	Mat src;
	image.copyTo(src);
	cv::resize(src, src, Size(), dx, dx);
	using namespace cv;
	int kernel_size = 5;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	/// Apply the Hough Transform to find the circles
	Mat abs_dst, src_gray, dst;


	/// Reduce the noise so we avoid false circle detection
	for (int i(0); i < 1; ++i)
	{
		medianBlur(src, src, 3);
		blur(src, src, Size(3, 3));
	}
	/// Convert it to gray
	//blur(src, src, Size(5,5));
	if (src.channels() == 3)  cvtColor(src, src, CV_BGR2GRAY);
	// split b g and r into different channels, process each seperately 
	// average the circle coords
	threshold(src, src, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	for (int i(0); i < 5; ++i) medianBlur(src, src, 7);
	// 	for (int i(0); i < 5; ++i) blur(src, src, Size(3, 3));

		//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
		//Laplacian(src, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	Canny(src, abs_dst, 2, 20 * 5);

	//convertScaleAbs(dst, abs_dst);
	//threshold(abs_dst, abs_dst, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	return abs_dst;
}

tuple <float, float, float, float, float>
circle_finder(cv::Mat ProcessedImage, float min_rad, float max_rad, int bordersize, cv::Mat Original = cv::Mat())
{

	vector<float> results = my_RANSAC(ProcessedImage, min_rad, max_rad);
	cv::Point center = cv::Point(floor(ProcessedImage.cols / 2), floor(ProcessedImage.rows / 2));

	float x, y, r, score;
	x = results[0] - bordersize;
	y = results[1] - bordersize;
	r = results[2];
	score = results[3];

	float smallestSide = (ProcessedImage.cols < ProcessedImage.rows) ? ProcessedImage.cols : ProcessedImage.rows;

	float fraction_filled = (2 * r) / smallestSide;
	float delta_z = 0.9 / fraction_filled * 100;

	std::cout << "\n Fraction Filled: " << fraction_filled << '\n';

	tuple<float, float, float, float, float> deltas{ x - center.x, y - center.y , delta_z, r, score };

	if (Original.empty())
	{
		return deltas;
	}

	cv::circle(Original, cv::Point(x, y), 5, cv::Scalar(0, 0, 255));
	cv::circle(Original, cv::Point(x, y), r, cv::Scalar(255, 100, 100));

	cv::circle(Original, center, 5, cv::Scalar(255, 255, 255));

	cv::line(Original, center, Point2d(x, y), cv::Scalar(255, 255, 255));

	std::cout << "delta X: " << get<0>(deltas) << "\ndelta Y: " << get<1>(deltas) << "\ndelta Z%: " << get<2>(deltas) << '\n';

	cv::imshow("RANSAC Example", Original);
	cv::waitKey(0);


	return deltas;
}