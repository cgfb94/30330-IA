// Disable depreciation warning
#pragma warning(disable : 4996)

//TODO: efficiency improvements, calculate r and stop if it is out of bounds
//		calc the expected size of the circle from the distance input

#include "mars.h"
#include "rover.h"

#include <random>
#include <tuple>

#include <time.h>
#include <sstream>

vector <float> 
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

	int erode_sz = 2;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(2 * erode_sz + 1, 2 * erode_sz + 1),
		cv::Point(erode_sz, erode_sz));

	//for (int i(0); i < 1; ++i) cv::dilate(src, src, element);
 	for (int i(0); i < 10; ++i) medianBlur(src, src, 7);
// 	for (int i(0); i < 5; ++i) blur(src, src, Size(3, 3));

	//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
	//Laplacian(src, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	Canny(src, abs_dst, 20, 20*5);
	
	//convertScaleAbs(dst, abs_dst);
	//threshold(abs_dst, abs_dst, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	return abs_dst;
}

tuple <float, float, float, float, float>
circle_finder(cv::Mat ProcessedImage, float min_rad, float max_rad, int bordersize, cv::Mat Original = cv::Mat() )
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

	tuple<float, float, float, float, float> deltas{ x - center.x, y - center.y , delta_z, r, score};

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

float estimate_radius(float distance, float focal_length, float expected_radius)
{
	return (expected_radius / distance) * focal_length;
}


int main(int argc, char* argv[])
// iternate through the list of contours given by canny, find average gradient
// of the last few points. split the contour if the gradient deviates from average
// by too much and then continue search on other half of the contour
// use the new list of contours to detect circles
{
	std::string name = utils::getAbsImagePath("Images//G//66-G-x2.y-2-0.105m-L1-R0.jpg");

	cv::Mat Image = cv::imread(name.c_str(), 1);
	cv::Mat Image_640x480 = cv::Mat(360, 640, CV_8UC3, Scalar());
	cv::resize(Image, Image_640x480, Image_640x480.size());
	cv::Mat processed = preprocess_main(Image_640x480, 1);
 	int bordersize = 0;
 	float r1, r2;
	// 640 480
	// 1265 : 678 focal length
	// 980 : G focal length
	float radius_estimate = estimate_radius(10.5, 960, 2.5);
 	r1 = radius_estimate * 0.75;
 	r2 = radius_estimate * 1.25;

 	circle_finder(processed, r1, r2, bordersize, Image_640x480);

	//vector<Mat> pics = { imread(imPath0.c_str(), 1), imread(imPath1.c_str(), 1), imread(imPath2.c_str(), 1), imread(imPath3.c_str(), 1), imread(imPath4.c_str(), 1) };

	//int x = test3(pics);
}




