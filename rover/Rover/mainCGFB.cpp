// Disable depreciation warning
#pragma warning(disable : 4996)

//TODO: efficiency improvements, calculate r and stop if it is out of bounds
//		calc the expected size of the circle from the distance input

#include "mars.h"
#include "rover.h"

#include <random>
#include <tuple>


//calculates absolute circle position


vector <int> 
my_RANSAC(cv::Mat img, float r1, float r2);

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

	threshold(src, src, 40, 255, THRESH_TOZERO | THRESH_OTSU);
 	for (int i(0); i < 50; ++i) medianBlur(src, src, 7);
// 	for (int i(0); i < 5; ++i) blur(src, src, Size(3, 3));

	//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
	//Laplacian(src, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	Canny(src, abs_dst, 20, 20*5);
	
	//convertScaleAbs(dst, abs_dst);
	//threshold(abs_dst, abs_dst, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	return abs_dst;
}

pair <int, int>
circle_finder(cv::Mat ProcessedImage, float min_rad, float max_rad, int bordersize, cv::Mat Original = cv::Mat() )
{

	vector<int> results = my_RANSAC(ProcessedImage, min_rad, max_rad);
	cv::Point center = cv::Point(floor(ProcessedImage.cols / 2), floor(ProcessedImage.rows / 2));

	int x, y, r;
	x = results[0] - bordersize;
	y = results[1] - bordersize;
	r = results[2];

	pair<int, int> deltas{ x - center.x, y - center.y };

	if (Original.empty())
	{
		return deltas;
	}

	cv::circle(Original, cv::Point(x, y), 5, cv::Scalar(0, 0, 255));
	cv::circle(Original, cv::Point(x, y), r, cv::Scalar(255, 100, 100));
		
	cv::circle(Original, center, 5, cv::Scalar(255, 255, 255));

	cv::line(Original, center, Point2d(x, y), cv::Scalar(255, 255, 255));

	std::cout << "delta X: " << deltas.first << "\n delta Y: " << deltas.second << '\n';

	cv::imshow("RANSAC Example", Original);
	cv::waitKey(0);

	
	return deltas;
}


int main(int argc, char* argv[])
// iternate through the list of contours given by canny, find average gradient
// of the last few points. split the contour if the gradient deviates from average
// by too much and then continue search on other half of the contour
// use the new list of contours to detect circles
{
	//IplImage* Image = webcam_capture();
	std::string imPath =  utils::getAbsImagePath("Images\\mars1.jpeg");

	//cv::Mat image = (utils::loadImageG(impath));
	cv::Mat dst;
	cv::Mat image = cv::imread(imPath.c_str(), 1);
	cv::resize(image, image, cv::Size(), 0.3, 0.3);
	cv::Mat processed = preprocess_main(image, 1);
	int bordersize = 10;
	copyMakeBorder(processed, dst, bordersize, bordersize, bordersize, bordersize, BORDER_CONSTANT);
	// collect the top 10 and plot in different colours
	// restrict score by most complete circle

	float r1, r2;
	r1 = 50;
	r2 = 120;
	
	circle_finder(dst, r1, r2, bordersize, image);
	cv::waitKey(0);


	// write a function  that accepts the max/min radii 
	// compare with the output of the hough circle algorithm

// 	Mat canny_output;
// 	vector<vector<Point> > contours;
// 	vector<Vec4i> hierarchy;
// 
// 	/// Detect edges using canny
// 	Canny(processed, canny_output, 150, 150 * 2, 3);
// 	/// Find contours
// 	findContours(canny_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));
// 
// 	/// Draw contours
// 	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
// 	for (int i = 0; i < contours.size(); i++)
// 	{
// 		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
// 		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
// 	}


	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());
	
	//Mat pic1 = imread(imPath1.c_str(), 1);
	//Mat pic2 = imread(imPath2.c_str(), 1);

	//int x = test2(pic1, pic2);

	//int x = test(pic1, pic2);
}




