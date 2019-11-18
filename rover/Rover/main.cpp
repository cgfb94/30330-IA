// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

#include <random>
#include <tuple>

cv::Mat test(cv::Mat input) { return input; }

cv::Mat preprocess(cv::Mat image) {
	Mat src;
	image.copyTo(src);
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
	for (int i(0); i < 5; ++i) blur(src, src, Size(3, 3));

	//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
	Laplacian(src, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	//Canny(src_gray, abs_dst, 20, 20*5);

	convertScaleAbs(dst, abs_dst);

	return abs_dst;
}

pair<int, int> circle_opt(cv::Mat input, int x, int y)
// Searches for best radius given a starting position in an image. Returns pair <score, radius>
{
	// Create a vector to hold scores of circles centered around this point
	vector<pair<int, int>> scores;

	for (int radius1(100), radius2(150); radius1 < input.rows * 1.5; radius1 += 5, radius2 += 5)
	{
		// sum area around point 
		int difference = radius1 - radius2;

		// check whether col range is allowed otherwise it will crash
		if (y - radius2 > 0 && y + radius2 < input.size().width)
		{
			if (x - radius2 > 0 && x + radius2 < input.size().width)
			{
				cv::Mat reduced = input.rowRange(y - radius2, y + radius2);
				reduced = reduced.colRange(x - radius2, x + radius2);
				int score = 0;
				for (int i(0); i < reduced.cols * reduced.rows; ++i)
				{
					// try to calculate distance with pythag
					int x_2 = (i % reduced.size().width - (reduced.size().width / 2));
					int y_2 = (i / reduced.size().width - (reduced.size().height / 2));
					x_2 = pow(x_2, 2);
					y_2 = pow(y_2, 2);
					if (x_2 + y_2 > pow(radius1, 2) && x_2 + y_2 < pow(radius2, 2))
					{
						score += reduced.data[i];
					}
				}
				std::pair<int, int> entry(score, radius1);
				scores.push_back(entry);
			}
		}
			
		}
	// find the radius that gives the best score
	sort(scores.begin(), scores.end());
	if (scores.data() == nullptr) return pair<int, int>(0, 0);
	return scores.data()[0];


}

int cirlce_finder(cv::Mat Image)
{
	// Sample points in image
	// expand 2 circles around this point with r1 r2
	// check pixel values that fall between the two radii (expand to also look for ellipse?)
	// score the circles on how many edge points they contain
	// return the 2 best non overlapping circles
	using namespace cv;
	Mat src, src_gray, dst;
	//vector<pair<int, pair <int,int>>> circles;
	// score, position, radius
	vector<tuple<int, int, int>> circles;



	// Add a border around the image so circles can have centers out of image

	int bordersize = 600;

	src = preprocess(Image);
	copyMakeBorder(src, dst, bordersize, bordersize, bordersize, bordersize, BORDER_CONSTANT);
	for (int i(0); i < dst.size().area(); i += 20)
	{
		int score, position, radius;
		pair<int,int> vals = circle_opt(dst, i % dst.size().width, i / dst.size().width);
		score = vals.second;
		position = i;
		if (i % 100 == 0) cout << i << '\n';
		radius = vals.first;
		circles.push_back(tuple<int,int,int>(score, position, radius));
	}
	sort(circles.begin(), circles.end());
	
	// Plot the best circle found 
	int x, y, radius;
	x = get<1>(circles.data()[0]) % dst.size().width;
	y = get<1>(circles.data()[0]) / dst.size().width;
	radius = get<2>(circles.data()[0]);
	Point center(x,y);
	// circle center

	if (Image.channels() == 1) cvtColor(Image, Image, CV_GRAY2BGR);
	circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	// circle outline
	circle(src, center, radius, Scalar(0, 0, 255), 10, 8, 0);

	imshow("Source", Image);
	imshow("Processed Image", dst);

	waitKey(0);

	// Draw the circles
//	for (size_t i = 0; i < circles.size(); i++)
//	{
//		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//		int radius = cvRound(circles[i][2]);
//		// circle center
//		circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
//		// circle outline
//		circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
//	}
	return 0;
}


int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath1 =  utils::getAbsImagePath("Images\\mars5.jpeg");
	std::string imPath2 =  utils::getAbsImagePath("Images\\mars6.jpeg");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;
	std::string impath = utils::getAbsImagePath("Images\\mars6.jpeg");
	//Look for circles
	//cv::Mat fourierIm = fourier(impath.c_str());

	//cv::Mat src = cv::imread(impath, 1);

	cv::Mat image = (utils::loadImageG(impath));

	cirlce_finder(image);

	//fourier(image, test);

	//threshold_exe(image);

	//circle(impath.c_str());


	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());
	
	//Mat pic1 = imread(imPath1.c_str(), 1);
	//Mat pic2 = imread(imPath2.c_str(), 1);

	//int x = test2(pic1, pic2);

	//int x = test(pic1, pic2);


	return 0;
}


