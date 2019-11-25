// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

cv::Mat test(cv::Mat input) { return input; }

int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath0 = utils::getAbsImagePath("Images\\mars8.jpeg");
	std::string imPath1 =  utils::getAbsImagePath("Images\\mars7.jpeg"); //7,6
	std::string imPath2 =  utils::getAbsImagePath("Images\\mars6.jpeg");
	std::string imPath3 = utils::getAbsImagePath("Images\\mars5.jpeg");
	std::string imPath4 = utils::getAbsImagePath("Images\\mars3.jpeg");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;
	std::string impath = utils::getAbsImagePath("Images\\mars5.jpeg");
	//Look for circles
	//cv::Mat fourierIm = fourier(impath.c_str());

	cv::Mat src = cv::imread(impath, 1);

	cv::Mat image = (utils::loadImageG(impath));

	//fourier(image, test);

	//circle(fourierIm);


	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());

	vector<Mat> pics = { imread(imPath0.c_str(), 1), imread(imPath1.c_str(), 1), imread(imPath2.c_str(), 1), imread(imPath3.c_str(), 1), imread(imPath4.c_str(), 1) };

	int x = test3(pics);

	//int x = test(pic1, pic2);


	return 0;
}
