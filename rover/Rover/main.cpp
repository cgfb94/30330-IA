// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

cv::Mat test(cv::Mat input) { return input; }

int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath1 =  utils::getAbsImagePath("Images\\mars5.jpeg");
	std::string imPath2 =  utils::getAbsImagePath("Images\\mars6.jpeg");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;
	std::string impath = utils::getAbsImagePath("Images\\mars5.jpeg");
	//Look for circles
	//cv::Mat fourierIm = fourier(impath.c_str());

	cv::Mat src = cv::imread(impath, 1);

	cv::Mat image = (utils::loadImageG(impath));

	fourier(image, test);

	//circle(fourierIm);


	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());
	
	/*Mat pic1 = imread(imPath1.c_str(), 1);
	Mat pic2 = imread(imPath2.c_str(), 1);
	int x = test(pic1, pic2);*/

	return 0;
}
