// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath1 =  utils::getAbsImagePath("Images\\mars5.jpeg");
	std::string imPath2 =  utils::getAbsImagePath("Images\\mars6.jpeg");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;

	//Look for circles
	//circle(imPath.c_str());

	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());
	
	Mat pic1 = imread(imPath1.c_str(), 1);
	Mat pic2 = imread(imPath2.c_str(), 1);
	int x = test(pic1, pic2);

	return 0;
}
