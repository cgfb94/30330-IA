// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath =  utils::getAbsImagePath("Images\\mars3.jpeg");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;

	//Look for circles
	//circle(imPath.c_str());

	//Terrain navigation
	int position;
	//IplImage* pic = cvLoadImage(imPath.c_str(), 0);
	position = first_image(imPath.c_str());

	return 0;
}
