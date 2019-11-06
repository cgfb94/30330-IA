// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

int main(int argc, char* argv[])
{
	//IplImage* Image = webcam_capture();
	std::string imPath1 =  utils::getAbsImagePath("Images\\search.png");
	std::string imPath2 =  utils::getAbsImagePath("Images\\test.png");
	// Produce some contour images
	//if (ex4::contour(imPath.c_str())) return 1;

	//Look for circles
	//circle(imPath.c_str());

	//Terrain navigation
	//int position;
	//position = first_image(imPath.c_str());
	
	Mat shape = imread(imPath1.c_str(), 1);
	Mat place = imread(imPath2.c_str(), 1);
	int x = try_fit_feature(shape, place);

	return 0;
}
