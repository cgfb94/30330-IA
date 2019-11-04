// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv;

int first_image(const char* source)
{
	location pos;
	Mat pic2, pic3;

	Mat pic = imread(source, 1);
	resize(pic, pic, Size(), 0.4, 0.4);

	//Turn to gray
	cvtColor(pic, pic, CV_BGR2GRAY);
	
	//pic2 = ex4::remove_SaltPepper(pic, 1);
	medianBlur(pic, pic2, 3);


	// BINARIZE (pic2 --> pic3)
	int t = 150;
	threshold(pic2, pic3, t, 255, 0);


	//SHOW IMAGES IN NEW WINDOWS 
	namedWindow("pic1", CV_WINDOW_AUTOSIZE);
	imshow("pic1", Mat(pic));
	namedWindow("pic2", CV_WINDOW_AUTOSIZE);
	imshow("pic2", Mat(pic3));
	
	cv::waitKey();
	return 0;
}

