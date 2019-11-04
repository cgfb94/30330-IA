// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"



int first_image(IplImage* pic)
{
	location pos;

	IplImage* pic2 = 0;
	//pic2 = ex4::remove_SaltPepper(pic, 2);

	//SHOW IMAGES IN NEW WINDOWS 
	const char* Wname = "pic";
	const char* Wname2 = "pic2";
	cvNamedWindow(Wname, 0);
	cvNamedWindow(Wname2, 0);
	cvResizeWindow(Wname, pic->width, pic->height);
	cvResizeWindow(Wname2, pic2->width, pic2->height);
	cvMoveWindow(Wname, 1000, 100);
	cvMoveWindow(Wname2, 500, 500);
	cvShowImage(Wname, pic);
	cvShowImage(Wname2, pic2);
	
	cv::waitKey();
	return 0;
}

