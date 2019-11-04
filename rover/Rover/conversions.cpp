// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv;


namespace convert
{
	IplImage* mat2Ipl_RGB(Mat image1) {
		IplImage* image2;
		image2 = cvCreateImage(cvSize(image1.cols, image1.rows), 8, 3);
		IplImage ipltemp = image1;
		cvCopy(&ipltemp, image2);
		return image2;
	}

	IplImage* mat2Ipl_grey(Mat image1) {
		IplImage* image2;
		image2 = cvCloneImage(&(IplImage)image1);
		return image2;
	}
}
