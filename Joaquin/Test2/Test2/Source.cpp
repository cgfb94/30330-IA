#pragma warning(disable : 4996)
#include <stdio.h>
#include "cv.h"
#include "cxcore.h" 
#include "highgui.h" 
#include <conio.h> 
#include <iostream>
#include <math.h>

using namespace std;

float median3x3(char* n, int width)
{
	float sum = 0;
	vector<unsigned char> list;
	n = n - width - 1;
	for (int i = 0; i < 3; n += (-2 + width), ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			list.push_back((unsigned char)*n);
			++n;
		}
	}
	sort(list.begin(), list.end());
	return list[4];
}


int main(int argc, char* argv[])
{
	//CREATE IMAGE OBJECT FOR PICTURE
	
	IplImage* colour = 0;  // Only if picture recorded by cam.
	IplImage * pic1 = 0;
	const char* name1 = "D:\\joaro\\Documents\\GitHub\\30330-IA\\rover\\Rover\\Images\\mars1.jpeg";
	
	//LOAD PICTURE FROM FOLDER
	pic1 = cvLoadImage(name1, 0);

	////CAPTURE IMAGE
	//CvCapture* capture = 0;

	//double capProp = 0;

	//// pointers to images
	//capture = cvCaptureFromCAM(0); // initialize capture device
	//if (capture)
	//{
	//	for (;;)
	//	{
	//		if (!cvGrabFrame(capture))
	//			return 12;
	//		colour = cvRetrieveFrame(capture);
	//		if (colour) break;

	//	}
	//}
	//else { printf("fuck"); return 11; };

	////TURN TO GREY
	//pic1 = cvCreateImage(cvSize(colour->width, colour->height), IPL_DEPTH_8U, 1);
	//cvCvtColor(colour, pic1, CV_BGR2GRAY);
	

	cout << "Size image: " << pic1->width << "x" << pic1->height << "\n";

	unsigned char *channel; // pre-allocated array holding image data for the color channel or the grayscale image. unsigned char value=0; // index value for the histogram (not really needed) int histogram[256]; // histogram array - remember to set to zero initially int width; // say, 320 int height; // say, 240 
	channel = (unsigned char*)pic1->imageData; //Imagedata is converted to unsigned char


	//CREATE COPIES OF THE IMAGE
	IplImage* pic2 = cvCloneImage(pic1);
	IplImage* pic3 = cvCloneImage(pic1);
	IplImage* pic4 = cvCloneImage(pic1);








	//SHOW IMAGES IN NEW WINDOWS 
	const char* Hname1 = "pic1";
	cvNamedWindow(Hname1, 0);
	cvResizeWindow(Hname1, pic1->width/2, pic1->height/2);
	cvShowImage(Hname1, pic1);
	//cvMoveWindow(Hname1, 1000, 100);

	//cvWaitKey(5);

	//RELEASE MEMORY
	cvReleaseImage(&pic1);

	cvWaitKey();
}