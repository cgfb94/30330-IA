#pragma warning(disable : 4996)
#include "rover.h"



int 
webcam_capture(void)
{
	const char* wName = "kill me!";
	// window namecvNamedWindow(wName, 0); // create simple window

	CvCapture* capture = 0;

	double capProp = 0;
	IplImage *frame, *frame_copy = 0;
	// pointers to images
	capture = cvCaptureFromCAM(0); // initialize capture device
	if (capture)
	{
		for (;;)
		{
			if (!cvGrabFrame(capture))
				break;
			frame = cvRetrieveFrame(capture);
			(*frame).width = 1500;
			if (!frame)break;
			if (!frame_copy)
			{
				printf("Frame settings:\n Width: %d\n Height: %d\n", frame->width, frame->height);
				frame_copy = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, frame->nChannels);
				cvResizeWindow(wName, frame->width, frame->height);
			}
			if (frame->origin == IPL_ORIGIN_TL)
				cvCopy(frame, frame_copy, 0);
			else
				cvFlip(frame, frame_copy, 0);

			IplImage *grey = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
			cvCvtColor(frame, grey, CV_BGR2GRAY);

			cvShowImage(wName, grey);

			hist(*frame);

			cvWaitKey();

			if (cvWaitKey(5) > 0)
				break;
		}
	}
	cvReleaseImage(&frame_copy);
	cvDestroyWindow("kill me!");
	return 0;

}