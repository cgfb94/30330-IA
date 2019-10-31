// Disable depreciation warning
#pragma warning(disable : 4996)

#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <conio.h>

int hist(IplImage source)
{
  using namespace cv;
  Mat src, hsv;

  src = Mat(&source);
  cvtColor(src, hsv, CV_BGR2HSV);

  // Quantize the hue to 30 levels
  // and the saturation to 32 levels
  int hbins = 30, sbins = 32;
  int histSize[] = { hbins, sbins };
  // hue varies from 0 to 179, see cvtColor
  float hranges[] = { 0, 180 };
  // saturation varies from 0 (black-gray-white) to
  // 255 (pure spectrum color)
  float sranges[] = { 0, 256 };
  const float* ranges[] = { hranges, sranges };
  MatND hist;
  // we compute the histogram from the 0-th and 1-st channels
  int channels[] = { 0, 1 };

  calcHist(&hsv, 1, channels, Mat(), // do not use mask
    hist, 2, histSize, ranges,
    true, // the histogram is uniform
    false);
  double maxVal = 0;
  minMaxLoc(hist, 0, &maxVal, 0, 0);

  int scale = 10;
  Mat histImg = Mat::zeros(sbins*scale, hbins * 10, CV_8UC3);

  for (int h = 0; h < hbins; h++)
    for (int s = 0; s < sbins; s++)
    {
      float binVal = hist.at<float>(h, s);
      int intensity = cvRound(binVal * 255 / maxVal);
      rectangle(histImg, Point(h*scale, s*scale),
        Point((h + 1)*scale - 1, (s + 1)*scale - 1),
        Scalar::all(intensity),
        CV_FILLED);
    }

  namedWindow("Source", 1);
  imshow("Source", src);

  namedWindow("H-S Histogram", 1);
  imshow("H-S Histogram", histImg);
  waitKey();
  return 0;
}


int main(int argc, char* argv[])
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
