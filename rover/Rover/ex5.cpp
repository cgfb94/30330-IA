#pragma warning(disable : 4996)

#include "rover.h"

using namespace std;

namespace ex5 {
//IplImage* band(IplImage* in_pic, int b[2]) {
//	IplImage* out_pic = cvCloneImage(in_pic);
//
//	for (int i = 0; i < in_pic->width*in_pic->height; i++) {
//		if (unsigned char(in_pic->imageData[i]) < b[1] && unsigned char(in_pic->imageData[i]) > b[0]) {
//			out_pic->imageData[i] = unsigned char(255);
//		}
//		else {
//			out_pic->imageData[i] = unsigned char(0);
//		}
//	}
//
//	return out_pic;
//}
	vector<unsigned char> region3x3(char* n, int width)
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
		return list;
	}

	unsigned char* region9x9(char* n, int width, unsigned char* list)
	{
		int count = 0;
		float sum = 0;
		n = n - 4 * (width + 1);
		for (int i = 0; i < 9; n += (-8 + width), ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				list[i * 8 + j] = ((unsigned char)*n);
				++n;
				count++;
			}
		}
		return list;
	}

	float compare9x9(unsigned char* ref, unsigned char* other)
	{
		float sum = 0.0;
		for (int i = 0; i < 81; ++i)
			sum += ref[i] - other[i];
		return sum;
	}

	/*int min(IplImage *pic1, IplImage *pic2, int reference)
	{
		vector<tuple<float, int>> scores;
		vector<unsigned char> ref = region3x3(&pic1->imageData[reference], pic1->width);
		for (int i = (pic1->width + 1); i < (pic1->width - 1)*(pic1->height - 1); ++i)
		{
			if (i % pic1->width == pic2->width - 1) {
				i += 3;
			}
			vector<unsigned char> reg = region3x3(&pic2->imageData[i], pic2->width);
			float score = compare(ref, reg);
			scores.push_back(tuple<float, int>(score, i));
		}
		sort(scores.begin(), scores.end());
		return get<1>(scores[0]);
	}*/

	int min9x9(IplImage* pic1, IplImage* pic2, int reference)
	{
		vector<tuple<float, int>> scores;
		unsigned char* ref = (unsigned char*)malloc(81 * sizeof(char));
		unsigned char* reg = (unsigned char*)malloc(81 * sizeof(char));

		region9x9(&pic1->imageData[reference], pic1->width, ref);
		for (int i = 4 * (pic1->width + 1); i < (pic1->width - 4) * (pic1->height - 4); ++i)
		{
			if (i % pic1->width == pic2->width - 4) {
				i += 9;
			}

			region9x9(&pic2->imageData[i], pic2->width, reg);
			float score = compare9x9(ref, reg);
			scores.push_back(tuple<float, int>(score, i));
		}
		std::sort(scores.begin(), scores.end());
		return get<1>(scores[0]);
	}

	int analyse_movement(char* pathL, char* pathR) {
		using namespace ex5;
		//CREATE IMAGE OBJECT FOR PICTURE
		IplImage* pic1 = 0, * pic2 = 0;
		const char* name1 = pathL;
		const char* name2 = pathR;
		pic1 = cvLoadImage(name1, 0);
		pic2 = cvLoadImage(name2, 0);

		int B1 = pic1->width;
		int B2 = pic2->width;

		// REF POINT
		int X = 350;
		int Y = 36;

		cout << "Size image 1: " << pic1->width << "x" << pic1->height << "\nSize image 2: " << pic2->width << "x" << pic2->height << "\n ";

		//CREATE COPIES TO WORK WITH

		IplImage* pic11 = cvCloneImage(pic1);
		IplImage* pic12 = cvCloneImage(pic1);
		IplImage* pic21 = cvCloneImage(pic2);
		IplImage* pic22 = cvCloneImage(pic2);

		int index = min9x9(pic1, pic2, B1 * Y + X);

		int X_ = index % B2;
		int Y_ = index / B2;

		cout << "X' =" << index % B2 << ",  Y'= " << index / B2 << "\n ";


		cvCircle(pic1, cvPoint(X, Y), 10, cvScalar(0), 1);
		cvCircle(pic2, cvPoint(X_, Y_), 10, cvScalar(0), 1);

		//SHOW IMAGES IN NEW WINDOWS 
		const char* Hname1 = "pic1";
		cvNamedWindow(Hname1, 0);
		const char* Hname2 = "pic2";
		cvNamedWindow(Hname2, 0);
		cvResizeWindow(Hname1, pic1->width, pic1->height);
		cvResizeWindow(Hname2, pic2->width, pic2->height);
		cvShowImage(Hname1, pic1);
		cvShowImage(Hname2, pic2);
		cvMoveWindow(Hname1, 1000, 100);
		cvMoveWindow(Hname2, 500, 500);

		cvWaitKey();

		//RELEASE MEMORY
		cvReleaseImage(&pic1);
		cvReleaseImage(&pic2);

		return 0;
	}
}