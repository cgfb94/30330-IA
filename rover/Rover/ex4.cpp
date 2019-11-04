#pragma warning(disable : 4996)

#include "rover.h"

using namespace std;

namespace ex4
{

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

	//float filt9x9(char* n, int m, int width, char kernel[9][9], float K = 1000)
	//{
	//	float sum = 0;
	//
	//	n = n - 4*(width+1);
	//	for (int i = 0; i < 9; n += (-8 + width), ++i)
	//	{
	//		for (int j = 0; j < 9; ++j)
	//		{
	//			sum += ((unsigned char)(*n)) * (kernel[i][j]);
	//			++n;
	//		}
	//	}
	//	return sum / K;
	//}

	IplImage* remove_SaltPepper(IplImage* pic, int amount) {
		IplImage* filtered_pic = cvCloneImage(pic);

		for (int j = 0; j < amount; j++) {
			for (int i = (pic->width + 1); i < (pic->width - 1) * (pic->height - 1); i++) {
				char* pointer = &pic->imageData[i];
				filtered_pic->imageData[i] = unsigned char(median3x3(pointer, pic->width));
			}
		}

		return filtered_pic;
	}

	int contour(const char* name) {


		//CREATE IMAGE OBJECT FOR PICTURE
		IplImage* colour = 0, * pic = 0;
		
		//LOAD PICTURE INFO
		pic = cvLoadImage(name, 0);
						  

		if (!pic) return 1;


		//CAPTURE IMAGE
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

		//TURN TO GREY
		//pic = cvCreateImage(cvSize(colour->width, colour->height), IPL_DEPTH_8U, 1);
		//cvCvtColor(colour, pic, CV_BGR2GRAY);



		cout << "Size image: " << pic->width << "x" << pic->height << "\n";

		unsigned char* channel; // pre-allocated array holding image data for the color channel or the grayscale image. unsigned char value=0; // index value for the histogram (not really needed) int histogram[256]; // histogram array - remember to set to zero initially int width; // say, 320 int height; // say, 240 
		channel = (unsigned char*)pic->imageData; //Imagedata is converted to unsigned char


		//CREATE A COPY OF THE IMAGE
		IplImage* pic2 = cvCloneImage(pic);
		IplImage* pic3 = cvCloneImage(pic);
		IplImage* pic4 = cvCloneImage(pic);

		double count1 = cvGetTickCount();

		// REMOVE NOISE

		//Our version

		//Differential filter
		char avg_kern[3][3] = { {1, 1, 1},{1, 1, 1},{1, 1, 1} };
		//Differential filter
		char dif_kern[3][3] = { {0, -1, 0},{-1,4,-1},{0, -1, 0} };
		//Tent filter
		char tent_kern[3][3] = { {1, 2, 1},{2,4,2},{1, 2, 1} };

		// Median filter (pic --> pic2)
		int times1 = 2;

		for (int j = 0; j < times1; j++) {
			for (int i = (pic->width + 1); i < (pic->width - 1) * (pic->height - 1); i++) {
				char* pointer = &pic->imageData[i];
				pic2->imageData[i] = unsigned char(median3x3(pointer, pic->width));
			}
		}


		////Average filter (pic -> pic2)
		//CvMat *kernel1 = cvCreateMat(3, 3, CV_32F);
		//float avg[9] = { 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9, 1.0 / 9};
		//float tent[9] = { 1.0/16, 2.0/16, 1.0/16, 2.0/16, 4.0/16, 2.0/16, 1.0/16, 2.0/16, 1.0/16 };
		//kernel1->data.fl = avg;
		// 

		//for (int i = 0; i < 0; i++) {
		// cvFilter2D(pic, pic4, kernel1);
		// cvFilter2D(pic4, pic, kernel1);
		//}

		//pic2 = cvCloneImage(pic);

		// BINARIZE (pic2 --> pic3)
		int t = 70;
		for (int i = 0; i < pic2->width * pic2->height; i++) {
			if (unsigned char(pic3->imageData[i]) > t) {
				pic3->imageData[i] = unsigned char(255);
			}
			else {
				pic3->imageData[i] = unsigned char(0);
			}
		}


		//Laplacian filter (pic3 -> pic4)
		CvMat* kernel = cvCreateMat(3, 3, CV_32F);
		float laplace[9] = { 0.0, 1.0, 0.0, 1.0, -4.0, 1.0, 0.0, 1.0, 0.0 };
		float laplace2[81] = { 0,0,1,2,2,2,1,0,0,0,1,5,10,12,10,5,1,0,
								1,5,15,19,16,19,15,5,1,2,10,19,-19,-64,-19,19,10,2,
								2,12,16,-64,-148,-64,16,12,2,2,10,19,-19,-64,-19,19,10,2,
								1,5,15,19,16,19,15,5,1 , 0,1,5,10,12,10,5,1,0,
								0,0,1,2,2,2,1,0,0 };
		kernel->data.fl = laplace;

		cvFilter2D(pic3, pic4, kernel);


		// CONTOUR SEARCH
		const int MAX_RAND = 1000;
		unsigned char* pict = (unsigned char*) & (pic4->imageData[0]); // placeholder for image data 
		int rimx[MAX_RAND], rimy[MAX_RAND], pos_i[MAX_RAND];
		int newpos, local_tresh, draw_type; draw_type = 0;
		int pos = 0;
		newpos = 0;
		int count = 0, B = pic4->width;
		local_tresh = 10;
		for (int i = B * 125 + 40; i < pic4->width * pic4->height; i++)
		{
			cout << pict[i] << " ";
			if (pict[i] > local_tresh) { pos = i; newpos = pos; cout << "\n" << pos << "\n FOUND\n"; break; }
		}


		while (newpos >= 0L && newpos < (pic4->width) * (pic4->height))
		{
			pos_i[count] = newpos;
			rimx[count] = newpos % B; // save current position in list
			rimy[count] = newpos / B;
			count++;
			draw_type = (draw_type + 6) % 8; // Select next search direction
			switch (draw_type)
			{
			case 0: if (pict[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
			case 1: if (pict[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
			case 2: if (pict[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
			case 3: if (pict[newpos + B - 1] > local_tresh) { newpos += B - 1; draw_type = 3; break; }
			case 4:if (pict[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
			case 5: if (pict[newpos - B - 1] > local_tresh) { newpos -= B + 1; draw_type = 5; break; }
			case 6: if (pict[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
			case 7: if (pict[newpos - B + 1] > local_tresh) { newpos -= B - 1; draw_type = 7; break; }
			case 8: if (pict[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
			case 9: if (pict[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
			case 10: if (pict[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
			case 11: if (pict[newpos + B - 1] > local_tresh) { newpos += B - 1; draw_type = 3; break; }
			case 12: if (pict[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
			case 13: if (pict[newpos - B - 1] > local_tresh) { newpos -= B + 1; draw_type = 5; break; }
			case 14: if (pict[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
			}// If weare back at the beginning, we declare success
			if (newpos == pos) break;// Abortif the contour is too complex.
			if (count >= MAX_RAND)break;
		}

		//PRINT CONTOUR
		for (int j = 0; j < count; j++) {
			cout << pos_i[j] << " ";
			pic2->imageData[pos_i[j]] = unsigned char(255);
		}


		////Time count
		//double time = (cvGetTickCount() - count1) / (cvGetTickFrequency());
		//cout << "\n Time invested = " << time << " units\n";



		//SHOW IMAGES IN NEW WINDOWS 
		const char* Hname2 = "pic2";
		cvNamedWindow(Hname2, 0);
		const char* Hname3 = "pic3";
		cvNamedWindow(Hname3, 0);
		cvResizeWindow(Hname2, pic->width, pic->height);
		cvResizeWindow(Hname3, pic->width, pic->height);
		cvShowImage(Hname2, pic2);
		cvShowImage(Hname3, pic3);
		cvMoveWindow(Hname2, 1000, 100);
		cvMoveWindow(Hname3, 500, 500);
		const char* Hname4 = "pic4";
		cvNamedWindow(Hname4, 0);
		cvResizeWindow(Hname4, pic->width, pic->height);
		cvShowImage(Hname4, pic4);
		cvMoveWindow(Hname3, 1000, 500);

		cvShowImage(name, pic);
		cvWaitKey(5);

		//RELEASE MEMORY
		cvReleaseImage(&pic);
		cvReleaseImage(&pic2);
		cvReleaseImage(&pic3);
		cvReleaseImage(&pic4);
		cvReleaseImage(&colour);

		cvWaitKey();
		return 0;
	}
}