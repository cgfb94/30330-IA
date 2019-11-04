#pragma once
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <conio.h>

// to find cwd
#include <direct.h>

#include <iostream>
#include <math.h>
#include <algorithm>
#include <vector>
#include <map>
#include <string>

int circle(const char* source);

int hist(IplImage source);
IplImage* webcam_capture(void);	// #TODO change webcam capture to return an IpImage

struct location {
	float x; float y; float z; float angle;
};
int first_image(IplImage* pic);

namespace utils
{
	std::string getAbsImagePath(const char* localPath);
}

namespace ex4
{
	// Median filter
	// #TODO: Make 9x9
	float median3x3(char* n, int width);

	// Draws around the first contour it finds
	// #TODO: See if this works for all pic sizes
	int contour(const char* name);

	IplImage* remove_SaltPepper(IplImage* pic, int amount);
}


namespace ex5
// Functions written for ex 5: stereo vision/ identifying movement
// Implimentation of reg9x9 uses c arrays rather than vectors
{
	using namespace std;

	// Returns a square of pixel values around a point
	vector<uchar> region3x3(char* n, int width);
	unsigned char* region9x9(char* n, int width, unsigned char* list);
	
	// Compares two 9x9 regions to generate a score of similarity
	float compare9x9(unsigned char* ref, unsigned char* other);

	// returns the best matched region in second pic given reference point
	int min9x9(IplImage* pic1, IplImage* pic2, int reference);
	
	// tries to find point in one image in another and draw circle
	int analyse_movement(char* pathL, char* pathR);
}

namespace convert
{
	IplImage* mat2Ipl_RGB(cv::Mat image1);
	IplImage* mat2Ipl_grey(cv::Mat image1);
}
