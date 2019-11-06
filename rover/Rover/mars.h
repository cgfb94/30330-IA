#pragma once
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv; using namespace std;

struct location {
	float x=0; float y=0; float z=0;
	float angle=0;
	float error = 100000;
	Mat overlap;
};

struct feature
{
	Mat picture;
	int o_x; int o_y;
	Moments M;
};

struct region
{
	feature light_feature[36];
	feature dark_feature[36];
	int pos_x; int pos_y;
};

struct picture
{
	Mat original;
	Mat diferential;
	Mat starfield;
	vector<Point> locations;
	//int spots[2][100];
};

int test(Mat object, Mat image);

