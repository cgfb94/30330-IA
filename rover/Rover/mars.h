#pragma once
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv; using namespace std;


struct location {
	float x; float y; float z;
	float angle;
	float error=100000000;
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


