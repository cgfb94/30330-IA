#pragma once
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv; using namespace std;


struct point {
	bool light;
	float x; float y;
	float near_dist[10];
	float near_angle[10];
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
};


