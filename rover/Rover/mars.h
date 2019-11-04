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

class constellation
{
private:
	point star[10];
	Mat pic;
};
