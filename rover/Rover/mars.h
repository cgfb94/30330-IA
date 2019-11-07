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

class matchings
{
	static const int MAX_MATCHINGS = 50;
	static const int MAX_CAPTURES = 50;
	int ref_in_pic[MAX_CAPTURES][MAX_MATCHINGS];

	matchings() {
		for (int i = 0; i < MAX_CAPTURES; i++) for (int j = 0; j < MAX_MATCHINGS; j++) ref_in_pic[i][j] = -1;
	}
};



class picture
{
public:
	Mat original;
	Mat diferential;
	Mat starfield;
	vector<Point> locations;
	location captured_from;
	//int spots[2][100];

	picture() {};

	Mat get_piece(Point spot, int n_pixels) {
		Mat piece;
		int l = n_pixels / 2;
		piece = original(Rect(spot.x - l, spot.y - l, n_pixels, n_pixels));
		return piece;
	}
	
	vector<Mat> get_landmark_pics(int size, bool show=false)
	{
		int n = locations.size();
		vector<Mat> feat(n);
		for (int i = 0; i < n; i++) {
			feat[i] = get_piece(locations[i], size);
		}
		
		if (show) {
			for (int i = 0; i < n; i++) {
				imshow(to_string(i), feat[i]);
			}
		}

		return feat;
	}
};

int test(Mat object, Mat image);

