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


class square_matchings
{
public:
	static const int MAX_MATCHINGS = 10;
	//static const int MAX_CAPTURES = 50;
	int match_ref[4][MAX_MATCHINGS];
	// [0] --> number
	// [1] --> ref in pic1
	// [2] --> ref in pic2
	// [3] --> discrepancy
	int num_found;

	square_matchings() {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < MAX_MATCHINGS; j++) {
				if (i == 0) match_ref[0][j] = j + 1;
				else match_ref[i][j] = -1;
			}
		}
		num_found = 0;
	}

	void retrieve_SQUARE_matchings(float** weights, int n, int m, int max_error) {
		int full = 0;
		for (int k = 0; k <= max_error; k++) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					if (weights[i][j] == k){
						if (full >= MAX_MATCHINGS) break;
						match_ref[1][full] = i;
						match_ref[2][full] = j;
						match_ref[3][full] = k;
						full++;
					}
				}
			}
		}
		num_found = full;
	}

	void retrieve_ANGLE_matchings(float** weights, int n, int m, int min_accuracy, int max_match) {
		int full = 0;
		for (int k = max_match; k >= min_accuracy; k--) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					if (weights[i][j] == k) {
						if (full >= MAX_MATCHINGS) break;
						match_ref[1][full] = i;
						match_ref[2][full] = j;
						match_ref[3][full] = k;
						full++;
					}
				}
			}
		}
		num_found = full;
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
int test2(Mat object, Mat image);


