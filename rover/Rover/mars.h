#pragma once
#pragma warning(disable : 4996)

#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv; using namespace std;

float euclideanDist(Point p, Point q);

struct location {
	Point2f traslation = { 0,0 };
	Point2f abs_centre = { 0,0 };
	Point2f map_centre = { 0,0 };
	float dz = 0;
	float z = 1;
	float d_angle = 0;
	float angle=0;
	float step_distance = 0;
	float error = 0;
	Mat rel_homography;
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

template<typename T>
vector<T> subvector(vector<T> const& v, int m, int n) {
	auto first = v.begin() + m;
	auto last = v.begin() + n + 1;
	vector<T> vector(first, last);
	return vector;
}

class KPstat 
{
public:
	float avg = -10000, sd;
	string name;
	vector<int> N;
	vector<float> values;
	vector<float> weight;
	vector<float> avg_dist;
	vector<float> ranked; // ordered by distance to average
	vector<float> w_ranked;

	////ADD FUNCTIONS
	KPstat();
	KPstat(string text) {
		name = text;
	}
	void initial_normal_distribution(bool show = false) {
		avg = 0; sd = 0; float Z = 0;
		int n = values.size();

		for (int i = 0; i < n; i++) {
			if (values[i] > -1000 && values[i] < 1000) {
				avg = avg + min((float)10000, (values[i] * weight[i]));
				Z = Z + weight[i];
			}
		}
		avg = avg / Z;

		for (int i = 0; i < n; i++) {
			if (values[i] > -1000 && values[i] < 1000) sd = sd + weight[i]*(values[i] - avg) * (values[i] - avg);
		}
		sd = sqrt(sd / (Z - 1.0));

		if (show) cout << "\n Normal distribution --> " << name << " ~ N ( " << avg << ", " << sd << " ),  from " << n << " values.";
	}
	int normal_distribution(bool show = false) {
		avg = 0; sd = 0; float Z = 0;
		int n = ranked.size();
		if (n == 0) {
			cout << "\n WARNING: Data still in raw state. Performing instead <<initial normal distribution>>.";
			initial_normal_distribution(show);
			return 0;
		}

		for (int i = 0; i < n; i++) {
			if (ranked[i] > -1000 && ranked[i] < 1000) {
				avg = avg + min((float)10000, (ranked[i] * w_ranked[i]));
				Z = Z + w_ranked[i];
			}
		}
		avg = avg / Z;

		for (int i = 0; i < n; i++) {
			if (ranked[i] > -1000 && ranked[i] < 1000) sd = sd + w_ranked[i] * (ranked[i] - avg) * (ranked[i] - avg);
		}
		sd = sqrt(sd / (Z - 1));

		if (show) cout << "\n Normal distribution --> " << name << " ~ N ( " << avg << ", " << sd << " ),  from " << n << " values.";
		return 0;
	}
	void weighted_average() {
		avg = 0; float Z = 0;
		int n = values.size();

		for (int i = 0; i < n; i++) {
			avg = avg + (values[i] * weight[i]);
			Z = Z + weight[i];
		}
		avg = avg / Z;
	}
	void initial_orderbydistancetoX() {
		int n = values.size();
		ranked = { values[0] }, avg_dist = { abs({ values[0] - avg }) }; N = { 0 };
		w_ranked = { weight[0] };

		int vmax = 1;
		for (int i = 1; i < n; i++) {
			float dist = min((float)1000, abs(values[i] - avg));
			if (dist >= avg_dist.back()) {
				ranked.push_back(values[i]);
				avg_dist.push_back(dist);
				N.push_back(i);
				w_ranked.push_back(weight[i]);
				vmax++;
			}
			else if (dist <= avg_dist[0]) {
				ranked.insert(ranked.begin(), values[i]);
				avg_dist.insert(avg_dist.begin(), dist);
				N.insert(N.begin(), i);
				w_ranked.insert(w_ranked.begin(), weight[i]);
				vmax++;
			}
			else
				for (int j = 1; j <= vmax; j++) {
					if (dist < avg_dist[j]) {
						ranked.insert(ranked.begin() + j, values[i]);
						avg_dist.insert(avg_dist.begin() + j, dist);
						N.insert(N.begin() + j, i);
						w_ranked.insert(w_ranked.begin() + j, weight[i]);
						vmax++;
						break;
					}
				}
		}
	}
	void orderbydistancetoX() {
		int n = ranked.size();
		vector<float> ranked2; vector<float> w_ranked2; vector<int> N2;
		ranked2 = { ranked[0] }, avg_dist = { abs({ ranked[0] - avg }) }; N2 = { N[0] };
		w_ranked2 = { w_ranked[0] };

		int vmax = 1;
		for (int i = 1; i < n; i++) {
			float dist = min((float)1000, abs(ranked[i] - avg));
			if (dist >= avg_dist.back()) {
				ranked2.push_back(ranked[i]);
				avg_dist.push_back(dist);
				N2.push_back(N[i]);
				w_ranked2.push_back(w_ranked[i]);
				vmax++;
			}
			else if (dist <= avg_dist[0]) {
				ranked2.insert(ranked2.begin(), ranked[i]);
				avg_dist.insert(avg_dist.begin(), dist);
				N2.insert(N2.begin(), N[i]);
				w_ranked2.insert(w_ranked2.begin(), w_ranked[i]);
				vmax++;
			}
			else
				for (int j = 1; j <= vmax; j++) {
					if (dist < avg_dist[j]) {
						ranked2.insert(ranked2.begin() + j, ranked[i]);
						avg_dist.insert(avg_dist.begin() + j, dist);
						N2.insert(N2.begin() + j, N[i]);
						w_ranked2.insert(w_ranked2.begin() + j, w_ranked[i]);
						vmax++;
						break;
					}
				}
		}
	}
	bool destroy_outliers(float rate = 0.1, float tol = 1) {
		if (avg == -10000) initial_normal_distribution();
		if (ranked.empty()) initial_orderbydistancetoX();
		else orderbydistancetoX();
		int n = ranked.size();
		int good = (float)n * (1.0 - rate);

		// Check if we are eliminating good points
		bool unsure = true;
		while (unsure && good<n){
			float deviation = abs(ranked[good] - avg);
			float range = tol * sd;
			if (deviation > range) unsure = false;
			else good++;
		}
	
		bool too_much = false;
		if (good < n - 1) {
			ranked = subvector(ranked, 0, good);
			w_ranked = subvector(w_ranked, 0, good);
			N = subvector(N, 0, good);
		}
		else {
			too_much = true;
		}
		// Return a warning if we do not need to clean more
		return too_much;
	}
	vector<int> findCommonInfo(KPstat S) {
		vector <int> common;
		for (int i = 0; i < N.size(); i++) {
			for (int j = 0; j < S.N.size(); j++) {
				if (N[i] == S.N[j]) {
					common.push_back(N[i]);
					break;
				}
			}
		}
		return common;


	};
	KPstat useCommonInfo(KPstat S) {
		vector <int> commonN;
		vector <float> Sranked;
		ranked.clear();
		for (int i = 0; i < N.size(); i++) {
			for (int j = 0; j < S.N.size(); j++) {
				if (N[i] == S.N[j]) {
					commonN.push_back(N[i]);
					ranked.push_back(values[N[i]]);
					Sranked.push_back(S.values[N[i]]);
					w_ranked.push_back(weight[N[i]]);
					break;
				}
			}
		}
		N = commonN;
		S.ranked = Sranked;
		S.N = commonN;
		S.w_ranked = w_ranked;
		return S;
	}
	void copyProcessedInfo(KPstat S) {
		N = S.N;
		ranked = S.ranked;
		w_ranked = S.w_ranked;
	}
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
	Point2f circle_rel; float circle_dZ;
	Point2f circle_abs; float circle_Z;

	picture() {};

	void get_rel_circle(float dx, float dy, float dz) {
		circle_rel.x = dx;
		circle_rel.y = dy;
		circle_dZ = dz;
	}
	void rel2abs_circle() {
		circle_abs = captured_from.abs_centre + circle_rel;
		circle_Z = circle_dZ * captured_from.z;
	}

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

//pair<int, int> cirRangeCalc(float Z, float focalLength, float realR, float error = 0.1) {
//	float R = (realR / Z) * focalLength;
//	pair<int, int> R_range;
//	R_range.first = R - error;
//	R_range.second = R + error;
//	return R_range;
//}

int test(Mat object, Mat image);
int test2(Mat object, Mat image);
int test3(vector<Mat> pics);


