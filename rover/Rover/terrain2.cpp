// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"

using namespace std;
using namespace cv;

/*

float weighted_average(vector<float> values, vector<float> weights) {
	float avg = 0; float Z = 0;
	int n = values.size();

	for (int i = 0; i < n; i++) {
		avg = avg + (values[i] * weights[i]);
		Z = Z + weights[i];
	}
	avg = avg / Z;

	return avg;
}

KPstat orderbydistancetoX(vector<float> values, float avg) {
	int n = values.size();

	KPstat S;
	S.avg = avg; S.values = values;
	S.ranked = { values[0] }, S.avg_dist = { abs({ values[0] - avg }) }; S.N = { 0 };
	
	for (int i = 1; i < n; i++) {
		float dist = values[i] - avg;
		if (dist >= S.avg_dist.back()) {
			S.ranked.push_back(values[i]);
			S.avg_dist.push_back(dist);
			S.N.push_back(i);
		}
		else if (dist <= S.avg_dist[0]) {
			S.ranked.insert(S.ranked.begin(), values[i]);
			S.avg_dist.insert(S.avg_dist.begin(), dist);
			S.N.insert(S.N.begin(), i);
		}
	}

	return S;
}

Mat translateImg(Mat& img, int offsetx, int offsety) {
	Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	warpAffine(img, img, trans_mat, img.size());
	return img;
}

*/
/*

int test2(Mat pic1_0, Mat pic2_0) {
	resize(pic1_0, pic1_0, Size(), 0.4, 0.4); resize(pic2_0, pic2_0, Size(), 0.4, 0.4);
	if (pic1_0.channels() == 3)  cvtColor(pic1_0, pic1_0, CV_BGR2GRAY);
	if (pic2_0.channels() == 3)  cvtColor(pic2_0, pic2_0, CV_BGR2GRAY);

	Mat pic1, pic2;
	
	int blur_ker = 3; int med_blur_ker = 3;
	blur(pic1_0, pic1, Size(blur_ker, blur_ker)); blur(pic2_0, pic2, Size(blur_ker, blur_ker));
	medianBlur(pic1, pic1, med_blur_ker); medianBlur(pic2, pic2, med_blur_ker);
	
	int method = 0;

	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;

	cv::ORB orb;

	//FEATURE DETECTION AND DESCRIPTION

	cv::OrbFeatureDetector detector(200, 1.2f, 8, 50, 0, 2, ORB::FAST_SCORE, 50);
	/*1 - The first parameter tells the extractor to only use the top 25 results from the detector.For a reliable estimation of an 8 DOF homography with no constraints on parameters,
		  you should have an order of magnitude more features than parameters, i.e. 80, or just make it an even 100.
	  2 - The second parameter is for scaling the images down(or the detector patch up) between octaves(or levels). using the number 1.0f means you don't change the scale between
		  octaves, this makes no sense, especially since your third parameter is the number of levels which is 2 and not 1. The default is 1.2f for scale and 8 levels, for less
		  calculations, use a scaling of 1.5f and 4 levels (again, just a suggestion, other parameters will work too).
	  3 - your fourth and last parameters say that the patch size to calculate on is 10x10, that's pretty small, but if you work on low resolution that's fine.
	  4 - your score type(one before last parameter) can change runtime a bit, you can use the ORB::FAST_SCORE instead of the ORB::HARRIS_SCORE but it doesn't matter much.*/
/*

	cv::OrbDescriptorExtractor extractor;

	//-- object
	if (method == 0) { //-- ORB
		orb.detect(pic1, keypoints1);
		orb.compute(pic1, keypoints1, descriptors1);
	}
	else { //-- SURF test
		detector.detect(pic1, keypoints1);
		extractor.compute(pic1, keypoints1, descriptors1);
	}
	
	if (method == 0) { //-- ORB
		orb.detect(pic2, keypoints2);
		orb.compute(pic2, keypoints2, descriptors2);
	}
	else { //-- SURF
		detector.detect(pic2, keypoints2);
		extractor.compute(pic2, keypoints2, descriptors2);
	}

	// MATCHING
	std::vector< DMatch > matches;

	if (method == 1) {
		BFMatcher matcher = BFMatcher(NORM_HAMMING, false);
		matcher.match(descriptors1, descriptors2, matches);
	}
	else {
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
		matcher->match(descriptors1, descriptors2, matches);
	}


	
	double max_dist = 0; double min_dist = 2000;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);


	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< DMatch > good_matches;
	float filter = 1.02; int counter = 0;
	while (good_matches.size() < 2) {
		filter = filter + 0.02;
		good_matches.clear();
		for (int i = 0; i < descriptors1.rows; i++)
		{
			if (matches[i].distance <= max(filter * min_dist, 0.02))
			{
				good_matches.push_back(matches[i]);
			}
		}
		counter++; if (counter > 100) break;
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches(pic1_0, keypoints1, pic2_0, keypoints2,
		good_matches, img_matches, Scalar::all(255), Scalar::all(255),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	imshow("Good Matches", img_matches);

	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d :  distance --> %f \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx, good_matches[i].distance);
	}

	cv::waitKey();


	// CORRECT POSSIBLE POSITION ERRORS

	
	return 0;
}
int test3(vector<Mat> pic) {
	int n = pic.size();

	// 0 - (LOOP) PREPROCESSING
	for (int i = 0; i < n; i++) {
		resize(pic[i], pic[i], Size(), 0.4, 0.4);
		if (pic[i].channels() == 3)  cvtColor(pic[i], pic[i], CV_BGR2GRAY);
	}
	cout << "\n >> Picture size: " << pic[0].cols << " x " << pic[0].rows << "\n";

	vector<picture> frame(1);

	// 1 - INITIALIZE ORIGIN
	frame[0].original = pic[0];
	frame[0].captured_from.error = 0;
	frame[0].captured_from.z = 20; //mm
	//cout << "\nFrame (0): ORIGIN -->  dX = " << frame[0].captured_from.xo << ", dY = " << frame[0].captured_from.yo << ", dZ = " << frame[0].captured_from.z << "; dAngle =  " << frame[0].captured_from.angle;

	// 2 - (LOOP) COMPUTE NEXT PICTURES - RELATIVE AND ABSOLUTE POSITIONS
	for (int i = 1; i < n; i++) {
		cout << "\n\n>> FRAME " << i;
		frame.push_back(newpic_relpos(frame[i - 1], pic[i]));
		cout << "\nFrame (" << i << ") moved " << frame[i].captured_from.step_distance << "units -->  dX = " << frame[i].captured_from.traslation.x << ", dY = " << frame[i].captured_from.traslation.y << ", dZ = " << frame[i].captured_from.dz << "; dAngle =  " << frame[i].captured_from.d_angle;
		cout << "\n                                   X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;

	}

	// 3 - DISPLAY COLLAGE
	cout << "\n\n DRAWING AREA MAP:";
	display_map(frame);
	
	waitKey();
	return 0;
}

*/