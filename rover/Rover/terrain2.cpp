// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/gpu.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
//#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
//using namespace xfeatures2d;

pair<float, float> normal_distribution(vector<float> values, vector<float> weights) {
	float avg = 0; float sd = 0; float Z = 0;
	int n = values.size();

	for (int i = 0; i < n; i++) {
		avg = avg + (values[i] * weights[i]);
		Z = Z + weights[i];
	}
	avg = avg / Z;

	for (int i = 0; i < n; i++) {
		sd = sd + (values[i] - avg) * (values[i] - avg);
	}
	sd = sqrt(sd / (Z - 1));

	pair<float, float> N{ avg, sd };
	return N;
}

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

picture newpic_relpos(picture previous, Mat pic2, int n_kp = 15, int method = 1) {

	method = 1;

	Mat pic1 = previous.original;

	picture newpic;
	newpic.original = pic2;

	// PREPROCESSING - FILTERS
	Mat pic_match1; Mat pic_match2;
	int blur_ker = 3; int med_blur_ker = 3; //int diff_ker = 3;
	blur(pic1, pic_match1, Size(blur_ker, blur_ker)); blur(pic2, pic_match2, Size(blur_ker, blur_ker));
	medianBlur(pic_match1, pic_match1, med_blur_ker); medianBlur(pic_match2, pic_match2, med_blur_ker);
	//Laplacian(pic_match1, pic_match1, CV_16S, diff_ker); Laplacian(pic_match2, pic_match2, CV_16S, diff_ker);

	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;

	cv::ORB orb(800, 1.2f, 10, 30, 0, 2, ORB::FAST_SCORE, 30);
	/*
	  1 - The first parameter tells the extractor to only use the top 25 results from the detector.For a reliable estimation of an 8 DOF homography with no constraints on parameters,
		  you should have an order of magnitude more features than parameters, i.e. 80, or just make it an even 100.
	  2 - The second parameter is for scaling the images down(or the detector patch up) between octaves(or levels). using the number 1.0f means you don't change the scale between
		  octaves, this makes no sense, especially since your third parameter is the number of levels which is 2 and not 1. The default is 1.2f for scale and 8 levels, for less
		  calculations, use a scaling of 1.5f and 4 levels (again, just a suggestion, other parameters will work too).
	  3 - your fourth and last parameters say that the patch size to calculate on is 10x10, that's pretty small, but if you work on low resolution that's fine.
	  4 - your score type(one before last parameter) can change runtime a bit, you can use the ORB::FAST_SCORE instead of the ORB::HARRIS_SCORE but it doesn't matter much.
	*/

	//FEATURE DETECTION AND DESCRIPTION

	cv::OrbDescriptorExtractor extractor;

	orb.detect(pic_match1, keypoints1);
	orb.compute(pic_match1, keypoints1, descriptors1);
	orb.detect(pic_match2, keypoints2);
	orb.compute(pic_match2, keypoints2, descriptors2);

	// MATCHING
	std::vector< DMatch > matches;

	if (method == 1) {
		BFMatcher matcher = BFMatcher(NORM_HAMMING, false);
		matcher.match(descriptors1, descriptors2, matches);
	}
	//else if (method == 2) {
	//	/*FlannBasedMatcher matcher;
	//	std::vector< DMatch > matches;
	//	matcher.match(descriptors1, descriptors2, matches);*/
	//}
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

	printf("\nRelative position computation:");
	printf("\n-- Max dist : %f", max_dist);
	printf("\n-- Min dist : %f", min_dist);

	std::vector< DMatch > good_matches;
	float filter0 = 0, filter = 1.005; int counter = 0;
	while (good_matches.size() < n_kp) {
		//good_matches.clear();
		for (int i = 0; i < descriptors1.rows; i++)
		{
			if (matches[i].distance < max(filter * min_dist, 0.02)  && matches[i].distance >= filter0* min_dist)
			{
				good_matches.push_back(matches[i]);
			}
		}
		counter++; if (counter > 400) break;
		filter0 = filter;
		filter = filter + 0.005;
	}

	//-- Localize the object
	vector<Point2f> goodKP1;
	vector<Point2f> goodKP2;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		printf("\n-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d :  distance --> %f ", i, good_matches[i].queryIdx, good_matches[i].trainIdx, good_matches[i].distance);
		goodKP1.push_back(keypoints1[good_matches[i].queryIdx].pt);
		goodKP2.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}

	//Homography stuff :(
	Mat H = findHomography(goodKP2, goodKP1, CV_RANSAC);
	/*
	//Mat H = getPerspectiveTransform(goodKP2, goodKP1);

	std::vector<Point2f> rel_vec =
	{
		Point2f((float)pic2.cols / 2, (float)pic2.rows / 2),
		Point2f(0, 0),
		Point2f((float)pic2.cols, 0),
		Point2f((float)pic2.cols, (float)pic2.rows),
		Point2f(0, (float)pic2.rows)
	};

	std::vector<Point2f> abs_vec(5);
	//perspectiveTransform(rel_vec, abs_vec, H);

	
	for (int i = 0; i < 5; i++) {
		circle(pic2, rel_vec[i], 10, Scalar(255), 2);
		if(abs_vec[i].x<=pic1.cols && abs_vec[i].y <= pic1.rows) circle(pic1, abs_vec[i], 10, Scalar(255), 2);
	}
	*/

	//-- Show detected matches
	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches(pic1, keypoints1, pic2, keypoints2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("Good Matches & Object detection", img_matches);


	// CALCULATE ROTATION AND POSITION DIFFERENCE
	location pos;

	// Create matrixes to store all computations
	int nk = goodKP1.size();
	vector<float> wC, w;
	KPstat AZ("AZ"), Aangle("Aangle"), AX("AX"), AY("AY");

	// Loop to verify zoom and rotation given by different keypoint couples:------------
	int nit1 = 5, nit2 = 5;
	bool show1 = true, show2 = true;
	float numerator = 10;

	// 1 - Calculate rotations and zoom:
	for (int i = 0; i < nk; i++) {
		for (int j = i+1; j < nk; j++) {
			AZ.weight.push_back(numerator/(good_matches[i].distance *good_matches[i].distance) * numerator/(good_matches[j].distance *good_matches[j].distance));
			AZ.values.push_back((euclideanDist(goodKP2[i], goodKP2[j]) / euclideanDist(goodKP1[i], goodKP1[j])));
			float a1 = atan2(goodKP1[j].y - goodKP1[i].y, goodKP1[j].x - goodKP1[i].x);
			float a2 = atan2(goodKP2[j].y - goodKP2[i].y, goodKP2[j].x - goodKP2[i].x);
			Aangle.values.push_back(a1 - a2);
		}
	}
	Aangle.weight = AZ.weight;

	// 2 - Fit into normal distribution
	AZ.initial_normal_distribution(show1); Aangle.initial_normal_distribution(show1);
	 
	for (int i = 0; i < n_kp; i++) {
		// 3 - Order points by distance to mean --> eliminate furthest
		AZ.destroy_outliers(0.15); Aangle.destroy_outliers(0.15);
		// 4 - Rely on common results
		AZ = Aangle.useCommonInfo(AZ);
		// 5 - repeat until outlier percentage is lower than %
		AZ.normal_distribution(show1); Aangle.normal_distribution(show1);
		cout << "<< { ";
		for (int j = 0; j < AZ.N.size(); j++) {
			cout << AZ.N[j]/n_kp << ", ";
		}
		cout << "}\n";
	}
	// endloop---------------------------------------------------------------------------

	//pos.dz = AZ.avg;
	//pos.d_angle = Aangle.avg;
	//pos.error = max(Aangle.sd, AZ.sd);

	// old %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	pos.dz = (euclideanDist(goodKP2[0], goodKP2[1]) / euclideanDist(goodKP1[0], goodKP1[1])); //* previous.captured_from.z
	float a1 = atan2(goodKP1[1].y - goodKP1[0].y, goodKP1[1].x - goodKP1[0].x);
	float a2 = atan2(goodKP2[1].y - goodKP2[0].y, goodKP2[1].x - goodKP2[0].x);
	pos.d_angle = a1 - a2;
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	// Zoom and rotate second image to match second one's characteristics
	Point2f centre((float)pic1.cols / 2, (float)pic1.rows / 2);
	Mat rot = cv::getRotationMatrix2D(centre, pos.d_angle, pos.z);	
	Mat aux; vector<Point2f> aux2(goodKP2.size());
	warpAffine(pic1, aux, rot, pic1.size());
	for (int k = 0; k < goodKP1.size(); k++) {
		aux2[k].x = rot.at<double>(0, 0) * goodKP1[k].x + rot.at<double>(0, 1) * goodKP1[k].y + rot.at<double>(0, 2);
		aux2[k].y = rot.at<double>(1, 0) * goodKP1[k].x + rot.at<double>(1, 1) * goodKP1[k].y + rot.at<double>(1, 2);
		circle(aux, aux2[k], 10, Scalar(255), 2);
	}


	// Loop to verify translation given by different keypoint couples:------------
	//float sfx = 0.81; float sfy = 0.7;
	float sfx = 1; float sfy = 1;

	// 1 - Calculate translations:
	bool getpreviousinfo = false;
	if (getpreviousinfo){
		//Taking into account previously found outliers
		vector<int> common = AZ.findCommonInfo(Aangle);
		for (int i = 0; i < common.size(); i++) {
			AX.weight.push_back(numerator / (good_matches[common[i]].distance *good_matches[common[i]].distance));
			AX.values.push_back(sfx * max((float)-10000, min((float)10000,  goodKP2[common[i]].x - aux2[common[i]].x)));
			AY.values.push_back(sfy* max((float)-10000, min((float)10000,  goodKP2[common[i]].y - aux2[common[i]].y)));
		}
	}
	else for (int i = 0; i < nk; i++) {
		AX.weight.push_back(numerator / (good_matches[i].distance *good_matches[i].distance));
		AX.values.push_back(sfx * max((float)-10000, min((float)10000, (goodKP2[i].x - aux2[i].x))));
		AY.values.push_back(sfy * max((float)-10000, min((float)10000, (goodKP2[i].y - aux2[i].y))));
		//cout << "\n (" << i << ") --> AX = " << AX.values[i]  << " ||  AY = " << AY.values[i] << "  || W = " << AX.weight[i];
	}
	//AX.weight[0] = AX.weight[0] * 3; AX.weight[1] = AX.weight[1] * 2;
	AY.weight = AX.weight;

	// 2 - Fit into normal distribution
	AX.initial_normal_distribution(show2); AY.initial_normal_distribution(show2);

	for (int i = 0; i <n_kp/1.5; i++) {
		// 3 - Order points by distance to mean --> eliminate furthest
		AX.destroy_outliers(0.1); AY.destroy_outliers(0.1);
		// 4 - Rely on common results
		AY = AX.useCommonInfo(AY);
		// 5 - repeat until outlier percentage is lower than %
		AX.normal_distribution(show2); AY.normal_distribution(show2);
		cout << "<< { ";
		for (int j = 0; j < AX.N.size(); j++) {
			cout << AX.N[j] << ", ";
		}
		cout << "}\n";
	}
	

	// endloop----------------------------------------------------------------

	pos.traslation.x = AX.avg;
	pos.traslation.y = AY.avg;
	pos.error = max(pos.error, AX.sd); pos.error = max(pos.error, AY.sd);
	pos.step_distance = euclideanDist(Point2f(0, 0), pos.traslation);

	//old %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
	//pos.traslation.x = (goodKP2[0].x - aux2[0].x);
	//pos.traslation.y = (goodKP2[0].y - aux2[0].y);
	//pos.step_distance = euclideanDist(Point2f(0, 0), pos.traslation);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	// Get the correct relative measurements watching the loop results

	// Using relative measurements compute absolute location


	pos.z = (1/pos.dz) * previous.captured_from.z;
	pos.angle = pos.d_angle + previous.captured_from.angle;
	pos.abs_centre = pos.traslation + previous.captured_from.abs_centre;
	pos.rel_homography = H;
	pos.error = pos.error+previous.captured_from.error;

	// Return:
	newpic.captured_from = pos;
	return newpic;
}



int display_map(vector<picture> piece) {

	int n = piece.size();
	vector<vector<Point2f>> abs_points(n, vector<Point2f>(5));

	float maxX = 0; float maxY = 0;
	int diag = euclideanDist(Point2f(0, 0), Point2f(piece[0].original.rows, piece[0].original.rows));

	for (int i = 0; i < n; i++) {
		int X = abs(piece[i].captured_from.abs_centre.x);
		int Y = abs(piece[i].captured_from.abs_centre.y);
		if (X > maxX) maxX = X;
		if (Y > maxY) maxY = Y;
	}
	Point2f centre = { maxX + (int)(diag/2), maxY + (int)(diag/2) };

	Mat map(Size((int)centre.x * 2, (int)centre.y * 2), piece[0].original.type(), Scalar(0));
	//Mat map_see = cv::Mat::zeros(map.size(), CV_8U);
	
	Point2f centre_aux = centre;

	// DRAW IMAGES
	for (int j = 0; j < n; j++) {
		Mat g = piece[j].original;
		Mat g_mask(g.size(), CV_8U, Scalar(255));
		Mat rot_aux = cv::Mat::zeros(g.size(), g.type());
		Mat map_mask = cv::Mat::zeros(map.size(), CV_8U);

		centre_aux = centre - piece[j].captured_from.abs_centre;
		piece[j].captured_from.map_centre = centre_aux;

		float scale = piece[j].captured_from.z / piece[0].captured_from.z;

		//Rotation and zoom
		Mat H = getRotationMatrix2D(Point2f(g.cols/2, g.rows/2), -piece[j].captured_from.angle, scale);
		//Translation
		H.at<double>(0, 2) = centre_aux.x - g.cols * scale / 2; 
		H.at<double>(1, 2) = centre_aux.y - g.rows * scale / 2;
		warpAffine(g, rot_aux, H, map.size());
		warpAffine(g_mask, map_mask, H, map.size());
		
 		std::vector<Point2f> rel_vec =
		{
			Point2f((float)0, (float)0),
			Point2f((float)-g.cols / 2, (float)-g.rows / 2),
			Point2f((float)g.cols / 2, (float)-g.rows / 2),
			Point2f((float)g.cols / 2, (float)g.rows / 2),
			Point2f((float)-g.cols / 2, (float)g.rows / 2)
		};

		for (int k = 0; k < rel_vec.size(); k++) {
			abs_points[j][k].x = H.at<double>(0, 0) * rel_vec[k].x + H.at<double>(0, 1) * rel_vec[k].y + H.at<double>(0, 2) + g.cols * scale / 2;
			abs_points[j][k].y = H.at<double>(1, 0) * rel_vec[k].x + H.at<double>(1, 1) * rel_vec[k].y + H.at<double>(1, 2) + g.rows * scale / 2;
		}

		rot_aux.copyTo(map, map_mask);
		
		//warpPerspective(g, map_aux, piece[j].captured_from.rel_homography, map.size());
		//map_mask(Rect(a2[0], a2[2])) = 255;
		//map_aux.convertTo(map_aux, map.type());
		//map_aux.copyTo(map, map_mask);
		
		//line(map, abs_points[j][1], abs_points[j][1], 255, 1); line(map, abs_points[j][2], abs_points[j][3], 255, 1);
		//line(map, abs_points[j][3], abs_points[j][4], 255, 1); line(map, abs_points[j][4], abs_points[j][1], 255, 1);
		circle(map, centre_aux, 5, Scalar(255), 2);
	}

	// DRAW VECTORS
	for (int j = 1; j < n; j++) {
		arrowedLine(map, piece[j - 1].captured_from.map_centre, piece[j].captured_from.map_centre, Scalar(255), 2);
		putText(map, "-(" + to_string(j) + ")", piece[j].captured_from.map_centre, FONT_HERSHEY_COMPLEX_SMALL, 1.5, 255, 2);
		putText(map, " " + to_string((int)piece[j].captured_from.step_distance) + " pixels", Point2f((piece[j-1].captured_from.map_centre.x + piece[j].captured_from.map_centre.x)/2, (piece[j - 1].captured_from.map_centre.y + piece[j].captured_from.map_centre.y) / 2), FONT_HERSHEY_COMPLEX_SMALL, 1.1, 255, 2);
		cout << "\n\nFrame " << j << " --> Drawn.";
	}


	resize(map, map, Size(), 0.5, 0.5);
	
	imshow("Full map", map);

	return 0;
}


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