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

Mat translateImg(Mat& img, int offsetx, int offsety) {
	Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	warpAffine(img, img, trans_mat, img.size());
	return img;
}

picture newpic_relpos(picture previous, Mat pic2, int n_kp = 7, int method = 1) {

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
	//pos.centre = abs_vec[0];
	/*for (int i = 0; i < 4; i++) {
		pos.corner[i] = abs_vec[i+1] - pos.centre;
	}*/
	pos.dz = (euclideanDist(goodKP2[0], goodKP2[1]) / euclideanDist(goodKP1[0], goodKP1[1])); //* previous.captured_from.z
	float a1 = atan2(goodKP1[1].y - goodKP1[0].y, goodKP1[1].x - goodKP1[0].x);
	float a2 = atan2(goodKP2[1].y - goodKP2[0].y, goodKP2[1].x - goodKP2[0].x);
	pos.d_angle = a1 - a2;


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
	// ... then, compare both and get relative displacement between keypoints:
	// Loop to verify results given by different keypoint couples:------------
	pos.traslation.x = 0; pos.traslation.y = 0; float N = goodKP2.size();
	for (int k = 0; k < N; k++) {
		pos.traslation.x = pos.traslation.x + (goodKP2[k].x - aux2[k].x);
		pos.traslation.y = pos.traslation.y + (goodKP2[k].y - aux2[k].y);
	}
	pos.traslation.x = pos.traslation.x / N;
	pos.traslation.y = pos.traslation.y / N;
	pos.step_distance = euclideanDist(Point2f(0, 0), pos.traslation);
	// endloop----------------------------------------------------------------

	// Get the correct relative measurements watching the loop results

	// Using relative measurements compute absolute location
	pos.z = (1/pos.dz) * previous.captured_from.z;
	pos.angle = pos.d_angle + previous.captured_from.angle;
	pos.abs_centre = pos.traslation + previous.captured_from.abs_centre;
	pos.rel_homography = H;
	pos.error = previous.captured_from.error;

	// Return:
	newpic.captured_from = pos;
		return newpic;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
	Point2f centre = { maxX + diag/2, maxY + diag/2 };

	Mat map(Size((int)centre.x * 2, (int)centre.y * 2), piece[0].original.type(), Scalar(0));
	//Mat map_see = cv::Mat::zeros(map.size(), CV_8U);
	
	Point2f centre_aux = centre;

	for (int j = 0; j < 2; j++) {
		cout << "\n\nFRAME " << j;
		Mat g = piece[j].original;
		Mat rot_aux = cv::Mat::zeros(g.size(), g.type());
		Mat map_aux = cv::Mat::zeros(map.size(), g.type());
		Mat map_mask = cv::Mat::zeros(map.size(), CV_8U);
		centre_aux = centre - piece[j].captured_from.abs_centre;

		Mat rot = cv::getRotationMatrix2D(Point2f(g.rows/2, g.cols/2), -piece[j].captured_from.angle, piece[0].captured_from.z / piece[j].captured_from.z);
		warpAffine(g, rot_aux, rot, g.size());

		std::vector<Point2f> rel_vec =
		{
			Point2f((float)0, (float)0),
			Point2f((float)-g.cols / 2, (float)-g.rows / 2),
			Point2f((float)g.cols / 2, (float)-g.rows / 2),
			Point2f((float)g.cols / 2, (float)g.rows / 2),
			Point2f((float)-g.cols / 2, (float)g.rows / 2)
		};

		for (int k = 0; k < rel_vec.size(); k++) {
			abs_points[j][k].x = rot.at<double>(0, 0) * rel_vec[k].x + rot.at<double>(0, 1) * rel_vec[k].y + rot.at<double>(0, 2) + centre_aux.x;
			abs_points[j][k].y = rot.at<double>(1, 0) * rel_vec[k].x + rot.at<double>(1, 1) * rel_vec[k].y + rot.at<double>(1, 2) + centre_aux.y;
		}

		Rect r = Rect(abs_points[j][1].x, abs_points[j][1].y, rot_aux.cols, rot_aux.rows);
		rectangle(map, r, 255, 1);
		rot_aux.copyTo(map(r));
		
		//warpPerspective(g, map_aux, piece[j].captured_from.rel_homography, map.size());
		//map_mask(Rect(a2[0], a2[2])) = 255;
		//map_aux.convertTo(map_aux, map.type());
		//map_aux.copyTo(map, map_mask);
		
		line(map, abs_points[j][1], abs_points[j][1], 255, 1); line(map, abs_points[j][2], abs_points[j][3], 255, 1);
		line(map, abs_points[j][3], abs_points[j][4], 255, 1); line(map, abs_points[j][4], abs_points[j][1], 255, 1);
		circle(map, centre_aux, 5, Scalar(255), 2);
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

int test3(Mat pic1, Mat pic2) {
	resize(pic1, pic1, Size(), 0.4, 0.4); resize(pic2, pic2, Size(), 0.4, 0.4);
	cout << "\n >> Picture size: " << pic1.cols << " x " << pic1.rows << "\n";
	if (pic1.channels() == 3)  cvtColor(pic1, pic1, CV_BGR2GRAY);
	if (pic2.channels() == 3)  cvtColor(pic2, pic2, CV_BGR2GRAY);

	vector<picture> frame(1);

	//INITIALIZE ORIGIN
	frame[0].original = pic1;
	frame[0].captured_from.error = 0;
	frame[0].captured_from.z = 20; //mm
	//cout << "\nFrame (0): ORIGIN -->  dX = " << frame[0].captured_from.xo << ", dY = " << frame[0].captured_from.yo << ", dZ = " << frame[0].captured_from.z << "; dAngle =  " << frame[0].captured_from.angle;

	// COMPUTE NEXT PICTURE

	//newframe_relpos(frame[0], pic2);

	frame.push_back(newpic_relpos(frame[0], pic2));
	cout << "\nFrame (" << 1 << ") moved " << frame[1].captured_from.step_distance << "units -->  dX = " << frame[1].captured_from.traslation.x << ", dY = " << frame[1].captured_from.traslation.y << ", dZ = " << frame[1].captured_from.dz << "; dAngle =  " << frame[1].captured_from.angle;
	cout << "\n                                   X = " << frame[1].captured_from.abs_centre.x << ",  Y = " << frame[1].captured_from.abs_centre.y << ",  Z = " << frame[1].captured_from.z;
	display_map(frame);
	
	waitKey();
	return 0;
}