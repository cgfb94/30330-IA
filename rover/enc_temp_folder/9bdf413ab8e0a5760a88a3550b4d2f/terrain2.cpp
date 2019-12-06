#include <opencv2/opencv.hpp>
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/gpu.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core.hpp"

using namespace std;
using namespace cv;
//using namespace xfeatures2d;

int test2(Mat pic1, Mat pic2) {
	resize(pic1, pic1, Size(), 0.4, 0.4); resize(pic2, pic2, Size(), 0.4, 0.4);
	if (pic1.channels() == 3)  cvtColor(pic1, pic1, CV_BGR2GRAY);
	if (pic2.channels() == 3)  cvtColor(pic2, pic2, CV_BGR2GRAY);
	
	int blur_ker = 3; int med_blur_ker = 3;
	blur(pic1, pic1, Size(blur_ker, blur_ker)); blur(pic2, pic2, Size(blur_ker, blur_ker));
	medianBlur(pic1, pic1, med_blur_ker); medianBlur(pic2, pic2, med_blur_ker);
	
	int method = 0;

	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;

	cv::ORB orb;

	//FEATURE DETECTION AND DESCRIPTION

	cv::OrbFeatureDetector detector(100, 1.5f, 4, 40, 0, 2, ORB::FAST_SCORE, 40);
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

	
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
	std::vector< DMatch > matches;
	matcher->match(descriptors1, descriptors2, matches);
	
	double max_dist = 0; double min_dist = 120;

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

	for (int i = 0; i < descriptors1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches(pic1, keypoints1, pic2, keypoints2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	imshow("Good Matches", img_matches);

	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
	}


	cv::waitKey();
	

	return 0;
}