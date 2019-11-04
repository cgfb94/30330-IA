// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

using namespace cv; using namespace std;

float bin_m00(Mat img) {
	float mass = 0;
	for (int y = 0; y < img.rows; y++)
	{
		for (int x = 0; x < img.cols; x++)
		{
			if (img.at<uchar>(y, x) > 150) mass++;
		}
	}
	return mass;
}

//CREATE CONSTELLATIONS:
Mat zodiak(Mat pic, double wanted_ratio) {
	Mat bin_pic;
	float island_ratio = 1; float mass = 0;
	int local_treshold = 0;
	float im_size = pic.rows * pic.cols;

	Moments M;
	while (island_ratio > wanted_ratio) {
		local_treshold++;

		// Binarize
		threshold(pic, bin_pic, local_treshold, 255, 0);

		//Calculate moments
		//mass = bin_m00(bin_pic);
		M = moments(bin_pic, 1); //maybe too slow?

		//New island ratio
		island_ratio = M.m00 / im_size;
	}

	std::cout << "\n\n>> Constellations created: \n    The mass of the image is: " << M.m00 << "\n    The star-sky ratio is: " << island_ratio;
	return bin_pic;
}



int first_image(const char* source)
{
	location pos;
	Mat inv_pic_RGB, inv_pic, pic;

	Mat pic_RGB = imread(source, 1);
	resize(pic_RGB, pic_RGB, Size(), 0.4, 0.4);

	//TURN TO GRAYSCALE
	//Mat gray_pic(pic.size(), CV_8U);
	cvtColor(pic_RGB, pic, CV_BGR2GRAY);

	//REMOVE SOME NOISE
	//pic2 = ex4::remove_SaltPepper(pic, 1);
	medianBlur(pic, pic, 3);

	//CREATE INVERTED PICTURE
	bitwise_not(pic_RGB, inv_pic_RGB);
	bitwise_not(pic, inv_pic);


	//CREATE CONSTELLATIONS
	//Find lightspots and turn them into stars
	Mat star_pic = dist_transf_slopes(pic_RGB, 150, THRESH_BINARY);
	star_pic = zodiak(star_pic, 0.01);
	
	//Remove big shadows
	threshold(inv_pic, inv_pic, 200, 255, THRESH_TOZERO_INV);
	//Find darkspots and turn them into stars
	Mat dark_pic = zodiak(inv_pic, 0.01);
	Mat sky_pic;
	star_pic.convertTo(star_pic, dark_pic.type());
	add(star_pic, dark_pic, sky_pic);

	//SHOW IMAGES IN NEW WINDOWS 
	namedWindow("pic", CV_WINDOW_AUTOSIZE);
	imshow("pic", Mat(pic));
	namedWindow("star_pic", CV_WINDOW_AUTOSIZE);
	imshow("star_pic", Mat(star_pic));

	namedWindow("dark_pic", CV_WINDOW_AUTOSIZE);
	imshow("dark_pic", Mat(dark_pic));
	namedWindow("sky_pic", CV_WINDOW_AUTOSIZE);
	imshow("sky_pic", Mat(sky_pic));
	
	cv::waitKey();
	return 0;
}

