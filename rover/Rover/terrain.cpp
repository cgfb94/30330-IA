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
	Mat inv_pic, pic;

	Mat pic1 = imread(source, 1);
	resize(pic1, pic1, Size(), 0.4, 0.4);

	//TURN TO GRAYSCALE
	//Mat gray_pic(pic.size(), CV_8U);
	cvtColor(pic1, pic, CV_BGR2GRAY);

	//REMOVE SOME NOISE
	//pic2 = ex4::remove_SaltPepper(pic, 1);
	medianBlur(pic, pic, 3);

	//CREATE INVERTED PICTURE
	bitwise_not(pic, inv_pic);


	//CREATE CONSTELLATIONS
	Mat star_pic = zodiak(pic, 0.05);
	/*Mat star_pic;
	threshold(pic, star_pic, 250, 255, THRESH_BINARY | THRESH_OTSU);*/
	threshold(inv_pic, inv_pic, 250, 255, 4);
	Mat dark_pic = zodiak(inv_pic, 0.01);
	Mat sky_pic;
	add(star_pic, dark_pic, sky_pic);

	Mat countries = watershed_regions(pic1, 100, THRESH_BINARY);



	//SHOW IMAGES IN NEW WINDOWS 
	/*namedWindow("pic", CV_WINDOW_AUTOSIZE);
	imshow("pic", Mat(pic));
	namedWindow("star_pic", CV_WINDOW_AUTOSIZE);
	imshow("star_pic", Mat(star_pic));
	namedWindow("dark_pic", CV_WINDOW_AUTOSIZE);
	imshow("dark_pic", Mat(dark_pic));
	namedWindow("sky_pic", CV_WINDOW_AUTOSIZE);
	imshow("sky_pic", Mat(sky_pic));*/
	
	cv::waitKey();
	return 0;
}

