// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

using namespace std;
using namespace cv;

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

//Create night sky
Mat firmament(Mat pic, double wanted_ratio) {
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

	std::cout << "\n\n>> Constellations created: \n    The mass of the image is: " << M.m00 << "\n    The star-sky ratio is: " << island_ratio << "\n    Threshold: " << local_treshold;
	return bin_pic;
}

//Search for nearby stars
int bodysearch(Mat sky_pic, point centre) {

	return 0;
}

region get_region(Mat light_pic, int mat_side) {
	region captured;
	Mat show_regions = light_pic;
	int m = light_pic.rows / mat_side;
	int n = light_pic.cols / mat_side;

	for (int i = 0; i < mat_side*mat_side; i++) {
		int r = i % mat_side; int s = i / mat_side;
		captured.light_feature[i].picture = light_pic(Rect(r, s, n, m));
		captured.dark_feature[i].picture = shadow_pic(Rect(r, s, n, m));


		//rectangle(show_regions, Rect(r * n, s * m, n, m), 200); //Draw grid
	}
	imshow("sections", show_regions);
	return captured;
}


int first_image(const char* source)
{
	location pos;
	Mat inv_pic_RGB, inv_pic, pic;

	Mat pic_RGB = imread(source, 1);
	resize(pic_RGB, pic_RGB, Size(), 0.4, 0.4);

	//TURN TO GRAYSCALE
	//Mat gray_pic(pic.size(), CV_8U);
	//cvtColor(pic_RGB, pic, CV_BGR2GRAY);

	//REMOVE SOME NOISE
	//pic2 = ex4::remove_SaltPepper(pic, 1);
	medianBlur(pic_RGB, pic_RGB, 3);

	//CREATE INVERTED PICTURE
	bitwise_not(pic_RGB, inv_pic_RGB);

	//CREATE CONSTELLATIONS
	//Find light spots and turn them into stars
	Mat star_pic = dist_transf_slopes(pic_RGB, 150, THRESH_BINARY);
	
	//Remove big shadows
	//Find dark spots and turn them into stars
	//Mat dark_pic = dist_transf_slopes(inv_pic_RGB, 150, THRESH_BINARY);
	//Mat sky_pic;
	//star_pic.convertTo(star_pic, dark_pic.type());
	//add(star_pic, dark_pic, sky_pic);


	//GET INFO
	region data = get_region(star_pic, 6);

	//SHOW IMAGES IN NEW WINDOWS 
	imshow("random region", Mat(data.light_feature[5].picture));
	//imshow("dark_pic", Mat(dark_pic));
	//imshow("sky_pic", Mat(sky_pic));

	cv::waitKey();
	return 0;
}

