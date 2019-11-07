// Disable depreciation warning
#pragma warning(disable : 4996)

#include "mars.h"
#include "rover.h"

using namespace std;
using namespace cv;


//Calculate mass
float bin_m00(Mat img) {
	float mass = 0;
	for (int y = 0; y < img.rows; y++)
	{
		for (int x = 0; x < img.cols; x++)
		{
			if (img.at<uchar>(y, x) > 0) mass++;
		}
	}
	return mass;
}

//Count objects (binary)
int count_objects(Mat img) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<Vec4i> hierarchy;
	cv::findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	return contours.size();
}

//Get star locations
vector<Point> locate_stars(Mat sky)
{
	vector<Point> stars;
	findNonZero(sky, stars);
	return stars;
}

//Create night sky
Mat firmament(Mat pic, double wanted_ratio, int minmass=3) {
	Mat bin_pic;
	float island_ratio = 1; float mass = 0;
	int local_treshold = 0;
	float im_size = pic.rows * pic.cols;
	bool too_much = false;

	Moments M;
	while (island_ratio > wanted_ratio || too_much) {
		
		if (too_much) local_treshold--;
		else local_treshold++;

		// Binarize
		threshold(pic, bin_pic, local_treshold, 255, 0);

		//Calculate moments
		//mass = bin_m00(bin_pic);
		M = moments(bin_pic, 1); //maybe too slow?

		//New island ratio
		island_ratio = float(M.m00) / im_size;
		if (M.m00 < minmass) too_much = true;
		else if (too_much && M.m00 > minmass) break;
	}

	//std::cout << "\n\n>> Constellations created: \n    The mass of the image is: " << M.m00 << "\n    The star-sky ratio is: " << island_ratio << "\n    Threshold: " << local_treshold;
	return bin_pic;
}
Mat firmament_AxBregions(Mat pic, double wanted_ratio, int sky_regions[2], int scope_regions[2], int minmass = 3, int margin = 10)
{
	int A = sky_regions[0]; int B = sky_regions[1];

	Mat full_sky = Mat::zeros(pic.size(), CV_32F);
	int m = (pic.rows-2*margin) / A;
	int n = (pic.cols-2*margin) / A;
	for (int i = 0; i < A * B; i++) {
		int r = i % A; int s = i / A;
		Mat partial_sky;
		Mat1b mask(pic.size(), uchar(0));
		rectangle(mask, Rect(r*n+margin, s*m+margin, n, m), Scalar(255), CV_FILLED);
		pic.copyTo(partial_sky, mask);
		partial_sky = firmament(partial_sky, wanted_ratio);
		normalize(partial_sky, partial_sky, 0, 255, NORM_MINMAX);
		bitwise_or(full_sky, partial_sky, full_sky);
	}
	normalize(full_sky, full_sky, 0, 1.0, NORM_MINMAX);
	full_sky.convertTo(full_sky, CV_8UC1);
	threshold(full_sky, full_sky, 0.4, 1.0, THRESH_BINARY);

	A = scope_regions[0]; B = scope_regions[1];

	Mat selective_sky = Mat::zeros(pic.size(), CV_8UC1);;
	m = (pic.rows - 2 * margin) / A;
	n = (pic.cols - 2 * margin) / A;
	for (int i = 0; i < A * B; i++) {
		int r = i % A; int s = i / A;
		Mat partial_sky;
		Mat1b mask(pic.size(), uchar(0));
		rectangle(mask, Rect(r * n + margin, s * m + margin, n, m), Scalar(255), CV_FILLED);
		full_sky.copyTo(partial_sky, mask);
		Moments M = moments(partial_sky);
		if ((M.m00) != 0) {
			vector<Point> search = locate_stars(partial_sky);
			line(selective_sky, search[0], search[0], 255, 1);
		}
	}

	return selective_sky;
}

//Show feature locations
int show_locations(string name, picture p) {
	for (int i = 0; i < p.locations.size(); i++) {
		circle(p.original, p.locations[i], 10, 200, 1, 8);
	}
	imshow(name, p.original);
	return 0;
}

//OLD
/*
region brake_in_AxB_features(Mat light_pic, int A, int B) {
	region captured;
	Mat show_regions = light_pic;
	int m = light_pic.rows / A;
	int n = light_pic.cols / A;

	for (int i = 0; i < A * B; i++) {
		int r = i % A; int s = i / A;
		captured.light_feature[i].picture = light_pic(Rect(r*n, s*m, n, m));
		captured.light_feature[i].M = moments(captured.light_feature[i].picture, 1);
		captured.light_feature[i].o_x = captured.light_feature[i].M.m10 / captured.light_feature[i].M.m00;
		captured.light_feature[i].o_y = captured.light_feature[i].M.m01 / captured.light_feature[i].M.m00;
		captured.pos_x = r; captured.pos_y = s;

		//rectangle(show_regions, Rect(r * n, s * m, n, m), 200); //Draw grid
	}
	cv::imshow("sections", show_regions);
	return captured;
}

location fit_feature(feature ft, Mat image)
{
	int expected_rotation = 90; int variance = 1.5;
	int trans_step = 10; int rot_step = 5; int sideways_range = 40;
	
	//Initialize position
	location found;
	Mat stuff = ft.picture;
	double index_stuff = (ft.M.nu20 + ft.M.nu02) / ft.M.m00;
	double stuff_angle = ((atan2(double(2 * ft.M.nu11), double(ft.M.nu20 - ft.M.nu02)))/2)*180/3.1415 + 90;
	int n = stuff.rows; int m = stuff.cols;
	float ro_x = (m - 1.0) / 2.0; float ro_y = (n - 1.0) / 2.0;
	double radius = sqrt(((m - 1.0) / 2.0)* ((m - 1.0) / 2.0) + ((n - 1.0) / 2.0)* ((n - 1.0) / 2.0));
	Point2f centre(radius, radius);

	for (int i = radius; i < image.cols-radius; i=i+trans_step) {
		for (int j = radius; j < image.rows-radius; j=j+trans_step) {
			//circle for rotation
			//int i = 799; int j = 107;

			Mat1b mask(image.size(), uchar(0));
			circle(mask, Point(i, j), radius, Scalar(255), CV_FILLED);
			// Compute the bounding box
			Rect bbox(i, j, 2 * radius, 2 * radius);
			// Create a black image
			//Mat res;
			// Copy only the image under the white circle to black image
			//image.copyTo(res, mask);
			Mat area(image.rows + 2 * radius, image.cols + 2 * radius, image.type());
			image.copyTo(area(cv::Rect(radius, radius, image.cols, image.rows)), mask);

			// Crop according to the roi
			area = area(bbox);
			Moments areaM = moments(area);
			double indexA = (areaM.nu20 + areaM.nu02) / areaM.m00;
			//cout << "index feature: " << index_stuff << "   index section: " << indexA;

			//Compare invariants
			if (indexA > index_stuff* (1.0 - variance) && indexA < index_stuff* (1.0 + variance)) {
				double area_angle = ((atan2(double(2 * areaM.nu11), double(areaM.nu20 - areaM.nu02))) / 2) * 180 / 3.1415 + 90;
				double delta_angle = stuff_angle - area_angle;
				//cout << "\nobject angle: " << stuff_angle << "  area angle: " << area_angle << "  delta angle: " << delta_angle;

				// Compare angles
				if (abs(delta_angle) < expected_rotation) {
					for (int k = delta_angle - sideways_range; k < delta_angle + sideways_range+1; k = k + rot_step) {
						double angle = k;

						// get rotation matrix for rotating the image around its center in pixel coordinates
						cv::Mat rot = cv::getRotationMatrix2D(centre, angle, 1.0);
						Mat rotated;
						warpAffine(area, rotated, rot, area.size());
						Mat comparing = rotated(Rect(radius - ro_x, radius - ro_y, m, n));
						Mat error;
						bitwise_xor(comparing, stuff, error);
						float difference = abs(bin_m00(error));

						if (difference < found.error) {
							found.x = i; found.y = j; found.angle = angle;
							found.error = difference; found.overlap = error;
							cout << "\nError: " << found.error << "   x: " << found.x << "   y: " << found.y << "   rho: " << found.angle;
						}
					}
				}
			}
		}
	}
	return found;
}

int first_image(const char* source)
{
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
	region data = brake_in_AxB_features(star_pic, 6, 6);

	fit_feature(data.light_feature[5], star_pic);


	//SHOW IMAGES IN NEW WINDOWS 
	cv::imshow("random region", Mat(data.light_feature[5].picture));
	//imshow("dark_pic", Mat(dark_pic));
	//imshow("sky_pic", Mat(sky_pic));

	cv::waitKey();
	return 0;
}

int try_fit_feature(Mat object, Mat image) {
	feature thing;

	cvtColor(object, object, CV_BGR2GRAY);
	bitwise_not(object, object);
	cvtColor(image, image, CV_BGR2GRAY);
	bitwise_not(image, image);
	imshow("searching", object);

	thing.picture = object;
	thing.M = moments(thing.picture);
	thing.o_x = thing.M.m10 / thing.M.m00;
	thing.o_y = thing.M.m01 / thing.M.m00;
	
	location spot = fit_feature(thing, image);
	rectangle(image, Rect(spot.x - thing.o_x, spot.y-thing.o_y, 2* thing.o_x, 2* thing.o_y), 200); //Draw finding
	imshow("found", image);
	imshow("overlap", spot.overlap);

	cvWaitKey();

	return 0;
}
*/

picture preprocessing(Mat pic, int sky_regions[2], int scope_regions[2])
{
	//Mat pic = imread(source, 1);
	resize(pic, pic, Size(), 0.4, 0.4);

	//TURN TO GRAYSCALE
	//Mat gray_pic(pic.size(), CV_8U);
	cvtColor(pic, pic, CV_BGR2GRAY);

	//REMOVE SOME NOISE
	//pic2 = ex4::remove_SaltPepper(pic, 1);
	medianBlur(pic, pic, 9);

	//LAPLACIAN FILTER
	Mat aux, dif;
	Laplacian(pic, aux, CV_16S, 5);
	convertScaleAbs(aux, dif);
	aux.release();

	//CREATE INVERTED PICTURE
	//bitwise_not(pic_RGB, inv_pic_RGB);

	//CREATE CONSTELLATIONS
	threshold(dif, dif, 150, 255, THRESH_BINARY | THRESH_OTSU);
	// Perform the distance transform algorithm
	Mat dist;
	distanceTransform(dif, dif, 2, 3);
	//Find light spots and turn them into stars
	Mat star_pic;

	star_pic = firmament_AxBregions(dif, 0, sky_regions, scope_regions);
	//normalize(star_pic, star_pic, 0, 1.0, NORM_MINMAX);
	//star_pic.convertTo(star_pic, CV_8UC1);
	//threshold(star_pic, star_pic, 0.4, 1.0, THRESH_BINARY);

	//TRANSFER DATA TO STRUCTURE
	picture p;
	p.diferential = dif;
	p.original = pic;
	p.starfield = star_pic;

	//GET STAR LOCATIONS
	p.locations = locate_stars(star_pic);

	//Show stuff
	//imshow("pic", pic);
	//imshow("dif", dif);
	//imshow("spots", star_pic);

	return p;
}


int test(Mat pic1, Mat pic2) {
	picture p1, p2;
	int sky_regions[2] = { 3, 3 };
	int scope_regions[2] = { 6, 6 };
	p1 = preprocessing(pic1, sky_regions, scope_regions);
	p2 = preprocessing(pic2, sky_regions, scope_regions);
	show_locations("p1", p1);
	show_locations("p2", p2);

	cvWaitKey();
	return 0;
}