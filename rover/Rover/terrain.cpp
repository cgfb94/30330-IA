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


//OLD
/*
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

region brake_in_AxB_features(Mat light_pic, int A, int B) {
	region captured;
	Mat show_regions = light_pic;
	int m = light_pic.rows / A;
	int n = light_pic.cols / A;

	for (int i = 0; i < A*B; i++) {
		int r = i % A; int s = i / A;
		captured.light_feature[i].picture = light_pic(Rect(r, s, n, m));
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


location solve_puzzle(feature ft, Mat image)
{
	int expected_rotation = 90; int variance = 1.5;
	int trans_step = 10; int rot_step = 5; int sideways_range = 40;

	//Initialize position
	location found;
	Mat stuff = ft.picture;
	double index_stuff = (ft.M.nu20 + ft.M.nu02) / ft.M.m00;
	double stuff_angle = ((atan2(double(2 * ft.M.nu11), double(ft.M.nu20 - ft.M.nu02))) / 2) * 180 / 3.1415 + 90;
	int n = stuff.rows; int m = stuff.cols;
	float ro_x = (m - 1.0) / 2.0; float ro_y = (n - 1.0) / 2.0;
	double radius = sqrt(((m - 1.0) / 2.0) * ((m - 1.0) / 2.0) + ((n - 1.0) / 2.0) * ((n - 1.0) / 2.0));
	Point2f centre(radius, radius);

	for (int i = radius; i < image.cols - radius; i = i + trans_step) {
		for (int j = radius; j < image.rows - radius; j = j + trans_step) {
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
			if (indexA > index_stuff* (1.0 - variance) && indexA < index_stuff * (1.0 + variance)) {
				double area_angle = ((atan2(double(2 * areaM.nu11), double(areaM.nu20 - areaM.nu02))) / 2) * 180 / 3.1415 + 90;
				double delta_angle = stuff_angle - area_angle;
				//cout << "\nobject angle: " << stuff_angle << "  area angle: " << area_angle << "  delta angle: " << delta_angle;

				// Compare angles
				if (abs(delta_angle) < expected_rotation) {
					for (int k = delta_angle - sideways_range; k < delta_angle + sideways_range + 1; k = k + rot_step) {
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