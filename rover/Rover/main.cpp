// Disable depreciation warning
#pragma warning(disable : 4996)

//TODO: efficiency improvements, calculate r and stop if it is out of bounds
//		calc the expected size of the circle from the distance input

#include "mars.h"
#include "rover.h"
#include "compass.h"

#include <random>
#include <tuple>

#include <time.h>

vector <int> 
my_RANSAC(cv::Mat img, float r1, float r2);

cv::Mat
webcam_capture_main(bool save = false, std::string name = "")
{
	VideoCapture cap(0);

// 	if (!cap.isOpened())
// 		// check if we succeeded
// 		std::cout << "Camera cannot be opened!\n";
// 		return cv::Mat();

	cv::Mat Image;
	cap >> Image;

	if (save) 
	{
		imwrite(name, Image);
	}

	return Image;
}

cv::Mat 
preprocess_main(cv::Mat image, float dx = 1.0) {
	Mat src;
	image.copyTo(src);
	cv::resize(src, src, Size(), dx, dx);
	using namespace cv;
	int kernel_size = 5;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	/// Apply the Hough Transform to find the circles
	Mat abs_dst, src_gray, dst;


	/// Reduce the noise so we avoid false circle detection
	for (int i(0); i < 1; ++i)
	{
		medianBlur(src, src, 3);
		blur(src, src, Size(3, 3));
	}
	/// Convert it to gray
	//blur(src, src, Size(5,5));
	if (src.channels() == 3)  cvtColor(src, src, CV_BGR2GRAY);
	// split b g and r into different channels, process each seperately 
	// average the circle coords
	threshold(src, src, 40, 255, THRESH_TOZERO | THRESH_OTSU);
 	for (int i(0); i < 5; ++i) medianBlur(src, src, 7);
// 	for (int i(0); i < 5; ++i) blur(src, src, Size(3, 3));

	//threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO_INV | THRESH_OTSU);
	//Laplacian(src, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	Canny(src, abs_dst, 2, 20*5);
	
	//convertScaleAbs(dst, abs_dst);
	//threshold(abs_dst, abs_dst, 40, 255, THRESH_TOZERO | THRESH_OTSU);
	return abs_dst;
}

tuple <float, float, float>
circle_finder(cv::Mat ProcessedImage, float min_rad, float max_rad, int bordersize, cv::Mat Original = cv::Mat() )
{

	vector<int> results = my_RANSAC(ProcessedImage, min_rad, max_rad);
	cv::Point center = cv::Point(floor(ProcessedImage.cols / 2), floor(ProcessedImage.rows / 2));

	float x, y, r;
	x = results[0] - bordersize;
	y = results[1] - bordersize;
	r = results[2];

	float smallestSide = (ProcessedImage.cols < ProcessedImage.rows) ? ProcessedImage.cols : ProcessedImage.rows;
	
	float fraction_filled = (2 * r) / smallestSide;
	float delta_z = 0.9 / fraction_filled * 100;

	std::cout << "\n Fraction Filled: " << fraction_filled << '\n';

	tuple<float, float, float> deltas{ x - center.x, y - center.y , delta_z};

	if (Original.empty())
	{
		return deltas;
	}

	cv::circle(Original, cv::Point(x, y), 5, cv::Scalar(0, 0, 255));
	cv::circle(Original, cv::Point(x, y), r, cv::Scalar(255, 100, 100));
		
	cv::circle(Original, center, 5, cv::Scalar(255, 255, 255));

	cv::line(Original, center, Point2d(x, y), cv::Scalar(255, 255, 255));

	std::cout << "delta X: " << get<0>(deltas) << "\ndelta Y: " << get<1>(deltas) << "\ndelta Z%: " << get<2>(deltas) << '\n';

	cv::imshow("RANSAC Example", Original);
	cv::waitKey(0);

	
	return deltas;
}


int main(int argc, char* argv[])
// iternate through the list of contours given by canny, find average gradient
// of the last few points. split the contour if the gradient deviates from average
// by too much and then continue search on other half of the contour
// use the new list of contours to detect circles
{
	//IplImage* Image = webcam_capture();

	bool search_for_circle = false;

	vector<string> imPath = {
		utils::getAbsImagePath("Images\\678B\\1.jpg"),
		utils::getAbsImagePath("Images\\678B\\2.jpg"),
		//utils::getAbsImagePath("Images\\678B\\3.jpg"),
		//utils::getAbsImagePath("Images\\678B\\4.jpg"),
		//utils::getAbsImagePath("Images\\678B\\5.jpg"),
		//utils::getAbsImagePath("Images\\678B\\6.jpg"),
		//utils::getAbsImagePath("Images\\678B\\7.jpg"),
		//utils::getAbsImagePath("Images\\678B\\8.jpg"),
		//utils::getAbsImagePath("Images\\678B\\9.jpg")
	};
	int dataSize = imPath.size();

	vector<picture> frame(1);

	// 1 - INITIALIZE ORIGIN
	cout << "\n\nFRAME 0";
	cv::Mat Image = cv::imread(imPath[0].c_str(), 1);
	cv::Mat Image_360x640 = cv::Mat(360, 640, CV_8UC3, Scalar());
	cv::resize(Image, Image_360x640, Image_360x640.size());
	frame[0].original = Image_360x640;
	frame[0].captured_from.error = 0;
	frame[0].captured_from.z = 250; //mm

	float focalLength = 1150; //pixels
	float realR = 50 / 2; //mm
	float R_error = 0.1; //%
	
	if (search_for_circle) {
		// 2 - Find circle in first image
		std::cout << "\n\n>>   Circle search:\n";
		cv::Mat processed = preprocess_main(frame[0].original, 1);
		float R = estimate_radius(frame[0].captured_from.z, focalLength, realR); //pixels
		Mat aux1;  frame[0].original.copyTo(aux1);
		tuple<float, float, float> circ_centre = circle_finder(processed, R * (1 - R_error), R * (1 + R_error), 0, aux1);
		frame[0].circle_abs = Point2f(-get<0>(circ_centre), -get<1>(circ_centre)); frame[0].circle_rel = frame[0].circle_abs;
		frame[0].circle_dZ = get<2>(circ_centre);
		frame[0].circle_Z = (1 / frame[0].circle_dZ) * frame[0].captured_from.z;
		frame[0].has_circle = true;
	}

	for (int i = 1; i < dataSize; i++) {

		cv::Mat Image = cv::imread(imPath[i].c_str(), 1);
		cv::Mat Image_360x640 = cv::Mat(360, 640, CV_8UC3, Scalar());
		cv::resize(Image, Image_360x640, Image_360x640.size());

		// 3 - Run terrain recognition to see how we moved
		cout << "\n\n\nFRAME " << i;
		cout << "\n\n >>   Terrain recognition:\n";
		frame.push_back(newpic_relpos(frame[i - 1], Image_360x640, 30));
		cout << "\nFrame (" << i << ") moved " << frame[i].captured_from.step_distance << "units -->  dX = " << frame[i].captured_from.traslation.x << ", dY = " << frame[i].captured_from.traslation.y << ", dZ = " << frame[i].captured_from.dz << "; dAngle =  " << frame[i].captured_from.d_angle;
		cout << "\n                                   X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;

		// If accumulated error is too high compare with previous images to reduce it
		if (frame[i].captured_from.error > 50) {
			int best_previous = i - 1;
			picture aux = frame[i];
			for (int j = i - 1; j >= 0; j--) {
				aux = newpic_relpos(frame[j], frame[i].original);
				if (aux.captured_from.error < frame[i].captured_from.error) {
					frame[i] = aux;
					best_previous = j;
				}
			}
			cout << "\nFrame (" << i << ") from ("<< best_previous <<")   X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;

		}
		if (frame[i].captured_from.error > 150) {
			cout << "\n\n ERROR IS TOO BIG TO HANDLE IT... USE A FKING GPS MAAAAAN!!!";
			break;
		}

		if (search_for_circle && frame[i].captured_from.error>30) {
			// 4 - Find circle
			std::cout << "\n\n>>   Circle search:\n";
			Mat aux2;  frame[i].original.copyTo(aux2);
			cv::Mat processed = preprocess_main(frame[i].original, 1);
			float R = estimate_radius(frame[i].captured_from.z, focalLength, realR); //pixels
			tuple<float, float, float> circ_centre2 = circle_finder(processed, R * (1 - frame[i].captured_from.error*0.5), R * (1 + frame[i].captured_from.error*0.5), 0, aux2);
			frame[i].circle_abs = Point2f(-get<0>(circ_centre2), - get<1>(circ_centre2));
			frame[i].circle_abs = frame[i].circle_rel + frame[i].captured_from.abs_centre;
			frame[i].circle_dZ = get<2>(circ_centre2);
			frame[i].circle_Z = (1 / frame[i].circle_dZ) * frame[i].captured_from.z;
			frame[i].has_circle = true;
		}

		// 5 - DISPLAY COLLAGE
		cout << "\n\n DRAWING AREA MAP:";
	}

	display_map(frame, 0.8, focalLength, realR);

	cv::waitKey(0);

	return 0;
}




