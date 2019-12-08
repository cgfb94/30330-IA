// Disable depreciation warning
#pragma warning(disable : 4996)

//TODO: efficiency improvements, calculate r and stop if it is out of bounds
//		calc the expected size of the circle from the distance input

#include "mars.h"
#include "rover.h"
#include "circles.h"
#include "compass.h"

#include <random>
#include <tuple>

#include <time.h>


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
	
	// 2 - Initial circle search (if requested)
	Mat aux1;  if (search_for_circle) frame[0].original.copyTo(aux1);
	tuple <float, float, float, float, float> circ_info;
	if (search_for_circle) {
		// 2 - Find circle in first image
		std::cout << "\n\n>>   Circle search:\n";
		cv::Mat processed = preprocess_main(frame[0].original, 1);
		float R = estimate_radius(frame[0].captured_from.z, focalLength, realR); //pixels
		circ_info = circle_finder(processed, R * (1.0 - R_error), R * (1.0 + R_error), 0, aux1);
		frame[0].circle_abs = Point2f(-get<0>(circ_info), -get<1>(circ_info)); frame[0].circle_rel = frame[0].circle_abs;
		frame[0].circle_dZ = get<2>(circ_info); frame[0].circle_Z = (1 / frame[0].circle_dZ) * frame[0].captured_from.z;
		frame[0].circle_R = get<3>(circ_info); frame[0].circle_R = get<4>(circ_info);
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
			frame[i].original.copyTo(aux1);
			cv::Mat processed = preprocess_main(frame[i].original, 1);
			float R = estimate_radius(frame[i].captured_from.z, focalLength, realR); //pixels
			circ_info = circle_finder(processed, R * (1.0 - frame[i].captured_from.error*0.5), R * (1.0 + frame[i].captured_from.error*0.5), 0, aux1);
			frame[i].circle_abs = Point2f(-get<0>(circ_info), - get<1>(circ_info));
			frame[i].circle_abs = frame[i].circle_rel + frame[i].captured_from.abs_centre;
			frame[i].circle_dZ = get<2>(circ_info); frame[i].circle_Z = (1 / frame[i].circle_dZ) * frame[i].captured_from.z;
			frame[i].circle_R = get<3>(circ_info); frame[i].circle_R = get<4>(circ_info);
			frame[i].has_circle = true;
		}

		// 5 - DISPLAY COLLAGE
		cout << "\n\n DRAWING AREA MAP:";
	}

	display_map(frame, 0.8, focalLength, realR);

	cv::waitKey(0);

	return 0;
}




