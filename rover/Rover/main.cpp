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
#include <sstream>


int main(int argc, char* argv[])
// iternate through the list of contours given by canny, find average gradient
// of the last few points. split the contour if the gradient deviates from average
// by too much and then continue search on other half of the contour
// use the new list of contours to detect circles
{
	//IplImage* Image = webcam_capture();

	int n_kp = 80;

	bool search_for_circle = true;
	float cir_err = 3;
	float prev_err = 15; float prev_tol = 8;

	vector<string> imPath = {
		utils::getAbsImagePath("Images\\678B\\1.jpg"),
		utils::getAbsImagePath("Images\\678B\\2.jpg"),
		utils::getAbsImagePath("Images\\678B\\3.jpg"),
		utils::getAbsImagePath("Images\\678B\\4.jpg"),
		utils::getAbsImagePath("Images\\678B\\5.jpg"),
		utils::getAbsImagePath("Images\\678B\\6.jpg"),
		utils::getAbsImagePath("Images\\678B\\7.jpg"),
		//utils::getAbsImagePath("Images\\678B\\8.jpg"),
		//utils::getAbsImagePath("Images\\678B\\9.jpg")
	};
	int dataSize = imPath.size();

	vector<picture> frame(1);
	float imSize[2] = { 360, 640 };

	// 1 - INITIALIZE ORIGIN
	cout << "\n\nFRAME 0  <----- Start of journey :)";
	cv::Mat Image = cv::imread(imPath[0].c_str(), 1);
	cv::Mat Image_360x640 = cv::Mat(imSize[0], imSize[1], CV_8UC3, Scalar());
	cv::resize(Image, Image_360x640, Image_360x640.size());
	frame[0].original = Image_360x640;
	frame[0].captured_from.error = 0;
	frame[0].captured_from.z = 250; //mm
	frame[0].captured_from.dz = 1;

	float focalLength = 1150; //pixels
	float realR = 50 / 2; //mm
	float R_error = 0.1; //%

	// 2 - Initial circle search (if requested)
	Mat aux1; //<-- writable image
	tuple <float, float, float, float, float> circ_info;
	Mat where;  frame[0].original.copyTo(where); //<-- search image
	if (search_for_circle) {
		// 2 - Find circle in first image
		while (1) {
			char accept = 'M';
			std::cout << "\n\n>>   Circle search:\n";
			frame[0].original.copyTo(aux1);
			cv::Mat processed = preprocess_main(where, 1);
			float R = estimate_radius(frame[0].captured_from.z, focalLength, realR); //pixels
			circ_info = circle_finder(processed, R * (1.0 - R_error), R * (1.0 + R_error), 0, aux1);
			frame[0].circle_abs = Point2f(get<0>(circ_info), get<1>(circ_info) * (-1)); frame[0].circle_rel = frame[0].circle_abs;
			frame[0].circle_dZ = get<2>(circ_info); frame[0].circle_Z = (1 / frame[0].circle_dZ) * frame[0].captured_from.z;
			frame[0].circle_R = get<3>(circ_info); frame[0].circle_error = get<4>(circ_info);
			frame[0].has_circle = true;
			cout << "Found radius = " << get<3>(circ_info) << " <--> Previously estimated ~ N( " << R << ", " << R_error << " )";
			while (accept != 'Y' && accept != 'y' && accept != 'N' && accept != 'n' && accept != 'X' && accept != 'x') {
				cout << "\n Do you accept the found circle? (Y/N) | Deactivate circle search (X) :   ";
				cv:waitKey(1);
				cin >> accept;
				cout << " -- Imput received: " << accept;
			}
			if (getWindowProperty("RANSAC result", 0) >= 0) cvDestroyWindow("RANSAC result");
			if (accept == 'Y' || accept == 'y') break;
			else if (accept == 'X' || accept == 'x') {
				search_for_circle = false;
				break;
			}
			else where = circle_reduceArea(frame[0].original, where, imSize);
		}
	}

	for (int i = 1; i < dataSize; i++) {

		cv::Mat Image = cv::imread(imPath[i].c_str(), 1);
		cv::Mat Image_360x640 = cv::Mat(imSize[0], imSize[1], CV_8UC3, Scalar());
		cv::resize(Image, Image_360x640, Image_360x640.size());

		// 3 - Run terrain recognition to see how we moved
		cout << "\n\n\nFRAME " << i;
		cout << "\n\n >>   Terrain recognition:\n";
		frame.push_back(newpic_relpos(frame[0], Image_360x640, 30));
		cout << "\nFrame (" << i << ") moved " << frame[i].captured_from.step_distance << "units -->  dX = " << frame[i].captured_from.traslation.x << ", dY = " << frame[i].captured_from.traslation.y << ", dZ = " << frame[i].captured_from.dz << "; dAngle =  " << frame[i].captured_from.d_angle;
		cout << "\n                                   X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;

		// If accumulated error is too high compare with previous images to reduce it
		if (frame[i].captured_from.error > prev_err) {
			int best_previous = i - 1;
			picture aux = frame[i];
			for (int j = i - 1; j >= 0; j--) {
				aux = newpic_relpos(frame[j], frame[i].original, n_kp);
				if (aux.captured_from.error < frame[i].captured_from.error || aux.captured_from.error < 15) {
					frame[i] = aux;
					best_previous = j;
				}
				if(frame[i].captured_from.error < prev_tol) break;
			}
			cout << "\nFrame (" << i << ") from ("<< best_previous <<")   X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;

		}
		if (frame[i].captured_from.error > 150) {
			cout << "\n\n ERROR IS TOO BIG TO HANDLE IT... USE A FKING GPS MAAAAAN!!!";
			break;
		}

		if (search_for_circle && frame[i].captured_from.error > cir_err) {
			// 4 - Find circle
			std::cout << "\n\n>>   Circle search:\n";
			frame[i].original.copyTo(aux1);
			cv::Mat processed = preprocess_main(frame[i].original, 1);
			float R = frame[0].circle_R * frame[0].captured_from.z / frame[i].captured_from.z;//estimate_radius(frame[i].captured_from.z, focalLength, realR); //pixels
			
			circ_info = circle_finder(processed, R - frame[i].captured_from.error * 3, R + frame[i].captured_from.error * 3, 0); //, aux1
			frame[i].circle_rel = Point2f(get<0>(circ_info), get<1>(circ_info) * (-1));
			float scale = frame[i].captured_from.z / frame[0].captured_from.z;
			frame[i].circle_abs = frame[i].captured_from.abs_centre + frame[i].circle_rel * scale;
			frame[i].circle_dZ = get<2>(circ_info); frame[i].circle_Z = (1 / frame[i].circle_dZ) * frame[i].captured_from.z;
			frame[i].circle_R = get<3>(circ_info); frame[i].circle_error = get<4>(circ_info);
			frame[i].has_circle = true;
			cout << "Found radius = " << get<3>(circ_info) << " <--> Previously estimated ~ N( " << R << ", " << frame[i].captured_from.error << " )";
			//cvWaitKey();
			if (getWindowProperty("RANSAC result", 0) >= 0) cvDestroyWindow("RANSAC result");
			frame[i] = SensorFusion(frame[0], frame[i]);
			cout << "\nFrame (" << i << ")  X = " << frame[i].captured_from.abs_centre.x << ",  Y = " << frame[i].captured_from.abs_centre.y << ",  Z = " << frame[i].captured_from.z << "; Angle =  " << frame[i].captured_from.angle << ",  >> ERROR << = " << frame[i].captured_from.error;
		}
	}

	// 5 - DISPLAY COLLAGE
	cout << "\n\n DRAWING AREA MAP:";
	display_map(frame, 0.9, focalLength, realR);

	return 0;
}




