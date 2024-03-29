#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
using namespace cv;
using namespace std;


Mat fourier(const char* source)
// Function for displaying the frequency domain of an input image
{

	Mat I = imread(source, IMREAD_GRAYSCALE);
	resize(I, I, Size(), 0.4, 0.4);

	if (I.empty()) {
		cout << "Error opening image" << endl;
		return I;
	}
	Mat padded;                            //expand input image to optimal size
	int m = getOptimalDFTSize(I.rows);
	int n = getOptimalDFTSize(I.cols); // on the border add zero values
	copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
	Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };
	Mat complexI;
	merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
	dft(complexI, complexI);            // this way the result may fit in the source matrix
	// compute the magnitude and switch to logarithmic scale
	// => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
	magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
	Mat magI = planes[0];
	magI += Scalar::all(1);                    // switch to logarithmic scale
	log(magI, magI);
	// crop the spectrum, if it has an odd number of rows or columns
	magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));
	// rearrange the quadrants of Fourier image  so that the origin is at the image center
	int cx = magI.cols / 2;
	int cy = magI.rows / 2;
	Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right
	Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);
	normalize(magI, magI, 0, 1, NORM_MINMAX); // Transform the matrix with float values into a
											// viewable image form (float between values 0 and 1).
	imshow("Input Image", I);    // Show the result
	imshow("spectrum magnitude", magI);

	blur(magI, magI, Size(3, 3));


	waitKey();
	return magI;
}

cv::Mat fourier(cv::Mat inputImage, cv::Mat(*function)(cv::Mat))
// Accepts a cv::Mat and a function pointer as input. Applies fourier transform, function pointed to,
// then applies a reverse fourier transform and returns the image as a cv::Matrix
{
	// Go float
	cv::Mat fImage, gImage;
	if (inputImage.channels() == 3)  cvtColor(inputImage, inputImage, CV_BGR2GRAY);
	
	inputImage.convertTo(fImage, CV_32F);

	// FFT
	std::cout << "Direct transform...\n";
	cv::Mat fourierTransform;
	cv::dft(fImage, fourierTransform, cv::DFT_SCALE | cv::DFT_COMPLEX_OUTPUT);

	// Some processing using the given function pointer
	cv::Mat output = (*function)(fourierTransform);

	// IFFT
	std::cout << "Inverse transform...\n";
	cv::Mat inverseTransform;
	cv::dft(fourierTransform, inverseTransform, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);

	// Back to 8-bits
	cv::Mat finalImage;
	inverseTransform.convertTo(finalImage, CV_8U);
	return finalImage;
}
