#pragma warning(disable : 4996)

#include "rover.h"

namespace utils {
	using namespace std;
	string getAbsImagePath(const char* localPath)
		// Only use for debugging purposes due to quirk with Visual Studio cwd
	{
		string buffer = _getcwd(NULL, 0);

		// Cut-off debug section of cwd
		string cwd = buffer.substr(0, buffer.length() - 5);
		string imagePath = cwd + "Rover\\" + localPath;

		return imagePath;

	}
	cv::Mat loadImageG(string path, float scale) {
		cv::Mat src, src2, src_gray;
		src = cv::imread(path, 1);
		cv::resize(src, src2, cv::Size(), scale, scale, cv::INTER_LANCZOS4);
		cvtColor(src2, src_gray, CV_BGR2GRAY);
		return src_gray;
	}
	cv::Mat preproccess(cv::Mat src, float imscale) {
		using namespace cv;

		int kernel_size = 5;
		int scale = 1;
		int delta = 0;
		int ddepth = CV_16S;

		Mat src_gray, dst, abs_dst;

		resize(src, src, Size(), imscale, imscale);
	
		/// Reduce the noise so we avoid false circle detection
		for (int i(0); i < 1; ++i)
		{
			medianBlur(src, src, 7);
			blur(src, src, Size(3, 3));
		}
		/// Convert it to gray
		//blur(src, src, Size(5,5));
		cvtColor(src, src_gray, CV_BGR2GRAY);

		threshold(src_gray, src_gray, 40, 255, THRESH_TOZERO | THRESH_OTSU);

		for (int i(0); i < 50; ++i) medianBlur(src_gray, src_gray, 7);

		Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);

		convertScaleAbs(dst, abs_dst);

		threshold(abs_dst, abs_dst, 40, 255, THRESH_BINARY | THRESH_OTSU);

		return abs_dst;

	}
}
