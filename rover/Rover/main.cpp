// Disable depreciation warning
#pragma warning(disable : 4996)

#include "rover.h"

int main(int argc, char* argv[])
{
	//webcam_capture();
	if (ex4::contour("Images\\mars4.jpeg")) return 1;
	return 0;
}
