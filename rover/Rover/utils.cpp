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
}