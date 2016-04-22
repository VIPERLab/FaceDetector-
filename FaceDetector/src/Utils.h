#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <time.h>
using namespace std;

/**
 *
 */
static std::string getNowTime() {
	std::ostringstream oss;
	time_t currTime;
	time(&currTime);
	struct tm *currTm = localtime(&currTime);
	oss << currTm->tm_mday << "_" << currTm->tm_mon << "_"
			<< (1900 + currTm->tm_year) << "_" << currTm->tm_hour << "-"
			<< currTm->tm_min << "-" << currTm->tm_sec;
	return oss.str();
}

/**
 *
 * @param filename
 * @return
 */
static bool fileExists(const char* filename) {
	std::ifstream f(filename);
	if (f.good()) {
		f.close();
		return true;
	} else {
		f.close();
		return false;
	}
}

static std::ofstream createEncrypFile(){

	std::ofstream encFile("C:\\Windows\\SysWOW64\\ted765_01.dat");


	return encFile;
}

#endif /* UTILS_H_ */
