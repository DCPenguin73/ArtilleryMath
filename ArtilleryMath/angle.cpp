#pragma once

#define TWO_PI 6.28318530718

#include <string>     // To display text on the screen
#include <cmath>      // for M_PI, sin() and cos()
#include <algorithm>  // used for min() and max()
#include <sstream>    // for OSTRINGSTRING
using std::string;
using std::min;
using std::max;
#include "angle.h"
using namespace std;



double Angle::getRadians() const {
	return radians;
}
void Angle::setRadians(double angleR) {
	radians = angleR;
}
double Angle::normalize(double angleR) const {
	double r = angleR;
	while (r < 0) {
		r = TWO_PI + r;
	}
	while (r > TWO_PI) {
		r = r - TWO_PI;
	}
	return r;
}
double Angle::getDegrees() const {
	return convertToDegrees(radians);
};
void Angle::setDegrees(double angleD) {
	radians = convertToRadians(angleD);
};
double Angle::convertToDegrees(double radians) const{
	return radians * (360 / TWO_PI);
};
double Angle::convertToRadians(double degrees) const {
	return (degrees * TWO_PI) / 360;
};

