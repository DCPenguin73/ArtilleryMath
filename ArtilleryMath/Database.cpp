#include <string>     // To display text on the screen
#include <cmath>      // for M_PI, sin() and cos()
#include <algorithm>  // used for min() and max()
#include <sstream>    // for OSTRINGSTRING
using std::string;
using std::min;
using std::max;
#include "database.h"
using namespace std;



/****************************************************************
 * LINEAR INTERPOLATION
 * Take the coordinates of two points, and finds part the point in the middle
 ****************************************************************/
inline double Database::linearInter(double pos1X, double pos1Y, double pos2X, double pos2Y, double pointX) {
    return pos1Y + (pos2Y - pos1Y) * (pointX - pos1X) / (pos2X - pos1X);
}

double Database::searchDatabase(double pointX) {
    for (int i = 1; i < length; i++) {
        if (pointX < domain[i]) {
            return linearInter(domain[i-1], range[i-1], domain[i], range[i], pointX);
        }
    }
};