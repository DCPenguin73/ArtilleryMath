/*************************************************************
 * 1. Name:
 *      team 7 Daniel & Cayden
 * 2. Assignment Name:
 *      Lab 07: Artillery Math
 * 3. Assignment Description:
 *      Simulate the Artillery Math
 * 4. What was the hardest part? Be as specific as possible.
 *      
 * 5. How long did it take for you to complete the assignment?
 *      
 *****************************************************************/


#include <iostream>  // for CIN and COUT
#include <math.h>
#include "angle.h"
#include "position.h"
#include "Database.h"
using namespace std;
#define PI  (2 * acos(0.0));
#define WEIGHT        46.7   // Weight in KG
#define GRAVITY     -9.8   // Vertical acceleration due to gravity, in m/s^2

#define THRUST   827.000 // Thrust of the initial bullet
#define TIME_INTERVAL .01





/***************************************************
 * COMPUTE DISTANCE
 * Apply inertia to compute a new position using the distance equation.
 * The equation is:
 *     s = s + v t + 1/2 a t^2
 * INPUT
 *     s : original position, in meters
 *     v : velocity, in meters/second
 *     a : acceleration, in meters/second^2
 *     t : time, in seconds
 * OUTPUT
 *     s : new position, in meters
 **************************************************/
 // your function goes here
double computeDistance(double s, double v, double a, double t) {
    double s2 = s + (v * t) + (0.5 * a * (t * t));
    return s2;
}

/**************************************************
 * COMPUTE ACCELERATION
 * Find the acceleration given a thrust and mass.
 * This will be done using Newton's second law of motion:
 *     f = m * a
 * INPUT
 *     f : force, in Newtons (kg * m / s^2)
 *     m : mass, in kilograms
 * OUTPUT
 *     a : acceleration, in meters/second^2
 ***************************************************/
 // your function goes here
double computeAcceleration(double f, double m) {
    double a = f / m;
    return a;
}

/***********************************************
 * COMPUTE VELOCITY
 * Starting with a given velocity, find the new
 * velocity once acceleration is applied. This is
 * called the Kinematics equation. The
 * equation is:
 *     v = v + a t
 * INPUT
 *     v : velocity, in meters/second
 *     a : acceleration, in meters/second^2
 *     t : time, in seconds
 * OUTPUT
 *     v : new velocity, in meters/second
 ***********************************************/
 // your function goes here
double computeVelocity(double v, double a, double t) {
    double v2 = v + (a * t);
    return v2;
}


/***********************************************
 * COMPUTE VERTICAL COMPONENT
 * Find the vertical component of a velocity or acceleration.
 * The equation is:
 *     cos(a) = y / total
 * This can be expressed graphically:
 *      x
 *    +-----
 *    |   /
 *  y |  / total
 *    |a/
 *    |/
 * INPUT
 *     a : angle, in radians
 *     total : total velocity or acceleration
 * OUTPUT
 *     y : the vertical component of the total
 ***********************************************/
 // your function goes here
double computeVertical(Angle a, double total) {
    double y = cos(a.getRadians()) * total;
    return y;
}

/***********************************************
 * COMPUTE HORIZONTAL COMPONENT
 * Find the horizontal component of a velocity or acceleration.
 * The equation is:
 *     sin(a) = x / total
 * This can be expressed graphically:
 *      x
 *    +-----
 *    |   /
 *  y |  / total
 *    |a/
 *    |/
 * INPUT
 *     a : angle, in radians
 *     total : total velocity or acceleration
 * OUTPUT
 *     x : the vertical component of the total
 ***********************************************/
 // your function goes here
double computeHorizontal(Angle a, double total) {
    double x = sin(a.getRadians()) * total;
    return x;
}

/************************************************
 * COMPUTE TOTAL COMPONENT
 * Given the horizontal and vertical components of
 * something (velocity or acceleration), determine
 * the total component. To do this, use the Pythagorean Theorem:
 *    x^2 + y^2 = t^2
 * where:
 *      x
 *    +-----
 *    |   /
 *  y |  / total
 *    | /
 *    |/
 * INPUT
 *    x : horizontal component
 *    y : vertical component
 * OUTPUT
 *    total : total component
 ***********************************************/
double computeTotal(double x, double y) {
    double total;
    total = sqrt(x * x + y * y);
    return total;
}

/**************************************************
 * PROMPT
 * A generic function to prompt the user for a double
 * INPUT
 *      message : the message to display to the user
 * OUTPUT
 *      response : the user's response
 ***************************************************/
double prompt(string message) {
    double response;
    cout << message << endl;
    cin >> response;
    return response;
}
/****************************************************************
 * ANGLE FROM COMPONENTS
 * Get the new angle from the old angle and the horizonal and vertical speed
 ****************************************************************/
double changeAngle(double horX, double verY){
    return atan2(horX, verY);
}

/****************************************************************
 * DRAG FORCE EQUATION
 * Figure out how much drag there is on the projectile
 ****************************************************************/
double dragForce(double coefficient, double density, double velocity, double area) {
    if (velocity > 0){
        return .5 * coefficient * density * velocity * velocity * area * -1;
    }
    else {
        return .5 * coefficient * density * velocity * velocity * area;
    }
    
}
/****************************************************************
 * LINEAR INTERPOLATION
 * Take the coordinates of two points, and finds part the point in the middle
 ****************************************************************/
inline double linearInter(double pos1X, double pos1Y, double pos2X, double pos2Y, double pointX) {
    return pos1Y + (pos2Y - pos1Y) * (pointX - pos1X) / (pos2X - pos1X);
}

/****************************************************************
 * MAIN
 * Prompt for input, compute new position, and display output
 ****************************************************************/
int main()
{
    double surface_area = (154.89 / 2000) * (154.89 / 2000)  * PI;
    double domain1[] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000, 20000, 25000};
    double range1[] = { 9.807, 9.804, 9.801, 9.797, 9.794, 9.791, 9.788, 9.785, 9.782, 9.779, 9.776, 9.761, 9.745, 9.730 };
    Database gravity = Database(domain1, range1, 14);

    double domain2[] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000, 20000, 25000, 30000, 40000, 50000, 60000, 70000, 80000 };
    double range2[] = { 1.2250000, 1.1120000, 1.0070000, 0.9093000, 0.8194000, 0.7364000, 0.6601000, 0.5900000, 0.5258000, 0.4671000, 0.4135000, 0.1948000, 0.0889100, 0.0400800, 0.0184100,
    0.0039960, 0.0010270, 0.0003097, 0.0000828, 0.0000185 };
    Database airDensity = Database(domain2, range2, 20);

    double domain3[] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000, 20000, 25000, 30000, 40000 };
    double range3[] = { 340, 336, 332, 328, 324, 320, 316, 312, 308, 303, 299, 295, 295, 295, 305, 324 };
    Database speedSound = Database(domain3, range3, 16);

    double domain4[] = {0.300, 0.500, 0.700, 0.890, 0.920, 0.960, 0.980, 1.000, 1.020, 1.060, 1.240, 1.530, 1.990, 2.870, 2.890, 5.000 };
    double range4[] = {0.1629, 0.1659, 0.2031, 0.2597, 0.3010, 0.3287, 0.4002, 0.4258, 0.4335, 0.4483, 0.4064, 0.3663, 0.2897, 0.2297, 0.2306, 0.2656};
    Database dragCoef = Database(domain4, range4, 16);

        



    Angle aDegrees(prompt("What is the angle of the howitzer where 0 is up? "));    // Prompt for angle
    double speed = THRUST;   // Total speed
    Position location = Position(0.0, 0.0); //Location of Bullet
    double speedX = computeHorizontal(aDegrees, speed); //Horizontal Speed
    double speedY = computeVertical(aDegrees, speed);
    //Vertical Speed
    double drag; //total drag
    double dragX; //x drag
    double dragY; //y drag
    double accelX = 0; //Horizontal Acceleration
    double accelY = 0; //Vertical Acceleration
    double hangTime = 0;
    double mach = 0;
    Position oldLocation = Position(0.0, 0.0);
    while (location.getMetersY() >= 0) {
        oldLocation = location;
        aDegrees.setRadians(changeAngle(speedX, speedY));
        mach = speed / speedSound.searchDatabase(location.getMetersY());
        drag = dragForce(dragCoef.searchDatabase(mach), airDensity.searchDatabase(location.getMetersY()), speed, surface_area) / WEIGHT;
        dragX = computeHorizontal(aDegrees, drag);
        dragY = computeVertical(aDegrees, drag);
        accelX = dragX;
        accelY = (gravity.searchDatabase(location.getMetersY()) * -1) + dragY;
        speedX = computeVelocity(speedX, accelX, TIME_INTERVAL);
        speedY = computeVelocity(speedY, accelY, TIME_INTERVAL);
        location.setMetersX(computeDistance(location.getMetersX(), speedX, accelX, TIME_INTERVAL));
        location.setMetersY(computeDistance(location.getMetersY(), speedY, accelY, TIME_INTERVAL));
        speed = computeTotal(speedX, speedY);
        hangTime += TIME_INTERVAL;
    }
    if (location.getMetersY() != 0) {
        hangTime = linearInter(oldLocation.getMetersY(), hangTime - TIME_INTERVAL, location.getMetersY(), hangTime, 0);
        location.setMetersX(linearInter(oldLocation.getMetersY(), oldLocation.getMetersX(), location.getMetersY(), location.getMetersX(), 0));
        location.setMetersY(0.0);
    }
    cout.setf(ios::fixed);
    cout.setf(ios::showpoint);
    cout.precision(1);
    cout << "Distance: " << location.getMetersX() << "m  Hang Time: " << hangTime << "s" << endl;
    
    return 0;
}