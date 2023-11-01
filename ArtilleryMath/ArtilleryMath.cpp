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
 * MAIN
 * Prompt for input, compute new position, and display output
 ****************************************************************/
int main()
{
    double surface_area = (154.89 / 2000) * (154.89 / 2000)  * PI;
    double domain1[] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 15000, 20000, 25000};
    double range1[] = { 9.807, 9.804, 9.801, 9.797, 9.794, 9.791, 9.788, 9.785, 9.782, 9.779, 9.776, 9.761, 9.745, 9.730 };
    Database gravity = Database(domain1, range1, 14);
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
    Position oldLocation = Position(0.0, 0.0);
    while (location.getMetersY() >= 0) {
        oldLocation = location;
        aDegrees.setRadians(changeAngle(speedX, speedY));
        drag = dragForce(0.3, 0.6, speed, surface_area) / WEIGHT;
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
    //if (location.getMetersY() != 0) {
    //    hangTime = linearInter(hangTime - TIME_INTERVAL, oldLocation.getMetersY(), hangTime, location.getMetersY(), 0);
    //    location.setMetersX(linearInter(oldLocation.getMetersX(), oldLocation.getMetersY(), location.getMetersX(), location.getMetersY(), 0));
    //    location.setMetersY(0.0);
    //}
    cout << "Distance: " << location.getMetersX() << "m   Altitude: " << location.getMetersY() << "m    Hang Time: " << hangTime << "s" << endl;
    
    return 0;
}