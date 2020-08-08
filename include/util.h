#ifndef UTIL_H
#define UTIL_H

#include "math.h"

#define CW true
#define CCW false
#define ON true
#define OFF false

const float PI = 3.1415;

inline int sign(int num)
{
	if (num < 0)
		return -1;
	return 1;
}
inline float sqr(float x)
{
	return x * x;
}
inline float toRad(float deg)
{
	//PI/180 = 0.017453
	return (deg * 0.017453);
}
inline float toDeg(float rad)
{
	return (rad / 0.017453);
}
inline float avg(float a, float b)
{
	return ((a + b) / 2.0);
}
inline float limUpTo(float max, float x)
{
	if (abs(x) < max)
		return x;
	else
		return sign(x) * max;
}
inline float limDownTo(float min, float x)
{
	if (abs(x) > min)
		return x;
	else
		return sign(x) * min;
}
inline float clamp(float max, float min, float amnt)
{
	if (amnt > max)
		return max;
	if (amnt < min)
		return min;
	return amnt;
}
inline float normAngle(float degrees)
{
	if (degrees > 180)
		return (degrees - 360);
	else if (degrees < -180)
		return (degrees + 360);
	return degrees;
}
inline float boundAngle(float radians)
{ //keeps radians within -PI to +PI
	while (radians < -PI)
		radians += 2 * PI;
	while (radians >= PI)
		radians -= 2 * PI;
	return radians;
}
inline float encoderDistInch(int rawSensor)
{
	const float wheelDiam = 2.75;
	const int countsPerRev = 360;
	const int gearRatio = 1; //1 to 1
	return (rawSensor * PI * wheelDiam) / (countsPerRev * gearRatio);
}
inline float encoderDistInchBASE(int rawSensor)
{
	const float wheelDiam = 4.140; //base wheels
	const int countsPerRev = 360;
	const int gearRatio = 1; //1 to 1
	return (rawSensor * PI * wheelDiam) / (countsPerRev * gearRatio);
}
inline bool isWithinBounds(const float current, const float goal, const float thresh)
{
	return (fabs(current - goal) < thresh);
}
inline bool isWithinAngleBounds(const float current, const float goal, const float thresh)
{
	return (fabs(normAngle(current - goal)) < thresh);
}

//easy button shortcuts
#define btnL1 get_digital(E_CONTROLLER_DIGITAL_L1)
#define btnL2 get_digital(E_CONTROLLER_DIGITAL_L2)
#define btnR1 get_digital(E_CONTROLLER_DIGITAL_R1)
#define btnR2 get_digital(E_CONTROLLER_DIGITAL_R2)
#define btnA get_digital(E_CONTROLLER_DIGITAL_A)
#define btnB get_digital(E_CONTROLLER_DIGITAL_B)
#define btnX get_digital(E_CONTROLLER_DIGITAL_X)
#define btnY get_digital(E_CONTROLLER_DIGITAL_Y)
#define btnUP get_digital(E_CONTROLLER_DIGITAL_UP)
#define btnDOWN get_digital(E_CONTROLLER_DIGITAL_DOWN)
#define btnLEFT get_digital(E_CONTROLLER_DIGITAL_LEFT)
#define btnRIGHT get_digital(E_CONTROLLER_DIGITAL_RIGHT)
//analog sticks
#define leftY get_analog(E_CONTROLLER_ANALOG_LEFT_Y)
#define leftX get_analog(E_CONTROLLER_ANALOG_LEFT_X)
#define rightY get_analog(E_CONTROLLER_ANALOG_RIGHT_Y)
#define rightX get_analog(E_CONTROLLER_ANALOG_RIGHT_X)

#define btnSpeedToggle btnRIGHT

#define ON true
#define OFF false
#define DRIVE 0
#define ANGLE 1
#define DRIVE2 2
#define CURVE 3

//defining motor ports:
#define RFront 9
#define RBack 1
#define LFront 8
#define LBack 17
#define LIFT 7
#define INTAKE1 3
#define INTAKE2 13
#define TRAY 10

#define WHEELWIDTH 3.5
#define DEFINE_TRACKERS ADIEncoder encMo(3, 4, true), encRo(7, 8, false), encLo(5, 6, false);

//with backauton
#define LIFTTHRESHOLD_LOW (-800)
#define LIFTTHRESHOLD_HIGH (-1200)
#define LIFTLOW (-1500)
#define LIFTMED (-2000)
#define TRAYHIGH 1500
#define TRAYLOW 800
#define TRAYNEUTRAL (-1)

// with frontauton
// #define LIFTTHRESHOLD_LOW (-800)
// #define LIFTTHRESHOLD_HIGH (-1200)
// #define LIFTLOW (-2300)
// #define LIFTMED (-3000)
// #define TRAYHIGH 800
// #define TRAYLOW  0
// #define TRAYNEUTRAL (-1)
// //#define TRAYNOPID 500

#define MEC_BASE 1
#define MEC_TRAY 2
#define MEC_LIFT 3
#define MEC_INTAKE 4

#define SLOW_INTAKE 2.0
#define SLOW_TRAY 2.5
#define SLOW_NORMAL 1.0
#endif
