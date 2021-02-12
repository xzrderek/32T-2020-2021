#pragma once
#include "api.h"

namespace io {
  extern pros::Controller master;

  extern pros::Motor RollerL, RollerR, RollerT, RollerB;

  // Rotates the rollers in a given speed. Use negative values for going down. Uses RPM for speed.
  void roller(int speed);

  void driveRoller(int speed);

  // Rotates the rollers in a given speed to a given point. Use negative values for going down. Uses RPM for speed.
  void roller(double rot, int speed);

  void scorer(int speed);
  void driveTScorer(int speed);
  void driveBScorer(int speed);
  void driveScorer(int speed);
  void score(double rot, int speed);
  void poop(double rot, int speed);
  void drivePooper(int speed);
  void down(double rot, int speed);
  void autonscore(double rot, int speed, int time);
  void afterscore();
  void index(int speed = 30);
}

namespace macro {
  class Slew {
    public:
      Slew(double accel_);
      Slew(double accel_, double decel_);
      Slew(double accel_, double decel_, bool reversible_);

      Slew& withLimit(double input);

      double calculate(double input);

      void setOutput(double output_);
      double getOutput();

      void reset();

    private:
      double accel, decel;
      double input, output, limit;
      bool isReversible, noDecel, isLimited;
  };

  class PID {
    public:
      PID(double kP_);
      PID(double kP_, double kD_);

      PID& withGain(double kP_);
      PID& withGain(double kP_, double kD_);

      double calculate(double target, double input);

      double getError();
      double getOutput();

    private:
      double kP, kD;

      double current, error, last, derivative, output;
  };

  void wait(int ms);
  void print(const char * text);
}

#ifndef PI
const float PI = 3.1415;
#endif

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