#include "main.h"

#include "kari/control/chassis.h"
#include "kari/control/rack.h"
#include "kari/control/arm.h"

#include "kari/util/odometry.h"
#include "kari/util/misc.h"
#include "kari/util/vision.h"

static Chassis chassis;
static Rack rack;
static Arm arm;
static Odom odom;

int angleoffset = 0;

using namespace io;

// Make Trigger

/*===========================================
  DICTIONARY
===========================================*/
// IDLE = 0
// DRIVING_POINT = 1
// DRIVING_DIST = 2
// TURNING = 3
// STRAFING = 4

void tester()
{
  odom.calibrateGyro();

  // roller(127);
  // chassis.drive(2100,65,4).withAngle(0, 80).withTol(30).waitUntilSettled();
  // chassis.drive(-300,70,4).withAngle(0, 80).withTol(30).waitUntilSettled();
  // chassis.turn(45,70).withTol(10).waitUntilSettled();
  // chassis.drive(500,40,2).withAngle(45, 80).withTol(30).waitUntilSettled();

  master.rumble(" . .");
}

/*===========================================
  PREMADE FUNCTIONS
===========================================*/
void deploy()
{
  arm.move(ARM_LOW_TOWER, 127).withTol(0.3).waitUntilSettled();
  arm.zero();
}

/*===========================================
  RED MATCH AUTONOMOUSES
===========================================*/
void redsmallzone11()
{
}

void redsmallzone8()
{
}

void redsmallzone7()
{
}

void redsmallzone5()
{
}

void redbigzone()
{
}

void redbigzonescore()
{
}

/*===========================================
  BLUE MATCH AUTONOMOUSES
===========================================*/
void bluesmallzone11()
{
}

void bluesmallzone8()
{
}

void bluesmallzone7()
{
}

void bluesmallzone5()
{
}

void bluebigzone()
{
}

void bluebigzonescore()
{
}

/*===========================================
  SKILLS AUTONOMOUSES
===========================================*/
void skills4()
{
}

void skills5()
{
  //init
  chassis.lock();
  odom.zero();
}

void skills3()
{

}

void skills2()
{
  chassis.lock();
}

void skills1()
{
  chassis.lock();
  odom.zero();
  roller(100);
  scorer(30);
  // chassis.drive(1300, 100, 2).withGain(1).withTol(50).waitUntilSettled();
  chassis.turn(90, 127).withTol(5).waitUntilSettled();
  // scorer(127);
  // delay(500);
  chassis.drive(-500, 127, 2).withGain(1).withTol(50).waitUntilSettled();
}