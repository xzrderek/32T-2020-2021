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

void redsmallzone7()
{
}

/*===========================================
  BLUE MATCH AUTONOMOUSES
===========================================*/
void bluesmallzone11()
{
}

void bluesmallzone7()
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

  chassis.lock();
  odom.zero();
  driveRoller(-40);
  delay(500);
  driveRoller(127);
  chassis.left(65);
  chassis.right(100);
  delay(500);
  chassis.turn(45, 127, 2).withTol(5).waitUntilSettled();
  chassis.left(0);
  chassis.right(0);
  driveScorer(127);
  delay(400);
  chassis.left(0);
  chassis.right(0);
  delay(500); //was 500
  driveRoller(0);
  delay(500);
  driveScorer(-127);
  driveRoller(-127);
  chassis.drive(-500, 127, 2).withAngle(45, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  //backing out from first goal
  // chassis.drive(3700, 100, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled();
  chassis.drive(-2100, 110, 1).withAngle(0, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  driveScorer(0);
  delay(500);
  driveRoller(127);
  chassis.turn(85, 100, 1).withTol(5).waitUntilSettled();
  delay(250);
  chassis.left(80);
  chassis.right(80);
  delay(500);
  chassis.left(20);
  chassis.right(20);
  touchBall(false, 127, 1);
  driveScorer(127);
  // score(1, 600);
  delay(500);
  chassis.left(0);
  chassis.right(0);
  driveRoller(0);
  delay(500);
  touchBall(false, 127, 1);
  driveScorer(-127);
  driveRoller(-127);

  //second goal done
  chassis.drive(-600, 127, 2).withAngle(90, 127, 4).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90
  chassis.drive(1300, 127, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled(); //180 might not work
  driveRoller(127);
  // driveScorer(127);
  chassis.turn(145, 127, 2).withTol(5).waitUntilSettled();
  chassis.left(80);
  chassis.right(80);
  driveRoller(127);
  delay(1000);
  chassis.left(20);
  chassis.right(20);
  scoreBalls(true, 127, 3);
  // touchBall(true, 127, 3);
  driveRoller(-127);
  driveScorer(-127);

  //third goal done
  chassis.drive(-500, 127, 2).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90

}

void skills2()
{

  chassis.lock();
  odom.zero();
  driveRoller(-40);
  delay(500);
  driveRoller(127);
  chassis.left(65);
  chassis.right(100);
  delay(500);
  chassis.turn(45, 127, 2).withTol(5).waitUntilSettled();
  chassis.left(0);
  chassis.right(0);
  driveScorer(127);
  delay(400);
  chassis.left(0);
  chassis.right(0);
  delay(500); //was 500
  driveRoller(0);
  delay(500);
  driveScorer(-127);
  driveRoller(-127);
  chassis.drive(-500, 127, 2).withAngle(45, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  //backing out from first goal
  // chassis.drive(3700, 100, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled();
  chassis.drive(-2100, 110, 1).withAngle(0, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  driveScorer(0);
  delay(500);
  driveRoller(127);
  chassis.turn(85, 100, 1).withTol(5).waitUntilSettled();
  delay(250);
  chassis.left(80);
  chassis.right(80);
  delay(500);
  chassis.left(20);
  chassis.right(20);
  touchBall(false, 127, 1);
  driveScorer(127);
  // score(1, 600);
  delay(500);
  chassis.left(0);
  chassis.right(0);
  driveRoller(0);
  delay(500);
  touchBall(false, 127, 1);
  driveRoller(0);
  driveScorer(-127);
  // driveRoller(-80);

  //second goal done
  chassis.drive(-600, 127, 2).withAngle(90, 127, 4).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90
  chassis.drive(1400, 127, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled(); //180 might not work
  driveRoller(-127);
  // driveScorer(127);
  chassis.turn(145, 127, 2).withTol(5).waitUntilSettled();
  // driveRoller(127);
  chassis.left(80);
  chassis.right(80);
  driveRoller(127);
  delay(1000);
  chassis.left(20);
  chassis.right(20);
  scoreBalls(true, 127, 3);
  // touchBall(true, 127, 3);
  driveRoller(-127);
  driveScorer(-127);

  //third goal done
  chassis.drive(-500, 127, 2).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90
}

void skills1()
{
  chassis.lock();
  odom.zero();
  driveRoller(-40);
  delay(500);
  driveRoller(127);
  chassis.left(65);
  chassis.right(100);
  delay(500);
  chassis.turn(45, 127, 2).withTol(5).waitUntilSettled();
  chassis.left(40);
  chassis.right(40);
  driveRoller(0);
  scoreBalls(true, 127, 1);
  driveScorer(127);
  delay(300);
  driveScorer(0);
  driveScorer(-127);
  // driveScorer(-127);
  chassis.left(0);
  chassis.right(0);
  delay(50);
  driveScorer(0);
  driveRoller(-127);
  chassis.drive(-500, 127, 2).withAngle(45, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  //backing out from first goal
  // chassis.drive(3700, 100, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled();
  chassis.drive(-2100, 127, 1).withAngle(0, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  driveScorer(0);
  delay(250);
  driveRoller(-127);
  chassis.turn(80, 127, 1).withTol(5).waitUntilSettled();
  delay(50);
  chassis.left(80);
  chassis.right(80);
  delay(500);
  chassis.left(20);
  chassis.right(20);
  // touchBall(false, 127, 1);
  driveScorer(127);
  // score(1, 600);
  delay(400);
  chassis.left(0);
  chassis.right(0);
  // driveRoller(0);
  // delay(500);
  // touchBall(false, 127, 1);
  driveScorer(-127);
  driveRoller(-127);

  //second goal done
  chassis.drive(-1600, 127, 1).withAngle(90, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  chassis.drive(700, 127, 1).withAngle(90, 127, 4).withGain(1).withTol(50).waitUntilSettled();
  // chassis.drive(-600, 127, 2).withAngle(90, 127, 4).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90
  chassis.drive(1775, 127, 1).withAngle(180, 127, 4).withTol(50).waitUntilSettled(); //180 might not work
  driveRoller(127);
  // driveScorer(127);
  chassis.turn(150, 127, 2).withTol(5).waitUntilSettled();
  chassis.left(100);
  chassis.right(100);
  // driveScorer(127);
  driveRoller(127);
  delay(1000);
  driveRoller(-127);
  chassis.left(20);
  chassis.right(20);
  delay(1000);
  // chassis.left(-127);
  // chassis.right(-127);
  // delay(1000);
  // scoreBalls(true, 127, 1);
  // driveScorer(127);
  // driveRoller(-127);
  // delay(500);
  // touchBall(true, 127, 3);
  // driveRoller(-127);
  // driveScorer(-127);

  //third goal done
  // chassis.drive(-500, 127, 2).withGain(1).withTol(50).waitUntilSettled();  //angle should be 90
}