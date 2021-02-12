#include "main.h"

#include "kari/control/chassis.h"
#include "kari/control/rack.h"
#include "kari/control/arm.h"

#include "kari/util/odometry.h"
#include "kari/util/misc.h"

#include "kari/displayController.h"
#include "kari/autonController.h"

void initialize() {
  okapi::Timer timer;
  double init = timer.millis().convert(millisecond);
  macro::print("Starting Init Routine");

  Autonomous Auton;
  macro::print("Auton Set!");

  // Class Initialization
  Odom odom;

  //tracking wheel version. using IMU angle
  Chassis chassis(odom.getL(), odom.getR(), odom.getM(), odom.getThetaDeg(), odom.getX(), odom.getY(), odom.getXInch(), odom.getYInch()); 
  
  // Rack rack;
  // Arm arm;
  Display Disp;

  // Roller Init
  io::RollerL.set_brake_mode(MOTOR_BRAKE_HOLD);
	io::RollerR.set_brake_mode(MOTOR_BRAKE_HOLD);

  // Sensor Init
  odom.calibrateGyro(); 
  std::cout << "Motors / Sensors Initialized!" << std::endl; 

  // Threads
  pros::Task odomController(odom.start, NULL, "Odometry Tracker"); 

  pros::Task baseController(chassis.start, NULL, "Chassis Controller");

  // pros::Task rackController(rack.start, NULL, "Rack Controller");
  // rack.setBrakeType(MOTOR_BRAKE_HOLD);

  // pros::Task armController(arm.start, NULL, "Arm Controller");
  // arm.setBrakeType(MOTOR_BRAKE_HOLD);
  // arm.tarePos();

  pros::Task b_display(Disp.start, NULL, "Display Controller");
  b_display.set_priority(TASK_PRIORITY_MIN);

  pros::Task b_auton(Auton.start, NULL, "Auton Controller");
  b_display.set_priority(TASK_PRIORITY_MIN);

  std::cout << "Tasks Initialized!" << std::endl;

  Disp.addInfo("X Pos", 'd', odom.getXInch())
      .addInfo("Y Pos", 'd', odom.getYInch())
      .addInfo("Deg Heading", 'd', odom.getThetaDeg())
      .addInfo("Rad Heading", 'd', odom.getThetaRad())
      .addInfo("Left Wheel", 'i', odom.getL())
      .addInfo("Right Wheel", 'i', odom.getR())
      .addInfo("Middle Wheel", 'i', odom.getM())
      // .addInfo("X inches", 'd', odom.getXInch())
      // .addInfo("Y inches", 'd', odom.getYInch())
      // .addInfo("X1 LF", 'd', odom.getLF())
      // .addInfo("Y1 RF", 'd', odom.getRF())
      // .addInfo("Y2 LB", 'd', odom.getLB())
      // .addInfo("X2 RB", 'd', odom.getRB())
      ;


  // Disp.addInfo("Rack", 'i', rack.getPot())
  //     .addInfo("Arm", 'd', arm.getPos())
  //     .addInfo("Arm Limit", 'b', arm.getLimit());

  Disp.cleanUp();

  double end = timer.millis().convert(millisecond);
  std::cout << "Initialization Done! Took " << end - init << "ms." << std::endl;
}

void disabled() {
  Chassis chassis;
  Rack rack;
  Arm arm;

  chassis.withGain().withTurnGain().withTol().withSlop().reset();
  chassis.setMode(IDLE);
  chassis.clearArr();
  rack.withTol().reset();
  arm.withTol().reset();
}

void competition_initialize() { }
