#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"
#include "odom.h"

using namespace pros;
using std::string;

// TODO
DEFINE_TRACKERS

Controller master(E_CONTROLLER_MASTER);
Robot rob = Robot();

// ADIEncoder encoderL (1, 2, true), encoderR (3, 4, false), encoderM (5, 6, false);
float encM, encL, encR;
int baseLine;
float LeftBAvg, RightBAvg;
volatile int gAdjustTray = TRAYNEUTRAL;

void updatePIDs(void *param)
{
  gAdjustTray = TRAYNEUTRAL;
  Robot *r = (Robot *)param;
  const float delayAmnt = 2;
  //temp
  bool intakeZeroed = false, trayPidOff = false;
  while (true)
  {
    //tray value usually 1000
    //if(r->tray.getSensorVal() > 1000 && r->lift.getSensorVal() > -1000) {

    //turns intake pid off when outtaking a stack (hinges)
    if (r->tray.getSensorVal() > 500 && r->lift.getSensorVal() > -1000)
    {
      //r->intake.setPIDState(OFF);
      rob.intake.setSlow(SLOW_INTAKE);
      // rob.intake.setPID(0.03, 0.0, 0.0);

      // // if(rob.tray.get_actual_velocity == 0){
      //   intakeZeroed = true;
      //   if (intakeZeroed) {
      //     rob.intake.set_zero_position();
      //   }
      //   rob.intake.moveTo(rob.tray.getSensorVal()/3);
      // // }
    }
    else
    {
      // intakeZeroed = false;
      rob.intake.setSlow(SLOW_NORMAL);
      // rob.intake.setPID(1.0, 0.0, 2.0);
    }

    //tray moves forward when lift moves up and pid off when under value to prevent stalling
    // if(gAdjustTray != TRAYNEUTRAL) {
    //   rob.tray.moveTo(gAdjustTray);
    //   if (abs(rob.tray.getSensorVal() - gAdjustTray) < 50) {
    //     gAdjustTray = TRAYNEUTRAL;
    //   }
    // } else {

    //fix later if necessary
    if (rob.tray.getSensorVal() < 1000)
    {
      rob.tray.setPIDState(OFF);
    }

    // }

    // if(gAdjustTray != TRAYNEUTRAL) {
    //   rob.tray.moveTo(gAdjustTray);
    //   gAdjustTray = TRAYNEUTRAL;
    // }

    r->tray.PID();
    r->lift.PID();
    r->intake.PID();
    r->base.PID();
    r->rollers.PID();

    // if(master.btnA) {
    //   rob.tray.moveTo(3800);
    //   rob.lift.moveTo(700);
    //rob.intake.moveNew(20);
    // rob.lift.moveNew(20);
    // rob.tray.moveNew(20);

    //  lcd::print(7, (string("traytoggle kd: ") + std::to_string(r->trayToggle.pid.kD)).c_str());
    //}

    //debug
    lcd::print(7, (string("Lift: ") + std::to_string(r->lift.getSensorVal())).c_str());
    lcd::print(6, (string("Iintake: ") + std::to_string(r->intake.getSensorVal())).c_str());
    // lcd::print(5, (string("Base: ") + std::to_string(r->base.getSensorVal())).c_str());
    lcd::print(4, (string("Tray: ") + std::to_string(r->tray.getSensorVal())).c_str());
    //
    // lcd::print(3, (string("Tray Goal: ") + std::to_string(r->tray.getPIDGoal())).c_str());
    // lcd::print(2, (string("Base Goal: ") + std::to_string(r->base.getPIDGoal())).c_str());
    // lcd::print(1, (string("Intake Goal: ") + std::to_string(r->intake.getPIDGoal())).c_str());
    // lcd::print(0, (string("gAdjustTray: ") + std::to_string(gAdjustTray)).c_str());

    //master.set_text(1, 1, (string("Speed: ") + std::to_string(r->tray.getSlow())).c_str());

    // delay
    delay(delayAmnt);
  }
}

// void trayToggleFunc(void *param) {
//   Robot* r = (Robot*) param;
//   const float delayAmnt = 2;
//   while(true){
//     if(master.btnA) {
//       rob.tray.setPIDState(OFF);
//       rob.intake.setPIDState(OFF);
//       rob.trayToggle.moveToUntil(3800);
//       //rob.intake.moveNew(20);
//       // rob.lift.moveNew(20);
//       // rob.tray.moveNew(20);

//       lcd::print(7, (string("traytoggle kd: ") + std::to_string(r->trayToggle.pid.kD)).c_str());

//       // delay
//       delay(delayAmnt);
//     }
//   }
// }

// void updateSensor(void* param) {
//   Robot* r = (Robot*) param;
//   const float delayAmnt = 20;//ms delay for velocity calculations
//   while(true){
//     r->base.computeVel();
//     delay(delayAmnt);
//   }
// }

void resetEncoders()
{
  encMo.reset();
  encLo.reset();
  encRo.reset();
  encL = encM = encR = 0;
}

void updateTask(void *param)
{
  Robot *r = (Robot *)param;
  // ADILineSensor light (2);
  while (true)
  {

    if (rob.base.odom.resetEncoders)
    {
      resetEncoders();
    }

    LeftBAvg = avg(r->base.mots[1].get_position(), r->base.mots[2].get_position());   //1, 2
    RightBAvg = -avg(r->base.mots[0].get_position(), r->base.mots[3].get_position()); //0, 3
    encM = encMo.get_value();
    encL = encLo.get_value();
    encR = encRo.get_value();
    // // baseLine = light.get_value();
    lcd::print(0, (string("Pos X: ") + std::to_string(rob.base.odom.pos.X)).c_str());
    lcd::print(1, (string("Pos Y: ") + std::to_string(rob.base.odom.pos.Y)).c_str());
    lcd::print(2, (string("Heading: ") + std::to_string(rob.base.odom.pos.heading)).c_str());
    // lcd::print(3, (string("kP: ") + std::to_string( rob.intake.pid.kP)).c_str());
    // lcd::print(4, (string("kI: ") + std::to_string( rob.intake.pid.kI)).c_str());
    // lcd::print(5, (string("kD: ") + std::to_string( rob.intake.pid.kD)).c_str());
    //
    lcd::print(3, (string("LLLEnc: ") + std::to_string(encoderDistInch(encL))).c_str());
    lcd::print(4, (string("REnc: ") + std::to_string(encoderDistInch(encR))).c_str());
    lcd::print(5, (string("MEncoder : ") + std::to_string(encM)).c_str());
    // lcd::print(6, (string("Line : ") + std::to_string( baseLine )).c_str());

    delay(2);
  }
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
  static bool pressed = false;
  pressed = !pressed;
  if (pressed)
  {
    pros::lcd::set_text(2, "I was pressed!");
  }
  else
  {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

  rob.reset();
  resetEncoders();

  rob.base.odom.resetEncoders = true;

  // Task sensorUpdates(updateSensor, &rob, "");
  Task taskUpdate(updateTask, &rob, "");
  Task PIDsUpdate(updatePIDs, &rob, "");
  // Task trayToggleTask(trayToggleFunc, &rob, "");
  Task odometryCalculations(calculatePosBASE, &rob.base.odom, "");

  bool pressedIntake = false, pressedTray = false, pressedLift = false, pressedReset = false, intakeZeroed = false, pressedResetPos = false, trayPidOff = false;
  bool pressedBase = false, pressedSlow = false, pressedLiftTo = false, pressedLiftToLow = false;

  while (true)
  {

    // lift to med
    if (master.btnDOWN)
    {
      pressedLiftTo = true;
    }
    else if (pressedLiftTo == true)
    {
      if (master.btnY)
      { //with shift
        rob.lift.moveTo(LIFTMED);
        rob.intake.moveTo(rob.intake.getSensorVal() + 1000);
      }
      else
      {
        rob.intake.setSlow(SLOW_INTAKE);
        rob.lift.moveTo(LIFTMED);
        gAdjustTray = TRAYHIGH;
      }
      pressedLiftTo = false;
    }
    // lift to low
    if (master.btnB)
    {
      pressedLiftToLow = true;
    }
    else if (pressedLiftToLow == true)
    {
      if (master.btnY)
      { //with shift
        rob.lift.moveTo(LIFTLOW);
        rob.intake.moveTo(rob.intake.getSensorVal() + 1000);
      }
      else
      {
        rob.intake.setSlow(SLOW_INTAKE);
        rob.lift.moveTo(LIFTLOW);
        gAdjustTray = TRAYHIGH;
      }
      pressedLiftToLow = false;
    }
    // slow button
    //        INIT  Lift Height>Threshold  Height<Threshold   BTNL2 & Height>Threshold   BTNL2 & Height<Threshold
    // intake  1         2(slow)             1                  Toggle (2->1)
    // tray    1                                                                             Toggle
    if (master.btnSpeedToggle)
    {
      pressedSlow = true;
    }
    else if (pressedSlow == true)
    {
      if (rob.lift.getSensorVal() < LIFTTHRESHOLD_LOW)
      {
        rob.intake.toggleSlow();
      }
      else
      {
        rob.tray.toggleSlow();
      }
      pressedSlow = false;
    }

    // intake
    rob.intake.simpleControl(master.btnR2, master.btnR1);
    if (master.btnR2 || master.btnR1)
    {
      rob.intake.setPIDState(OFF);
      pressedIntake = true;
    }
    else if (pressedIntake)
    {
      rob.intake.moveTo(rob.intake.getSensorVal());
      rob.intake.setPIDState(ON);
      pressedIntake = false;
    }

    // lift
    rob.lift.simpleControl(master.btnL2, master.btnL1);
    if (master.btnL1 || master.btnL2)
    {
      rob.lift.setPIDState(OFF);
      pressedLift = true;

      if (rob.lift.getSensorVal() < LIFTTHRESHOLD_LOW)
      {
        gAdjustTray = TRAYHIGH;
        rob.intake.setSlow(SLOW_INTAKE);
      }
      else
      {
        gAdjustTray = TRAYLOW;
        rob.intake.setSlow(SLOW_NORMAL);
      }

      // if (master.btnL2 && rob.lift.getSensorVal() > LIFTTHRESHOLD_HIGH) {
      //   gAdjustTray = TRAYNEUTRAL; // TRAYRESTING;
      // }
    }
    else if (pressedLift)
    {
      rob.lift.moveTo(rob.lift.getSensorVal());
      pressedLift = false;
    }

    // tray
    rob.tray.simpleControl(master.btnX, master.btnA);
    if (master.btnX || master.btnA)
    {
      rob.tray.setPIDState(OFF);
      pressedTray = true;
    }
    else if (pressedTray)
    {
      rob.tray.moveTo(rob.tray.getSensorVal());
      // rob.trayToggle.setPIDState(OFF);
      // rob.intake.setPIDState(OFF);
      pressedTray = false;
    }

    // base
    rob.base.driveArcadeXDrive(master.leftY, master.leftX, master.rightX);
    if (master.rightX != 0 || master.leftY != 0 || master.leftX != 0)
    {
      rob.base.setPIDState(OFF);
      pressedBase = true;
    }
    else if (pressedBase)
    {
      rob.base.moveTo(rob.base.getSensorVal());
      pressedBase = false;
    }

    //resets encoder values in op control if auton skews values
    if (master.btnUP)
    {
      pressedReset = true;
    }
    else if (pressedReset)
    {
      rob.reset();
      pressedReset = false;
      // trayPidOff = true;
    }

    if (master.btnY)
    {
      pressedResetPos = true;
    }
    else if (pressedResetPos)
    {
      rob.lift.moveTo(0);
      rob.tray.moveTo(0);
      pressedResetPos = false;
    }

    // Base with odometry
    // rob.base.driveArcade(master.leftY, master.rightX);
    // if (master.rightX != 0 || master.leftY != 0) {
    //   rob.base.setPIDState(OFF);
    //   pressedBase = true;
    // } else if (pressedBase) {
    //   rob.base.setPIDGoal(rob.base.odom.pos.X, rob.base.odom.pos.Y);
    //   pressedBase = false;
    // }

    // if(brake == 1) rob.base.driveLR(master.leftY, master.rightX);
    // if(master.btnY) {
    //   brake = !brake;
    //   delay(300);
    // }

    //Debug Absolute Position
    // if(master.btnUP){
    //   rob.base.moveToUntil(400);
    // }

    // if(master.btnY){
    //   rob.base.odom.resetEncoders = true;
    //   rob.base.odom.resetAngleSentinel = 90;
    //   rob.base.driveToPoint(10, 20);
    // }

    // if(master.btnLEFT){
    //   rob.base.turnUntil(90);
    // }

    delay(2);
  }
}
