#ifndef ROBOT_H
#define ROBOT_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>
#include <cerrno>
#include "lowLevel.h"

using pros::Motor;
pros::ADIEncoder;
std::string; //changed commas to semicolons , might b broken idk
using namespace pros;

#define BACK true

// extern ADIPotentiometer pot;
class Robot
{
public:
	//CONSTRUCTOR:
	Robot() : //mechanisms
			  tray(
				  MEC_TRAY,
				  {Motor(TRAY, true)},								//motors
				  {},												//no sensors for indexer, thus use indexer motor
				  PIDcontroller(0.5, 0.0, 0.1, 10, 10, true, false) //PID
				  ),
			  intake(
				  MEC_INTAKE,
				  {Motor(INTAKE1), Motor(INTAKE2, true)},			//motors
				  {},												//no sensors for intake, thus use indexer motor
				  PIDcontroller(1.0, 0.0, 2.0, 10, 10, true, false) //PID
				  ),
			  lift(
				  MEC_LIFT,
				  {Motor(LIFT)},									//motors
				  {},												//no sensors for lift, thus use indexer motor
				  PIDcontroller(2.0, 0.0, 2.0, 10, 10, true, false) //PID
				  ),
			  base(
				  {Motor(RFront, true), Motor(LFront), Motor(LBack), Motor(RBack, true)}, //motors
				  {PIDcontroller(40, 0.0, 2.0, 0.5, 10, true, false),					  //forward back
				   PIDcontroller(2, 0.0, 0.5, 5.0, 10, true, false),					  //turnuntil angle, if too slow, increase kp, lower = slower power
				   PIDcontroller(0.1, 0.0, 0, 0.5, 10, true, false),
				   PIDcontroller(2.5, 0.0, 0.0, 1.0, 10, false, false)}, //opdrive PID
				  Odometry(Position(0, 0, 0), Position(0, 0, 0))		 //actual position, tracker mech's position
			  )
	// trayToggle(
	// 	MEC_TRAY,
	// 	{ Motor(TRAY) }, //motors
	// 	{},//no sensors for indexer, thus use indexer motor
	// 	PIDcontroller(0.07, 1.0, 0.1, 100,  10, true, true)//PID
	// )
	// base(//motors
	// 	{ Motor(RFront), Motor(LFront), Motor(LBack), Motor(RBack) },
	// 	//drive PID, then angle PID, then curve
	// 	{ PIDcontroller(12, 0.0, 0.05, 1.75, 10, true, false),
	// 	  PIDcontroller(3.5, 0.0, 0.5, 2.0,  10, false, false),
	// 	  PIDcontroller(2.5, 0.0, 0.0, 1.0,  10, false, false) },
	// 	//Odometry
	// 	Odometry(Position(0, 0, 0), Position(0, 0, 0)//,//actual position, tracker mech's position
	// 	//Odom Sensors:
	// 	)
	// )
	{
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		base.odom.pos.heading = 90;
	}

	Mechanism tray;
	Mechanism intake;
	Mechanism lift;
	Mechanism rollers;
	// Mechanism trayToggle;
	Chassis base; //ChassisNoOdom baseNoOdom;
	float FWVelGoal = 0;

	void reset()
	{
		tray.reset();
		intake.reset();
		lift.reset();
		base.reset();
		rollers.reset();
		base.odom.pos.X = 0;
		base.odom.pos.Y = 0;
		base.odom.pos.heading = 90;
	}
	// void indexerAdvance(int amntTicks = 100){//bring indexer ball up once (given number of encoder ticks)
	// 	indexer.moveAmnt(amntTicks, 10);
	// }

	//tests

	// std::vector<string> debugString(){
	// std::vector<string> ret;
	// //ret.push_back(string("BATTERY percent:") + std::to_string( pros::battery::get_capacity()));
	// /*ret.push_back(string("Flywheel1 Vel:") + std::to_string(sprocket1.get_actual_velocity()));
	// ret.push_back(string("Flywheel1 Temp:") + std::to_string(sprocket1.get_temperature()));
	// ret.push_back(string("Flywheel2 Vel:") + std::to_string(sprocket2.get_actual_velocity()));
	// ret.push_back(string("Flywheel2 Temp:") + std::to_string(sprocket2.get_temperature()));
	// if(flyWheelVelPID.getRunningState()) ret.push_back(string("PID Running: YES"));
	// else ret.push_back(string("PID Running: NO"));
	// ret.pus h_back(string("PID Goal:") + std::to_string(flyWheelVelPID.getGoal()));
	// */

	// //ret.push_back(string("EncL: ") + std::to_string( encoderL.get_value())
	// //+string("; EncR: ") + std::to_string( encoderR.get_value())
	// //+string("; EncM: ") + std::to_string( encoderM.get_value()));
	// ret.push_back(string("Pos X: ") + std::to_string( base.odom.pos.X) + string(" Pos Y: ") + std::to_string( base.odom.pos.Y));
	// ret.push_back(string("Heading: ") + std::to_string( base.odom.pos.heading));
	// //ret.push_back(string("FlywheelPos: ") + std::to_string( flywheelEnc.get_value()));
	// //ret.push_back(string("FlywheelVel(rpm): ") + std::to_string( flywheel.velocity/2) + string(" Motors: ") + std::to_string( flywheel.getMotorVel() ));
	// ret.push_back(string("base Vel: ") + std::to_string( base.driveVel) + string("; rot Vel: ") + std::to_string( base.rotVel));
	// ret.push_back(string("Angle Err: ") + std::to_string( base.pids[ANGLE].error));

	// //ret.push_back(string("Error Val: ") + strerror(errno));
	// return ret;
	// }
};
#endif
