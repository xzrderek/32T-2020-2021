#include "main.h"

#include "kari/control/chassis.h"
#include "kari/control/rack.h"
#include "kari/control/arm.h"
#include "kari/util/odometry.h"
#include "kari/util/misc.h"
#include "kari/displayController.h"

#include "api.h"


void opcontrol()
{
	using namespace io;

	int towerMode = 0, lastPos = 0;
	bool isTrack = false;

	double kP = 0.12, kP_rev = 0.2;

	bool rel = false;

	Rack rack;
	Arm arm;

	macro::Slew rackSlew(9, 9, true); // Accel, Decel
	macro::PID rackPID(kP);			  // kP, kD
	macro::Slew roller(10, 2);		  // Accel, Decel
	macro::Slew topScorer(4);
	macro::Slew botScorer(9);

	rack.setBrakeType(MOTOR_BRAKE_HOLD);
	arm.setBrakeType(MOTOR_BRAKE_HOLD);

	LF.set_brake_mode(MOTOR_BRAKE_COAST);
	LB.set_brake_mode(MOTOR_BRAKE_COAST);
	RF.set_brake_mode(MOTOR_BRAKE_COAST);
	RB.set_brake_mode(MOTOR_BRAKE_COAST);

	while (true)
	{
		// printf("%lf", LEncoder.get_value());
		std::cout << master.get_analog(ANALOG_LEFT_Y) << std::endl;
		std::cout << LEncoder.get_value() << std::endl;
		
		LF.move_velocity(master.get_analog(ANALOG_LEFT_Y) * 1.58 + master.get_analog(ANALOG_LEFT_X) * 1.58 + master.get_analog(ANALOG_RIGHT_X) * 1.58);
		LB.move_velocity(master.get_analog(ANALOG_LEFT_Y) * 1.58 - master.get_analog(ANALOG_LEFT_X) * 1.58 + master.get_analog(ANALOG_RIGHT_X) * 1.58);
		RF.move_velocity(-master.get_analog(ANALOG_LEFT_Y) * 1.58 + master.get_analog(ANALOG_LEFT_X) * 1.58 + master.get_analog(ANALOG_RIGHT_X) * 1.58);
		RB.move_velocity(-master.get_analog(ANALOG_LEFT_Y) * 1.58 - master.get_analog(ANALOG_LEFT_X) * 1.58 + master.get_analog(ANALOG_RIGHT_X) * 1.58);



		/*--------------------------------
				ROLLERS
		--------------------------------*/
		if (master.get_digital(DIGITAL_R1))
		{
			roller.calculate(127);
		}
		else
		{
			roller.calculate(0);
		}
		if (master.get_digital(DIGITAL_R2))
		{

			roller.calculate(-127);
		}
		else
		{

			roller.calculate(0);
		}

		io::driveRoller(roller.getOutput());

		if (master.get_digital(DIGITAL_L1)) //both up
		{
			topScorer.calculate(127);
			botScorer.calculate(127);
		}
		else
		{
			topScorer.calculate(0);
			botScorer.calculate(0);
		}
		if (master.get_digital(DIGITAL_L2)) //pooping
		{
			topScorer.calculate(127);
			botScorer.calculate(-127);
		}
		else
		{
			topScorer.calculate(0);
			botScorer.calculate(0);
		}

		if (master.get_digital(DIGITAL_RIGHT)) //outake
		{
			topScorer.calculate(-127);
			botScorer.calculate(-127);
		}
		else
		{
			topScorer.calculate(0);
			botScorer.calculate(0);
		}

		io::driveScorer(topScorer.getOutput());
		io::driveScorer(botScorer.getOutput());

		// std::cout << "Rack: " << RackMotor.get_current_draw() << "mA, Arm: " << ArmMotor.get_current_draw() << "mA, RollerL: " << RollerL.get_current_draw() << "mA, RollerR: " << RollerR.get_current_draw() << "mA" << std::endl;
		// std::cout << "Rack Output: " << rackSlew.getOutput() << ", Rack PID Output: " << rackPID.getOutput() << std::endl;

		std::cout << "Rack: " << *rack.getPot() << ", Rack Output: " << rackSlew.getOutput() << std::endl;

		pros::delay(10);
	}
}
