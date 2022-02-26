#include "main.h"
#include <string>
#include <math.h>
#include "global.h"
#include "PID.h"
using namespace pros;
using namespace std;
using namespace glb;

//wasdawsd
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "e");

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.d
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

int currAuton = 0;
void competition_initialize() {
	imu.reset();
	while(imu.is_calibrating()) delay(5);
	bool selected = true;
	int localTime = 0;
	int totalAutons = 5;
	con.clear();

	while(true) {

		if(button.get_value() == 0) {
			if(selected) {
				currAuton ++;
				if(currAuton == totalAutons+1) {
					currAuton = 1;
				}
				selected = false;
			}
			selected = false;
		}
		else selected = true;

		if(localTime%50 == 0) {
			// con.clear();
			switch(currAuton) {
				case (1):
					con.print(0, 0, "Selected: %d AWP Left", currAuton);
					break;
				case(2):
					con.print(0, 0, "Selected: %d AWP Right", currAuton);
					break;
				case(3):
					con.print(0, 0, "Selected: %d Neutral", currAuton);
					break;
				case(4):
					con.print(0, 0, "Selected: %d Center", currAuton);
					break;
				case(5):
					con.print(0, 0, "Selected: %d None", currAuton);
					break;
			}
			// con.print(0, 0, "Selected: %d", currAuton);
		}
		localTime ++;
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	halfRightAwp();
	// soloAwpLeft();

	// INTAKE.move_absolute(1100, 100);
	// soloLeftAwp();
	// imuTurn(90);
	// imu.reset();
	// while(imu.is_calibrating()) delay(50);
	// con.clear();
	// con.print(0, 0, "look at the field lol");
	// if(currAuton == 1) {
	// 	// imuTurn(90);
	// 	halfLeftAwp();
	// }
	// if(currAuton == 2) {
	// 	halfRightAwp();
	// }
	// if(currAuton == 3) {
	// 	grabNeutral();
	// }
	// if(currAuton == 4) {
	// 	grabCenter();
	// }
	// if(currAuton == 5) {
	//
	// }
}

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
void opcontrol() {
	// con.clear();
	// imu.reset();
	// while(imu.is_calibrating()) delay(50);
	int localTime = 0;
	bool first = true;
	bool pistonValue = true;
	int currentINTAKEpos = INTAKE.get_position();
	bool putUp = true;
	bool isInPos = false;
	// con.clear();
	//the piston starting actuated or unactuated is all decided by whereever the tubes are
	while (true) {

		int power = con.get_analog(ANALOG_LEFT_Y);
		int turn = con.get_analog(ANALOG_RIGHT_X);

		int left = power + turn;
		int right = power - turn;


		LF.move(left);
		LM.move(left);
		LB.move(left);
		RF.move(right);
		RM.move(right);
		RB.move(right);
//r1 up, r2 down, l1 air,
		if(localTime%150 == 0) {

			con.print(0, 0,  "%f", LIFT.get_position());
			// else con.print(0, 0, "false");
		}
		//air
		if(con.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			if(first == true) {
				if(pistonValue == true) {
					piston.set_value(false);
					pistonValue = false;
				}
				else {
					piston.set_value(true);
					pistonValue = true;
				}
				first = false;
			}
		}
		else first = true;

		//Lift
		if(con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			LIFT.move(100);
		}
		else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			LIFT.move(-100);
		}
		else {
			LIFT.set_brake_mode(E_MOTOR_BRAKE_COAST);
			LIFT.move_velocity(0);
		}

		//intake
	 	if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			if(putUp == true) {
				if(isInPos == true) {
					INTAKE.move_absolute(15, -100);
					isInPos = false;
					// INTAKE.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
					// INTAKE.move_velocity(0);
				}
				else {
					INTAKE.move_absolute(1875, 100);
					isInPos = true;
					// INTAKE.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
					// INTAKE.move_velocity(0);
				}
				putUp = false;
				}
			}
			else putUp = true;

		// else {
		// 	putUp = true;
		// 	if(con.get_digital(E_CONTROLLER_DIGITAL_X)) {
		// 		INTAKE.move(-100);
		// 	}
		// 	else if (con.get_digital(E_CONTROLLER_DIGITAL_A)) {
		// 		INTAKE.move(100);
		// 	}
		// 	else {
		// 		INTAKE.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
		// 		INTAKE.move_velocity(0);
		// 	}
		// }
		localTime = localTime + 5;

		delay(5);
	}
}
