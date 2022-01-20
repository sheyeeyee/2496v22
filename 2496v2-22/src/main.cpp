#include "main.h"
#include <string>
#include <math.h>
#include "global.h"
#include "PID.h"
using namespace pros;
using namespace std;
using namespace glb;

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

int currAuton = 1;
void competition_initialize() {

	bool selected = true;
	int localTime = 0;
	int totalAutons = 4;
	con.clear();

	while(true) {

		if(button.get_value() == 0) {
			if(selected) {
				currAuton ++;
				if(currAuton == 5) {
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
					con.print(0, 0, "Selected: AWP Left");
					break;
				case(2):
					con.print(0, 0, "Selected: AWP Right");
					break;
				case(3):
					con.print(0, 0, "Selected: Right Side");
					break;
				case(4):
					con.print(0, 0, "Selected: Left Side");
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
	grabNeutral();
	if(currAuton == 1) {
		// grabNeutral();
	}
	else if(currAuton == 2) {

	}
	else if(currAuton == 3) {

	}
	else if(currAuton == 4) {

	}
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
	con.clear();
	int localTime = 0;
	bool first = true;
	bool pistonValue = true;

	//the piston starting actuated or unactuated is all decided by whereever the tubes are
	while (true) {
		if(localTime%150 == 0) {

			if(pistonValue) con.print(0, 0, "true");
			else con.print(0, 0, "false");
		}
		int power = con.get_analog(ANALOG_RIGHT_X); // right = left?
		int turn = con.get_analog(ANALOG_LEFT_Y);

		int left = power + turn;
		int right = power - turn;

		LF.move(left);
		LM.move(left);
		LB.move(left);
		RF.move(right);
		RM.move(right);
		RB.move(right);
//r1 up, r2 down, l1 air,

		//air
		if(con.get_digital(E_CONTROLLER_DIGITAL_X)) {
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

		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			INTAKE.move(-100);
		}
		else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			INTAKE.move(100);
		}
		else {
			INTAKE.set_brake_mode(E_MOTOR_BRAKE_COAST);
			INTAKE.move_velocity(0);
		}
		localTime += 5;
		delay(5);
	}
}
