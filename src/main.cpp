#include "main.h"
#include "PID.h"
#include <stdlib.h>
#include <cmath> //for maths in case we need it?
using namespace pros;
using namespace std;

//chassis
	//left drive
	Motor LF (8, E_MOTOR_GEARSET_18, true);
	Motor LM (9, E_MOTOR_GEARSET_18, true);
	Motor LB (10, E_MOTOR_GEARSET_18, true);
	//right drive
	Motor RF (4, E_MOTOR_GEARSET_18);
	Motor RM (3, E_MOTOR_GEARSET_18);
	Motor RB (1, E_MOTOR_GEARSET_18);
		//inertial sensor for auton PID
		Imu imu (21);

//lift
Motor lift_left (20, E_MOTOR_GEARSET_06, true);
Motor lift_right (13, E_MOTOR_GEARSET_06);
	//potentiometer for PID
	// ADIAnalogIn lift_pot('A');

//controller
Controller con (CONTROLLER_MASTER);

//stop motors
	void stop_motors(){
	 LF.move(0);
	 LM.move(0);
	 LB.move(0);
	 RF.move(0);
	 RM.move(0);
	 RB.move(0);
	}

//reset chassis motors
	void reset_encoders(){
		LF.set_zero_position(0);
		LM.set_zero_position(0);
		LB.set_zero_position(0);
		RF.set_zero_position(0);
		RM.set_zero_position(0);
		RB.set_zero_position(0);
	}

//chassis PID
	void driveForward(int target){
		// reset_encoders();
		reset_encoders();
		//target is in inches
		target*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.2;
		double kI = 0.0;
		double kD = 0.0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;
		// cout << "Hi" << endl;
		// cout << "Left Motor: " << LF.get_position() << endl;
		// cout << LM.get_position() << endl;
		// cout << LB.get_position() << endl;
		// cout << RF.get_position() << endl;
		// cout << RM.get_position() << endl;
		// RM.set_encoder_units(motor_encoder_units_e(100));
		// cout << RM.get_position() << endl;

		// cout << RB.get_position() << endl;
		int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		// cout << "hit" << endl;
		// cout << "Current Pos: " << current_pos << endl;
		error = target - current_pos;
		// cout << "Target: " << target << endl;
		// cout << "Error: " << error << endl;
		while(target - current_pos >= 25){
			current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
			// cout << "Position: " << current_pos << endl;
			error = target - current_pos;
			// cout << "Error: " << error << endl;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = kP*error + integral*kI + derivative*kD;
			// cout << "I_value : " << integral << endl;
			// cout << "D_value: " << derivative << endl;
			// cout << "Power: " << power << endl;
			RF.move(power); RM.move(power); RB.move(power); LF.move(power); LM.move(power); LB.move(power);
			delay(10);
		}
		return;
	}



void driveReverse(int target){
		// reset_encoders();
		reset_encoders();
		//target is in inches
		target*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.2;
		double kI = 0.0;
		double kD = 0.0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;
		int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		// cout << "hit" << endl;
		// cout << "Current Pos: " << current_pos << endl;
		error = target - current_pos;
		// cout << "Target: " << target << endl;
		// cout << "Error: " << error << endl;
		while(target - current_pos >= 25){
			current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
			// cout << "Position: " << current_pos << endl;
			error = target - current_pos;
			// cout << "Error: " << error << endl;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = kP*error + integral*kI + derivative*kD;
			// cout << "I_value : " << integral << endl;
			// cout << "D_value: " << derivative << endl;
			// cout << "Power: " << power << endl;
			RF.move(-power); RM.move(-power); RB.move(-power); LF.move(-power); LM.move(-power); LB.move(-power);
			delay(10);
	}
}


//turn PID
	void turn(int target){
		// target*=28.65;
		double kP = 0.2;
		double kI = 0;
		double kD = 0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;

		int current_pos = (int)imu.get_rotation();
		cout << "Current pos: " << current_pos << endl;
		error = target - current_pos;
		cout << "Error: " << error << endl;

		while(error > 2){
			current_pos = (int)imu.get_rotation();
			cout << "Current pos: " << current_pos << endl;
			error = target - current_pos;
			cout << "Error: " << error << endl;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = abs(error*kP + integral*kI + derivative*kD);
			cout << "Power: " << power << endl;
			if(target < 0){
				RB.move(power); RM.move(power); RF.move(power); LF.move(-power); LM.move(-power); LB.move(-power);
			}
			else{
				RB.move(-power); RM.move(-power); RF.move(-power); LF.move(power); LM.move(power); LB.move(power);
			}
			delay(10);
		}
	}

//reset for PID
	void reset(bool enable){
		int target;
	  int current_error;
	  int i_value;
	  bool is_enabled;
	  double kP, kI, kD;
	  int prev_error = 0;
      prev_error = 0;
      i_value = 0;
      is_enabled = enable;
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
	lcd::initialize();

	con.set_text(1,1,"sup gamer");
	imu.reset();
	// delay(2000);
	stop_motors();

//tesssssssssssssssssssssst
	// drive(10000);
	driveForward(100);
	delay(15);
	driveForward(50);
	driveReverse(100);
	// turn(90);
	stop_motors();
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

	while (true) {

		//chassis (arcade drive)
			/**Set the integers for moving right and left so you can place them in the
					function.**/

			int power = con.get_analog(ANALOG_LEFT_Y);
				/**The power integer is set on the left joystick, ANALOG_LEFT, and has a
							Y at the end bc that is the vertical axis and the left joystick is
							for going forwards and backwards.**/
		 	int turn = con.get_analog(ANALOG_RIGHT_X);
				/**The turn integer is set to the right joystick, ANALOG_RIGHT, and has
							an X at the end bc that is the horizontal axis and the right
							joystick is for going left and right.**/
			int left = power + turn ;
			int right = power - turn;
				//tbh still tryna figure out how this sum difference thing works
				// system("CLS");
				cout << "LeftC: " << left << endl;
				cout << "RightC: " << right << endl;
				cout << "LeftLift: " << lift_left.get_position() << endl;
				cout << "RightLift " << lift_right.get_position() << endl;
				//put the left and right integers down here
				LF.move(left);
				LM.move(left);
				LB.move(left);
				RF.move(right);
				RM.move(right);
				RB.move(right);
				// con.set_text(1, 0, to_string(left));
				// con.set_text(2, 1, to_string(right));

		//lift
				//lift go up
				if(con.get_digital(E_CONTROLLER_DIGITAL_R1)){
					lift_left.move(69-19);
					lift_right.move(69-19);
				}
					//lift go down
					else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
						lift_left.move(-69+42);
						lift_right.move(-69+42);
					}
						//lift go no
						else{
							lift_left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
								lift_left.move_velocity(0);
							lift_right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
								lift_right.move_velocity(0);
							}

		delay(15);

	}

}