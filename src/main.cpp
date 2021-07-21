#include "main.h"
#include "PID.h"
#include <cmath> //for maths in case we need it?
using namespace pros;
using namespace std;

//CONSTRUCTORS
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
			Imu imu (10);

	//lift
	Motor lift_left (11, E_MOTOR_GEARSET_06, true);
	Motor lift_right (20, E_MOTOR_GEARSET_06);
		//potentiometer for PID
		ADIAnalogIn lift_pot('A');

	//controller
	Controller con (CONTROLLER_MASTER);


//chassis PID
	void drive (int target){
		double kP = 0.2;
		double kI = 0.0;
		double kD = 0.0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;

		int current_pos = (LF.get_position() + LM.get_position() + LB.get_position() + RF.get_position() + RM.get_position() + RB.get_position())/6;

		error = target - current_pos;
		while(target - current_pos >= 15){
			current_pos = (LF.get_position() + LM.get_position() + LB.get_position() + RF.get_position() + RM.get_position() + RB.get_position())/6;
			error = target - current_pos;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = kP*error + integral*kI + derivative*kD;
			LF.move(power); LM.move(power); LB.move(power); RF.move(power); RM.move(power); RB.move(power);
			delay(15);
		}
	}

	int iter_pos(Imu imu){
		return -1;
	}

	void turn(int target){
		double kP = 0.2;
		double kI = 0;
		double kD = 0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;


		int current_pos = (int)imu.get_yaw();
		error = target - current_pos;

		while(error > 2){
			current_pos = (int)imu.get_yaw();
			error = target - current_pos;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = abs(error*kP + integral*kI + derivative*kD);
			if(target < 0){
				RB.move(power); RM.move(power); RF.move(power); LF.move(-power); LM.move(-power); LB.move(-power);
			}
			else{
				RB.move(-power); RM.move(-power); RF.move(-power); LF.move(power); LM.move(power); LB.move(power);
			}
			delay(15);
		}
	}

bool autobalance = false;
	void autoBalance(){
		int target = 0;
		double kP = 0.2;
		double kI = 0;
		double kD = 0;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;

		int current_pos = (int)imu.get_pitch();
		error = target - current_pos;
		while(abs(error) > 0){
			if(autobalance == false){
				break;
			}
			current_pos = (int)imu.get_pitch();
			error = target - current_pos;
			integral += error;
			derivative = error - prev_error;
			prev_error = error;
			power = kP*error + integral*kI + derivative*kD;
			LF.move(power); LM.move(power); LB.move(power); RF.move(power); RM.move(power); RB.move(power);
			delay(15);
			}
	}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "I was pressed!");
	} else {
		lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::set_text(1, "sup gamer");

	lcd::register_btn1_cb(on_center_button);
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


	//tesssssssssssssssssssssst
	drive(300);
	turn(90);
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
			int left = power + turn;
			int right = power - turn;
				//tbh still tryna figure out how this sum difference thing works

				//put the left and right integers down here
				LF.move(left);
				LM.move(left);
				LB.move(left);
				RF.move(right);
				RM.move(right);
				RB.move(right);
				con.set_text(1, 1, to_string(LF.get_position()));
				con.set_text(2, 2, to_string(RF.get_position()));
		//lift
			//lift go up
			if(con.get_digital(E_CONTROLLER_DIGITAL_R1)){
				lift_left.move(69);
				lift_right.move(69);
			}
				//lift go down
				else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
					lift_left.move(-69);
					lift_right.move(-69);
				}
					//lift go no
					else{
						lift_left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
							lift_left.move_velocity(0);
						lift_right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
							lift_right.move_velocity(0);
						}

		//autobalance
		if(con.get_digital(E_CONTROLLER_DIGITAL_A)){
			autobalance = true;
			autoBalance();
				}
				else{
					autobalance = false;
						}

	}

}
