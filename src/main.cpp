#include "main.h"
#include "PID.h"
#include <cmath> //for maths in case we need it?
using namespace pros;
using namespace std;

//CONSTRUCTORS
	//chassis
		//left drive
		Motor LF (3, E_MOTOR_GEARSET_18, true);
		Motor LM (5, E_MOTOR_GEARSET_18, true);
		Motor LB (6, E_MOTOR_GEARSET_18, true);
		//right drive
		Motor RF (9, E_MOTOR_GEARSET_18);
		Motor RM (8, E_MOTOR_GEARSET_18);
		Motor RB (7, E_MOTOR_GEARSET_18);
			//inertial sensor for auton PID
			// Imu imu (21);

	//lift
	Motor lift_left (1, E_MOTOR_GEARSET_06, true);
	Motor lift_right (10, E_MOTOR_GEARSET_06);
		//potentiometer for PID
		ADIAnalogIn lift_pot('A');

	//controller
	Controller con (CONTROLLER_MASTER);


//chassis PID

void reset_encoders(){
	RF.set_zero_position(0);
	RM.set_zero_position(0);
	RB.set_zero_position(0);
	LF.set_zero_position(0);
	LM.set_zero_position(0);
	LB.set_zero_position(0);
}

void reset_lift(){
	lift_left.set_zero_position(0);
	lift_right.set_zero_position(0);
}

void stop_motors(){
	RF.move(0);
	RM.move(0);
	RB.move(0);
	LF.move(0);
	LM.move(0);
	LB.move(0);
}

void stop_lift(){
	lift_left.move(0);
	lift_right.move(0);
}

void park_lift(){
	lift_left.move_velocity(0);
	lift_right.move_velocity(0);
	lift_left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	lift_right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}


	void drive(int target){
		// reset_encoders();
		reset_encoders();
		//target is in inches
		target*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.5;
		double kI = 0.01;
		double kD = 0.02;
		int integral = 0;
		int derivative = 0;
		int error;
		int prev_error;
		int power;
		int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		error = target - current_pos;
		while(abs(error) >= 15){
			current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
			error = target - current_pos;
			integral += error;
			if(error == 0){
				integral = 0;
			}
			if(integral > 3000){
				integral = 0;
			}

			derivative = error - prev_error;
			prev_error = error;
			power = kP*error + integral*kI + derivative*kD;
			LF.move(power); LM.move(power); LB.move(power); RF.move(power-10); RM.move(power-10); RB.move(power-10);
			delay(10);
		}
		stop_motors();
		if(target > 0){
		RF.move(-15);
		RM.move(-15);
		RB.move(-15);
	}
		delay(75);
		stop_motors();
	}

	int iter_pos(Imu imu){
		return -1;
	}

//this is just a brainstrom for turning
void turn(int degrees){
	reset_encoders();
	degrees *= 8.8; //this is honestly just some random number
	double kP = 0.225;
	double kI = 0.009;
	double kD = 0.06;
	int integral;
	int derivative;
	int error;
	int prev_error;
	int power;
	int current_pos;

	current_pos = (LF.get_position() + LM.get_position() + LB.get_position()) / 3;
	error = degrees - current_pos;
	while(abs(error) >= 1){
    // for(int i = 0; i < 4 ; i++){
    //   cout << "" << endl;
    // }
		current_pos = (LF.get_position() + LM.get_position() + LB.get_position()) / 3;
    // cout << "Current pos: " << current_pos << endl;
		error = degrees - current_pos;
    // cout << "Error: " << error << endl;
		integral += error;
		if(error == 0){
			integral = 0;
		}

		if(integral >= 2500){
			integral = 0;
		}
		// if(abs(error) > 150){
		// 	integral = 0;
		// }

		derivative = error - prev_error;
		prev_error = error;
    // cout << "P: " << kP*error << endl;
    // cout << "I: " << kI * integral << endl;
    // cout << "D: " << kD*derivative << endl;
		power = kP*error + kI*integral + kD*derivative;
    // cout << "Power: " << power << endl;
		LF.move(power); LM.move(power); LB.move(power); RF.move(-power); RM.move(-power); RB.move(-power);
		delay(10);
	}
	stop_motors();
}

void liftMobileGoal(){
	reset_lift();
	double kP = 0.1;
	double kI = 0.0025;
	double kD = 0.02;
	int integral = 0;
	int derivative = 0;
	int power = 0;
	int current_pos = 0;
	int error = 0;
	int prev_error = 0;
	error = 1300;
	while(error > 5){
		if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			break;
		}
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = 1400 - current_pos;
		integral += error;
		if(error == 0){
			integral = 0;
		}
		if(error > 300){
			integral = 0;
		}
		derivative = error - prev_error;
		power = kP * error + integral * kI + derivative * kD;
		prev_error = 0;
		power -= 15;
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
}

void moveLift(int target){
	reset_lift();
	double kP = 0.1;
	double kI = 0.0025;
	double kD = 0.01;
	int integral = 0;
	int derivative = 0;
	int power = 0;
	int current_pos = 0;
	int error = 0;
	int prev_error = 0;
	error = target - current_pos;
	while(abs(error)>5){
		// if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
		// 	break;
		// }
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = target - current_pos;
		integral += error;
		if(error == 0){
			integral = 0;
		}
		if(error > 300){
			integral = 0;
		}
		derivative = error - prev_error;
		power = kP * error + integral * kI + derivative * kD;
		prev_error = 0;
		// power -= 15;
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
}

void moveMogo(int target){
	reset_lift();
	double kP = 0.1;
	double kI = 0.0025;
	double kD = 0.01;
	int integral = 0;
	int derivative = 0;
	int power = 0;
	int current_pos = 0;
	int error = 0;
	int prev_error = 0;
	error = target - current_pos;
	while(abs(error)>target/2-100){
		// if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
		// 	break;
		// }
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = target - current_pos;
		integral += error;
		if(error == 0){
			integral = 0;
		}
		if(error > 600){
			integral = 0;
		}
		derivative = error - prev_error;
		power = kP * error + integral * kI + derivative * kD;
		prev_error = 0;
		// power -= 15;
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	park_lift();
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

	// imu.reset();
	// delay(2300);
	// while(imu.is_calibrating());
	// stop_motors();
// 	for(int i = 0 ; i < 12 ; i
	//when turning left subtract 10 from wanted degree amount
	// turn(90);
	// delay(5000);
	// turn(-80);
	// // turn(90);
	// stop_motors();


	//sherk programming arc
		moveLift(-1900);
		delay(2);
		//get goal
		turn(90);
		delay(5);
		drive(50);
		moveMogo(1400);
		delay(5);
		//back out
		drive(-50);
		delay(5);
		turn(-90);

	//RED RIGHT
	// moveMogo(1200);
	// delay(5);
	// moveLift(-1900); // Lift Down, the Lift starts at like 20 degrees les than a flat 90 from the top.
	// delay(5);
	// drive(100); //Drive to neutral
	// delay(5);
	// moveMogo(1200); // Pick up neutral ( value needs to be higher because of added weight from mobile goal, 2x)
	// delay(5);
	// drive(-80); // go backwards
	// delay(5); // turn right just cuz

	//RED RIGHT but maybe more

	// moveLift(-1900); // Lift Down, the Lift starts at like 20 degrees les than a flat 90 from the top.
	// delay(5);
	// drive(100); //Drive to neutral
	// delay(5);
	// moveMogo(1400);
	// // park_lift(); // Pick up neutral ( value needs to be higher because of added weight from mobile goal, 2x)
	// delay(5);
	// drive(-80); // go backwards
	// delay(5);
	// turn(75); // turn right just cuz
	// delay(5);
	// drive(30);
	// delay(15);
	// moveLift(-150);
	// delay(5);
	// drive(-20);
	// delay(5);
	// turn(-130);
	// delay(5);
	// drive(109);
	// delay(5);
	// moveMogo(1200);
	// delay(5);
	// park_lift();
	// drive(-110);
	// delay(5);
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
	// imu.reset();
	// delay(2300);
	// while(imu.is_calibrating());
	// stop_motors();
	// cout << "this is working" << endl;
	// cout << "This is working" << endl;
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
				// con.clear();

				// cout << "Lift Left - " << lift_left.get_position() << endl;
				// cout << "Lift Right - " << lift_right.get_position() << endl;
				// cout << "Pitch: " << imu.get_pitch() << endl;
				// cout << "Yaw: " << imu.get_yaw() << endl;
				// cout << "Roll: " << imu.get_roll() << endl;
				// for(int i = 0; i < 5; i ++){
				// 	cout << endl;
				// }
		//lift
			//lift go up
			if(con.get_digital(E_CONTROLLER_DIGITAL_R1)){
				// cout << "Pressed R1" << endl;
				lift_left.move(69);
				lift_right.move(69);
			}
				//lift go down
				else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
					// cout << "Pressed R2" << endl;
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
				if(con.get_digital(E_CONTROLLER_DIGITAL_A)){
					cout << "Pressed A" << endl;
					liftMobileGoal();
				}

				delay(5);
	}

	}
