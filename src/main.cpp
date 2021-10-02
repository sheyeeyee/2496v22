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
		Imu imu (17);

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
		int powerAdj = 0;
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
			if(power < 0){
				power = min(power, -127);
			}
			else{
				if(power > 0){
					power = min(power, 127);
				}
			}
			// powerAdj = power/10;
			LF.move(power); LM.move(power); LB.move(power); RF.move(power-powerAdj); RM.move(power-powerAdj); RB.move(power-powerAdj);
			delay(10);
		}
		stop_motors();
	}

	int iter_pos(Imu imu){
		return -1;
	}

// only do -180-180 turns
void imuTurn(double degrees)
{
	if(degrees < 0)
	{
		imu.set_heading(350);
	}
	else
	{
		imu.set_heading(10);
	}
	float kP = 0.3;
	float kI = 0.1;
	float kD = 0.2;
	double target = imu.get_heading() + degrees;
	double error = target - imu.get_heading(); // -90
	double lastError = error;
	int power = 0;
	double integral = 0.0;
	double derivative = 0.0;

	while(abs(error) > 1.0)	{
		error = target - imu.get_heading();
		integral += error;
		if(abs(integral) >= 600){
			integral = 0;
		}
		derivative = error - lastError;
		power = error * kP + integral * kI + derivative *kD;
		lastError = error;
		LF.move(power); LM.move(power); LB.move(power); RF.move(-power); RM.move(-power); RB.move(-power);
		delay(10);
	}
	stop_motors();
}

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
	double kP = 0.4;
	double kI = 0.0025;
	double kD = 0.01;
	int integral = 0;
	int derivative = 0;
	int power = 0;
	int current_pos = 0;
	int error = 0;
	int prev_error = 0;
	error = 1700;
	while(error > 5){
		if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			break;
		}
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = 1700 - current_pos;
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
	stop_lift();
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
	while(abs(error)>7){
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
}


void winPointMoveDown(int target){
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
		power = min(abs(power), 69);
		power *= -1;
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

	void autoBalance(){
		float kP = 2.0;
		float kD = 0.2;
		float kI = 0.005;
		int power;
		float error;
		float pError;
		int integral;

		int derivative;
		while(abs(imu.get_pitch()) >= 1.5){
			if(con.get_digital(E_CONTROLLER_DIGITAL_X)){
				break;
			}
			error = 1.5 - imu.get_pitch();
			if(abs(error) <= 15 && error > 8){
				integral += error;
			}
			// else{
			// 	integral = 0;
			// }
			if(abs(integral) >= 250){
				integral = 0;
			}
			derivative = pError - error;
			power = kP*error + kI*integral + kD*derivative;
			LF.move(-power); LM.move(-power); LB.move(-power); RF.move(-power); RM.move(-power); RB.move(-power);
			pError = error;
			delay(5);
			if(abs(imu.get_pitch()) <= 9){
				stop_motors();
			}
		}
	}
	void driveLiftDown(int dTarget, int lTarget){
		reset_encoders();
		reset_lift();
		//target is in inches
		dTarget*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.1;
		double kI = 0.0025;
		double kD = 0.01;
		int integral = 0;
		int derivative = 0;
		int power = 0;
		int current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		int error = 0;
		int prev_error = 0;
		error = lTarget - current_pos;
		//----------
		double dkP = 0.5;
		double dkI = 0.01;
		double dkD = 0.02;
		int dintegral = 0;
		int dderivative = 0;
		int derror;
		int dprev_error;
		int dpower;
		int powerAdj = 0;
		int d_current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		derror = dTarget - d_current_pos;
		//-------------------
		while(abs(derror) >= 15 || abs(error)>7){
			d_current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
			current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
			error = lTarget - current_pos;
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
			if(abs(error) < 5){
				stop_lift();
			}
			else{
				lift_left.move(power); lift_right.move(power);
			}

			derror = dTarget - d_current_pos;
			dintegral += derror;
			if(derror == 0){
				dintegral = 0;
			}
			if(dintegral > 3000){
				dintegral = 0;
			}

			dderivative = derror - dprev_error;
			dprev_error = derror;
			dpower = dkP*derror + dintegral*dkI + dderivative*dkD;
			if(dpower < 0){
				dpower = min(dpower, -127);
			}
			else{
				if(dpower > 0){
					dpower = min(dpower, 127);
				}
			}
			powerAdj = dpower/10;

			if(abs(derror) < 15){
				stop_motors();
		}
		else{
			LF.move(dpower); LM.move(dpower); LB.move(dpower); RF.move(dpower-powerAdj); RM.move(dpower-powerAdj); RB.move(dpower-powerAdj);
		}
			delay(10);
		}
		stop_motors();
		stop_lift();
	}


	void currAuton(){

		driveLiftDown(110, -1900);
		delay(5);
		moveMogo(1300);
		turn(-180);
		// turnLift(-180, 1400);
		delay(5);
		drive(105);
		delay(5);

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
void competition_initialize() {
	imu.reset();
	while(imu.is_calibrating()) delay(100);
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


 //For left turns do -10 from wanted values
void autonomous() {
	lcd::initialize();

	// 	imu.reset();
	// 	delay(100);
	// 	while(imu.is_calibrating()) stop_motors();
	// con.set_text(1,1,"sup gamer");

  // //GLOBAL SAFE
	driveLiftDown(120, -1900);
	delay(5);
	moveMogo(1100);
	delay(5);
	drive(-105);
	delay(5);



	// imuTurn(90);

	// Left with imu

	// driveLiftDown(120, -1900);
	// delay(5);
	// moveMogo(1100);
	// delay(5);
	// drive(-75);
	// delay(5);
	// delay(5);
	// imuTurn(-110);
	// drive(20);
	// delay(5);
	// moveLift(-525);
	// delay(5);
	// drive(-35);
	// delay(5);
	// imuTurn(175);
	// delay(5);
	// drive(87);
	// delay(5);
	// moveMogo(1000);
	// delay(5);
	// drive(-100);
	// delay(5);

	//RIGHT Global but more imu
	//SETUP IS KEY

	// driveLiftDown(115, -1900); //Drive to neutral and set lift down
	// delay(5);
	// moveMogo(1200);// Lift the Neutral
	// delay(5);
	// drive(-70); // go backwards
	// delay(5);
	// imuTurn(126); // turn right
	// delay(5);
	// drive(30); // go forward a little
	// delay(15);
	// moveLift(-275); // drop the mobile goal
	// delay(5);
	// drive(-30); // Go back
	// delay(5);
	// imuTurn(-180); // turn to face the tall goal
	// delay(5);
	// drive(87); // drive to pick up
	// delay(15);
	// moveMogo(1550); // pick up
	// delay(6);
	// drive(-84); // go back
	// delay(5);

	// Winpoint
	// winPointMoveDown(-1900); // lift Down
	// delay(5);
	// drive(25);
	// delay(5);
	// moveMogo(1400);
	// delay(5);
	// drive(-30);

	//Skills?
	// moveLift(-1900); // Lift Down, the Lift starts at like 20 degrees les than a flat 90 from the top.
	// delay(5);
	// drive(20); //drive a little forward
	// delay(5);
	// moveMogo(1800); // lift mobile goal to position where if i run  function then it can get to the back
	// liftMobileGoal(); // put on back
	// delay(5);
	// turn(-80); // turn left
	// delay(5);
	// drive(200); // drive to toher side of field
	// delay(5);
	// turn(-90); // turn to face the mobile goal weighing the platform down
	// delay(5);
	// moveLift(-2000); // move lift down
	// delay(5);
	// drive(35); // drive to mogo
	// delay(5);
	// liftMobileGoal(); // pick it up and drop it into cage
	// delay(5);
	// turn(200); // turn to face other side
	// delay(5);
	// drive(220); // push neutral with us to other side
	// delay(5);
	// drive(-20); // go back a litle to line up with other mogo
	// delay(5);
	// moveLift(-1400); // put lift down
	// delay(5);
	// turn(-80); // turn to face mogo
	// delay(5);
	// drive(40); // drive and pick up mogo
	// delay(5);
	// moveMogo(1400); // hold mogo in lift
	// delay(5);
	// turn(-80); // turn to face other side of field
	// delay(5);
	// drive(200); // drive to the other side of the field
	// delay(5);
	// moveLift(-700); //put the lift down
	// delay(5);
	// drive(-100); // reverse out to face center neut mogo
	// delay(5);
	// turn(-75); // turn to facce neutral mogo
	// delay(5);
	// drive(50); // go forward and grab it
	// delay(5);
	// moveMogo(1400); // pick it up
	// delay(5);
	// turn(85); // turn right
	// delay(5);
	// drive(50); // put in front of platform
	// delay(5);
	// moveLift(-700); // put lift down
	// delay(5);
	// drive(-50); // reverse back to where once was
	// delay(5);
	// turn(-80); // turn left again
	// delay(5);
	// drive(60); // drive forward and pick up 3rd mogo
	// delay(5);
	// moveMogo(1400); //pick up neutral
	// delay(5);
	// turn(90); // turn right to face platform
	// delay(5);
	// drive(90); // drive into a home zone
	// delay(5);
	// moveLift(-700); // put mogo down
	// delay(5);
	// drive(-170); // drive to other color side of mogo alliance

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
	imu.reset();
	delay(100);
	while(imu.is_calibrating())	stop_motors();
	// cout << "this is working" << endl;
	// cout << "This is working" << endl;
	while (true) {
		// cout << "Heading Value: " << imu.get_heading() << endl;
		// cout << "Pitch: " << imu.get_pitch() << endl;
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
			int right = power - turn-(power/25);
			//if drives forward, right side goes faster than left or left goes slower, left
			// most likely will not continue to any faster, so plan is to reduce right side speed
			// if it is turning, then left side will turn slower than before theoretically, but
			// turn faster for the right side as well, making it even?
				//tbh still tryna figure out how this sum difference thing works

				//put the left and right integers down here
				LF.move(left);
				LM.move(left);
				LB.move(left);
				RF.move(right);
				RM.move(right);
				RB.move(right);

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
					lift_left.move(-127);
					lift_right.move(-127);
				}
					//lift go no
					else{
						lift_left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
							lift_left.move_velocity(0);
						lift_right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
							lift_right.move_velocity(0);

						}
				if(con.get_digital(E_CONTROLLER_DIGITAL_A)){
					// cout << "Pressed A" << endl;
					liftMobileGoal();
				}

				if(con.get_digital(E_CONTROLLER_DIGITAL_Y)) autoBalance();

				if(con.get_digital(E_CONTROLLER_DIGITAL_UP)){
						currAuton();
				}
 //OK
				delay(5);
			}
	}
