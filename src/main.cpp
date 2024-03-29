#include "main.h"
#include "PID.h" //header file for organization ig
#include <cmath> //for maths in case we need it?
using namespace pros;
using namespace std;

//CONSTRUCTORS
	//chassis
		//left drive

		Motor LF (16, E_MOTOR_GEARSET_18, true);
		Motor LM (15, E_MOTOR_GEARSET_18, true);
		Motor LB (10, E_MOTOR_GEARSET_18, true);
		//right drive
		Motor RF (17, E_MOTOR_GEARSET_18);
		Motor RM (20, E_MOTOR_GEARSET_18);
		Motor RB (14, E_MOTOR_GEARSET_18);
			//inertial sensor for auton PID
		Imu imu (19);

	//lift
	Motor lift_left (3, E_MOTOR_GEARSET_06, true);
	Motor lift_right (6, E_MOTOR_GEARSET_06);
		//potentiometer for PID
		// ADIAnalogIn lift_pot('A');


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
		double kP = 0.6;
		double kI = 0.01;
		double kD = 0.0;
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
			powerAdj = 0;
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
	int localTime = 0;
	if(degrees < 0)
	{
		imu.set_heading(350);
	}
	else
	{
		imu.set_heading(10);
	}
	float kP = 0.7;
	float kI = 0.053;
	float kD = 0.0;
	double target = imu.get_heading() + degrees;
	double error = target - imu.get_heading(); // -90
	double lastError = error;
	int power = 0;
	double integral = 0.0;
	double derivative = 0.0;

	while(abs(error) > 0.5)	{
		error = target - imu.get_heading();
		integral += error;
		if(abs(integral) >= 600){
			integral = 0;
		}
		derivative = error - lastError;
		power = error * kP + integral * kI + derivative *kD;
		lastError = error;
		LF.move(power); LM.move(power); LB.move(power); RF.move(-power); RM.move(-power); RB.move(-power);
		delay(5);
	}
	stop_motors();
}


void liftMobileGoal(){
	stop_motors();
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
	error = 2100;
	while(error > 5){
		if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			break;
		}
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = 2100 - current_pos;
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
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
	stop_motors();
}

void liftMobileGoal2() {
	stop_motors();
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
	error = 2400;
	while(error > 5){
		if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			break;
		}
		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = 2400 - current_pos;
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
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
	stop_motors();
	reset_lift();
}

void autonLiftMobileGoal(){
	stop_motors();
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
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
	stop_motors();
	reset_lift();
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
		stop_lift();
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
	void park(){

			LF.move_velocity(0);
			LM.move_velocity(0);
			LB.move_velocity(0);
			RF.move_velocity(0);
			RM.move_velocity(0);
			RB.move_velocity(0);
			RF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			RM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			RB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			LM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			LB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	}
	void autoBalance(){
		float kP = 2.35;
		float kD = 0.0;
		float kI = 0.0;
		int power;
		float error;
		float pError;
		int integral;

		int derivative;
		int localTime= 0;
		con.clear();
		while(abs(imu.get_pitch()) >= 1.5){
			if(con.get_digital(E_CONTROLLER_DIGITAL_X)){
				break;
			}
			error = 1.5 - imu.get_pitch();
			if(abs(error) <= 15 && error > 8){
				integral += error;
			}
			else {
				integral = 0;
			}
			// else{
			// 	integral = 0;
			// }
			if(abs(integral) >= 350){
				integral = 0;
			}
			derivative = pError - error;
			// power = kP*error + kI*integral + kD*derivative;
			power = imu.get_pitch() * kP;
			LF.move(power); LM.move(power); LB.move(power); RF.move(power); RM.move(power); RB.move(power);
			pError = error;

			if(localTime % 50 == 0) {
				con.print(0,0, "Pitch: %.2f", imu.get_pitch());
			}
			if(abs(imu.get_pitch()) <= 20.5){
				park();
			}
			delay(5);
			localTime += 5;

		}
	}
	void driveLiftDown(int dTarget, int lTarget){
		reset_encoders();
		reset_lift();
		//target is in inches
		dTarget*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.4;
		double kI = 0.01;
		double kD = 0.01;
		int integral = 0;
		int derivative = 0;
		int power = 0;
		int current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		int error = 0;
		int prev_error = 0;
		error = lTarget - current_pos;
		//----------
		double dkP = 0.2;
		double dkI = 0.0;
		double dkD = 0.3;
		int dintegral = 0;
		int dderivative = 0;
		int derror;
		int dprev_error;
		int dpower;
		int powerAdj = 0;
		int d_current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		derror = dTarget - d_current_pos;
		//-------------------
		while(abs(derror) >= 100 || abs(error)>100){
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
			powerAdj = 0;

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


	void turnLift(double degrees, int target)
	{
		if(degrees < 0)
		{
			imu.set_heading(350);
		}
		else
		{
			imu.set_heading(10);
		}
		reset_lift();
		float tkP = 0.2;
		float tkI = 0.1;
		float tkD = 0.2;
		double tTarget = imu.get_heading() + degrees;
		double tError = tTarget - imu.get_heading(); // -90
		double tlastError = tError;
		int tpower = 0;
		double tintegral = 0.0;
		double tderivative = 0.0;
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

		while(abs(tError) > 1.0 || abs(error)>target/2-100) {
			tError = tTarget - imu.get_heading();
			tintegral += tError;
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
			if(abs(tintegral) >= 600){
				tintegral = 0;
			}
			tderivative = tError - tlastError;
			tpower = tError * tkP + tintegral * tkI + tderivative * tkD;
			tlastError = tError;
			power = kP * error + integral * kI + derivative * kD;
			prev_error = 0;
			if(abs(tError) < 1.0){
				stop_motors();
			}
			if(abs(error)<target/2-100) {
				stop_lift();
				park_lift();
			}
			LF.move(tpower); LM.move(tpower); LB.move(tpower); RF.move(-tpower); RM.move(-tpower); RB.move(-tpower);
			lift_left.move(power); lift_right.move(power);
			delay(10);
		}
		park_lift();
		stop_motors();
	}

	void currAuton(){
		driveLiftDown(95, -2000);
		delay(5);
		turnLift(-173, 500);
		delay(5);
		drive(80);

	}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * competition-specific initialization routines, such as an autonomous selector
 * Management System or the VEX Competition Switch. This is intended for
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
	// imu.reset();
	// while(imu.is_calibrating()) delay(100);
	// 	delay(100);
	// 	while(imu.is_calibrating()) stop_motors();
	// con.set_text(1,1,"sup gamer");

	//Right awp
	// imuTurn(45);
	// delay(5);
	// drive(32);
	// delay(1000);
	// moveLift(-900);
	// delay(15);
	// moveLift(900);
	// delay(5);
	// drive(13);
	// delay(5);
	// drive(-15);
	// delay(5);
	// moveLift(-1000);
	// delay(5);

	//Left 1/2 Awp + central neutral?
	// moveLift(-1000);
	// delay(1000);
	// drive(-15);
	// imuTurn(-90);
	// delay(5);
	// drive(70);
	// delay(5);
	// imuTurn(90);
	// delay(5);
	// drive(-10);
	// delay(5);
	// moveLift(-900);
	// delay(5);
	// drive(220);
	// delay(5);
	// drive(-29);
	// delay(5);
	// moveLift(600);
	// delay(5);
	// imuTurn(-90);
	// delay(5);
	// drive(-10);
	// delay(5);
	// moveLift(-600);
	// delay(5);
	// drive(62);
	// delay(5);
	// moveMogo(1100);
	// delay(5);
	// drive(-75);

	//GLOBAL SAFE
	// driveLiftDown(95, -2000);
	// delay(5);
	// turnLift(-173, 500);
	// delay(5);
	// drive(80);


	// imuTurn(90);

			// Left with imu

				// driveLiftDown(105, -1850);
				// delay(5);
				// moveMogo(1100);
				// delay(5);
				// drive(-95);
				// delay(5);
				// imuTurn(-125);
				// delay(5);
				// drive(10);
				// delay(5);
				// moveLift(-725);
				// delay(5);
				// drive(-30);
				// delay(5);
				// imuTurn(168);
				// delay(5);
				// drive(87);
				// delay(5);
				// moveMogo(1250);
				// delay(5);
				// drive(-100);

//anti-pneumatic
			driveLiftDown(85, -1500); //Drive to neutral and set lift down
			delay(5);
			imuTurn(90); //tip mogo
			delay(5);
			drive(-40);//drive back
			delay(5);
			imuTurn(55);//turn towards central
			delay(5);
			drive(55);//drive to central
			delay(5);
			imuTurn(45);//tip central
			delay(5);
			drive(-40);//back up

//not sure what i was thinking
			// imuTurn(-45);
			// delay(5);
			// drive(15);
			// delay(5);
			// moveLift(-1000);
			// delay(5);
			// drive(15);
			// delay(5);
			// moveMogo(750);




	//RIGHT Global but more imu
	//SETUP IS KEY
	//
	// driveLiftDown(95, -2000); //Drive to neutral and set lift down
	// delay(5);
	// moveMogo(700);// Lift the Neutral
	// delay(5);
	// drive(-40); // go backwards
	// delay(5);
	// imuTurn(-90); // turn right
	// delay(5);
	// drive(30); // go forward a little
	// delay(15);
	// moveLift(-1100); // drop the mobile goal
	// delay(5);
	// drive(-25); // Go back
	// delay(5);
	// imuTurn(180); // turn to face the tall goal
	// delay(5);
	// drive(40); // drive to pick up
	// delay(15);
	// liftMobileGoal(); // pick up
	// delay(5);
	// imuTurn(-180);
	// delay(5);
	// moveLift(-1800);
	// delay(5);
	// drive(30); // go back
	// delay(5);

	//Skills ?
	// moveLift(-2250); // move the lift down
	// delay(5);
	// drive(40); // go forward for mogo
	// delay(5);
	// autonLiftMobileGoal(); // put mogo into rack
	// delay(1000);
	// drive(12); // go forward to line up with the neutral
	// delay(1000);
	// imuTurn(-89); // turning left
	// delay(5);
	// drive(-15); //wall reset
	// delay(5);
	// moveLift(-1900); //
	// delay(1000);
	// drive(120);
	// delay(1000);
	// moveMogo(2700);
	// delay(1000);
	// drive(98);
	// delay(5);
	// imuTurn(92);
	// delay(5);
	// drive(245);
	// delay(5);
	// drive(-20);
	// delay(5);
	// imuTurn(-90);
	// delay(5);
	// drive(30);
	// delay(5);
	// imuTurn(-80);
	// delay(5);
	// drive(100);
	// delay(5);
	// imuTurn(-5);
	// delay(5);
	// while(1 == 1) autoBalance();

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
	// delay(100);
	// while(imu.is_calibrating())	stop_motors();
	// cout << "this is working" << endl;
	// cout << "This is working" << endl;
	int localTime = 0;
	// con.clear();
	while (true) {
		// if(localTime % 50 == 0) {con.print(0,0, "Pitch: %.2f", imu.get_pitch());}
		// if(localTime % 50 == 0) {
		// 	con.print(0,0, "Right Front: %f", RF.get_position());
		// }
		// cout << RF.get_position() << endl;
		// cout << "Heading Value: " << imu.get_heading() << endl;
		// cout << "Pitch: " << imu.get_pitch() << endl;

		//chassis (arcade drive)
			/**Set the integers for moving right and left so you can place them in the
					function.
					-SWJ**/
			int power = con.get_analog(ANALOG_LEFT_Y);
				/**The power integer is set on the left joystick, ANALOG_LEFT, and has a
							Y at the end bc that is the vertical axis and the left joystick is
							for going forwards and backwards.
							-SWJ**/
		 	int turn = con.get_analog(ANALOG_RIGHT_X);
				/**The turn integer is set to the right joystick, ANALOG_RIGHT, and has
							an X at the end bc that is the horizontal axis and the right
							joystick is for going left and right.**/
			int adj = 0;
			int left = power + turn;
			int right = power - turn-adj;
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
				else if(con.get_digital(E_CONTROLLER_DIGITAL_Y)){
					lift_left.move(-15);
					lift_right.move(-15);
				}//gerald was here
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
				//gerald was here too
				// if(con.get_digital(E_CONTROLLER_DIGITAL_Y)) autoBalance();

				if(con.get_digital(E_CONTROLLER_DIGITAL_UP)){
						currAuton();
				}
				localTime+=5;
				delay(5);
			}
	}
