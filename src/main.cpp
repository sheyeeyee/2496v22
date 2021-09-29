#include "main.h"
#include "PID.h" //header file for organization ig
#include <cmath> //for maths in case we need it?
using namespace pros;
using namespace std;

//testseithithaihihasihfihfiahfihfiah

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
		//ADIAnalogIn lift_pot('A');

	//controller
	Controller con (CONTROLLER_MASTER);



	//main auton and used for the singular button press
	void currAuton(){
		driveLiftDown(115, -1900); //Drive to neutral and set lift down
		delay(5);
		moveMogo(1200);// Lift the Neutral
		delay(5);
		drive(-70); // go backwards
		delay(5);
		imuTurn(126); // turn right
		delay(5);
		drive(30); // go forward a little
		delay(15);
		moveLift(-750); // drop the mobile goal
		delay(5);
		drive(-30); // Go back
		delay(5);
		imuTurn(-180); // turn to face the tall goal
		delay(5);
		drive(92); // drive to pick up
		delay(15);
		moveMogo(1200); // pick up
		delay(5);
		drive(-80); // go back
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

		imu.reset();
		delay(100);
		while(imu.is_calibrating()) stop_motors();
	con.set_text(1,1,"sup gamer");


	// imuTurn(90);

	// Left with imu
	// driveLiftDown(120, -1900);
	// delay(5);
	// moveMogo(1100);
	// delay(5);
	// drive(-55);
	// delay(5);
	// delay(5);
	// imuTurn(-110);
	// drive(30);
	// delay(5);
	// moveLift(-600);
	// delay(5);
	// drive(-35);
	// delay(5);
	// imuTurn(180);
	// delay(5);
	// drive(87);
	// delay(5);
	// moveMogo(1000);
	// delay(5);
	// drive(-100);
	// delay(5);


	//RIGHT Global but more imu
	//SETUP IS KEY

	driveLiftDown(115, -1900); //Drive to neutral and set lift down
	delay(5);
	moveMogo(1200);// Lift the Neutral
	delay(5);
	drive(-70); // go backwards
	delay(5);
	imuTurn(126); // turn right
	delay(5);
	drive(30); // go forward a little
	delay(15);
	moveLift(-750); // drop the mobile goal
	delay(5);
	drive(-30); // Go back
	delay(5);
	imuTurn(-180); // turn to face the tall goal
	delay(5);
	drive(92); // drive to pick up
	delay(15);
	moveMogo(1200); // pick up
	delay(5);
	drive(-80); // go back
	delay(5);

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
	delay(90);
	while(imu.is_calibrating())	stop_motors();
	// cout << "this is working" << endl;

	while (true) {
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
							joystick is for going left and right.
							-SWJ**/
			int left = power + turn;
			int right = power - turn - power/10;
			/**if drives forward, right side goes faster than left or left goes slower, left
			 most likely will not continue to any faster, so plan is to reduce right side speed
			if it is turning, then left side will turn slower than before theoretically, but
			turn faster for the right side as well, making it even?
				tbh still tryna figure out how this sum difference thing works
				-SWJ**/

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

				//autobalance
				if(con.get_digital(E_CONTROLLER_DIGITAL_Y)) autoBalance();

				//run auton during scrims
				if(con.get_digital(E_CONTROLLER_DIGITAL_UP)) {
					currAuton();
				}

				delay(5);
			}
	}
