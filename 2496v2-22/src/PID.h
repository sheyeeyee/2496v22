#include "main.h"
#include "global.h"
#include <cmath>
#ifndef _PID_
#define _PID_
using namespace glb;
using namespace std;

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

  void stop_motors(){
    RF.move(0);
    RM.move(0);
    RB.move(0);
    LF.move(0);
    LM.move(0);
    LB.move(0);
    // RF.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    // RM.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    // RB.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    // LF.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    // LM.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    // LB.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  }
 void reset_chassis() {
   RF.tare_position();
   RM.tare_position();
   RB.tare_position();
   LF.tare_position();
   LM.tare_position();
   LB.tare_position();
 }

 void holdLift() {
   LIFT.move_velocity(0);
   LIFT.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
 }

 void drive(int target){
   // reset_encoders();
   reset_chassis();
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
     powerAdj = power / 9.5;
     LF.move(power-powerAdj); LM.move(power-powerAdj); LB.move(power-powerAdj); RF.move(power); RM.move(power); RB.move(power);
     delay(10);
   }
   stop_motors();
 }

 void straightDrive(int target) {
   reset_chassis();
   target *= 28.65;
   double kP = 0.6;
   double kI = 0.01;
   double kD = 0.0;
   int integral = 0;
   int derivative = 0;
   int error;
   int prev_error;
   double power;
   int powerAdj = 5;
   double powerAdjConst = 11;
   int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
   imu.set_heading(90);
   while(abs(error) >= 15) {
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
       if(power < -127) {
         power = -127;
       }
     }
     else{
       if(power > 0){
         if(power > 127) {
           power = 127;
         }
       }
     }
     powerAdj = (imu.get_heading()-90) * powerAdjConst;
     LF.move(power- powerAdj); LM.move(power-powerAdj); LB.move(power-powerAdj); RF.move(power+powerAdj); RM.move(power+powerAdj); RB.move(power+powerAdj);
     delay(10);
   }
   stop_motors();
  }

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
   	float kI = 0.043;
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
      con.print(0, 0, "turn", power);
   	}
   	stop_motors();
   }

 void toggleClamp() {
 		static bool autonPiston = false;
    if(autonPiston) {
      piston.set_value(false);
      autonPiston = false;
    }
    else {
      piston.set_value(true);
      autonPiston = true;
    }
 }

 void twoBarDown() {
   INTAKE.move_absolute(2000, 100);
 }

 void twoBarUp() {
   INTAKE.move_absolute(15, 100);
 }

 void liftUp() {
   LIFT.move_absolute(2800, 100);
 }

void liftMedUp() {
  LIFT.move_absolute(1400, 100);
}

 void liftDown() {
   LIFT.move_absolute(0, 100);
 }

 void grabNeutral() {
   drive(90);
   toggleClamp();
   delay(5);
   drive(-80);
   delay(5);
   // imuTurn(-90);
   // toggleClamp();
 }

 void grabCenter() {
   drive(36);
   delay(5);
   imuTurn(-50);
   delay(5);
   liftDown();
   delay(5);
   drive(90);
   delay(5);
   toggleClamp();
   delay(500);
   drive(-110);
 }

 void halfLeftAwp() {
   INTAKE.move_absolute(1250, 100);
   delay(1000);
   twoBarUp();
   //imuTurn no work)
   imuTurn(95);
   delay(5);
   drive(90);
   delay(5);
   toggleClamp();
   delay(400);
   drive(-50);
 }

 void halfRightAwp() {
   drive(-10);
   delay(300);
   INTAKE.move_absolute(1250, 100);
   delay(500);
   drive(-4);
   delay(1000);
   twoBarUp();
   delay(5);
   drive(15);
   //taking out turn bc imuTurn no work
   delay(5);
   imuTurn(90);
   delay(5);
   liftDown();
   delay(5);
   drive(120);
   toggleClamp();
   drive(-120);
   // delay(5);
   // imuTurn(-50);
   // delay(5);
   // liftDown();
   // delay(5);
   // drive(90);
   // delay(5);
   // toggleClamp();
   // delay(500);
   // drive(-110);

 }

 void soloAwpLeft(){
   INTAKE.move_absolute(800, 40);
   delay(800);
   INTAKE.move_absolute(15, 100);
   delay(5);
   straightDrive(5);
   delay(3.0);
   imuTurn(-90);
   delay(5);
   straightDrive(-10);
   delay(5);
   imuTurn(-90);
   delay(5);
   straightDrive(115);
   delay(5);
   LIFT.move_absolute(1500, 100);
   delay(600);
   holdLift();
   delay(5);
   imuTurn(-12);
   delay(5);
   straightDrive(25);
   //160 distance (110 + 50) + mogo shove (15)
   delay(500);
   toggleClamp(); //drop ring
   delay(5);
   // drive(10);
   // delay(5);
   imuTurn(10);
   delay(4);
   straightDrive(-10);
   delay(15);
   LIFT.move_absolute(15, 100);
   delay(5);
   straightDrive(15);
   delay(5);
   toggleClamp();
   delay(5);
   straightDrive(-20);

 }

 void soloAwpRight(){
    straightDrive(-20);
    delay(5);
    INTAKE.move_absolute(1500, 100);
    delay(5);
    INTAKE.move_absolute(15,100);
    delay(5);
    imuTurn(90);
    delay(50);
    straightDrive(160);
    delay(5);
    imuTurn(-90);
    delay(5);
    LIFT.move_absolute(1000, 100);
    delay(5);
    straightDrive(25);
    delay(5);
    toggleClamp();
    delay(5);
    drive(-5);
    delay(5);
    imuTurn(180);
    delay(5);


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

 #endif
