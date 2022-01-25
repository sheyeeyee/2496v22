#include "main.h"
#include "global.h"
#include <cmath>
#ifndef _PID_
#define _PID_
using namespace glb;
using namespace std;

  void stop_motors(){
    RF.move(0);
    RM.move(0);
    RB.move(0);
    LF.move(0);
    LM.move(0);
    LB.move(0);
  }
 void reset_chassis() {
   RF.tare_position();
   RM.tare_position();
   RB.tare_position();
   LF.tare_position();
   LM.tare_position();
   LB.tare_position();
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
     powerAdj = power / 20;
     LF.move(power-powerAdj); LM.move(power-powerAdj); LB.move(power-powerAdj); RF.move(power); RM.move(power); RB.move(power);
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
      autonPiston = false;
    }
 }

 void twoBarDown() {
   INTAKE.move_absolute(2000, 100);
 }

 void twoBarUp() {
   INTAKE.move_absolute(15, 100);
 }

 void liftUp() {

 }

 void liftDown() {
   LIFT.move_absolute(0, 100);
 }

 void grabNeutralLeft() {
   drive(110);
   toggleClamp();
   delay(5);
   drive(-40);
   delay(5);
   imuTurn(-90);
   toggleClamp();
 }

 void halfLeftAwp() {
   INTAKE.move_absolute(1250, 100);;
   delay(1000);
   twoBarUp();
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
   INTAKE.move_absolute(1250, 100);
   delay(1000);
   twoBarUp();
   delay(5);
   drive(10);
   delay(5);
   imuTurn(-90);
   delay(5);
   drive(-15);
   delay(5);
   imuTurn(-90);
   delay(5);
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


 #endif
