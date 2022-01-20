#include "main.h"
#include "global.h"

#ifndef _PID_
#define _PID_
using namespace glb;

 void reset_chassis() {
   RF.tare_position();
   RM.tare_position();
   RB.tare_position();
   LF.tare_position();
   LM.tare_position();
   LB.tare_position();
 }

 void drive(int target) {
   reset_chassis();
   // con.clear();
   int localTime = 0;
   imu.set_heading(180);
	 target *= 28.65;
 	 double heading = imu.get_heading();
   float kP = 0.4;
   float kI = 0.01;
   float kD = 0;
   float akP = 1;
   int integral = 0;
   int derivative = 0;
   int prev_error = 0;
   int currPos = (RF.get_position() + RM.get_position() + RB.get_position() + LF.get_position() + LM.get_position() + LB.get_position()) / 6;
   int error = target - currPos;

   while (abs(error) > 1) {
     heading = imu.get_heading();
     currPos = (RF.get_position() + RM.get_position() + RB.get_position() + LF.get_position() + LM.get_position() + LB.get_position()) / 6;
     error = target - currPos;
     integral += error;
     if(integral > 1500) {
       integral = 1500;
     }//Gerald was here
     if(error > 2000) {
       integral = 0;
     }
     derivative = error - prev_error;
     prev_error = error;
     int power = kP * error + integral * kI + derivative * kD;
     int powerDiff = akP * (180 - heading);
     if(localTime%50 == 0) {
       con.print(0, 0, "Heading: %d", heading);
       // con.print(1, 0, "Power Diff: %d", powerDiff);
       // con.print(2, 0, "Power: %d", power);
     }
     LF.move(power + powerDiff); LM.move(power + powerDiff); LB.move(power + powerDiff); RF.move(power - powerDiff); RM.move(power - powerDiff); RB.move(power - powerDiff);
     localTime += 5;
     delay(5);
   }
   LF.move(0); LM.move(0); LB.move(0); RF.move(0); RM.move(0); RB.move(0);
 }

 void turn(int degrees) {

 }

 void toggleClamp() {
 		static bool autonPiston = false;

 }

 void twoBarDown() {

 }

 void twoBarUp() {

 }

 void liftUp() {

 }

 void liftDown() {

 }

 void grabNeutral() {
   drive(100);
 }
 #endif
