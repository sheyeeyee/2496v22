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
   LIFT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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
     powerAdj = 0;
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

 void straightDriveDist(int target) {
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
    while(abs(error) >= 15 && dist.get() > 30) {
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

   void straightBalance(int target) {
      reset_chassis();
      target *= 28.65;
      double kP = 0.6;
      double kI = 0.01;
      double kD = 0.01; //increase? slowing it down?
      int integral = 0;
      int derivative = 0;
      int error;
      int prev_error;
      double power;
      int powerAdj = 5;
      double powerAdjConst = 11;
      int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
      imu.set_heading(90);
      // con.print(0, 0, imu.get_heading()); //print angle of robot? because i want to see the angle changes
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
    float kI = 0.055;
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
   LIFT.move_absolute(20, 100);
 }

 void grabNeutral() {
   toggleClamp();
   LIFT.move_absolute(40, 100);
   straightDriveDist(120);
   toggleClamp();
   holdLift();
   delay(5);
   straightDrive(-80);
   delay(5);

 }

 void soloAwpLeft(){
   //drop ring in FIRST
   INTAKE.move_absolute(1300, 80);
   delay(1000);
   INTAKE.move_absolute(15, 100);
   delay(5);

   //drive away from goal to avoid hitting it
   straightDrive(5);
   delay(3.0);

   //turn towards center of field
   imuTurn(90);
   delay(5);

   //drive towards center
   straightDrive(20);
   delay(5);

   //turn towards other goal
   imuTurn(89);
   delay(5);

   //drive to SECOND goal
   straightDrive(115);
   delay(5);

   //lift up so it's above goal and can drop ring
   LIFT.move_absolute(1600, 100);
   delay(600);
   holdLift();
   delay(5);

   //turn to accurately face goal
   imuTurn(-4);
   delay(5);

   //towards goal
   straightDrive(18);
   //160 distance (110 + 50) + mogo shove (15)
   delay(10);

   //drop ring
   toggleClamp();
   delay(1000);
   //back to position to grab goal
   straightDrive(-10);
   delay(5);

   //lift down to grab goal
   // LIFT.set_brake_mode(E_MOTOR_BRAKE_COAST);
   LIFT.move_absolute(35, 100);
   delay(1000);

   //go forward to goal
   // imuTurn(5);
   // delay(5);
   straightDrive(40);
   delay(5);

   //grab goal
   toggleClamp();
   delay(300);

   //turn away from platform to avoid hitting alliance robot
   // LIFT.move_absolute(500, 100);

   // delay(300);
   // holdLift();
   // imuTurn(10);
   // delay(4);

   //clear line
   straightDrive(-40);
   // imuTurn(10);
   // straightDrive(-40);
   delay(5);
   // //drop mogo
   toggleClamp();
   // delay(5);

 }

 void grabBothRight() {
   toggleClamp();
   LIFT.move_absolute(40, 100);
   straightDriveDist(100);
   toggleClamp();
   holdLift();
   delay(5);
   straightDrive(-50);
   delay(5);
   LIFT.move_absolute(450, 100);
   delay(5);
   imuTurn(-90);
   delay(5);
   straightDrive(20);
   delay(5);
   toggleClamp();
   delay(5);
   LIFT.move_absolute(250, 100);
   straightDrive(-20);
   delay(5);
   imuTurn(37);
   delay(5);
   LIFT.move_absolute(40, 100);
   delay(5);
   straightDrive(83);
   delay(5);
   toggleClamp();
   holdLift();
   INTAKE.move_absolute(2000, 100);
   delay(5);
   straightDrive(-80);
 }

 void grabCenter() {
   toggleClamp();
   straightDrive(30);
   delay(5);
   imuTurn(-50);
   delay(5);
   LIFT.move_absolute(40, 100);
   delay(5);
   straightDriveDist(85);
   delay(5);
   toggleClamp();
   holdLift();
   delay(500);
   straightDrive(-90);
 }

 void halfLeftAwp() {
   INTAKE.move_absolute(1200, 90);
   delay(1000);
   twoBarUp();
   imuTurn(104);
   LIFT.move_absolute(20, 100);
   toggleClamp();
   delay(5);
   holdLift();
   straightDriveDist(100);
   delay(5);
   toggleClamp();
   delay(400);
   straightDrive(-70);
 }

 void halfRightAwp() {
   straightDrive(-7);
   delay(00);
   INTAKE.move_absolute(1250, 55);
   delay(1000);
   straightDrive(-11);
   delay(1000);
   twoBarUp();
   delay(5);
   straightDrive(5);
   delay(5);
   imuTurn(93);
   LIFT.move_absolute(40, 100);
   toggleClamp();
   delay(5);
   liftDown();
   delay(5);
   straightDriveDist(105);
   toggleClamp();
   straightDrive(-105);

 }


void autoBalance(){
  double target = 0; //target is 0 degrees
  //float error; //wait can error just be equal to negative degree?
  int power; //establish power as a variable
  int integral = 0; //establish integral
  int derivative; //establish derivative
  float prevError;
  float error = -imu.get_pitch();
  float kP = 2.5; //values to be changed during testing
  float kI = 0.018;
  float kD = 0.1;
  int powerAdjConst = 11; //power adjustment constant
  double powerAdj; //establish power adjustment now because it's not in the while loop
  imu.set_heading(90);

  while(abs(error) > 3) { //as long as the absolute value of the current pitch value is greater than 1.5
    error = -imu.get_pitch();
    integral = integral + error;
    derivative = error - prevError; //derivative is equal to how much you've traveled since the last time it checked
    prevError = error; //current error is now what it was right before3

    power = error*kP + integral*kI + derivative*kD; //calculate the power by adding all
    powerAdj = (imu.get_heading()-90) * powerAdjConst; //adjust for straightness
    // powerAdj = 0;
    if(abs(integral) >= 800) {
      if(integral > 0) integral = 800;
      else integral = -800;
    }
    // if(abs(error) <= 4) {
    //   integral = 0;
    // }

    LF.move(power-powerAdj); LM.move(power-powerAdj); LB.move(power-powerAdj); RF.move(power+powerAdj); RM.move(power+powerAdj); RB.move(power+powerAdj);
    delay(5); //the interval at which it refreshes/recalculates the error, integral, and derivative
  }
  straightDrive(-5);
  park();
}

void progSkog() {
  // move the two bar down anad pick up the alliance
  INTAKE.move_absolute(2000, 100);
  delay(1000);
  straightDrive(-25);
  delay(1000);
  INTAKE.move_absolute(20, 100);

  // get into a position to grab the neutral
  delay(1000);
  imuTurn(90);
  delay(5);
  straightDrive(28);
  delay(5);
  imuTurn(-90);

  //Go grab neutral and get to other side

  INTAKE.move_absolute(2000, 100);
  delay(1000);
  straightDrive(-35);
  delay(1000);
  INTAKE.move_absolute(1000, 100);
  delay(1000);
  imuTurn(15);
  straightDrive(-22);
  delay(1000);
  imuTurn(75);
  delay(1000);
  straightDrive(-10);
  delay(5);
  INTAKE.move_absolute(2000, 100);
  delay(2000);

  //drive and clmap blue
  straightDrive(150);
  toggleClamp();
}

 #endif
