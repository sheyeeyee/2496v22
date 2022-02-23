#include "main.h"
#include <string>
#include <math.h>
#include "global.h"
#include "PID.h"
using namespace pros;
using namespace std;
using namespace glb;

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
