#include "main.h"
using namespace std;
using namespace pros;

class PID{
public:
  int target;
  int current_error;
  int prev_error;
  int d_value;
  double kP, kI, kD;
  int i_value;
  bool is_enabled;
  PID(double kP, double kI, double kD, bool is_enabled){
    this -> kP = kP; this -> kI = kI; this -> kD = kD; this -> is_enabled = is_enabled;
  }


  void reset(bool is_enabled){
    prev_error = 0;
    i_value = 0;
    this -> is_enabled = is_enabled;
  }

  int update(){
    if(is_enabled){
      i_value = i_value + current_error;
      d_value = current_error - prev_error;
      prev_error = current_error;
      return current_error*kP + i_value*kI + d_value*kD;
    }
    return -1;
  }

  void setTarget(int tar){
    target = tar;
  }
};

class chassisPID : PID{
  public:

    chassisPID(double kP, double kI, double kD, bool is_enabled):PID(kP, kI, kD, is_enabled){
    };

    int current_pos;

    int moveStraight(Motor RF, Motor RM, Motor RB, Motor LF, Motor LM, Motor LB){
      // current_error = target - (RF.get_position() + RM.get_position() + RB.get_position() + LF.get_position()+LM.get_position()+LB.get_position())/6;
      current_error = (LF.get_position() + LM.get_position() + LB.get_position())/3 - (RF.get_position() + RM.get_position() + RB.get_position())/3;
      int rightPower = PID::update();
      return rightPower;
    }

};
