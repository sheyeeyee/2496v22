#include "main.h"
using namespace std;

class PID{
private:
  int target;
  int current_error;
  double kP, kI, kD;
  int i_value;
  bool is_enabled;
  int prev_error;
public:
  PID(double kP, double kI, double, kD, bool is_enabled){
    this -> kP = kP; this -> kI = kI; this -> kD = kD; this is_enabled = is_enabled;
  }

  void reset(bool enable){
    prev_error = 0;
    i_value = 0;
    is_enabled = enable;
  }

  int update(){ 
    if(is_enabled){
      i_value = max(i_value += current_error, 500);
      int d_value = current_error - prev_error;
      prev_error = current_error
      return current_error * kP + i_value*kI + d_value*kD;
    }
  }

  void changeTarget(int tar){
    this -> target = tar;
  }
};
