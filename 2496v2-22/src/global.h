#include "main.h"
using namespace pros;
#ifndef _GLOBALS_
#define _GLOBALS_
namespace glb {

  #define P_LF 17
  #define P_LM 9
  #define P_LB 10
  #define P_RF 19
  #define P_RM 18
  #define P_RB 12
  #define P_LIFT 11
  #define P_INTAKE 8
  #define P_LPISTON 'A'
  #define P_SELECTOR 'B'
  #define P_IMU 21

  Motor LF (P_LF, E_MOTOR_GEARSET_06, true);
  Motor LM (P_LM, E_MOTOR_GEARSET_06);
  Motor LB (P_LB, E_MOTOR_GEARSET_06, true);
  Motor RF (P_RF, E_MOTOR_GEARSET_06);
  Motor RM (P_RM, E_MOTOR_GEARSET_06, true);
  Motor RB (P_RB, E_MOTOR_GEARSET_06);
  Motor LIFT (P_LIFT, E_MOTOR_GEARSET_36);
  Motor INTAKE (P_INTAKE, E_MOTOR_GEARSET_36);

  ADIDigitalOut piston (P_LPISTON);
  ADIDigitalIn button (P_SELECTOR);

  Controller con (CONTROLLER_MASTER);

  Imu imu (P_IMU);

}
#endif
