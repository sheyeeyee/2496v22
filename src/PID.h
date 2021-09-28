#include "main.cpp"

//chassis PID
		//reset the motor values to 0 so that PID can work (numbers will stay constant)
		void reset_encoders(){     //void just means it doesn't return a value and has no parameters
			RF.set_zero_position(0);
			RM.set_zero_position(0);
			RB.set_zero_position(0);
			LF.set_zero_position(0);
			LM.set_zero_position(0);
			LB.set_zero_position(0);
		}
		//stop the motors from moving in case they were
		void stop_motors(){
			RF.move(0);
			RM.move(0);
			RB.move(0);
			LF.move(0);
			LM.move(0);
			LB.move(0);
		}

void drive(int target){

	reset_encoders();
    //set all motor encoders to 0, hence reset

//target is in inches
	target*=28.65; //the conversion for 36:1 for 4-inch wheels

  //setting the PID constants
	double kP = 0.5;
	double kI = 0.01;
	double kD = 0.02;

  //setting the integers
	int integral = 0;
	int derivative = 0;
	int error;
	int prev_error;
	int power;
	int powerAdj = 0;
	int current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;

      //setting up "error"
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

      	powerAdj = power/10;
    		LF.move(power); LM.move(power); LB.move(power); RF.move(power-powerAdj); RM.move(power-powerAdj); RB.move(power-powerAdj);
    		delay(10);
    	}
    	stop_motors();
    }

      //setting up imu
      int iter_pos(Imu imu){
      	return -1;
      }

      // only do -180-180 turns
      void imuTurn(double degrees)
      {

      if(degrees < 0)
      {
      	imu.set_heading(350); //heading is x-axis
              }

        else
        {
        	imu.set_heading(10);
                }

      //setting constants and integers for imu (inertial sensor) x-axis?
      float kP = 0.3;
      float kI = 0.1;
      float kD = 0.2;
      double target = imu.get_heading() + degrees;
      double error = target - imu.get_heading(); // -90
      double lastError = error;
      int power = 0;
      double integral = 0.0;
      double derivative = 0.0;

      while(abs(error) > 1.0)	{
      	error = target - imu.get_heading();
      	integral += error;
      	if(abs(integral) >= 600){
      		integral = 0;
      	}
      	derivative = error - lastError;
      	power = error * kP + integral * kI + derivative *kD;
      	lastError = error;
      	LF.move(power); LM.move(power); LB.move(power); RF.move(-power); RM.move(-power); RB.move(-power);
      	delay(10);
      }
      stop_motors();
      }

      void turn(int degrees){
      reset_encoders();

      //setting constants and integers for turning imu
      degrees *= 8.8; //this is honestly just some random number
      double kP = 0.225;
      double kI = 0.009;
      double kD = 0.06;
      int integral;
      int derivative;
      int error;
      int prev_error;
      int power;
      int current_pos;

      current_pos = (LF.get_position() + LM.get_position() + LB.get_position()) / 3;
      error = degrees - current_pos;
      while(abs(error) >= 1){
      	// for(int i = 0; i < 4 ; i++){
      	//   cout << "" << endl;
      	// }
      	current_pos = (LF.get_position() + LM.get_position() + LB.get_position()) / 3;
      	// cout << "Current pos: " << current_pos << endl;
      	error = degrees - current_pos;
      	// cout << "Error: " << error << endl;
      	integral += error;
      	if(error == 0){
      		integral = 0;
      	}

      	if(integral >= 2500){
      		integral = 0;
      	}
      	// if(abs(error) > 150){
      	// 	integral = 0;
      	// }

      	derivative = error - prev_error;
      	prev_error = error;
      	// cout << "P: " << kP*error << endl;
      	// cout << "I: " << kI * integral << endl;
      	// cout << "D: " << kD*derivative << endl;
      	power = kP*error + kI*integral + kD*derivative;
      	// cout << "Power: " << power << endl;
      	LF.move(power); LM.move(power); LB.move(power); RF.move(-power); RM.move(-power); RB.move(-power);
      	delay(10);
      }
      stop_motors();
      }


//lift PID
  void reset_lift(){
  	lift_left.set_zero_position(0);
  	lift_right.set_zero_position(0);
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

void liftMobileGoal(){
	reset_lift();

  //setting constants and ints for lift PID when lifting mogo (it needs more strength)
	double kP = 0.1;
	double kI = 0.0025;
	double kD = 0.02;
	int integral = 0;
	int derivative = 0;
	int power = 0;
	int current_pos = 0;
	int error = 0;
	int prev_error = 0;
	error = 1300;

  //for the auto thingy
	while(error > 5){
		if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			break;
		}

		current_pos = (lift_left.get_position() + lift_right.get_position()) / 2;
		error = 1400 - current_pos;
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
		power -= 15;
		lift_left.move(power); lift_right.move(power);
		delay(5);
	}
	stop_lift();
}


void moveMogo(int target){ //why is there another mogo one
	reset_lift();

  //setting up ???
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

void moveLift(int target){
	stop_lift();
	reset_lift();

  //setting constants and ints for lift PID when it's not moving mogos
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
	while(abs(error)>10){
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
}


void winPointMoveDown(int target){
	reset_lift();

  //setting up for awp lift ring yes
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

	void autoBalance(){
		float kP = 2.0;
		float kD = 0.2;
		float kI = 0.005;
		int power;
		float error;
		float pError;
		int integral;

		int derivative;
		while(abs(imu.get_pitch()) >= 1.5){

  		if(con.get_digital(E_CONTROLLER_DIGITAL_X)){
				break;
			}
			error = 1.5 - imu.get_pitch();

  		if(abs(error) <= 15 && error > 8){
				integral += error;
			}
			// else{
			// 	integral = 0;
			// }
			if(abs(integral) >= 100){
				integral = 0;
			}

  		derivative = pError - error;
			power = kP*error + kI*integral + kD*derivative;
			LF.move(-power); LM.move(-power); LB.move(-power); RF.move(-power); RM.move(-power); RB.move(-power);
			pError = error;
			delay(5);

  		if(abs(imu.get_pitch()) <= 9){
				stop_motors();
			}
		}
	}


	void driveLiftDown(int dTarget, int lTarget){
		reset_encoders();
		reset_lift();
		//target is in inches
		dTarget*=28.65; // the conversion for 36:1 4 inch wheels
		// RF.set_zero_position(0);
		double kP = 0.1;
		double kI = 0.0025;
		double kD = 0.01;
		int integral = 0;
		int derivative = 0;
		int power = 0;
		int current_pos = 0;
		int error = 0;
		int prev_error = 0;
		error = lTarget - current_pos;
		//----------
		double dkP = 0.5;
		double dkI = 0.01;
		double dkD = 0.02;
		int dintegral = 0;
		int dderivative = 0;
		int derror;
		int dprev_error;
		int dpower;
		int powerAdj = 0;
		int d_current_pos = (LF.get_position() + LM.get_position() + LB.get_position())/3;
		derror = dTarget - d_current_pos;
		//-------------------
		while(abs(derror) >= 15 && abs(error)>5){
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
			powerAdj = dpower/10;

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
