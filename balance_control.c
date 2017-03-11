/*
 * balance_control.c
 *
 *This file is meant for control of a Balancing robot with 2 wheels using QEI sensors and
 *IMU to determine position and angle
 *
 *
 *Meant for working with the TM4c123GH6PM board from TI
 *
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */


#include "balance_control.h"

/***************************************************************************************
 * init_balance_control()
 *
 * This function initializes all the starting values for the balance_control
 * Sets the initial position as the reference and all X - Y values are from starting orientation
 * of the robot Y Axis is forward and X axis is parallel to wheels
 *
 *
 **************************************************************************************/

void init_balance_control()
{
	ctrl_robot.current_orientation = 0;

	ctrl_robot.x_abs_pos = 0;	//x_abs_pos
	ctrl_robot.y_abs_pos  = 0;	//y_abs_pos
	ctrl_robot.wheel_L_cpos = 19700;	//current position of left wheel
	ctrl_robot.wheel_R_cpos = 19700;	//current position of right wheel
	ctrl_robot.wheel_L_fpos= 19700;	//final position of left wheel
	ctrl_robot.wheel_R_fpos = 19700;	//final position right wheel
	ctrl_robot.current_orientation = 0 ;	//value is in RADIANS -- CONVERT TO ANGLE
	ctrl_robot.final_orientation = 0;	//direction of robot orientation of robot and in RADIANS
	ctrl_robot.IMU_orientation = 0;
	ctrl_robot.current_speed = 1;		//speed requested of the robot
	ctrl_robot.current_state = 2;
	ctrl_robot.turn_percent = 0;  //use values from -100% to 100% left to right
	qei_data.left_wheel.set_point = 19700+69625;
	qei_data.right_wheel.set_point = 19700;
	int counter = 0;
	for(counter = 0;counter < 20;counter++)
	{
	ctrl_robot.c_X_pos[counter] = 0;	//distance in meters X-Y
	ctrl_robot.c_Y_Pos[counter] = 0;	//absololute position in meters X-Y
	}
}

/***************************************************************************************
 * init_PID_values()
 *
 * This function initializes all the starting values for the balance_control
 * This will set the base PID values taht I have tuned to be somewhat workable - working on dynamic
 * changing of the PID with machine learning or just GUI control
 *
 *
 **************************************************************************************/
void init_pid_values()
{
    _PIDA.Pgain = 3990;			// good base value 4000;
    _PIDA.Igain = 215;			// good base values 200;
    _PIDA.Dgain = 280;			// good base value 300;


	_PIDS.Pgain = 0.001295;		//decent base value 0.012; 0.00075
    _PIDS.Igain = 0.000080;		//decent value 0.00015		0.000075
    _PIDS.Dgain = 0.01655;		// 0.018 good base			0.0017


    PID_CONTROL_DATA._pid_position.Pgain = 0.000725; // 0.00075 good start
	PID_CONTROL_DATA._pid_position.Igain = 0.00007;  // 0.0005 good start
	PID_CONTROL_DATA._pid_position.Dgain = 5.90;		//7.5 good sstart


	/*
		_PIDA.Pgain = 4000;			// good base value 3900;
	    _PIDA.Igain = 180;			// good base values 200;
	    _PIDA.Dgain = 250;			// good base value 300;

		_PIDS.Pgain = 0.000;		//decent base value 0.012; 0.00075
	    _PIDS.Igain = 0.00000;		//decent value 0.00015		0.000075
	    _PIDS.Dgain = 0.0000;		// 0.018 good base			0.0017

	    PID_CONTROL_DATA._pid_position.Pgain = 0; // 0.00075 good start
		PID_CONTROL_DATA._pid_position.Igain = 0;  // 0.0005 good start
		PID_CONTROL_DATA._pid_position.Dgain = 0;		//7.5 good sstart
		*/


}

//THIS FUNCTION NEEDS TO BE WRITTEN IT WILL SET THE SPEED BASED ON THE TOTAL DISPLACEMENT
//THE ROBOT NEEDS TO COVER -- THINKING OF USING ANOTHER PID LOOP FOR POSITION THAT WILL
//DYNAMICALLY DECREASE THE SPEED AS WE GET FASTER AND CLOSER TO THE TARGET POSITION
//OR I NEED TO WRITE A VELOCITY PROFILE FOR THE ROBOT BASED ON A SUSTAINABLE TRAJECTORY
//RIGHT NOW I AM STRUGGLING WITH QEI WRONG DIRECTION
float speed_set()
{
	float speed = 0;
	float Perror_position =  (ctrl_robot.wheel_L_cpos - qei_data.left_wheel.abs_pos);
	float Derror = (qei_data.left_wheel.vel[speed_loop] + qei_data.left_wheel.vel[speed_loop] + qei_data.right_wheel.vel[cycle_data[speed_loop].back1] +qei_data.right_wheel.vel[cycle_data[speed_loop].back1]) /4;
	PID_CONTROL_DATA._pid_position.Ierror +=Perror_position;
	if(PID_CONTROL_DATA._pid_position.Ierror >400000 )
	{
		PID_CONTROL_DATA._pid_position.Ierror = 400000;
	}
	if(PID_CONTROL_DATA._pid_position.Ierror< -400000)
	{
		PID_CONTROL_DATA._pid_position.Ierror = -400000;
	}

	PID_CONTROL_DATA._pid_position.Ierror +=Perror_position;
	speed = (Perror_position * PID_CONTROL_DATA._pid_position.Pgain - Derror *  PID_CONTROL_DATA._pid_position.Dgain +  PID_CONTROL_DATA._pid_position.Igain * PID_CONTROL_DATA._pid_position.Ierror);
	if(speed>7000)
	{
		speed = 7000;
	}
	if(speed < -7000)
	{
		speed = -7000;
	}

	if(!(speed > 20 || speed < 20))
	{
		speed = 0;
	}
	/*
	System_printf("PID_Postion setpoint to speed = %f\n\n",speed);
	System_flush();
	*/
	return speed;
}

/*****************************************************************************************
 * position_calculation()
 *
 * Given the wheel displacements read from the QEI this function will determine
 * the X and Y displacement of the robot from it's last position. All X -Y positions
 * are from the starting point of the robot
 *
 *
 ****************************************************************************************/
void position_calculations()
{

	float arcLengthL = qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] - qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1];
	float arcLengthR = qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] - qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1];
	float X_val;
	float Y_val;
	float previous_orientation = ctrl_robot.current_orientation;

	if(arcLengthL>0 && arcLengthR > 0)
	{
		if(arcLengthL > arcLengthR)
		{
			//call function to convert arclengths to X and Y position relative current position
			ctrl_robot.current_orientation +=position_XY_calc(arcLengthR, arcLengthL,&X_val,&Y_val);
			//perform rotation matrix on X-Y to get into starting X - Y coordinate system
			rotation_matrix(previous_orientation,&X_val,&Y_val);
		}
		if(arcLengthL < arcLengthR)
		{
			ctrl_robot.current_orientation -=position_XY_calc(arcLengthL, arcLengthR,&X_val,&Y_val);
			X_val *= -1;
			rotation_matrix(previous_orientation,&X_val,&Y_val);
		}
	}

	else if(arcLengthL < 0 && arcLengthR < 0)
	{
		if(arcLengthL > arcLengthR)
		{
			ctrl_robot.current_orientation -=position_XY_calc(arcLengthL, arcLengthR,&X_val,&Y_val);
			X_val *= -1;
			rotation_matrix(previous_orientation,&X_val,&Y_val);
		}
		if(arcLengthL < arcLengthR)
		{
			ctrl_robot.current_orientation +=position_XY_calc(arcLengthR, arcLengthL,&X_val,&Y_val);
			rotation_matrix(previous_orientation,&X_val,&Y_val);
		}

	}

	else if(arcLengthL == arcLengthR)	//if arclengths are equal robot just moved forward in Y direction
	{
		Y_val = (arcLengthL/39400)*WHEEL_CIRC; //find tangential displacement of the wheel
		X_val = 0;
		rotation_matrix(previous_orientation,&X_val, &Y_val);	//convert movement starting X-Y coordinates

	}
	else
	{
		if(arcLengthL < 0 && arcLengthR > 0)		//THINK OF ALGORITHM FOR CENTER POINT TURNING THAT IS UNEQUAL
		{
			if(abs(arcLengthL) > arcLengthR)
			{
				ctrl_robot.current_orientation += center_axis_rotation(arcLengthL,&X_val,&Y_val);
				rotation_matrix(previous_orientation,&X_val, &Y_val);
			}
			else
			{
				ctrl_robot.current_orientation += center_axis_rotation(arcLengthR,&X_val,&Y_val);
				rotation_matrix(previous_orientation,&X_val, &Y_val);
			}


		}
		if(arcLengthL >0  && arcLengthR < 0)		//THINK OF ALGORITHM FOR CENTER POINT TURNING, just choose 1 as pivot?
		{
			if(arcLengthL > abs(arcLengthR))
			{
				ctrl_robot.current_orientation -= center_axis_rotation(arcLengthL,&X_val,&Y_val);
				rotation_matrix(previous_orientation,&X_val, &Y_val);
			}
			else
			{
				ctrl_robot.current_orientation -= center_axis_rotation(arcLengthR,&X_val,&Y_val);
				rotation_matrix(previous_orientation,&X_val, &Y_val);
			}


		}
	}
	/*
	System_printf("X = %f\t",ctrl_robot.x_abs_pos);
	System_printf("Y = %f\t",ctrl_robot.y_abs_pos);
	System_flush();
	*/

	ctrl_robot.x_abs_pos += X_val;
	ctrl_robot.y_abs_pos += Y_val;
}


/**********************************************************************
 * rotation_matrix(orientation , x_val, y_val
 *
 * function calculates the X and Y values in euclidean space back into
 * starting orientation of the robot to find absolute position with reference
 * to starting position of the robot.
 *
 *********************************************************************/
void rotation_matrix(float orientation,float *x_val,float *y_val)
{
	float x_temp = *x_val;
	*x_val = cos(-orientation)**x_val-sin(-orientation)**y_val;	//2by2 matrix for 2D rotation in X and Y space
	*y_val = sin(-orientation)*x_temp+cos(-orientation)**y_val;	//2by2 matrix for 2D rotation in X and Y space
}

/*************************************************************************************
 * float position_XY_calc(float arc1, float arc2,float * x,float * y)
 *
 * This function will find the X_position of of the robot
 * returns the theta value to find new orientation of the robot in degrees
 *
 *the X and Y positions are not relative to starting position - need to convert them
 *with a rotation matrix back into the starting orientation to get ABS  X and Y positions
 *with relative to starting location
 *
 ************************************************************************************/
float position_XY_calc(float arc1,float arc2,float *x_movement,float *y_movement)
{
	float theta;
	float ratio = arc1/arc2;
	float arc1_meters = (arc1/39400)*WHEEL_CIRC; 	//convert arc1 pulse to meters
	float radius = (ratio*WHEEL_WIDTH)/(1-ratio);	//convert Arc1 arc1 to the radius to Wheel of arc1

	theta = arc1_meters/radius;						//find the common theta to use in X-Y calcs

	*x_movement = (radius+WHEEL_CENTER)-(radius+WHEEL_CENTER)*cos(theta); //store the DELTA X value
	*y_movement = (radius+WHEEL_CENTER)*sin(theta);						 //store the DELTA y value
	return theta;
}

/*************************************************************************************
 *  center_axis_rotation();
 *
 * Will take encoder count and the X and Y pointers and will return the X y movement
 *
 *
 *
 *
 *
 ******************************************************************************************/
float center_axis_rotation(int encoder_count,float * x_val, float*y_val)
{
	float theta = ((encoder_count/MAX_ENCODER)*WHEEL_CIRC)/WHEEL_WIDTH;
	*x_val = cos(theta)*WHEEL_CENTER;
	*y_val = sin(theta)*WHEEL_CENTER;
	return theta;
}


/*************************************************************************************
 * balance_control_algo()
 *
 * No Params yet
 *
 * this function will be in charge of determining the correct motion plan of the robot
 * and what state it needs to be in to achieve this motion
 *
 * the states will be forward motion - which is the same as reverse motion
 * soft turning which will be forward motion with a slight turn
 * pivot turning left/right which will lock one wheel in place and turn around it
 * center turning left/right which will let the robot turn in place moving both wheels around
 * its center of axis.
 *
 *
 *
 *
 ******************************************************************************************/
void balance_control_algo()
{
	if(qei_data.left_wheel.abs_pos > qei_data.left_wheel.set_point && ctrl_robot.current_state == 2)
	{
		ctrl_robot.current_speed ++;
		qei_data.right_wheel.set_point = 19700+69625*ctrl_robot.current_speed;
		ctrl_robot.current_state = 3;
		qei_data.left_wheel.set_point = qei_data.left_wheel.abs_pos;
		_PIDS.Ierror = 0;
	}
	if(qei_data.right_wheel.abs_pos > qei_data.right_wheel.set_point && ctrl_robot.current_state == 3)
	{
		ctrl_robot.current_speed ++;
		qei_data.left_wheel.set_point = 19700+69625*ctrl_robot.current_speed;
		ctrl_robot.current_state = 2;
		qei_data.right_wheel.set_point - qei_data.right_wheel.abs_pos;
		_PIDS.Ierror = 0;
	}
	getWheel_Data();
	position_calculations();
	float speed = 150;//speed_set();
	PID_speed(speed);
}


/*************************************************************************************
 * PID_speed()
 *
 * Params Float Speed - the speed setpoint the robot will try to achieve
 *
 *THIS FUNCTION WILL TAKE THE CURRENT AUTOMATED SPEED CALCULATION AND DECIDE WHAT ANGLE
 *THE ROBOT NEEDS TO BE AT TO ACHIEVE THIS SPEED SETTING- CURRENTLY WE HAVE THE PID
 *IN A CASCADING MANNER - THE SPEED PID CHANGES THE SETPOINT OF THE ANGLE PID
 *WE HAVE THE CURRENT SPEED PID CALLED 1 time for EVERY 10 TIMES OF THE ANGLE PID
 *WE DO THIS TO REDUCE THE INTERFERENCE OF THE TWO SIGNALS AND ALLOW THE ROBOT
 *TO ACHEIVE THE CURRENT SETPOINT BEFORE HAVING TO CHANGE VALUES AGAIN
 *
 * THE ROBOT AUTOMATION SOFTWARE WILL DECIDE THE CURRENT SPEED IT WISHES TO ACHIEVE TO GET TO
 * THE CORRECT POSITION OR TO MAINTAIN THE CORRECT POSITION
 *
 *
 */
void PID_speed(float speed)
{

	float set_point = 0;
	float Perror;
	float Derror;
	Perror = speed - ((float)(qei_data.left_wheel.vel[cycle_data[speed_loop].now] + qei_data.right_wheel.vel[cycle_data[speed_loop].now])/2);


	Derror = ((qei_data.left_wheel.accel[cycle_data[speed_loop].now] + qei_data.left_wheel.accel[cycle_data[speed_loop].back1] +
			qei_data.left_wheel.accel[cycle_data[speed_loop].back2]+ qei_data.left_wheel.accel[cycle_data[speed_loop].back2])/4);


	_PIDS.Ierror += Perror;
	if(qei_data.left_wheel.vel[cycle_data[speed_loop].now] > 0 && qei_data.left_wheel.vel[cycle_data[speed_loop].now] < 0)
	{
		_PIDS.Ierror /= 2;
	}
	if(qei_data.left_wheel.vel[cycle_data[speed_loop].now] < 0 && qei_data.left_wheel.vel[cycle_data[speed_loop].now] > 0)
	{
		_PIDS.Ierror /= 2;
	}
	if(_PIDS.Ierror > 100000 || _PIDS.Ierror < -100000)
	{
		if(_PIDS.Ierror > 100000)
		{
			_PIDS.Ierror = 100000;
		}
		else
		{
			_PIDS.Ierror = -100000;
		}
	}

	_PIDS.setPoint[cycle_data[speed_loop].now] = (_PIDS.Pgain * Perror + _PIDS.Dgain * Derror + _PIDS.Igain * _PIDS.Ierror);

	if(_PIDS.setPoint[cycle_data[speed_loop].now] > 20)
	{
		_PIDS.setPoint[cycle_data[speed_loop].now] =20;
	}
	if(_PIDS.setPoint[cycle_data[speed_loop].now] <-20)
	{
		_PIDS.setPoint[cycle_data[speed_loop].now] =-20;
	}
	set_point = (_PIDS.setPoint[cycle_data[speed_loop].now]);// +_PIDS.setPoint[cycle_data[speed_loop].back1]+_PIDS.setPoint[cycle_data[speed_loop].back2])/3;
						//_PIDS.setPoint[cycle_data[speed_loop].back3]+_PIDS.setPoint[cycle_data[speed_loop].back4]+_PIDS.setPoint[cycle_data[speed_loop].back5]) / 6;

		//if error is higher than 5 increase the offset - play around with value until nominal
		/*
		if(Perror > 20)
		{
		_PIDS.setPoint_offset -= 0.005;
		}
		if(Perror < -20)
		{
		_PIDS.setPoint_offset += 0.005;
		}
*/
		//this is used to reset the set_point offset if the speed crosses the reference point
		/*
		if(_PIDS.total_vel[cycle_data[speed_loop].now] > speed && _PIDS.total_vel[cycle_data[speed_loop].back1] < speed)
		{
			_PIDS.setPoint_offset = 0;
		}
		if(_PIDS.total_vel[cycle_data[speed_loop].now] < speed && _PIDS.total_vel[cycle_data[speed_loop].back1] > speed)
		{
			_PIDS.setPoint_offset = 0;
		}
		 */


		if(!(set_point > 0.25 || set_point < -0.25))
		{
		set_point = 0;
		}
		PID_angle(set_point);


	//reset the Loop


	//PID_angle(0);
	//PID_angle(setAngle[angle_loop]);

	speed_loop++;
	if(speed_loop == 20)
		{
		speed_loop=0;
		}

}

/***************************************************************************************************************
 * THIS FUNCTION GETS THE WHEEL DATA FROM THE QEI ENCODER API
 *
 * THIS FUNCTION WILL DECIPHER THE DATA FROM THE VELOCITY API AND POSITION API
 *
 *
 * WILL STORE ABSOLUTE POSITION AND VAR POSITION AND CURRENT DIRECTION OF MOTORS
 * INTO THE QEI_DATA STRUCTURE FILE FOR EACH WHEEL
 *
 */
void getWheel_Data()
{

	//THIS NEEDS TO BE REFACTORED THIS PART IS NOT WORKING BECAUSE OF CHANGES IN DIRECTION CAUSING PROBLEMS
	//THE QEI DATA IS GETTING LOGGED INCORRECTLY IN ABSOLUTE POOSITION - NEED TO THINK OF A NEW ALGORITHM TO DEAL WITH THIS
	// SHOULD BE A SIMPLE FIX OF JUST CHECKING PULSE VALUE and NEGATING DIRECTION FOR CALCULATIONS.
	qei_data.left_wheel.encoder_cnt[cycle_data[cycle_data[speed_loop].now].now] = QEIPositionGet(QEI0_BASE);
	qei_data.left_wheel.wheel_dir = QEIDirectionGet(QEI0_BASE);
	qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now] = QEIPositionGet(QEI1_BASE);
	qei_data.right_wheel.wheel_dir = QEIDirectionGet(QEI1_BASE);
	qei_data.left_wheel.vel[cycle_data[speed_loop].now] = QEIVelocityGet(QEI0_BASE);
	qei_data.right_wheel.vel[cycle_data[speed_loop].now] = QEIVelocityGet(QEI1_BASE);



	int displacement_left;
	int displacement_right;
	//day four - captains log, they still think I know how to code
	if(qei_data.left_wheel.wheel_dir == 1)
	{
		qei_data.left_wheel.vel[cycle_data[speed_loop].now] = QEIVelocityGet(QEI0_BASE);
		if(qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1]>19700 && qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]<19700)
		{
			 displacement_left = (39400+qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			if(displacement_left > 20000 ||  displacement_left < -20000)
			{
				 displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			}
		}
		else
		{
			displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			if(displacement_left > 20000 ||  displacement_left < -20000)
			{
			displacement_left = (qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
			}
		}
		qei_data.left_wheel.abs_pos +=  displacement_left;
		qei_data.left_wheel.dynamic_pos +=  displacement_left;
	}
	if(qei_data.left_wheel.wheel_dir == -1)
	{
			qei_data.left_wheel.vel[cycle_data[speed_loop].now] *= -1;
			if(qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1]<19700 && qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]>19700)
			{
				 displacement_left = (qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if( displacement_left > 20000 ||  displacement_left < -20000)
				{
					 displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			else
			{
				displacement_left = qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if( displacement_left > 20000|| displacement_left < -20000)
				{
				displacement_left = (39400+qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			qei_data.left_wheel.abs_pos +=  displacement_left;
			qei_data.left_wheel.dynamic_pos +=  displacement_left;

	}
	if(qei_data.right_wheel.wheel_dir == 1)
	{

			if(qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1]>19700 && qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]<19700)
			{
				displacement_right = (39400+qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right> 20000 || displacement_right < -20000)
				{
					displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}

			}
			else
			{
				displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right> 20000 || displacement_right < -20000)
				{
				displacement_right= (qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			qei_data.right_wheel.abs_pos += displacement_right;
			qei_data.right_wheel.dynamic_pos += displacement_right;
	}
	if(qei_data.right_wheel.wheel_dir == -1)
	{
		qei_data.right_wheel.vel[cycle_data[speed_loop].now] *= -1;
			if(qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1]<19700 && qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]>19700)
			{
				displacement_right= (qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-39400)-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right > 20000 || displacement_right< -20000)
				{
					displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}
			else
			{
				displacement_right = qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]-qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				if(displacement_right > 20000|| displacement_right< -20000)
				{
					displacement_right = (39400+qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].now]) -qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1];
				}
			}

			qei_data.right_wheel.abs_pos += displacement_right;
			qei_data.right_wheel.dynamic_pos += displacement_right;
	}

	if(displacement_right > 15000 || displacement_right < -15000 || displacement_left > 15000 || displacement_left < -15000)
	{

		displacement_right = 0;
	}

	qei_data.left_wheel.accel[cycle_data[speed_loop].now] = ((float)(qei_data.left_wheel.vel[cycle_data[speed_loop].now]-qei_data.left_wheel.vel[cycle_data[speed_loop].back1]));
	qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] = qei_data.left_wheel.abs_pos;
	qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] = qei_data.right_wheel.abs_pos;
	qei_data.right_wheel.accel[cycle_data[speed_loop].now] = (float)((qei_data.right_wheel.vel[cycle_data[speed_loop].now]-qei_data.right_wheel.vel[cycle_data[speed_loop].back1]));
	_PIDS.total_vel[cycle_data[speed_loop].now] = (qei_data.right_wheel.vel[cycle_data[speed_loop].now] +qei_data.left_wheel.vel[cycle_data[speed_loop].now])/2;



	//ABSOLUTE POSITION MIGHT BE GETTING JUNKED, MAYBE WRITE A FAILSAFE TO DISCARD RIDICULOUS POSITION DATA
}



/*****************************************************************************************************
 *
 * PID_ANGLE
 *
 * THIS FUNCTION WILL TAKE THE GIVEN FLOAT POS AND THEN SET THE PID VALUES BASED ON IMU DATA
 * THE PIDS WILL THEN DETERMINE THE OUTPUT OF THE PWM SIGNAL TO THE MOTORS TO ACHIEVE THE SET POINT
 *
 * THE INTEGRAL VALUE IS CAPPED AND UNWOUND WHENEVER CROSSING THE SETPOINT
 * D Value IS THE GYRO DATA
 * P Value IS THE ERROR FROM OFFSET
 * I Value is the Culmination of the error
 *
 *
 *
 */
void PID_angle(float pos)
{
	float errorP;
	float errorD;
	_PIDA.setPoint = pos;

	//FIRST TIME THE LOOP RUNS WE WANT NON GARBAGE VALUES SO WE INITALIZE THEM TO ZERO
	if(angle_loop== -1)
	{
	init_main_loop();
	pid_imu_position[angle_loop].current_pos =  imu_data[angle_loop].euler_values.p;
	errorP = _PIDA.setPoint-imu_data[angle_loop].euler_values.p;
	errorD = 0;
	_PIDA.Ierror = errorP;
	}
	else //After first run values are initialized as non-garbage We should clean this up to reduce timings
	{
	pid_imu_position[cycle_data[angle_loop].now].current_pos = imu_data[angle_loop].euler_values.p;
	pid_imu_position[cycle_data[angle_loop].now].current_vel = imu_data[angle_loop].x_gyro;
			errorP = (float) (_PIDA.setPoint - (pid_imu_position[cycle_data[angle_loop].now].current_pos));
			errorD = (float)((pid_imu_position[cycle_data[angle_loop].now].current_vel));

		if(pid_imu_position[cycle_data[angle_loop].now].current_pos<pos && pid_imu_position[cycle_data[angle_loop].back1].current_pos > pos)
		{
			_PIDA.Ierror /=2;
		}
		if(pid_imu_position[cycle_data[angle_loop].now].current_pos > pos && pid_imu_position[cycle_data[angle_loop].back1].current_pos <pos)
		{
			_PIDA.Ierror /=2;
		}

		if(!(_PIDA.Ierror > 75.0 || _PIDA.Ierror <-75.0))
		{
		_PIDA.Ierror += errorP;
		}
	}


	float errorKP;
	float errorKD;
	float errorKI;
	//THE PID VALUES ARE DYNAMICALLY CHANGED BASED ON THE ANGULAR POSITION- THE FURTHER AWAY FROM 0
	// THE HIGHER THE Perror_angle_factor is, increasing the output at higher angles
	//float Perror_angle_factor = sin(imu_data[angle_loop].euler_values.p*(3.14159/180.0));
	//Perror_angle_factor = sqrt(sqrt(sqrt(sqrt(sqrt(fabs(Perror_angle_factor))))));


	//the ErrorKP and KD and KI values to determine the output of the motors
	errorKP = errorP * _PIDA.Pgain;//* Perror_angle_factor;
	errorKD = errorD * _PIDA.Dgain;//* Perror_angle_factor;
	errorKI = _PIDA.Ierror *_PIDA.Igain;//*Perror_angle_factor;

	//cap the outputs from the errors so we avoid lopsided issues
	if(errorKP > 65535.0)
	{
		errorKP = 65535.0;
	}
	if(errorKP < -65535.0)
	{
	errorKP = -65535.0;
	}
	if(errorKD > 65535.0)
	{
		errorKD = 65535.0;
	}
	if(errorKD < -65535.0)
	{
	errorKD = -65535.0;
	}

	//Take the error values and set it to the output
	pid_imu_position[cycle_data[angle_loop].now].output = errorKP + (errorKI) + (errorKD);
	//Set the PWM out as the output value for the wheels
	qei_data.left_wheel.pwm_out = pid_imu_position[cycle_data[angle_loop].now].output;
	//pid_imu_position[cycle_data[angle_loop].back2].output)/3;
	qei_data.right_wheel.pwm_out = qei_data.left_wheel.pwm_out;

	wheel_control();

	//We loop the values of the error 20 times so we can look back at the data or average the signals if data is bad
	angle_loop++;
	if(angle_loop == 20)
	{
		angle_loop = 0;
	}
}

/**********************************************************************************
 * init_QEI()
 *
 * THIS FUNCTION INITIALIZES THE QEI PERIPHERALS ON THE BOARD
 * WE SET UP THE CORRECT PINS OF PD6 PD7 and PC5 PC6
 *
 * THIS FUNCTION ALSO SETS UP THE DIRECTION BITS THAT CONTROL MOTOR DIRECTION
 *
 * THIS FUNCTION ALSO INITIALIZES THE STARTING VALUES TO ZERO SO WE DON'T HAVE INITIAL CONDITION ERRORS
 *
 *	ALSO INITS THE DIRECTION -- DIRECTION PINS FOR THE MOTORS
 *
 */
void init_QEI()
{
		// Enable QEI Peripherals
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

		//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

		//Set Pins to be PHA0 and PHB0
		GPIOPinConfigure(GPIO_PD6_PHA0);
		GPIOPinConfigure(GPIO_PD7_PHB0);


		GPIOPinConfigure(GPIO_PD6_PHA0);


		GPIOPinConfigure(GPIO_PC5_PHA1);
		GPIOPinConfigure(GPIO_PC6_PHB1);
		//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
		GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
		GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);
		//DISable peripheral and int before configuration
		QEIDisable(QEI0_BASE);

		QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

		QEIDisable(QEI1_BASE);

		QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

		uint32_t MAXPOS = 39400;
		uint32_t MIDPOS = 19700;

		QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAXPOS);
		QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,1000000);

		QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), MAXPOS);
		QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1,1000000);
		// Enable the quadrature encoder.
		QEIEnable(QEI0_BASE);
		QEIEnable(QEI1_BASE);
		//Set position to a middle value so we can see if things are working
		QEIPositionSet(QEI0_BASE, MIDPOS);
		QEIPositionSet(QEI1_BASE, MIDPOS);
		//Configure velocity setup

		QEIVelocityEnable(QEI0_BASE);
		QEIVelocityEnable(QEI1_BASE);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);


		qei_data.right_wheel.encoder_cnt[cycle_data[speed_loop].back1] = 0;
		qei_data.right_wheel.vel[cycle_data[speed_loop].back1] = 0;
		qei_data.left_wheel.encoder_cnt[cycle_data[speed_loop].back1] = 0;
		qei_data.left_wheel.vel[cycle_data[speed_loop].back1] = 0;

		int k = 0;
		for(k = 0; k < 20;k++)
		{
			qei_data.right_wheel.accel[k] = 0;
			qei_data.right_wheel.encoder_cnt[k] = 0;
			qei_data.right_wheel.vel[k] = 0;

			qei_data.left_wheel.accel[k] = 0;
			qei_data.left_wheel.encoder_cnt[k] = 0;
			qei_data.left_wheel.vel[k] = 0;
		}
		//MOTOR DIRECTION PINS FOR BALANCE BOT
		//THEY CAN BE CHANGED IF NEEDED
	    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
	    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
	    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);


	    //GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3); /// TESTING PIN FOR SPEED PERFORMANCE
	   // GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

}


/*********************************************************************
 * wheel_control()
 *
 * THIS FUNCTION WILL DETERMINE THE CORRECT STATE OPERATION OF THE WHEELS BASED ON THE ROBOT OBJECTIVE
 *
 * THIS WILL TUNE THE PWM OUTPUT TO ACHIEVE SPECIFIC ACTIONS BASED ON THE CORRECT MOTION PLANNING
 *
 * WE WILL NEED TO DECIDE IF WE WANT TO GO STRAIGHT FORWARD - SOFT TURN - PIVOT TURN - OR CENTER TURN
 *
 *
 * ALGORITHM WILL NEED TO BE DEVISED BEFORE HAND SO THIS FUNCTION CAN CALL THE NESCESSARY LOGIC TO ACHIEVE
 * THE CORRECT MOTION PROFILE
 *
 *
 *
 *
 ****************************************************************************************/
void wheel_control()
{

	switch(ctrl_robot.current_state)
	{
	case 0:
	forward_wheel_control();		//FORWARD MOTION KEEP WHEELS ALIGNED
		break;
	case 1:
	soft_turn_control();			//FORWARD MOTION ONE WHEEL FASTER - BASED ON TURNING ANGLE
		break;
	case 2:
	right_wheel_pivot();			//RIGHT WHEEL PIVOT TURN - RIGHT WHEEL STAYS AT IT'S POSITION
		break;
	case 3:
	left_wheel_pivot();				//LEFT WHEEL PIVOT TURN - LEFT WHEEL STAYS AT IT'S POSITION
		break;
	case 4:
	center_turn_right();			//CENTER AXIS TURN TO THE RIGHT
		break;
	case 5:
	center_turn_left();				//CENTER AXIS TURN TO THE LEFT
		break;
	case 6:
		break;
	case 7:
		break;
	default:
		forward_wheel_control();	//DEFAULT STAY IN LINE FORWARD OR REVERSE
		break;
	}
}

/**************************************************************************************
 * soft_turn_control()
 *
 * THIS FUNCTION WILL PERFORM AN ALGORITHM TO PERFORM A GRADUAL TURN WHILE MOVING FORWARD
 * WE WOULD LIKE TO IMPLEMENT A INPUT THAT DECIDES HOW GRADUAL AND WHAT RATE THE ROBOT NEEDS TO TURN AT
 * THIS VALUE WILL NEED TO BE CALCULATED BY THE MOTION PLANNING OF THE ROBOT TO ACHIEVE ITS OBJECTIVE
 * GRADUAL TURN WILL BE USED WHEN THE ROBOT NEEDS TO GO FORWARD AND TURN AS WELL
 *
 *
 */
void soft_turn_control()
{
	//SHIFT THE OUTPUT PWM FROM ONE WHEEL TO THE OTHER WHICH WILL INCREASE THE TURNING
	// WILL NEED TO ADD CONTROL TO MAINTAIN THE CORRECT TURN RATIO
	// REWRITE THIS CODE TO BE -100 to 100 for turn angle
	if(turnAngle[angle_loop] < -1 && qei_data.right_wheel.wheel_dir ==1)
	{
		qei_data.left_wheel.pwm_out *= ((100+turnAngle[angle_loop])/100);	//turn Angle needs to be percent -100 to 100
		qei_data.right_wheel.pwm_out *= ((100-turnAngle[angle_loop])/100);

	}
	if(turnAngle[angle_loop] > 1 && qei_data.right_wheel.wheel_dir==1)
	{
		qei_data.left_wheel.pwm_out *= (100-turnAngle[angle_loop])/100;
		qei_data.right_wheel.pwm_out *= ((100+turnAngle[angle_loop])/100);
	}
}


/***************************************************************************
 * If moving straight forward or backward this function will
 * Pick the correct direction of the motors and will make sure
 * that the wheels stay in perfect alignment
 *
 *
 *
 */
void forward_wheel_control()
{
	if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
		{
			if(qei_data.right_wheel.pwm_out<0)
			{
				qei_data.right_wheel.wheel_dir = -1;
			}
			else
			{
				qei_data.right_wheel.wheel_dir = 1;
			}
			qei_data.right_wheel.pwm_out  = 65535.0;
		}
		else if(qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
			qei_data.right_wheel.wheel_dir = -1;
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}
	qei_data.left_wheel.wheel_dir = qei_data.right_wheel.wheel_dir;
	qei_data.left_wheel.pwm_out =qei_data.right_wheel.pwm_out;


	// DYNAMICALLY CHANGE THE WHEELS PWM TO FIX CURVING DIRECTIONS

	float val = 0;
	float right_percent = 1;
	float left_percent = 1;
	float pwm_value_right = qei_data.right_wheel.pwm_out;
	float pwm_value_left = qei_data.left_wheel.pwm_out;
	if(qei_data.right_wheel.wheel_dir==1)
	{
		if(qei_data.right_wheel.dynamic_pos > qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.right_wheel.dynamic_pos - qei_data.left_wheel.dynamic_pos;
			//This will slightly offset the PWM values of the outputs if one wheel lags behind
			right_percent = 1+(val/1000);
			left_percent = 1-(val/1000);
			//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
			if(right_percent > 1.4)
			{
				right_percent = 1.4;
			}
			if(left_percent < 0.6)
			{
				left_percent = 0.6;
			}
			pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
			pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out = pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;

		}
		if(qei_data.right_wheel.dynamic_pos < qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.left_wheel.dynamic_pos - qei_data.right_wheel.dynamic_pos;
						//This will slightly offset the PWM values of the outputs if one wheel lags behind
						right_percent = 1-(val/1000);
						left_percent = 1+(val/1000);
						//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
						if(right_percent < 0.6)
						{
							right_percent = 0.6;
						}
						if(left_percent > 1.4)
						{
							left_percent = 1.4;
						}
						pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
								pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
	}
	//DYNAMICALLY CHANGE WHEELS PWM TO FIX OFFSETS
	if(qei_data.right_wheel.wheel_dir==-1)
	{
		if(qei_data.right_wheel.dynamic_pos > qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.right_wheel.dynamic_pos - qei_data.left_wheel.dynamic_pos;
					//This will slightly offset the PWM values of the outputs if one wheel lags behind
					right_percent = 1-(val/1000);
					left_percent = 1+(val/1000);
					//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
					if(right_percent < 0.6)
					{
						right_percent = 0.6;
					}
					if(left_percent > 1.4)
					{
						left_percent = 1.4;
					}
					pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
							pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
		if(qei_data.right_wheel.dynamic_pos < qei_data.left_wheel.dynamic_pos)
		{
			val = qei_data.left_wheel.dynamic_pos - qei_data.right_wheel.dynamic_pos;
			//This will slightly offset the PWM values of the outputs if one wheel lags behind
			right_percent = 1+(val/1000);
			left_percent = 1-(val/1000);
			//CAP THE PERCENTAGE FOR MORE GRADUAL TURNS back to origin
			if(right_percent > 1.4)
			{
				right_percent = 1.4;
			}
			if(left_percent < 0.6)
			{
				left_percent = 0.6;
			}
			pwm_value_right = qei_data.right_wheel.pwm_out*right_percent;
					pwm_value_left = qei_data.left_wheel.pwm_out *left_percent;
			qei_data.right_wheel.pwm_out =  pwm_value_right;
			qei_data.left_wheel.pwm_out = pwm_value_left;
		}
	}
	if(qei_data.right_wheel.pwm_out > 65535.0)
	{
		qei_data.right_wheel.pwm_out = 65535.0;
	}
	if(qei_data.left_wheel.pwm_out > 65535.0)
	{
		qei_data.left_wheel.pwm_out = 65535.0;
	}
}

/***************************************************************************************
 * center_turn_left()
 *
 * This function will turn the robot in place to the left it will just keep turning while the function is called
 * control of when to call this function will happen elsewhere.
 *
 *
 *
 */
void center_turn_left()
{
	if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
		{
			if(qei_data.right_wheel.pwm_out<0)
			{
				qei_data.right_wheel.wheel_dir = -1;
			}
			qei_data.right_wheel.pwm_out = 65535.0;
		}
		else if(qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
			qei_data.right_wheel.wheel_dir = -1;
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}
	qei_data.right_wheel.wheel_dir = qei_data.left_wheel.wheel_dir;
}


/***************************************************************************************
 * center_turn_right()
 *
 * This function will turn the robot in place to the left it will just keep turning while the function is called
 * control of when to call this function will happen elsewhere.
 *
 *
 *
 */
void center_turn_right()
{
	if(qei_data.right_wheel.pwm_out > 65535.0 || qei_data.right_wheel.pwm_out < -65535.0)
			{
				if(qei_data.right_wheel.pwm_out<0)
				{
					qei_data.right_wheel.wheel_dir = -1;
				}
				qei_data.right_wheel.pwm_out = 65535.0;
			}
			else if(qei_data.right_wheel.pwm_out < 0)
			{
				qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
				qei_data.right_wheel.wheel_dir = -1;
			}
			else
			{
				qei_data.right_wheel.wheel_dir = 1;
			}
		qei_data.right_wheel.wheel_dir = qei_data.left_wheel.wheel_dir;
}



/***************************************************************************************
 * left_wheel_pivot()
 *
 *THIS FUNCTION WILL TIE ONE OF THE WHEELS TO IT"S CURRENT LOCATION AND HAVE THE OTHER WHEEL
 *DRIVE IT SLIGTHLY FORWARD TURNING THE ROBOT ON A PIVOT POINT OF IT'S LEFT WHEEL
 *THIS FUNCTION WILL BE CALLED WHEN YOU ROBOT DETERMINES IT NEEDS TO PIVOT TURN
 *
 *
 *
 */
void left_wheel_pivot()
{
	float Derror = qei_data.left_wheel.vel[cycle_data[speed_loop].now];
	int Perror = qei_data.left_wheel.set_point - qei_data.left_wheel.abs_pos;


	if(!(Perror<5000 && Perror<-5000))
	{
	PID_CONTROL_DATA._pid_position.Ierror += Perror;
	}

	//check if the value has passed setpoint then reduce the Ivalue
	if(qei_data.left_wheel.var_pos[cycle_data[speed_loop].now] <qei_data.left_wheel.set_point&& qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1]>qei_data.left_wheel.set_point)
	{
		PID_CONTROL_DATA._pid_position.Ierror /=5;
	}
	//check if the value has passed setpoint then reduce the Ivalue
	if(qei_data.left_wheel.var_pos[cycle_data[speed_loop].now]>qei_data.left_wheel.set_point && qei_data.left_wheel.var_pos[cycle_data[speed_loop].back1]<qei_data.left_wheel.set_point)
	{
		PID_CONTROL_DATA._pid_position.Ierror /=5;
	}

	float KDerror = 0; //Derror * PID_CONTROL_DATA._pid_position.Dgain;
	float KPerror = .25 * Perror ; //Perror * PID_CONTROL_DATA._pid_position.Pgain;
	float KIerror = 0; //PID_CONTROL_DATA._pid_position.Ierror * PID_CONTROL_DATA._pid_position.Igain;

	qei_data.left_wheel.pwm_out = (int)(KDerror + KPerror + KIerror);

	if(qei_data.left_wheel.pwm_out < 0)
	{
		qei_data.left_wheel.pwm_out= fabs(qei_data.left_wheel.pwm_out);
		qei_data.left_wheel.wheel_dir = -1;
	}
	else
	{
		qei_data.left_wheel.wheel_dir = 1;
	}
	if(qei_data.left_wheel.pwm_out > 65535)
	{
		qei_data.left_wheel.pwm_out = 65535;
	}

	qei_data.right_wheel.pwm_out *= 1.5;
	if(qei_data.right_wheel.pwm_out<0)
	{
	qei_data.right_wheel.wheel_dir = -1;
	qei_data.right_wheel.pwm_out = fabs(qei_data.right_wheel.pwm_out);
	}
	else
	{
		qei_data.right_wheel.wheel_dir = 1;
	}
	if(qei_data.right_wheel.pwm_out>65535)
	{
		qei_data.right_wheel.pwm_out = 65535;

	}
}

/***************************************************************************************
 * right_wheel_pivot()
 *
 *THIS FUNCTION WILL TIE ONE OF THE WHEELS TO IT"S CURRENT LOCATION AND HAVE THE OTHER WHEEL
 *DRIVE IT SLIGTHLY FORWARD TURNING THE ROBOT ON A PIVOT POINT OF IT'S RIGHT WHEEL
 *THIS FUNCTION WILL BE CALLED WHEN YOU ROBOT DETERMINES IT NEEDS TO PIVOT TURN
 *
 *
 *
 */
void right_wheel_pivot()
{
		float Derror = qei_data.right_wheel.vel[cycle_data[speed_loop].now];
		int Perror = qei_data.right_wheel.set_point - qei_data.right_wheel.abs_pos;


		if(!(Perror<5000 && Perror<-5000))
		{
		PID_CONTROL_DATA._pid_position.Ierror += Perror;
		}

		//check if the value has passed setpoint then reduce the Ivalue
		if(qei_data.right_wheel.var_pos[cycle_data[speed_loop].now] <qei_data.right_wheel.set_point&& qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1]>qei_data.right_wheel.set_point)
		{
			PID_CONTROL_DATA._pid_position.Ierror /=5;
		}
		//check if the value has passed setpoint then reduce the Ivalue
		if(qei_data.right_wheel.var_pos[cycle_data[speed_loop].now]>qei_data.right_wheel.set_point && qei_data.right_wheel.var_pos[cycle_data[speed_loop].back1]<qei_data.right_wheel.set_point)
		{
			PID_CONTROL_DATA._pid_position.Ierror /=5;
		}

		float KDerror = 0; //Derror * PID_CONTROL_DATA._pid_position.Dgain;
		float KPerror = .25*Perror ; //Perror * PID_CONTROL_DATA._pid_position.Pgain;
		float KIerror = 0; //PID_CONTROL_DATA._pid_position.Ierror * PID_CONTROL_DATA._pid_position.Igain;

		qei_data.right_wheel.pwm_out = (int)(KDerror + KPerror + KIerror);

		if(qei_data.right_wheel.pwm_out < 0)
		{
			qei_data.right_wheel.pwm_out= fabs(qei_data.right_wheel.pwm_out);
			qei_data.right_wheel.wheel_dir = -1;
		}
		else
		{
			qei_data.right_wheel.wheel_dir = 1;
		}

		if(qei_data.right_wheel.pwm_out > 65535)
		{
			qei_data.right_wheel.pwm_out = 65535;
		}
		qei_data.left_wheel.pwm_out *= 1.5;
		if(qei_data.left_wheel.pwm_out<0)
		{
		qei_data.left_wheel.wheel_dir = -1;
		qei_data.left_wheel.pwm_out = fabs(qei_data.left_wheel.pwm_out);
		}
		else
		{
			qei_data.left_wheel.wheel_dir = 1;
		}
		if(qei_data.left_wheel.pwm_out>65535)
		{
			qei_data.left_wheel.pwm_out = 65535;
		}
}


/************************************************************************************
 * i2c_check_data()
 *
 * function checks the data right after reading from I2C, if the data is deemed
 * not plausible the function returns 1 to tell the system to restart i2c transaction
 *
 *
 * Will write more in here that it will fault if multiple wrong data reads continuously
 *
 ************************************************************************************/
int i2c_check_data()
{
	int check = 0;

	float difference = imu_data[cycle_data[angle_loop].now].euler_values.p - imu_data[cycle_data[angle_loop].back1].euler_values.p;

	if(difference > 4.5) //check that the angle doesn't jump more than 5 degrees, shouldn't be plausible in timeframe of 10ms intervals
	{
		check = 1;
	}
	if(difference < -4.5)
	{
		check = 1;
	}

	//will add in gyro values to make prediction on what range it could be in.

	return check;
}

/*************************************************************************************
 * init_main_loop()
 *
 * This function initializes the main loop values to 0 so that we don't have initial
 * condition fail points or undefined behavior from the PID values
 *
 *
 *
 *
 */
char init_main_loop()
{
			angle_loop = 0;
			//IMU data init so we don't run into errors
				pid_imu_position[19].current_vel = 0;
				pid_imu_position[18].current_vel = 0;
				pid_imu_position[17].current_vel = 0;
				pid_imu_position[16].current_vel = 0;
				pid_imu_position[15].current_vel = 0;
				pid_imu_position[14].current_vel = 0;
				pid_imu_position[13].current_vel = 0;
				pid_imu_position[12].current_vel = 0;
				pid_imu_position[11].current_vel = 0;
				pid_imu_position[10].current_vel = 0;
				//IMU data init so we don't run into errors
					pid_imu_position[cycle_data[angle_loop].now].output = 0;
					pid_imu_position[cycle_data[angle_loop].back1].output =0;
					pid_imu_position[cycle_data[angle_loop].back2].output =0;
					pid_imu_position[cycle_data[angle_loop].back3].output =0;

					//IMU data init so we don't run into errors
					pid_imu_position[cycle_data[angle_loop].back1].current_pos=0;
					pid_imu_position[cycle_data[angle_loop].back2].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back3].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back4].current_pos =0;
					pid_imu_position[cycle_data[angle_loop].back5].current_pos =0;
					_PIDS.setPoint_offset = 0;
	return 0x1;
}


/*************************************************************************************
 * init_main_loop()
 *
 * THIS FUNCTION CREATES A LOOPING ARRAY SET OF VALUES SO THAT YOU CAN CALL THE CORRECT
 * PREVIOUS 20 VALUES BASED ON THE CURRENT LOOP CONDITION THROUGH A PLACEMENT ARRAY
 *
 * THIS LETS YOU CALL BACK ALL 20 SPOTS OF THE DATA ALLOWING YOU TO SEE EFFECTIVELY 200mS
 * IN THE PAST OF THE ROBOTS ACTIONS
 *
 * WE MAY IMPLEMENT POSITION PREDICTION IN THE FUTURE WHICH WE WILL USE THIS VALUES TO SLOPE THE NEXT
 * SPOT TO HELP INCREASE THE ROBOTS RESPONSIVENESS
 *
 *
 *
 */


void INIT_TIMECYCLE()
{
	cycle_data[0].now = 0;				//Initizalize reference numbers for 10 locations back 20 array loop cycle
	cycle_data[0].back1 = 19;
	cycle_data[0].back2 = 18;
	cycle_data[0].back3 = 17;
	cycle_data[0].back4 = 16;
	cycle_data[0].back5= 15;
	cycle_data[0].back6 = 14;
	cycle_data[0].back7 = 13;
	cycle_data[0].back8 = 12;
	cycle_data[0].back9 = 11;
	cycle_data[0].back10 = 10;

	cycle_data[1].now = 1;
	cycle_data[1].back1 = 0;
	cycle_data[1].back2 = 19;
	cycle_data[1].back3 = 18;
	cycle_data[1].back4 = 17;
	cycle_data[1].back5= 16;
	cycle_data[1].back6 = 15;
	cycle_data[1].back7 = 14;
	cycle_data[1].back8 = 13;
	cycle_data[1].back9 = 12;
	cycle_data[1].back10 = 11;

	cycle_data[2].now = 2;
	cycle_data[2].back1 = 1;
	cycle_data[2].back2 = 0;
	cycle_data[2].back3 = 19;
	cycle_data[2].back4 = 18;
	cycle_data[2].back5= 17;
	cycle_data[2].back6 = 16;
	cycle_data[2].back7 = 15;
	cycle_data[2].back8 = 14;
	cycle_data[2].back9 = 13;
	cycle_data[2].back10 = 12;

	cycle_data[3].now = 3;
	cycle_data[3].back1 = 2;
	cycle_data[3].back2 = 1;
	cycle_data[3].back3 = 0;
	cycle_data[3].back4 = 19;
	cycle_data[3].back5= 18;
	cycle_data[3].back6 = 17;
	cycle_data[3].back7 = 16;
	cycle_data[3].back8 = 15;
	cycle_data[3].back9 = 14;
	cycle_data[3].back10 = 13;

	cycle_data[4].now = 4;
	cycle_data[4].back1 = 3;
	cycle_data[4].back2 = 2;
	cycle_data[4].back3 = 1;
	cycle_data[4].back4 = 0;
	cycle_data[4].back5= 19;
	cycle_data[4].back6 = 18;
	cycle_data[4].back7 = 17;
	cycle_data[4].back8 = 16;
	cycle_data[4].back9 = 15;
	cycle_data[4].back10 = 14;

	cycle_data[5].now = 5;
	cycle_data[5].back1 = 4;
	cycle_data[5].back2 = 3;
	cycle_data[5].back3 = 2;
	cycle_data[5].back4 = 1;
	cycle_data[5].back5= 0;
	cycle_data[5].back6 = 19;
	cycle_data[5].back7 = 18;
	cycle_data[5].back8 = 17;
	cycle_data[5].back9 = 16;
	cycle_data[5].back10 = 15;

	cycle_data[6].now = 6;
	cycle_data[6].back1 = 5;
	cycle_data[6].back2 = 4;
	cycle_data[6].back3 = 3;
	cycle_data[6].back4 = 2;
	cycle_data[6].back5= 1;
	cycle_data[6].back6 = 0;
	cycle_data[6].back7 = 19;
	cycle_data[6].back8 = 18;
	cycle_data[6].back9 = 17;
	cycle_data[6].back10 = 16;

	cycle_data[7].now = 7;
	cycle_data[7].back1 = 6;
	cycle_data[7].back2 = 5;
	cycle_data[7].back3 = 4;
	cycle_data[7].back4 = 3;
	cycle_data[7].back5= 2;
	cycle_data[7].back6 = 1;
	cycle_data[7].back7 = 0;
	cycle_data[7].back8 = 19;
	cycle_data[7].back9 = 18;
	cycle_data[7].back10 = 17;

	cycle_data[8].now = 8;
	cycle_data[8].back1 = 7;
	cycle_data[8].back2 = 6;
	cycle_data[8].back3 = 5;
	cycle_data[8].back4 = 4;
	cycle_data[8].back5= 3;
	cycle_data[8].back6 = 2;
	cycle_data[8].back7 = 1;
	cycle_data[8].back8 = 0;
	cycle_data[8].back9 = 19;
	cycle_data[8].back10 = 18;

	cycle_data[9].now = 9;
	cycle_data[9].back1 = 8;
	cycle_data[9].back2 = 7;
	cycle_data[9].back3 = 6;
	cycle_data[9].back4 = 5;
	cycle_data[9].back5 = 4;
	cycle_data[9].back6 = 3;
	cycle_data[9].back7 = 2;
	cycle_data[9].back8 = 1;
	cycle_data[9].back9 = 0;
	cycle_data[9].back10 = 19;


	int i;
	for( i = 10;i<20;i++)
	{
	cycle_data[i].now = i;
	cycle_data[i].back1 = i-1;
	cycle_data[i].back2 = i-2;
	cycle_data[i].back3 = i-3;
	cycle_data[i].back4 = i-4;
	cycle_data[i].back5= i-5;
	cycle_data[i].back6 = i-6;
	cycle_data[i].back7 = i-7;
	cycle_data[i].back8 = i-8;
	cycle_data[i].back9 = i-9;
	cycle_data[i].back10 = i-10;
	}
}
