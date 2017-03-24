/*
 * motion_control.c
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 *
 *
 *      THIS FILE IS MEANT FOR THE MOTION CONTROL ALGORITHMS OF THE ROBOT
 *      IT WILL CONTRAIN ALL THE FUNCTIONS PERTAINING TO THE MOTION CONTROL
 *      ALGORITHMS THE BOT WILL USE.
 *
 *      BASICALLY PATH PLANNING AND STATE DECISIONS THAT WILL BE MADE
 */

#include "motion_control.h"

/***************************************************************************************
 * init_balance_control()
 *
 * This function initializes all the starting values for the balance_control
 * Sets the initial position as the reference and all X - Y values are from starting orientation
 * of the robot Y Axis is forward and X axis is parallel to wheels
 *
 *
 **************************************************************************************/

void init_balance_control() {
	ctrl_robot.current_orientation = 0;
	ctrl_robot.x_abs_pos = 0.000;				//x_abs_pos
	ctrl_robot.y_abs_pos = 0.000;				//y_abs_pos
	ctrl_robot.wheel_L_fpos = 0;	 			//final position of left wheel
	ctrl_robot.wheel_R_fpos = 0; 				//final position right wheel
	ctrl_robot.current_orientation = 0;	//value is in RADIANS -- CONVERT TO ANGLE
	ctrl_robot.final_orientation = 0;//direction of robot orientation of robot and in RADIANS
	ctrl_robot.IMU_orientation = 0;
	ctrl_robot.current_speed = 0;				//speed requested of the robot
	ctrl_robot.current_state = 0;
	ctrl_robot.turn_percent = 0;  //use values from -100% to 100% left to right
	ctrl_robot.control_state = 1;
	qei_data.left_wheel.set_point = 0;
	qei_data.right_wheel.set_point = 0;

	int counter = 0;
	for (counter = 0; counter < 20; counter++) {
		ctrl_robot.c_X_pos[counter] = 0;		//distance in meters X-Y
		ctrl_robot.c_Y_Pos[counter] = 0;	//absololute position in meters X-Y
		ctrl_robot.wheel_L_cpos[counter] = 0;	//current position of left wheel
		ctrl_robot.wheel_R_cpos[counter] = 0;//current position of right wheel
	}
}

/*************************************************************************************
 * current_state_calculation()
 *
 * Params are X Y and Orientation
 *
 * REturn VOID
 *
 * This function will take the X Y and Orientation target and Decide based on current Position
 * the Ideal state for the robot to achieve this position and target location
 *
 */
void current_state_calculation(float x_dest, float y_dest,
		float orientation_dest) {
	float direction_angle = tan((x_dest) / (y_dest));//angle needed to go to new dest in straight line
	float c_orientation;  					//current orientation placeholder
	float x_offset = x_dest - ctrl_robot.x_abs_pos;
	float y_offset = y_dest - ctrl_robot.y_abs_pos;

	if (ctrl_robot.current_orientation > 2 * PI
			|| ctrl_robot.current_orientation < -2 * PI) {
		c_orientation = ctrl_robot.current_orientation;
	} else {
		c_orientation = ctrl_robot.current_orientation;
	}
	float angle_offset = direction_angle - c_orientation;
	if (angle_offset > PI) {
		angle_offset -= 2 * PI;
	}
	if (angle_offset < - PI) {
		angle_offset += 2 * PI;
	}
	if (!(angle_offset > 0.087266) || !(angle_offset < -0.087266)) {
		if (!(x_offset > WHEEL_CENTER || x_offset < -WHEEL_CENTER)
				&& !(y_offset > WHEEL_CENTER || y_offset < -WHEEL_CENTER)) {
			ctrl_robot.current_state = 0;
		} else {
			ctrl_robot.current_state = 0;
		}
	} else if ((x_offset > WHEEL_CENTER || x_offset < -WHEEL_CENTER)
			&& (y_offset > WHEEL_CENTER || y_offset < -WHEEL_CENTER)) {
		if (angle_offset > 0.087266 || angle_offset < -0.087266) {
			if (angle_offset < 0) {
				ctrl_robot.current_state = 4;
				qei_data.right_wheel.dynamic_pos = 0;
				qei_data.left_wheel.dynamic_pos = 0;
			}
			if (angle_offset > 0) {
				ctrl_robot.current_state = 5;
				qei_data.right_wheel.dynamic_pos = 0;
				qei_data.left_wheel.dynamic_pos = 0;
			}
		} else {
			ctrl_robot.current_state = 0;
		}
	}

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
void balance_control_algo() {

	getWheel_Data();
	position_calculations();

	switch (ctrl_robot.control_state) {
	case 0:
		/*
		 current_state_calculation(ctrl_robot.x_set_point,
		 ctrl_robot.y_set_point,
		 ctrl_robot.final_orientation);
		 */
		PID_speed(PID_pos());
		break;
	case 1:
		if (ctrl_robot.current_orientation > PI / 2) {
			ctrl_robot.control_state = 0;
			ctrl_robot.current_state = 0;
			ctrl_robot.wheel_L_fpos =
					ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
			ctrl_robot.wheel_R_fpos =
					ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel
			qei_data.right_wheel.dynamic_pos = 0;
			qei_data.left_wheel.dynamic_pos = 0;

		}
		PID_speed(PID_pos());
		break;
	case 2:
		if (_requested_position_AB(ctrl_robot.x_set_point,
				ctrl_robot.y_set_point,ctrl_robot.final_orientation)) {
			ctrl_robot.control_state = 0;
		}
			PID_speed(PID_pos());
		break;
	case 3:
		// THIS STATE IS YOUR REMOTE CONTROL STATE - PUT THE REMOTE CONTROL DATA INTO A FUNCTION THAT CONVERTS IT TO SPEED
		// THEN SEND THAT SPEED TO THE PID_speed
		add_turn_values();
		PID_speed(0);//PUT YOUR REMOTE CONTROL SPEED VALUE INTO THE SPEED PID
		//SET THE CONTROL STATE TO BE state 3
		break;
	case 4:
		break;

	}
}

/*************************************************************************************
 * char _requested_position(float X_Requested, float Y_requested,float Orientation_requested)
 *
 * Params X requested Y requested and Orientatioon Requested
 *
 *Returns 1 if error occurs
 *
 *
 *This function will take the requested position and determine the best movement to get the robot
 *to this position.
 *
 *Currently it will be center point or pivot turns and then just straight forward
 *
 *Will work on implementing gradual turns to the robot to work with curved trajectories
 *
 ******************************************************************************************/
char _requested_position_AB(float X_requested, float Y_requested,
	float Orientation_requested) {
	float X_current = ctrl_robot.x_abs_pos;
	float Y_current = ctrl_robot.y_abs_pos;
	float theta_current = ctrl_robot.current_orientation;
	float theta_final_displacement = calc_theta_displacement(theta_current,
			Orientation_requested);	//FINAL THETA DISPLACE FROM MOVEMENT THETA
	float X_displacement = X_requested - X_current;
	float Y_displacement = Y_requested - Y_current;
	float total_distance = sqrt(X_displacement*X_displacement + Y_displacement*Y_displacement);
	float angle_to_path = angular_to_path(X_displacement,Y_displacement);
	float theta_displacement = calc_theta_displacement(theta_current,
			angle_to_path);

	if(!(theta_final_displacement > 0.017453292 || theta_final_displacement < -0.017453292)&&!(X_displacement > 0.10 || X_displacement < -0.10)
			&& !(Y_displacement > 0.10 || Y_displacement < -0.10))
	{
		ctrl_robot.control_state = 0;
		ctrl_robot.wheel_L_fpos = ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
		ctrl_robot.wheel_R_fpos = ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel

	}
	//IF THETA IS LESS THAN 2 DEGREES WE CAN MOVE FORWARD TO POINT
	else if((!(theta_displacement > 0.017453292 ||theta_displacement< -0.017453292))
			|| ctrl_robot.current_state == 0) {
		ctrl_robot.current_state = 0;
		//IS THE ROBOT WITHIN TURNING DISTANCE OF X and Y location
		if (!(X_displacement > 0.05 || X_displacement < -0.05)
				&& !(Y_displacement > 0.05 || Y_displacement < -0.05)) {
			if (theta_final_displacement > 0.017453292
					|| theta_final_displacement < -0.017453292) {
				calculate_turn_state(theta_final_displacement);
			}
		} else if (theta_displacement > 0.017453292) {
			qei_data.left_wheel.dynamic_pos += 5;

		} else if (theta_displacement < -0.017453292) {
			qei_data.right_wheel.dynamic_pos += 5;
		}
		else
		{
		 calc_forward_wheel_movement(total_distance);
		}
	}
	else if (!(X_displacement > 0.05 || X_displacement < -0.05)
			&& !(Y_displacement > 0.05 || Y_displacement < -0.05)) {
		calculate_turn_state(theta_final_displacement);
	} else if (theta_displacement) {
		calculate_turn_state(theta_displacement);
	}

	//if failure return 0
	if (0)
		return 1;

	return 0;
}

/*************************************************************************************
 * float calc_theta_displacement(float current,float final)
 *
 * Params Current Theta and Final THeta
 *
 *returns theta displacement angle
 *
 *This function will take two angles and then return the total angular displacement
 *It will determine the shortest angular displacment as well.
 *
 ******************************************************************************************/
float calc_theta_displacement(float current, float final) {
	float theta_displacement = final - current;
	if (theta_displacement > PI) {
		theta_displacement -= 2 * PI;
	} else if (theta_displacement < -PI) {
		theta_displacement += 2 * PI;
	}

	return theta_displacement;

}

/*************************************************************************************
 *void calculate_turn_state(float theta)
 *
 * Params THeta
 *
 * Calculates teh state needed to perform the Turn
 *
 * Need to add in Pivot turn control algorithm to decide when it's best to pivot turn
 *
 ******************************************************************************************/
void calculate_turn_state(float theta) {
	if (theta > 0) {
		ctrl_robot.current_state = 5;
	}
	if (theta < 0) {
		ctrl_robot.current_state = 4;
	}
	calc_wheel_movement_turn(theta);
}

/*************************************************************************************
 * void calc_wheel_movement_turn(float theta);
 *
 * Params Theta
 *
 *This function will see what the theta value is and add it to the wheel_L_fpos value
 *
 ******************************************************************************************/
void calc_wheel_movement_turn(float theta)
{
	if(theta < 0)
	{
		ctrl_robot.wheel_L_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
		ctrl_robot.wheel_R_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel
	}
	else
	{
		ctrl_robot.wheel_L_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];//final position of left wheel
		ctrl_robot.wheel_R_fpos = (theta * WHEEL_CENTER) * (39400/WHEEL_CIRC) + ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];//final position right wheel
	}
}


/*************************************************************************************
 * void angular_to_path()
 *
 * X and Y arguments
 *
 * this function will calculate the desired angle to get to the X and Y plane
 *
 ******************************************************************************************/
float angular_to_path(float X, float Y)
{
	if((X> 0.30 || X <-0.30)|| (Y > 0.30 || Y <-0.30) )
	{
	float theta;
	if(!(X > 0.30 || X <-0.30))
	{
		if(!(Y > 0.30 || Y <-0.30))
		{
		return 0.00;
		}
		else if(Y < -0.30)
		{
		return -PI/2;
		}
		else if(Y > 0.30);
		return PI/2;
	}
	else if(!(Y > 0.30 || Y <-0.30))
	{
		if(X > 0.30)
		{
			return 0;
		}
		else if(X < -0.30)
		{
			return PI;
		}
		else
		{
			return 0;
		}
	}
	else if(X != 0 && Y != 0)
	{
		theta = (float)atan2(Y,X);
		return theta;
	}
	}
	else return 0;
}



void calc_forward_wheel_movement(float distance)
{
	float encoder_pulses = (distance/WHEEL_CIRC) * 39400;
	ctrl_robot.wheel_L_fpos = encoder_pulses+ctrl_robot.wheel_L_cpos[cycle_data[speed_loop].now];
	ctrl_robot.wheel_R_fpos = encoder_pulses+ctrl_robot.wheel_R_cpos[cycle_data[speed_loop].now];
}

//PUT YOUR GLOBAL TURN VALUES THROUGH SCA AND JUST ADD TO THE DYNAMIC POS AND THE ROBOT WILL TURN
void add_turn_values()
{

	//GLOBAL TURN FROM YOUR FUNCTION TO THE BYTE

	if(1)
	{
	qei_data.left_wheel.dynamic_pos+=0;//INPUT THE TURN AMOUNT
	}
	if(1)
	{
	qei_data.right_wheel.dynamic_pos+=0;//INPUT THE TURN AMOUNT
	}



}
