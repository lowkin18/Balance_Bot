/*
 * balance_control.h
 *
 *  Created on: Feb 12, 2017
 *      Author: c_ker
 */

#include "project_types.h"
#include <math.h>

#ifndef BALANCE_CONTROL_H_
#define BALANCE_CONTROL_H_

//init the timecycle data for looping data and previous values
void INIT_TIMECYCLE();

//Main Balance Control Algorithm
void balance_control_algo();
void init_QEI();
void getWheel_Data();

//Given a input speed it will perform this action
void speed_control_loop(float speed);


//Given an input set-Point angle it will perform this action
void PID_angle(float pos);
void PID_speed(float pos);

char init_main_loop();


//INITIALIZE ALL THE STARTING VALUES OF ROBOT to 0 for initial position
void init_balance_control();
void init_pid_values();


//TURNING ALGORITHMS
void center_turn_left();
void center_turn_right();
void forward_wheel_control();
void soft_turn_control();
void wheel_control();
void right_wheel_pivot();
void left_wheel_pivot();
//CONTROL ALGORITHIM FUNCTIONS
void position_calculations();
float speed_set();

int i2c_check_data();




//POSITION X-Y calcs
float position_XY_calc(float arc1,float arc2,float *x_movement,float *y_movement);
void rotation_matrix(float orientation,float *x_val,float *y_val);
float center_axis_rotation(int encoder_count,float * x_val, float*y_val);


#endif /* BALANCE_CONTROL_H_ */
