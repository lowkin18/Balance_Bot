/*
 * motion_control.h
 *
 *  Created on: Mar 21, 2017
 *      Author: c_ker
 */

#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_


#include "project_types.h"

void init_balance_control();
void balance_control_algo();
void current_state_calculation(float x_dest, float y_dest, float orientation_dest);	//calculates the state needed to achieve desired position
char _requested_position_AB(float X_Requested, float Y_requested,float Orientation_requested); //REFACTORED STATE CALC


void calc_wheel_movement_turn(float theta);
//CALCULATES THETA DISPLACEMENT TO THE DESIRED ANGLE
float calc_theta_displacement(float current,float final);
void calculate_turn_state(float theta);


//THIS FUNCTION IS FOR YOUR TURNS FROM REMOTE CONTROLLER
//USE YOUR GLOBAL VARIABLES YOU MADE - MAKE A STRUCT IN PROJECT TYPE
//FOR REMOTE CONTROL DATA OR THE SPI DATA
void add_turn_values();


void calc_forward_wheel_movement(float distance);
float angular_to_path(float X, float Y);

#endif /* MOTION_CONTROL_H_ */
