/*
 * camera_control.c
 *
 *  Created on: Mar 16, 2017
 *      Author: c_ker
 */

#include "camera_control.h"

PWM_Handle pwm_distance_level;
PWM_Handle pwm_top_pitch;
PWM_Handle pwm_top_yaw;

#define DISTANCE_LEVEL_MAX_BACK 6553
#define DISTANCE_LEVEL_MAX_FORWARD 13762
#define CAMERA_TILT_MAX_BACK 3932
#define CAMERA_TILT_MAX_FORWARD 14417
#define CAMERA_TURN_MAX_BACK 3932
#define CAMERA_TURN_MAX_FORWARD 15531.8

#define CAMERA_TILT_HOME 10485
#define CAMERA_TURN_HOME 11599
#define DISTANCE_LEVEL_HOME	7209

void level_mounts(float angle)
{
	int distance_level_duty;
	int top_pitch_duty;




	PWM_setDuty(pwm_distance_level,5000 );
	PWM_setDuty(pwm_top_pitch, 5000);
}


int distance_level_calc(float angle)
{
	int duty_calc = 10000;


	return duty_calc;
}
int camera_tilt_calc(float angle)
{
	int duty_calc = 10000;



	return duty_calc;
}

void init_levelers() {

	PWM_Params params1;
	uint16_t pwmPeriod = 5000;      // Period and duty in microseconds
	PWM_Params_init(&params1);
	params1.period = pwmPeriod;
	params1.dutyMode =  PWM_DUTY_SCALAR;
	pwm_distance_level = PWM_open(2, &params1);
	if (pwm_distance_level == NULL) {
		System_abort("Board_PWM0 did not open");
	}
	pwm_top_pitch = PWM_open(3, &params1);
	if (pwm_top_pitch == NULL) {
		System_abort("Board_PWM0 did not open");
	}
	pwm_top_yaw = PWM_open(4, &params1);
	if (pwm_top_yaw == NULL) {
		System_abort("Board_PWM0 did not open");
	}
	PWM_setDuty(pwm_distance_level, DISTANCE_LEVEL_HOME);
	PWM_setDuty(pwm_top_pitch, CAMERA_TILT_HOME);
	PWM_setDuty(pwm_top_yaw, CAMERA_TURN_HOME);
}
