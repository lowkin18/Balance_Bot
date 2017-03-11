/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp006.c ========
 */

#include "project_types.h"
#include "balance_control.h"
#include "button_control.h"
#include "lcd_gui.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Log.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles
#include <ti/sysbios/knl/Clock.h>
/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"
#include "bno055.h"

/*Hardware Drivers*/
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

/* Driver Libs */
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"




void ADC_init();



I2C_Handle i2c;
I2C_Transaction i2cTransaction;


/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */



void ADC_init()
{




}


Void task_i2c_fxn(UArg arg0, UArg arg1)
{
	I2C_Params      i2cParams;
	/* Create I2C for usage */

	I2C_Params_init(&i2cParams);
	i2cParams.bitRate = I2C_400kHz;				//CLOCK SPEED FAST
	i2cTransaction.slaveAddress = 0X28;
	i2c = I2C_open(1, &i2cParams);	//initialize the params
	if (i2c == NULL) {
	        return NULL;
	    }

	//i2c_init_bno055_tm4c123(&i2c);
	for(;;)
	{
		/*
		bno055_convert_float_euler_hpr_deg(&imu_data[angle_loop].euler_values);
		bno055_convert_float_gyro_x_dps(&imu_data[angle_loop].x_gyro);

		int check = i2c_check_data();
		if(check)
		{
		bno055_convert_float_euler_hpr_deg(&imu_data[angle_loop].euler_values);
		bno055_convert_float_gyro_x_dps(&imu_data[angle_loop].x_gyro);
		}

		//System_printf("Euler Pitch = %f\t\n",imu_data[angle_loop].euler_values.p);
		//System_printf("Euler Pitch = %f\t\n",imu_data[angle_loop].euler_values.h);
		//System_printf("Absolute position left %d \t right %d\n",qei_data.left_wheel.abs_pos,qei_data.right_wheel.abs_pos );
		//System_printf("PID setpoint Speed %f\n",_PIDS.setPoint[cycle_data[speed_loop].now]);
		//System_printf("Gyro_accel = %f\n",imu_data[angle_loop].x_gyro);
		//System_printf("ANGLE_OUTPUT = %f\n", setAngle[angle_loop]);
		//System_printf("RIGHT_WHEEL DIR = %d\t LEFT_WHEEL DIR %d\n",qei_data.right_wheel.wheel_dir,qei_data.left_wheel.wheel_dir);
		//System_flush();
		//LCD_DATA LOOP POST WILL ENABLE WHEN WIRED UP
		 */

		if(1)
		{
		Semaphore_post(Semaphore_i2c_data);
		Semaphore_post(Semaphore_gui);
		}
		if(angle_loop%5 == 0)// && menu_state.main_state != 10)
		{

		}
		if(angle_loop==10 || angle_loop == 0)
		{
				//Hwi_enable();
		}
	Task_sleep(10); //SLEEP FOR 9 miliseconds cause you can only read at 100hz
	}
}


Void task_pwm_fxn(UArg arg0, UArg arg1)
{
		PWM_Handle pwm1;
	    PWM_Handle pwm2;
	    PWM_Params params;
	    uint16_t   pwmPeriod = 50;      // Period and duty in microseconds
	    uint16_t   duty = 0;
	    PWM_Params_init(&params);
	    params.period = pwmPeriod;
	    params.dutyMode = PWM_DUTY_SCALAR;
	    pwm1 = PWM_open(Board_PWM0, &params);
	    if (pwm1 == NULL) {
	        System_abort("Board_PWM0 did not open");
	    }
	    if (Board_PWM1 != Board_PWM0) {
	        pwm2 = PWM_open(Board_PWM1, &params);
	        if (pwm2 == NULL) {
	            System_abort("Board_PWM1 did not open");
	        }
	    }
	    //FOREVER TASK LOOP
	    for(;;)
		{
		Semaphore_pend(Semaphore_i2c_data, BIOS_WAIT_FOREVER);
		// GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
		balance_control_algo();										//PID LOOP DATA;
		// GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
		//SET THE DIRECTION BITS FOR THE MOTORS
		if(qei_data.left_wheel.pwm_out > 5000)
		{
		qei_data.left_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0): GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
		}
		if(qei_data.right_wheel.pwm_out > 5000)
		{
		qei_data.right_wheel.wheel_dir == 1 ? GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2):GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
		}
		//SET THE PWM VALUES FOR THE MOTORS
		PWM_setDuty(pwm1, qei_data.left_wheel.pwm_out);										//LEFT WHEEL
		PWM_setDuty(pwm2, qei_data.right_wheel.pwm_out);									//RIGHT WHEEL
		//System_printf("duty = %d\n",duty);
		}
}


Void task_qei_fxn(UArg arg0, UArg arg1)
{
	for(;;)
		{



		Task_sleep(40000);
		}
}



Void task_gui_fxn(UArg arg0, UArg arg1)
{
		Semaphore_pend(Semaphore_gui,BIOS_WAIT_FOREVER);
		/*
		I2C_Params      i2cParams;
		I2C_Params_init(&i2cParams);
		i2cParams.bitRate = I2C_400kHz;				//CLOCK SPEED FAST
		i2cTransaction.slaveAddress = 0X27;
		I2C_Handle i2c1 = I2C_open(1, &i2cParams);	//initialize the params
		if (i2c1 == NULL) {
			        return NULL;
			   }
		*/
		init_LCD_i2c(i2c);
		start_up_sequence();
		for(;;)
		{
		Semaphore_pend(Semaphore_gui,BIOS_WAIT_FOREVER);
		//robot_abs_position();
		menu_print_control();
		Task_sleep(50);
		}
}






/*
 *  ======== main ========
 */
int main(void)
{
	//TI_RTOS BOARD INIT PARAMS
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initPWM();
    ////////////////////////////////////////////////////////////



    INIT_TIMECYCLE();		//init time cycle foor looping arrays
    init_QEI(); 			//init QEI hardware for quadrature
    init_balance_control(); //init balance starting points set values two 0
    init_pid_values();
    //TIMER_Initialize();
    init_buttons();			//init buttons for the LED gui;

    BIOS_start();			//start the BIOS for the TI_RTOS - TASKS START HERE

    return (0);
}







