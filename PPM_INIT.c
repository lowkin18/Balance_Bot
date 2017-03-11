#define PART_TMC123GH6PM

#include "PPM_INIT.h"

volatile float posEdge=0;
volatile float negEdge=0;
volatile float pulseWidth=0;
volatile float posEdge1=0;
volatile float negEdge1=0;
volatile float pulseWidth1=0;

/*volatile unsigned int posEdge=0;
volatile unsigned int negEdge=0;
volatile unsigned int pulseWidth=0;
volatile unsigned int i=0;
float setAngle=0;*/

void TIMER_Initialize(void)
{
    // Enables timers 1 2 fwrd/rev
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enables timers 0 3 left/right
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Timer config for frwd/rev PPM
    GPIOPinConfigure(GPIO_PB4_T1CCP0);
    GPIOPinConfigure(GPIO_PB0_T2CCP0);

    // Timer config for left/right PPM
    GPIOPinConfigure(GPIO_PB7_T0CCP1);
    GPIOPinConfigure(GPIO_PB3_T3CCP1);

    // Timer type set
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Configure timers to edge time capture
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP);

    // Event set to both pulse edges
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);

    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMER3_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);


    // Enable timer interrupts
    TimerIntEnable(TIMER1_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);

    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    TimerIntEnable(TIMER3_BASE, TIMER_CAPB_EVENT);



    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER2A);
    IntEnable(INT_TIMER0B);
    IntEnable(INT_TIMER3B);

    // Enables processor interrupts
    IntMasterEnable();

    // Enables the timers
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);

    TimerEnable(TIMER0_BASE, TIMER_B);
    TimerEnable(TIMER3_BASE, TIMER_B);

}


/*********fwrd/rev PPM ISRs*********/

void capture_pos_fxn()
{
   TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);
   posEdge = TimerValueGet(TIMER1_BASE, TIMER_A);
   //TimerIntDisable(TIMER1_BASE, TIMER_CAPA_EVENT);
   //TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);
}

void capture_neg_fxn()
{
    TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);
    negEdge = TimerValueGet(TIMER2_BASE, TIMER_A);
    pulseWidth=negEdge-posEdge;
    /*setAngle[angle_loop]=((0.0004545*pulseWidth)-54.545454);
    if(setAngle[angle_loop] > 15 || setAngle[angle_loop] < -15)
            {
                setAngle[angle_loop] = setAngle[cycle_data[angle_loop].back1];
            }*/
    setAngle[angle_loop]=((0.0004545*pulseWidth)-54.545454);
}

/*********left/right PPM ISRs*********/

void capture_pos_fxn2()
{
     TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);
     posEdge1 = TimerValueGet(TIMER0_BASE, TIMER_B);
     //TimerIntDisable(WTIMER0_BASE, TIMER_CAPB_EVENT);
     //TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT);
}

void capture_neg_fxn2()
{
    TimerIntClear(TIMER3_BASE, TIMER_CAPB_EVENT);
    negEdge1 = TimerValueGet(TIMER3_BASE, TIMER_B);
    pulseWidth1=negEdge1-posEdge1;
  /*  turnAngle[angle_loop]=((0.0004545*pulseWidth1)-54.545454);
       if(turnAngle[angle_loop] > 15 || turnAngle[angle_loop] < -15)
           {
               turnAngle[angle_loop] = turnAngle[cycle_data[angle_loop].back1];
           }*/
    turnAngle[angle_loop]=((0.0030303*pulseWidth1)-363.6363636);


}


