/*
 * p5_globals.h
 *
 *  Created on: Nov 10, 2021
 *      Author: Bens PC
 */

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DataPipeLineHeader.h"
#include "ControlFunctionHeader.h"
#include "EncoderHeader.h"



#ifndef P5_GLOBALS_H_
#define P5_GLOBALS_H_



// include the struct that transfers data throughout the system
//#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
//#include "DataPipeLineHeader.h"







//
// Defines for the Maximum Dead Band values
//

#define  MOTOR_DEADBAND   50  // skal kigges på senere
#define  INVERTER_PERIOD 750     //TB CLOCK CYCLES // TIDLIGERE val
#define  INVERTER_START_DUTY  50 // duty cycle start value - duty value // [%]
#define  CONTROL_PERIOD 750      //
#define  Km 1.557               //Km for the generator





//// group defined function prototypes
extern void InitInverterPWM(void);
extern void InitMotorControlEPWM(void);
extern void InitChopperPWM(void);



//function protypes
// run in high speed isr
extern void pwm_update(DATA_PIPELINE_handle, CONTROL_handle);
extern void data_sampling(DATA_PIPELINE_HANDLE, DATA_PIPELINE_HANDLE,  POSSPEED_HANDLE);
//extern void data_sampling(DATA_PIPELINE_handle);
extern void encoder(void); // unclear
extern void motor_control(DATA_PIPELINE_handle, CONTROL_handle);
extern void chopper_control(DATA_PIPELINE_handle);


//Communication prototypes


extern void fifo_init(void);
extern void InitSciaGpio(void);
extern void scia_echoback_init(void);
extern void scia_xmit(int a);
extern void scia_msg (char * msg);



#endif /* P5_GLOBALS_H_ */



//
// End of File
//
