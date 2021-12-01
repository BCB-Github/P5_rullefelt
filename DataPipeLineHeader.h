/*
 * DataPipeLineHeader.h
 *
 *  Created on: Nov 11, 2021
 *      Author: Bens PC
 */

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#ifndef DATAPIPELINEHEADER_H_
#define DATAPIPELINEHEADER_H_





#ifndef __DATA_PIPELINE__
#define __DATA_PIPELINE__

//
// Included Files
//

//
// Typedef for the structure of the data pipeline struct
//
typedef struct
{
    Uint16 V_DC[10];             // voltage over power board
    float V_DC_AVRG;           //Average voltage over power board
    Uint16 inverter_duty_cycle;            // desired voltage over  inverter motor
    Uint16 I_inverter[10];       // averaged current over motor
    float I_avg;
    // new inclusions
    float I_chopper[10];
    float I_chopper_AVRG;

    float V_A[10];
    float V_A_AVRG;

    float V_B[10];
    float V_B_AVRG;

    float V_C[10];
    float V_C_AVRG;

    float I_p2p;            //Peak to Peak values of current and voltage
    float V_p2p;

    float T_ref;            // reference torque from non deterministic loop
    float rpm_ref;
    float V_ref;            // Reference voltage that controls the chopper
    float V_output_ref;     // Reference voltage of the inverter
    int time_on_s;         // time since device turned on
    int time_on_ms;         //
    int32 rpm;              // rpm from encoder

    int chop_dis;          //Whether chopper is discharging or not

    // Function prototypes. Actual functions are found in DataPipeline.c
    //void (*init)();         // Pointer to the init function
    //void (*calc)();         // Pointer to the calc function
} DATA_PIPELINE;


typedef DATA_PIPELINE *DATA_PIPELINE_handle;


// COME BACK TO
#define DATA_PIPELINE_DEFAULTS {0x0, 0x0, 50,0x0, 0x0, 0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0, 0x0, 0x0,0x0,0x0, 0x0, 0x0, 0x0, 0x0, 0}//, \
        (void (*)(long))DATA_PIPELINE_Init, \
        (void (*)(long))DATA_PIPELINE_Calc }



void DATA_PIPELINE_Init(void);
void DATA_PIPELINE_Calc(DATA_PIPELINE_handle);


#endif // NOW THE DATA PIPELINE SHOULD BE DEFINED





#endif /* DATAPIPELINEHEADER_H_ */


