/*
 * ControlFunctionHeader.h
 *
 *  Created on: Nov 14, 2021
 *      Author: henri
 */

#ifndef CONTROLFUNCTIONHEADER_H_
#define CONTROLFUNCTIONHEADER_H_


//
// Typedef for the structure of the control struct
//
typedef struct
{
    //CCC specific values
    float integrator_value_1;
    float integrator_value_2;
    float gain_1;
    float gain_2;
    float gain_3;

    // physical/artificial limiter
    float limiter_1;
    float limiter_2;

    float ref_val;

    //Physical values

    float time_constant;

} CONTROL;


typedef CONTROL *CONTROL_handle;


// COME BACK TO
#define CONTROL_defaults {50,0, 0.001,1,1,0,0,0, \
        0.01,}

void CONTROL_Init(void);
void CONTROL_ramp_filter(CONTROL_handle);





#endif /* CONTROLFUNCTIONHEADER_H_ */
