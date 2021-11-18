/*
 * ControlFunction.c
 *
 *  Created on: Nov 14, 2021
 *      Author: henri
 */
#include "ControlFunctionHeader.h"

void CONTROL_ramp_filter(CONTROL *control_loop){

    float old_intgr, int_add, ref_val, gain;
    int ramp_limiter;



    //Extract values from struct

    old_intgr = control_loop->integrator_value_1;
    ramp_limiter = control_loop->limiter_1;
    ref_val = control_loop->ref_val;
    gain = control_loop->gain_1;



    //Calculate the value to be added to the integrator value

    int_add = (ref_val-old_intgr) * gain;

    //Check if the add value is bigger than the ramp limiter

    if (int_add > ramp_limiter){
        int_add = ramp_limiter;

    }else if ( int_add < -ramp_limiter){
        int_add = -ramp_limiter;
    }
    //add value to integrator value

    control_loop->integrator_value_1 = old_intgr + int_add;

}

void CONTROL_Init(void){



}
