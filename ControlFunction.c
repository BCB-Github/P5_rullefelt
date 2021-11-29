/*
 * ControlFunction.c
 *
 *  Created on: Nov 14, 2021
 *      Author: henri
 */
#include "ControlFunctionHeader.h"
#include "DataPipeLineHeader.h"
#include "p5_globals.h"

void CONTROL_ramp_filter(CONTROL *control_loop){

    float old_intgr, int_add, ref_val, gain, ramp_limiter;




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

void CONTROL_PID_AW_controller(CONTROL *control_loop, float measured_val){

    float old_intgr, ref_val, old_measured_val, gain1, gain2, gain3, point1, point2, back_calc, intgr_add;

    float I_ref = control_loop->ref_val /  control_loop->kg; // I = t_ref / kg
    //float ka = 1/
    float in;
    in = I_ref - measured_val;



    //Pull data from struct
    old_intgr = control_loop->integrator_value_1;
    ref_val = control_loop->ref_val;
    gain1 = control_loop->gain_1;
    gain2 = control_loop->gain_2;
    gain3 = control_loop->gain_3;
    point1 = control_loop->point_1;
    old_measured_val = control_loop->old_measured_val;
    //measured_val = measured_value->I_avg; //Needs to be changed to correct value




    //Begin calculations

    point2 = ref_val - Km * measured_val;

    back_calc = point1 - old_measured_val;     //Back calculation constant is calculated from old values
    intgr_add = old_intgr + point2*gain2 - back_calc*gain3;
    point1 = point2*gain1 + intgr_add;

    //Update struct with values
    control_loop->integrator_value_1 = intgr_add;
    control_loop->point_1 = point1;
    control_loop->old_measured_val = measured_val;



}


void CONTROL_Init(void){



}
