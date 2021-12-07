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

void CONTROL_PI_AW_Current(CONTROL *control_loop, POSSPEED *encoder, DATA_PIPELINE *pipeline, float measured_val_curr){

    float old_intgr, ref_val, old_measured_val, gain1, gain2, gain3, point1, point2, back_calc, intgr_add, T, Kg, speed;
    float N_R2G = control_loop->n_roll / control_loop->n_generator;
    //float I_ref = control_loop->ref_val /  control_loop->gain_1; // I = t_ref / kg
    //float ka = 1/
    //float in;
    //in = I_ref - measured_val;



    //Pull data from struct
    old_intgr = control_loop->integrator_value_1;
    ref_val = (control_loop->ref_val);
    gain1 = control_loop->gain_1;
    gain2 = control_loop->gain_2;
    gain3 = control_loop->gain_3;
    point1 = control_loop->point_1;
    old_measured_val = control_loop->old_measured_val;
    T = control_loop->time_constant;
    Kg = control_loop->K_g;
    speed = encoder->SpeedRpm_pr * 0.10472; //Speed is converted from rpm to rad/s
    //measured_val = measured_value->I_avg; //Needs to be changed to correct value

    //Begin calculations

    ref_val -= (control_loop->b_dyno * N_R2G * speed + control_loop->c_dyno);
    ref_val = ref_val / ((control_loop->n_generator * control_loop->d_roll)/(control_loop->n_roll * control_loop->d_wheel));


    control_loop->limiter_1 = ref_val;
    point2 = (ref_val/Kg);
    point2 -= measured_val_curr;
    back_calc = point1 - old_measured_val;     //Back calculation is calculated from old values
    intgr_add = old_intgr + (point2 - back_calc*gain3)*T;
    point1 = point2*gain1 + intgr_add*gain2 + speed * N_R2G * Kg;

    //Update struct with values
    control_loop->integrator_value_1 = intgr_add;
    control_loop->point_1 = point1;
    control_loop->old_measured_val = measured_val_curr;

    //Calculate old measured val

    control_loop->old_measured_val = 0;

    //Debugging
    control_loop->point_2=point2;


}


void CONTROL_Init(void){



}
