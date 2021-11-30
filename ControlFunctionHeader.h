/*
 * ControlFunctionHeader.h
 *
 *  Created on: Nov 14, 2021
 *      Author: henri
 */


#ifndef CONTROLFUNCTIONHEADER_H_
#define CONTROLFUNCTIONHEADER_H_

#define B_DYNO 0.0091766
#define C_DYNO 0.66655
#define J_DYNO 0.01192
#define N_GENERATOR 22
#define N_ROLL 28
//#define N_R2G 1.272727
#define D_ROLL 0.09
#define D_WHEEL 0.55
#define K_G 0.072
#define KP 0.07952
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
    float point_1;
    float point_2;

    // physical/artificial limiter
    float limiter_1;
    float limiter_2;
    float ref_val;
    float old_measured_val;


    //Physical values
    float time_constant;
    float b_dyno;
    float c_dyno;
    float j_dyno;
    float n_generator;
    float n_roll;
    float d_roll;
    float d_wheel;
    float K_g;
    //float kg;
    //float kp;
    //float Ka;
    //float t_wheel_ref;


} CONTROL;


typedef CONTROL *CONTROL_handle;


// COME BACK TO
//#define CONTROL_defaults {50,0, 0.001,1,1,0,0,0,0,0,0, \
        0.01,0, 0, B_DYNO, C_DYNO, J_DYNO, N_GENERATOR, N_ROLL, D_ROLL, D_WHEEL, KG, KP}


#define CONTROL_defaults {50,0,0,0,0,0,0,  0,0,0,0, \
        0.0004, B_DYNO, C_DYNO, J_DYNO, N_GENERATOR, N_ROLL, D_ROLL, D_WHEEL, K_G}

//Time constant is 1/2500 right now (can/should be calculated more accurately)


void CONTROL_Init(void);
extern void CONTROL_ramp_filter(CONTROL_handle);





#endif /* CONTROLFUNCTIONHEADER_H_ */
