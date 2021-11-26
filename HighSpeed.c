/*
 * HighSpeed.c
 *
 *  Created on: Nov 11, 2021
 *      Author: Bens PC
 */

#include "p5_globals.h"
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DataPipeLineHeader.h"
#include "ControlFunctionHeader.h"




int ConversionCount = 0;
int duty_ramp_limit;

float dut = 50;
float duty_int_value = 50;
float V_DC, V_REF, duty_gain;



///////////////////////
//DEBUGGING CONSTANTS
int adc_timer_1 = 0;
int adc_timer_2 = 0;
int adc_difference = 0;

int pwm_timer_1 = 0;
int pwm_timer_2 = 0;
int pwm_difference = 0;
int pwm_count = 0;

long pwm_integer_counter = 0;
long adc_integer_counter = 0;

// now some constants to see the time difference between when we read the adc values and when we switch

long adc_time_stamp = 0;
long adc_time_stamp_counter = 0;
long pwm_time_stamp = 0;
long adc_pwm_time_difference = 0;
////////////




 void pwm_update(DATA_PIPELINE *data_pipeline, CONTROL *inverter_duty_control){

     /// DEBUGGING
     pwm_time_stamp = CpuTimer2Regs.TIM.all;

     pwm_integer_counter++;

     pwm_count++;
     if (pwm_count > 1){
         pwm_count = 0;
     }
     /// speed testing
     if (pwm_count == 0){
             pwm_timer_1 = CpuTimer2Regs.TIM.all;

     }
     else if (pwm_count == 1){

         pwm_timer_2 = CpuTimer2Regs.TIM.all;
         pwm_difference = pwm_timer_2 - pwm_timer_1;

     }

     /////////




    // this is where duty cycle for the inverter h bridge will be updated
    double Val = INVERTER_PERIOD;
    CONTROL_ramp_filter(inverter_duty_control);
    data_pipeline->inverter_duty_cycle = (Uint16) inverter_duty_control->integrator_value_1;//
    double DutyValue = data_pipeline->inverter_duty_cycle;
    double duty_cycle = (100 - DutyValue) * Val / 100;

    // Update the duty cycle value based on the high speed pipeline
    EPwm2Regs.CMPA.half.CMPA = (Uint16) duty_cycle;
    //EPwm2Regs.CMPB = (Uint16) duty_cycle;
}

extern void data_sampling(DATA_PIPELINE *data_pipeline, DATA_PIPELINE *data_sampling_pipeline,  POSSPEED *poss_speed){

    adc_time_stamp_counter++;
    if (adc_time_stamp_counter == 2){
        adc_time_stamp = CpuTimer2Regs.TIM.all;
        adc_pwm_time_difference = - pwm_time_stamp + adc_time_stamp;

    }else if (adc_time_stamp_counter == 4) {
        adc_time_stamp_counter = 0;
    }




    //// DEBUGGING
    adc_integer_counter++;

    /// speed testing
    if (ConversionCount == 1){
            adc_timer_1 = CpuTimer2Regs.TIM.all;

    }
    else if (ConversionCount == 2){

        adc_timer_2 = CpuTimer2Regs.TIM.all;
        adc_difference = adc_timer_2 - adc_timer_1;

    }
    ////////



            data_pipeline->I_inverter[ConversionCount] = AdcRegs.ADCRESULT0>>4;
            data_pipeline->V_DC[ConversionCount] = AdcRegs.ADCRESULT1>>4;
            data_pipeline->I_chopper[ConversionCount] = AdcRegs.ADCRESULT2>>4;
            data_pipeline->V_A[ConversionCount] = AdcRegs.ADCRESULT3>>4;
           // data_pipeline->V_B[ConversionCount] = AdcRegs.ADCRESULT4>>4;
           // data_pipeline->V_C[ConversionCount] = AdcRegs.ADCRESULT5>>4;



            float runnning_sum_voltage = 0;
            float running_i_chopper = 0;
            float running_sum_v_a = 0;
            float running_sum_v_b = 0;
            float running_sum_v_c = 0;

            float running_voltage_max = data_pipeline->V_DC[1];
            float running_voltage_min = data_pipeline->V_DC[1];
            int i = 0;
            for (i = 0; i < 10; i++)
            {
                if (data_pipeline->V_DC[i]>running_voltage_max){
                    running_voltage_max = data_pipeline->V_DC[i];
                }else if ((data_pipeline->V_DC[i]<running_voltage_min)){
                    running_voltage_min = data_pipeline->V_DC[i];
                }


                running_i_chopper += data_pipeline->I_chopper[i];
                running_sum_v_a += data_pipeline->V_A[i];
                //running_sum_v_b += data_pipeline->V_B[i];
                //running_sum_v_c += data_pipeline->V_C[i];

                if (data_pipeline->V_DC[i] <= 4095){
                runnning_sum_voltage +=data_pipeline->V_DC[i];

                }else
                    runnning_sum_voltage += 4095; /// look at this
            }

            data_pipeline->V_DC_AVRG= 18.6*(((runnning_sum_voltage/10))*3/4096); //Conversion from the digial value, to the measured value, to the average dc value

            data_pipeline->I_chopper_AVRG = running_i_chopper / 10;
            data_pipeline->V_A_AVRG = (float) running_sum_v_a / 10;
            //data_pipeline->V_B_AVRG = (float) running_sum_v_b / 10;
            //data_pipeline->V_C_AVRG = (float) running_sum_v_c / 10;




            running_voltage_max = ((((running_voltage_max))*3/4096));
            running_voltage_min = ((((running_voltage_min))*3/4096));
            data_pipeline->V_p2p = running_voltage_max-running_voltage_min;




            float runnning_sum_current = 0;
            float running_current_max = data_pipeline->I_inverter[0];
            float running_current_min = data_pipeline->I_inverter[0];
            i = 0;
            for (i = 0; i < 10; i++)
            {


                if (data_pipeline->I_inverter[i]>running_current_max){
                                  running_current_max = data_pipeline->I_inverter[i];
                              }else if ((data_pipeline->I_inverter[i]<running_current_min)){
                                  running_current_min = data_pipeline->I_inverter[i];
                              }

                if (data_pipeline->I_inverter[i] <= 4095){
                runnning_sum_current +=data_pipeline->I_inverter[i];
                }else
                    runnning_sum_current += 4095; // again += might be wrong


            }

            data_pipeline->I_avg = ((((runnning_sum_current/10))*3/4096));//-1.65)/(0.0002*50);

            data_pipeline->I_avg = ((((runnning_sum_current/10))*3/4096)*(-88.65))+147.31;

            running_current_max = ((((running_current_max))*3/4096)*(-88.65))+147.31;
            running_current_min = ((((running_current_min))*3/4096)*(-88.65))+147.31;
            data_pipeline->I_p2p = running_current_max-running_current_min;
            //147.31 -88.65

            // put data into the data_sampling_buffer
            data_sampling_pipeline->I_avg = data_pipeline->I_avg;
            data_sampling_pipeline->V_DC_AVRG = data_pipeline->V_DC_AVRG;
            data_sampling_pipeline->time_on_ms = CpuTimer1.InterruptCount;
            data_sampling_pipeline->time_on_s = CpuTimer2.InterruptCount;
            data_sampling_pipeline->rpm = poss_speed->SpeedRpm_fr;
            data_sampling_pipeline->V_A_AVRG = data_pipeline->V_A_AVRG;
            //data_sampling_pipeline->V_B_AVRG = data_pipeline->V_B_AVRG;
            //data_sampling_pipeline->V_C_AVRG = data_pipeline->V_C_AVRG;
            data_sampling_pipeline->I_chopper_AVRG = data_pipeline->I_chopper_AVRG;



    if(ConversionCount == 10)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }








}

extern void encoder(void){
    // unclear
}

extern void motor_control(DATA_PIPELINE *data_pipeline, CONTROL *duty_control){













/*
    // this will be the control loop for the motor-
    // data will be read from the high speed pipeline and then setpoints will be updated
    if (data_pipeline->inverter_duty_cycle > 98) {

        data_pipeline->inverter_duty_cycle = 1;
    }else {
        data_pipeline->inverter_duty_cycle ++;
    }
*/
    V_DC = data_pipeline->V_DC_AVRG;
    V_REF = data_pipeline->V_ref;

    dut = (( V_REF / V_DC) + 1)/2;
    dut = dut*100;
    if (dut >= 100){
        dut = 100;
    }else if (dut <= 0){
        dut = 0;
    }
    duty_control->ref_val = dut;
    //CONTROL_ramp_filter(duty_control);

    //data_pipeline->inverter_duty_cycle = (Uint16) duty_control->integrator_value_1;//


}

extern void chopper_control(DATA_PIPELINE *data_pipeline){
    if (data_pipeline->V_DC_AVRG > (data_pipeline->V_ref * 1.1) ){

        // make duty cycle 100 %percent
        EPwm6Regs.CMPA.half.CMPA = (Uint16) INVERTER_PERIOD;
    }else{
        EPwm6Regs.CMPA.half.CMPA = 0;
    }

    // chopper control
    // here we want to check if the voltage over the motor is over the rated voltage
    EPwm6Regs.ETCLR.bit.INT = 1; // who know
    float a;

}


//
//END OF FILE
//
