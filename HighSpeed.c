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




 void pwm_update(DATA_PIPELINE *data_pipeline, CONTROL *inverter_duty_control){


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

extern void data_sampling(DATA_PIPELINE *data_pipeline){


    data_pipeline->I_inverter[ConversionCount] = AdcRegs.ADCRESULT0>>4;
    data_pipeline->V_DC[ConversionCount] = AdcRegs.ADCRESULT1>>4;



            float runnning_sum_voltage = 0;
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

                if (data_pipeline->V_DC[i] <= 4095){
                runnning_sum_voltage +=data_pipeline->V_DC[i];
                }else
                    runnning_sum_voltage += 4095;
            }

            data_pipeline->V_DC_AVRG= 18.6*(((runnning_sum_voltage/10))*3/4096); //Conversion from the digial value, to the measured value, to the average dc value

            running_voltage_max = ((((running_voltage_max))*3/4096));
            running_voltage_min = ((((running_voltage_min))*3/4096));
            data_pipeline->V_p2p = running_voltage_max-running_voltage_min;



            float runnning_sum_current = 0;
            float running_current_max = data_pipeline->I_inverter[1];
            float running_current_min = data_pipeline->I_inverter[1];
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
                    runnning_sum_current += 4095;


            }

            data_pipeline->I_avg = ((((runnning_sum_current/10))*3/4096));//-1.65)/(0.0002*50);

            data_pipeline->I_avg = ((((runnning_sum_current/10))*3/4096)*(-88.65))+147.31;

            running_current_max = ((((running_current_max))*3/4096)*(-88.65))+147.31;
            running_current_min = ((((running_current_min))*3/4096)*(-88.65))+147.31;
            data_pipeline->I_p2p = running_current_max-running_current_min;
            //147.31 -88.65


    if(ConversionCount == 9)
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



}


//
//END OF FILE
//
