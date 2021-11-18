//###########################################################################

//! Each ePWM is configured to interrupt on the 3rd zero event


//! \b External \b Connections \n

//!  - EPWM2A is on GPIO2
//!  - EPWM2B is on GPIO3
//
//
//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "p5_globals.h"







// ADC DEFINES
__interrupt void adc_isr(void);

//
// ADC module clock = HSPCLK/1      = 25.5MHz/(1)   = 25.0 MHz

//
// S/H width in ADC module periods = 2 ADC cycle
//
#define ADC_SHCLK  0x1
#define ADC_CKPS   0x0
#define ADC_MODCLK 0x3

#define CPU_CLK   150e6
#define PWM_CLK   5e3
#define SP        CPU_CLK/(2*PWM_CLK)
#define TBCTLVAL  0x200E      // up-down count, timebase=SYSCLKOUT







//
// Function Prototypes
//
void InitInverterEPWM(void);
void InitEncoderEPWM(void);
void InitEQepGpio(void);
void BoardStartup(void);
__interrupt void high_speed_isr(void);
__interrupt void motor_control_isr(void);
__interrupt void prdTick(void);


//
// Globals
//
Uint16  EPwm2_DB_Direction;
Uint32  EPwm2TimerIntCount;
Uint16 Interrupt_Count = 0;

DATA_PIPELINE high_speed_pipeline = DATA_PIPELINE_DEFAULTS;

CONTROL inverter_duty_control = CONTROL_defaults;

POSSPEED qep_posspeed=POSSPEED_DEFAULTS;










//
// Defines to keep track of which way the Dead Band is moving
//

//
// Main
//
void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2833x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //


    InitInverterPWM();       // Initialize the PWM GPIO we actually want to use // GPIO 2, 3


    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.

    InitPieCtrl();    // The default state is all PIE interrupts disabled and flags are cleared.




    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    // This function is found in DSP2833x_PieVect.c.
    InitPieVectTable();


    //
    // Initialise the control loop structs with the correct values (ie. the values that deviate from the defaults)
    //

    //Duty control

    inverter_duty_control.integrator_value_1 = 50;
    inverter_duty_control.gain_1 = 0.01;
    inverter_duty_control.limiter_1 = 0;
    inverter_duty_control.ref_val = 50;


    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;    // This is needed to write to EALLOW protected registers
    // Changes the duty cycle in the epwm function
    PieVectTable.EPWM2_INT = &high_speed_isr; // function that runs during interrupt
    PieVectTable.EPWM3_INT = &motor_control_isr;

    PieVectTable.ADCINT = &adc_isr;

    //Encoder interrupt

    PieVectTable.EPWM1_INT = &prdTick;

    EDIS;      // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2833x_InitPeripherals.c
    //
    // InitPeripherals();  // Not required for this example
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;



    //Init everything here:

    InitInverterEPWM();
    InitMotorControlEPWM();

    InitEncoderEPWM();

    InitEQepGpio();

    BoardStartup();

    //
    // Specific clock setting for this example:
    //

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    // ADC CLOCK SETTINGS

    //Bitchboi bein a BIT of a problem
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;    // HSPCLK = SYSCLKOUT/ADC_MODCLK
    EDIS;

    //
    // Step 5. User specific code, enable interrupts





    // ADC
    //
    // InitPeripherals(); // Not required for this example
    InitAdc();         // For this example, init the ADC





/*

    //
    // Configure ADC
    //
    AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x1; // Setup ADCINA3 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x0; // Setup ADCINA2 as 2nd SEQ1 conv.

    //
    // Enable SOCA from ePWM to start SEQ1
    //
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA = 0x0080;  // Set compare A value
    EPwm1Regs.TBPRD = 0xFFFF;           // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;    // count up and start


*/
/*
    //
    // Specific ADC setup for this example:
    //

    //
    // Sequential mode: Sample rate   = 1/[(2+ACQ_PS)*ADC clock in ns]
    //                      = 1/(3*40ns) =8.3MHz (for 150 MHz SYSCLKOUT)
    //                     = 1/(3*80ns) =4.17MHz (for 100 MHz SYSCLKOUT)
    // If Simultaneous mode enabled: Sample rate=1/[(3+ACQ_PS)*ADC clock in ns]
    //
    AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;     // 1  Cascaded mode
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1;     // Setup continuous run
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;     // Enable Sequencer override feature
    AdcRegs.ADCCHSELSEQ1.all = 0x0; // Initialize all ADC channel selects to A0
    AdcRegs.ADCCHSELSEQ2.all = 0x0;
    AdcRegs.ADCCHSELSEQ3.all = 0x0;
    AdcRegs.ADCCHSELSEQ4.all = 0x0;

    //
    // convert and store in 8 results registers
    //
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x7;

    //
    // Start SEQ1
    //
    AdcRegs.ADCTRL2.all = 0x2000;










    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
*/

    // Configure ADC
    //
    AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x3; // Setup ADCINA3 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.

    //
    // Enable SOCA from ePWM to start SEQ1
    //
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;

    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm4Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
    EPwm4Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC from from CPMA on upcount
    EPwm4Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm4Regs.CMPA.half.CMPA = 0x0080;  // Set compare A value
    EPwm4Regs.TBPRD = (Uint16) INVERTER_PERIOD/3;           // Set period for ePWM1
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;    // count up and start



    //
    // Initialize counters:
    EPwm2TimerIntCount = 0;





    //
    // Enable ADCINT in PIE
    //
    IER |= M_INT1;      // Enable CPU Interrupt 1
    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT
    //
    IER |= M_INT3;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    //
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1; /// NOT USED
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // adc probably

    PieCtrlRegs.PIEIER3.bit.INTx2 = 1; // inverter PWM
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // PIE for  inverter control (MAIN CONTROL LOOP)


    PieCtrlRegs.PIEIER3.bit.INTx4 = 1;


    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;       // Enable Global interrupt INTM
    ERTM;       // Enable Global realtime interrupt DBGM

    qep_posspeed.init(&qep_posspeed);


    GpioDataRegs.GPASET.bit.GPIO11 = 1;         //Set pin 29 to on
    //
    // Step 6. IDLE loop. Just sit and loop forever (optional)
    //
    for(;;)
    {
       // __asm("          NOP");
    }
}




//COME BACK TO
__interrupt void
motor_control_isr(void)
{
    EPwm3Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}




__interrupt void
adc_isr(void){
    // sample analogue signals
    data_sampling(&high_speed_pipeline);

    //
    // Reinitialize for next ADC sequence
    //
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}





//
// high_speed_isr -
//





__interrupt void
high_speed_isr(void)
{
    EPwm2TimerIntCount++;
    // can be found in HighSpeed.c
    // highest speed - run every time
    pwm_update(&high_speed_pipeline, &inverter_duty_control); // internal filter
    encoder(); // unclear

    // only run the motor and chopper control at one tenth of the frequency of the sampling
    if (EPwm2TimerIntCount == 10) {

    // 1/10 of the speed of the high speed
    motor_control(&high_speed_pipeline, &inverter_duty_control);
    //
    chopper_control(&high_speed_pipeline);



    // reset counter
    EPwm2TimerIntCount = 0;
    }

    //qep_posspeed.calc(&qep_posspeed);

    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// prdTick - EPWM1 Interrupts once every 4 QCLK counts (one period)
//
__interrupt void
prdTick(void)
{

    //
    // Position and Speed measurement
    //
    qep_posspeed.calc(&qep_posspeed);

    //
    // Control loop code for position control & Speed control
    //

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT=1;
}



void
InitInverterEPWM()
{

    // Initialize the time relationships for the h bridge switching frequency
    Uint32 Val = INVERTER_PERIOD;
    Uint32 DutyValue = INVERTER_START_DUTY;

    Uint32 duty_cycle = (100 - DutyValue) *(double) Val/ 100 ;

    EPwm2Regs.TBPRD = INVERTER_PERIOD;                        // Set timer period
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;  // Slow just to observe on the scope

    //
    // Setup compare
    //
    EPwm2Regs.CMPA.half.CMPA = duty_cycle;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;           // Set PWM2A on Zero
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;


    //Deadband is set here
    EPwm2Regs.DBRED = MOTOR_DEADBAND;
    EPwm2Regs.DBFED = MOTOR_DEADBAND;



    // Interrupt where we will modify the deadband
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                 // Enable INT

    // ET_3RD SHOULD MAYBE BE CHAnged
    EPwm2Regs.ETPS.bit.INTPRD =  ET_3RD;           // Generate INT on 3rd event
}





void InitMotorControlEPWM(){
        // initialize the clock and frequency for the motor control interrupt
        EPwm3Regs.TBPRD = CONTROL_PERIOD;                        // Set timer period
        EPwm3Regs.TBPHS.half.TBPHS = 0x0000;                     // Phase is 0
        EPwm3Regs.TBCTR = 0x0000;                                // Clear counter

        //
        // Setup TBCLK
        //
        EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
        EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
        EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
        EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;  // Slow just to observe on the scope



        // Interrupt where we will modify the deadband
        //
        EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
        EPwm3Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
        EPwm3Regs.ETPS.bit.INTPRD =  ET_1ST;           // Generate INT on 1st event
}


void InitEncoderEPWM(){
    EPwm1Regs.TBSTS.all=0;
    EPwm1Regs.TBPHS.half.TBPHS =0;
    EPwm1Regs.TBCTR=0;

    EPwm1Regs.CMPCTL.all=0x50;     // immediate mode for CMPA and CMPB
    EPwm1Regs.CMPA.half.CMPA=SP/2;
    EPwm1Regs.CMPB=0;

    //
    // CTR=CMPA when inc->EPWM1A=1, when dec->EPWM1A=0
    //
    EPwm1Regs.AQCTLA.all=0x60;

    EPwm1Regs.AQCTLB.all=0x09;     // CTR=PRD ->EPWM1B=1, CTR=0 ->EPWM1B=0
    EPwm1Regs.AQSFRC.all=0;
    EPwm1Regs.AQCSFRC.all=0;

    EPwm1Regs.TZSEL.all=0;
    EPwm1Regs.TZCTL.all=0;
    EPwm1Regs.TZEINT.all=0;
    EPwm1Regs.TZFLG.all=0;
    EPwm1Regs.TZCLR.all=0;
    EPwm1Regs.TZFRC.all=0;

    EPwm1Regs.ETSEL.all=0x0A;      // Interrupt on PRD
    EPwm1Regs.ETPS.all=1;
    EPwm1Regs.ETFLG.all=0;
    EPwm1Regs.ETCLR.all=0;
    EPwm1Regs.ETFRC.all=0;

    EPwm1Regs.PCCTL.all=0;

    EPwm1Regs.TBCTL.all=0x0010+TBCTLVAL; // Enable Timer
    EPwm1Regs.TBPRD=SP;
}


void
InitEQepGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;    // Enable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;    // Enable pull-up on GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up on GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27 (EQEP2S)

    //
    // Inputs are synchronized to SYSCLKOUT by default.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;  // Sync to SYSCLKOUT GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;  // Sync to SYSCLKOUT GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;  // Sync to SYSCLKOUT GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  // Sync to SYSCLKOUT GPIO27 (EQEP2S)

    //
    // Configure eQEP-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be eQEP2 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO25 as EQEP2B
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO26 as EQEP2I
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;   // Configure GPIO27 as EQEP2S

    EDIS;
}


void BoardStartup(){
    //This function enables the en_gate pin on the board, allowing for switching, etc.

    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;     //Use Pin 11

    //GpioCtrlRegs.GPAMUX2.bit.GPIO11 = 00;   //set mux to GPIO



    EDIS;



}
//
// End of File
//

