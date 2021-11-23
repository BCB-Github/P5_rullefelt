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
#include <stdio.h>
#include <math.h>
#include <string.h>







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


#define LSPCLK_FREQ CPU_CLK/4
#define SCI_FREQ 115200
#define SCI_PRD (LSPCLK_FREQ/(SCI_FREQ*8))-1



#define ADC_TO_PWM_RATIO 2 // this is how many pulses are read per invter duty period



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

// TIMERS
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);



void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char* res, int afterpoint);


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

    char *msg;

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

    // TIMERS
    PieVectTable.XINT13 = &cpu_timer1_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;

    PieVectTable.ADCINT = &adc_isr;

    //Encoder interrupt
    PieVectTable.EPWM1_INT = &prdTick;

    EDIS;      // This is needed to disable write to EALLOW protected registers
    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2833x_InitPeripherals.c
    //
    // InitPeripherals();  // Not required for this example


    // MORE TIMER STUFF
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
    ConfigCpuTimer(&CpuTimer1, 150, 100); // milliseconds
    ConfigCpuTimer(&CpuTimer2, 150, 1000000); // seconds
    CpuTimer1Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    CpuTimer2Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0






    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;



    //Init everything here:

    InitInverterEPWM();
    InitMotorControlEPWM();

    InitEncoderEPWM();

    InitEQepGpio();

    InitSciaGpio();         //Init GPIO for communication

    fifo_init();      // Initialize the SCI FIFO
    scia_echoback_init();  // Initialize SCI for echoback

    BoardStartup();



    //
    // Specific clock setting for this example:
    //

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Here we synchronize pwm clocks
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
    EPwm4Regs.TBPRD = (Uint16) INVERTER_PERIOD;           // Set period for ePWM1
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

    // TIMER THINGS
    IER |= M_INT13;
    IER |= M_INT14;
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM



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




        char recieved = SciaRegs.SCIRXBUF.all;


        int res_length = 8;
        int msg_length = 2 * res_length + 1; // temperorary definition
        char send_msg[17];

        char comma = ',';
        char endline = 'n';
        char period = '.';
        char negative_sign = '-';
        char zero_char = '0';
        int negative_res2 = 0;


        char serial_output[100];

        char i_out_string[10];
        char v_out_string[10];
        char time_stamp_ms_string[10];
        char time_stamp_s_string[10];

        float v_out = high_speed_pipeline.V_DC_AVRG;
        float i_out = high_speed_pipeline.I_avg;

        int time_stamp_ms = CpuTimer1.InterruptCount;
        int time_stamp_s = CpuTimer2.InterruptCount;

        intToStr(time_stamp_ms,time_stamp_ms_string, 0);
        intToStr(time_stamp_s, time_stamp_s_string, 0);


        if (i_out < 0)
        {
            // well we can't get negative numbers.....
            i_out = (-1) * i_out;
            ftoa(i_out, i_out_string, 4);
            negative_res2 = 1; // boolean for negative numbers
        }

        ftoa(v_out, v_out_string, 4);
        ftoa(i_out, i_out_string, 4);


        int zeros_before_ms = 0;
        // start with putting in the seconds
        strcpy(serial_output, time_stamp_s_string);
        strncat(serial_output, &period, 1);
        for (zeros_before_ms = 0; zeros_before_ms <4 - strlen(time_stamp_ms_string); zeros_before_ms ++ ) {
            strncat(serial_output, &zero_char, 1);

        }
        strcat(serial_output, time_stamp_ms_string);
        strncat(serial_output, &comma, 1);
         strcat (serial_output,v_out_string);
         strncat(serial_output, &comma, 1);
         if (negative_res2 == 1){
             strncat(serial_output, &negative_sign, 1); // negative check
             negative_res2 = 0;
         }
         strcat (serial_output,i_out_string);
         strncat(serial_output, &comma, 1);
         strncat(serial_output, &endline, 1);

        scia_msg(serial_output);
        int clearer;
        // This is just to clear the array
        for (clearer = 0; clearer < 50; clearer++)
        {
            serial_output[clearer] = '\n';
        }







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

void
fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;
}



//
// scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F,
// default, 1 STOP bit, no parity
//
void
scia_echoback_init()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    // 1 stop bit,  No loopback, No parity,8 char bits,
    // async mode, idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;
    SciaRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =0;
    SciaRegs.SCICTL2.bit.RXBKINTENA =0;
#if (CPU_FRQ_150MHZ)
    SciaRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 37.5MHz.
    SciaRegs.SCILBAUD    =SCI_PRD;
#endif
#if (CPU_FRQ_100MHZ)
    SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
    SciaRegs.SCILBAUD    =0x0044;
#endif
    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

//
// scia_xmit - Transmit a character from the SCI
//
void
scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    SciaRegs.SCITXBUF=a;
}

//'1'
// scia_msg -
//
void
scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}



//
// End of File
//







void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}



// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        //fpart = fpart * pow(10, afterpoint);
        fpart = fpart * 10000;
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}



// TIMERS
//
// cpu_timer1_isr -
//
__interrupt void
cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
    if (CpuTimer1.InterruptCount == 10000){
        CpuTimer1.InterruptCount = 0;
    }

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

//
// cpu_timer2_isr -
//
__interrupt void
cpu_timer2_isr(void)
{
    EALLOW;
    CpuTimer2.InterruptCount++;

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

