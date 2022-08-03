/* ****************************************************************************************************** *
 * main.c                                                                                                 *
 *                                                                                                        *
 *  Created on: Jun 25, 2018                                                                              *
 *      Author: a0233200                                                                                  *
 *                                                                                                        *
 * Copyright (c) 2018, Texas Instruments Incorporated                                                     *
 * All rights reserved.                                                                                   *
 *                                                                                                        *
 * Redistribution and use in source and binary forms, with or without                                     *
 * modification, are permitted provided that the following conditions                                     *
 * are met:                                                                                               *
 *                                                                                                        *
 * *  Redistributions of source code must retain the above copyright                                      *
 *    notice, this list of conditions and the following disclaimer.                                       *
 *                                                                                                        *
 * *  Redistributions in binary form must reproduce the above copyright                                   *
 *    notice, this list of conditions and the following disclaimer in the                                 *
 *    documentation and/or other materials provided with the distribution.                                *
 *                                                                                                        *
 * *  Neither the name of Texas Instruments Incorporated nor the names of                                 *
 *    its contributors may be used to endorse or promote products derived                                 *
 *    from this software without specific prior written permission.                                       *
 *                                                                                                        *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"                            *
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,                                  *
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR                                 *
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR                                       *
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,                                  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,                                    *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;                            *
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,                               *
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR                                *
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,                                         *
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                     *
 *                                                                                                        *
 * -------------------------- PIN MAP --------------------------                                          *
 *                      __________________________                                                        *
 *                     |                          |                                                       *
 *               AVCC -| 1               (P3.6) 38|-                                                      *
 *   Battery Status 2 -| 2 (PJ.4)        (P3.5) 37|- Buzzer                                               *
 *   Battery Status 3 -| 3 (PJ.5)               36|- RESET                                                *
 *               AVSS -| 4                      35|- TEST                                                 *
 *      Panel_V_Sense -| 5 (P1.0)        (P3.3) 34|- Panel_Enable                                         *
 *    Battery_V_Sense -| 6 (P1.1)        (P3.2) 33|- Load_Enable                                          *
 *      Panel_I_Sense -| 7 (P1.2)        (PJ.6) 32|-                                                      *
 *    Battery_I_Sense -| 8 (P1.3)               31|- DVCC                                                 *
 *      OPT3001 (SDA) -| 9 (P1.4)               30|- DVSS                                                 *
 *      OPT3001 (SCL) -|10 (P1.5) MSP430        29|-                                                      *
 *       Load_I_Sense -|11 (PJ.0)  F5132 (P3.1) 28|-                                                      *
 *                    -|12 (PJ.1)        (P3.0) 27|-                                                      *
 *                    -|13 (PJ.2)        (P2.7) 26|- LED 4                                                *
 *   Battery Status 1 -|14 (PJ.3)        (P2.6) 25|- LED 3                                                *
 *      OPT3001 (INT) -|15 (P1.6)        (P2.5) 24|- LED 2/RX                                             *
 *  Buck (Low-Side 1) -|16 (P1.7)        (P2.4) 23|- LED 1/TX                                             *
 * Buck (High-Side 1) -|17 (P2.0)               22|- DVSS                                                 *
 * Temperature Sensor -|18 (P2.1)               21|-                                                      *
 *  Buck (Low-Side 2) -|19 (P2.2)        (P2.3) 20|- Buck (High-Side 2)                                   *
 *                     |__________________________|                                                       *
 *                                                                                                        *
 * ****************************************************************************************************** */
//-----------------------------------------------------------------------------------------//
//-------------------------------------- Notes/Revisions ----------------------------------//
//-----------------------------------------------------------------------------------------//
// 1. Diode Emulation mode added to prevent boosting of panel side voltage as a protection feature
// 2. TimerA interrupt added. Refer the sections on TimerA_ISR
// 3.


#include "msp430f5132.h"
#include "hal_tlv.h"
#include<string.h>
#include<stdio.h>
#define CALTDH0CTL1_256     *((unsigned int *)0x1A36)

//-----------------------------------------------------------------------------------------//
//-------------------------------------- Definitions --------------------------------------//
//-----------------------------------------------------------------------------------------//

//-------------------------------User Settings---------------------------------------------//
//#define USE_FLOAT   // Uncomment to use floating-point calculations

// MAKE SURE ONLY ONE OF THE FOLLOWING IS UNCOMMENTED
#define SYS12       // Uncomment if using 12-V battery system
//#define SYS24       // Uncomment if using 24-V battery system
//#define SYS48       // Uncomment if using 48-V battery system
//-----------------------------------------------------------------------------------------//

#define BUCK_DISABLE    TD0CCTL1 = TD0CCTL2 = OUTMOD_0;\
                        TD1CCTL1 = TD1CCTL2 = OUTMOD_0;     // Turn PWM outputs off
#define BUCK_ENABLE     TD0CCTL1 = TD1CCTL1 = OUTMOD_2;\
                        TD0CCTL2 = TD1CCTL2 = OUTMOD_6;     // TDx.1 OUTPUT MODE = TOGGLE/RESET, TDx.2 OUTPUT MODE = TOGGLE/SET in Buck mode

#define CUTOFF_COUNTER_THRESHOLD        10  // 0.05 second delay
#define RECONNECT_COUNTER_THRESHOLD     10  // 0.05 second delay
#define OC_TRIGGERED_COUNTER_THRESHOLD  100 // 0.5 second delay

#define DEADBAND    20 // Difference between the two PWM phases in order to implement a deadband so that no H/L-side pair are turned on at the same time

#define BUCK_LOWER_THRESHOLD    400
#define BUCK_UPPER_THRESHOLD    680

#define P_V     3       // Setting indices for ADC_Readings vector
#define B_V     2
#define P_I     1
#define B_I     0

#define REF_AVG_MPPT_COUNTER    8 // MPPT control done with 8 sample average
#define REF_INST_PROT_COUNTER 2 //Protection running with 2 sample average

#ifdef USE_FLOAT
#define PANEL_UPPER_LIMIT       60.0    // Maximum panel voltage of 60V
#define MIN_BATTERY_CURRENT     0.0     // Minimum battery current of 0.5A (change as needed for battery)
#else
#define PANEL_UPPER_LIMIT       830     // Maximum panel voltage of 60V
#define MIN_BATTERY_CURRENT     3       // Minimum battery current of 0.5A (change as needed for battery)
#endif

#define PANEL_ENABLE    P3OUT |= BIT3   // Set to enable panel *** P3.3
#define PANEL_DISABLE   P3OUT &= ~BIT3  // Reset to disable panel
#define LOAD_ENABLE     P3OUT |= BIT2   // Set to enable load *** P3.2
#define LOAD_DISABLE    P3OUT &= ~BIT2  // Reset to disable load

#define LOOP_EXIT_LIMIT     10         // 1 second delay

#define DUTY_START          520

#define SLAVE_ADDR          0x44        // set Slave Address for Ambient Light Sensor to 44h (ADDR pin is connected to GND)
//#define SLAVE_ADDR          0x45        // set Slave Address for Ambient Light Sensor to 45h (ADDR pin is connected to VDD)
//#define SLAVE_ADDR          0x46        // set Slave Address for Ambient Light Sensor to 46h (ADDR pin is connected to SDA)
//#define SLAVE_ADDR          0x47        // set Slave Address for Ambient Light Sensor to 47h (ADDR pin is connected to SCL)

//-----------------------------------------------------------------------------------------//
//--------------------------------- Variable Declarations ---------------------------------//
//-----------------------------------------------------------------------------------------//
// Variables for user settings
uint16_t    Load_I_Limit = 0x01;        // Defining load overcurrent limit: 0x00 = 5A, 0x01 = 10A, 0x02 = 15A, 0x03 = 20A

// Variables for flags
uint16_t    System_reset_mode_ON, Load_monitor_mode_ON,     // Flags for load current comparator ISR
            OC_Triggered = 0, OC_Triggered_Counter = 0,     // Flags for load overcurrent protection
            CV_Mode = 0,
            Read_Done,
            Wait_State = 0,
            Start_Delay = 0,
            Is_Load_On = 0,
            MPPT_Loop = 1,
            Phase_Shifting_Done = 0,
            Hysteresis_ON = 0;

// Variables for counters
uint16_t    Wait_Counter = 0,
            Battery_Current_Counter = 0,
            Cutoff_Counter = 0,
            Reconnect_Counter = 0,
            MPP_Loop_Exit_Counter = 0,
            CC_Loop_Exit_Counter = 0,
            maxFlips =0, // Keep track for semi Global Search
            protection_count =0; // Counter to add reliability for protection trip

uint32_t    Panel_Voltage_inst = 0,
            Battery_Voltage_inst = 0,
            Panel_Current_inst = 0,
            Battery_Current_inst = 0;

// Variables for ADC
uint16_t    ADC_Readings[4],
            Avg_MPPT_Counter = 0,
            inst_prot_Counter=0;

//This buffer stores MPPT control variables
uint32_t    Panel_Voltage_Buffer = 0,
            Battery_Voltage_Buffer = 0,
            Panel_Current_Buffer = 0,
            Battery_Current_Buffer = 0;

//This buffer stores protection releated variables
uint32_t    Panel_Voltage_inst_Buffer = 0,
            Battery_Voltage_inst_Buffer = 0,
            Panel_Current_inst_Buffer = 0,
            Battery_Current_inst_Buffer = 0;

unsigned int Mode_sel = 0; //selects between Diode Emulation mode & normal synchronous operation
unsigned int boot_count = 0;
unsigned int boot_flag = 0;

// Variables for calculations
#ifdef USE_FLOAT
float       Panel_Voltage = 0.0,
            Battery_Voltage = 0.0,
            Panel_Current = 0.0,
            Battery_Current = 0.0,
            Prev_Battery_Current = 0.0,
            Panel_Power = 0.0,
            Prev_Panel_Power = 0.0;
#else
uint16_t    Panel_Voltage = 0,
            Battery_Voltage = 0,
            Panel_Current = 0,
            Battery_Current = 0,
            Prev_Battery_Current = 0,
            Panel_Power = 0,
            Prev_Panel_Power = 0;

#endif

// Variables for MPPT
int16_t     MPPT_Direction = 1;
uint16_t    Duty = DUTY_START;
uint16_t    REF_Battery_Charge_Current_State;

/**********************Needs to be changed with different Batteries*******************************************/
#ifdef USE_FLOAT
const float CC_LIMIT           =   20.0;        // Set the maximum charging current

#ifdef SYS12
// Typical battery threshold values         	// System Voltage   | 12V		| 24V		| 48V		||
const float CC_TO_CV_LIMIT     =   12;        //                  | 14.2V		| 28.4V		| 56.8V		||
const float FLOAT_VOLTAGE      =   13.8;        //                  | 13.8V		| 27.6V		| 55.2V		||
const float BATTERY_CUTOFF     =   10.2;        //                  | 10.2V		| 20.4V		| 40.8V		||
const float BATTERY_RECONNECT  =   11.2;        //                  | 11.2V		| 22.4V		| 44.8V		||
#endif

#ifdef SYS24
const float CC_TO_CV_LIMIT     =   24;
const float FLOAT_VOLTAGE      =   18;
const float BATTERY_CUTOFF     =   20.4;
const float BATTERY_RECONNECT  =   22.4;
#endif

#ifdef SYS48
const float CC_TO_CV_LIMIT     =   48;
const float FLOAT_VOLTAGE      =   37;
const float BATTERY_CUTOFF     =   40.8;
const float BATTERY_RECONNECT  =   44.8;
#endif

#else
                                                    // System Current   | 10A   | 15A   | 20A   ||
const uint16_t CC_LIMIT           =   305;          //                  | 305   | 457   | 610   ||

#ifdef SYS12
// Typical battery threshold values             	// System Voltage   | 12V           | 24V           | 48V           ||
const uint16_t CC_TO_CV_LIMIT     =   201;          //       14.4           | 219 (14.2V)   | 436 (28.4V)   |     (56.8V)   ||
const uint16_t FLOAT_VOLTAGE      =   213;          //                  | 213 (13.8V)   | 424 (27.6V)   |     (55.2V)   ||
const uint16_t BATTERY_CUTOFF     =   158;          //                  | 158 (10.2V)   | 314 (20.4V)   | 627 (40.8V)   ||
const uint16_t BATTERY_RECONNECT  =   173;          //                  | 173 (11.2V)   | 345 (22.4V)   | 688 (44.8V)   ||
#endif

#ifdef SYS24
const uint16_t CC_TO_CV_LIMIT     =   436;
const uint16_t FLOAT_VOLTAGE      =   424;
const uint16_t BATTERY_CUTOFF     =   314;
const uint16_t BATTERY_RECONNECT  =   345;
#endif

#ifdef SYS48
const uint16_t CC_TO_CV_LIMIT     =   870;  // Needs to be checked
const uint16_t FLOAT_VOLTAGE      =   846;  // Needs to be checked
const uint16_t BATTERY_CUTOFF     =   627;
const uint16_t BATTERY_RECONNECT  =   688;
#endif
#endif
/*************************************************************************************************************/

//-----------------------------------------------------------------------------------------//
//---------------------------------- Function Prototypes ----------------------------------//
//-----------------------------------------------------------------------------------------//
void Init_IO(void);
void Init_ADC(void);
void Init_WDT(void);
void Init_Comparator(void);
void Init_Timer(void);
void Init_I2C(void);
void Init_Clocks(void);
void SetVcoreUp(uint16_t level);
void MPPT(void);
void Battery_Charge_Profiling(void);
void Load_Management(void);
void Average_MPPT_ADC_Values();
void Mode_Select(int Mode_sel);
void init_TimerA(void);
void Inst_protection_Values(void);



//-----------------------------------------------------------------------------------------//
//--------------------------------------- Functions ---------------------------------------//
//-----------------------------------------------------------------------------------------//
void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;       // Stop WDT
    __bic_SR_register(GIE);
    __delay_cycles(2000000);        // Allow system to settle before initializations
    Init_Clocks();                  // Initialize clocks for 25MHz

    PANEL_DISABLE;
    LOAD_DISABLE;

    Init_Comparator();              // Initialize comparator for load overcurrent protection
    Init_Timer();                   // Initialize timer for buck converter gate signals

    __bis_SR_register(GIE);
    while(Phase_Shifting_Done);     // Wait until the phase shifting of the interleaved buck is done

    BUCK_DISABLE;

    Init_IO();                      // Set MCU input/output pins

    _delay_cycles (500);

    __bic_SR_register(GIE);
    //Init_I2C();                     // Initialize I2C for ambient light sensor
    Init_ADC();                     // Initialize ADC for panel and battery voltage and current monitoring
    Init_WDT();                     // Initialize watchdog timer for timing of ADC captures
    init_TimerA();                  // Initializing TimerA
    __bis_SR_register(GIE);         // Enable global interrupts


    while(1)
    {
        if(Read_Done)
        {
/*
// Protection with instantaneous values of variables. 2 Sample averaging is implemented. This is currently commented as it can cause momentary tripping in MPPT operation

            if (inst_prot_Counter == REF_INST_PROT_COUNTER){
                Inst_protection_Values();
                if (Panel_Voltage_inst > PANEL_UPPER_LIMIT || Battery_Voltage_inst > CC_TO_CV_LIMIT + 20){
                    protection_count++; //This variable is just added to deal with momentary spikes which might trigger shutdown
                    if (proection_count > 3){
                        PANEL_DISABLE;
                        BUCK_DISABLE;
                        protection_count = 0;
                    }

                }
            }

*/
            if(Avg_MPPT_Counter == REF_AVG_MPPT_COUNTER)
            {
                Average_MPPT_ADC_Values();

                REF_Battery_Charge_Current_State = 1;

                /* Battery charging only occurs when:
                 * 1. The panel voltage is greater than the battery voltage
                 * 2. The panel voltage is less than the predefined maximum threshold
                 */
                if((Panel_Voltage > (Battery_Voltage-30)) && (Panel_Voltage < PANEL_UPPER_LIMIT) && (!Wait_State) && (REF_Battery_Charge_Current_State))
                {       // Since we meet all specifications for battery charging, commence creating PWM signals for gate drivers
                    BUCK_ENABLE;
                    PANEL_ENABLE;

                    //MPPT_Loop=0;
                    if(MPPT_Loop == 1)
                        MPPT();                     // Calculate the MPP if the battery current and voltage are below maximum ratings
                    else
                        Battery_Charge_Profiling(); // Otherwise, maintain the battery current and voltage in a Float stage

                    TD0CCTL0 &= ~CCIFG;             // Wait until the timer completes its current cycle
                    while(!(TD0CCTL0 & CCIFG));

                    TD0CCR1 = TD1CCR1 = Duty;       // Set the duty cycle for the buck
                    TD0CCR2 = TD1CCR2 = Duty - DEADBAND;



                } // end if

                // Since the battery current is falling under the minimum charging current, we shut off the panel and buck controller
                if(((Battery_Current < MIN_BATTERY_CURRENT) && (!Wait_State)))
                {
                    Battery_Current_Counter++;      // Count how many times this statement is entered
                    if((Battery_Current_Counter > 10) || (Panel_Voltage > PANEL_UPPER_LIMIT))
                    {
                        // Once current has fallen for some time, disable panel and buck and go into a waiting state
                        PANEL_DISABLE;
                        BUCK_DISABLE;
                        Duty = BUCK_LOWER_THRESHOLD;
                        Wait_State = 1;
                        Wait_Counter = 0;
                    } // end if
                } // end if

                // Go to diode emulation mode(DEM) / Synchronous buck mode
                // If boot_flag is set in DEM go to synchronous buck mode for 1ms to charge boot strap capacitor
                else if((Battery_Current >= MIN_BATTERY_CURRENT) && (Battery_Current < 10))
                    {
                    if (boot_flag == 1){
                        BUCK_ENABLE;
                        Mode_Select(1);
                        Wait_State = 1;
                        Wait_Counter = 0;
                    }
                    else{
                        BUCK_ENABLE;
                        Mode_Select(0);
                        Wait_State = 1;
                        Wait_Counter = 0;
                    }
                    }
                // Sync buck mode
                else if((Battery_Current >= 10)){
                    BUCK_ENABLE;
                    Mode_Select(1);
                    Wait_State = 1;
                    Wait_Counter = 0;
                }

            else
                Battery_Current_Counter = 0; // end if-else




                if(Wait_State)
                {
                    // Wait in this statement until sufficient power is running through the system, checking every ~5 seconds
                    Wait_Counter++;
                    if(Wait_Counter > 50)
                    {
                        Wait_State = 0;
                        Battery_Current_Counter = 0;
                    } // end if
                } // end if
                Read_Done = 0;
            } // end if

        //    Load_Management();

        } // end if
    } // end while


} // end main

/* *************************************************************************************** *
 * Call this function to initialize all pins and ports for the various system signals      *
 * *************************************************************************************** */
void Init_IO(void)
{
    P1SEL |= BIT7;                  // P1.7/TD0.1 selected
    P1DIR |= BIT7;                  // P1.7 set as output for Timer D0.1
    P2SEL |= BIT0 + BIT2 + BIT3;    // P2.0/TD0.2, P2.2/TD1.1, and P2.3/TD1.2 selected
    P2DIR |= BIT0 + BIT2 + BIT3;    // P2.0, P2.2, P2.3 set as output for Timer D0.2, 1.1, 1.2

    PMAPPWD = 0x02D52;              // Enable write-access to modify port mapping registers
    PMAPCTL = PMAPRECFG;            // Allow reconfiguration during runtime
    P1MAP0 |= PM_ANALOG;            // Modify all PxMAPy registers
    P1MAP1 |= PM_ANALOG;            // Modify all PxMAPy registers
    P1MAP2 |= PM_ANALOG;            // Modify all PxMAPy registers
    P1MAP3 |= PM_ANALOG;            // Modify all PxMAPy registers
    P1MAP4 |= PM_UCB0SCL;           // Modify all PxMAPy registers
    P1MAP5 |= PM_UCB0SDA;           // Modify all PxMAPy registers
    PMAPPWD = 0;                    // Disable write-access to modify port mapping registers
    P1SEL |= BIT0 + BIT1 + BIT2 + BIT3; // P1.0, P1.1, P1.2, P1.3 set for panel and battery current and voltage sensing
    P1SEL |= BIT4 + BIT5;           // P1.4, P1.5 set to I2C for OPT3001

    P3DIR |= BIT2 + BIT3;           // P3.2, P3.3 set as output for Panel, Load enable
    PJDIR |= BIT3 + BIT4 + BIT5;    // PJ.3, PJ.4, PJ.5 set as output for Battery Status Indicators
    P2DIR &= ~BIT1;                 // P2.1 set as input for Temperature Sensor
    P2DIR |= BIT6 + BIT7;           // P2.6, P2.7 set as output for LED Indicators
    P3DIR |= BIT5;                  // P3.5 set as output for Buzzer
} // end Init_IO

/* *************************************************************************************** *
 * Call this function to initialize ADC pins for current and voltage sense of the          *
 *  solar panel (P_I, P_V) and battery (B_I, B_V)                                          *
 *  P1.0 (A0), P1.1 (A1), P1.2 (A2), and P1.3 (A3) are used for these signals              *
 * *************************************************************************************** */
void Init_ADC(void)
{
    ADC10CTL0 = ADC10SHT_2 + ADC10MSC + ADC10ON;    // 8 clock cycles, single trigger, conversion disabled
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_1;           // Sampling timer, sequence of channels
    ADC10CTL2 |= ADC10RES;                          // 10-bit conversion results
    ADC10MCTL0 = ADC10INCH_3;                       // A0, A1, A2, A3 (EoS), Vref+ = AVcc, Vref- = gnd

    DMACTL0 = DMA0TSEL_24;                          // ADC10IFG trigger
    __data16_write_addr((unsigned short) &DMA0SA, (unsigned long) &ADC10MEM0);          // Setting source address
    __data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &ADC_Readings[0]);    // Setting destination array address
    DMA0SZ = 0x04;                                  // 5 words (conversion results) transferred
    DMA0CTL = DMADT_4 + DMADSTINCR_3 + DMAEN + DMAIE;   // Source unchanged, destination increments, enable DMA, enable DMA interrupts
} // end Init_ADC

/* *************************************************************************************** *
 * Call this function to initialize the Watchdog Timer, used in gathering of analog data   *
 *  The Watchdog timer is set to trigger an interrupt every 1.3ms                          *
 * *************************************************************************************** */
void Init_WDT(void)
{
    WDTCTL = WDT_MDLY_32;   // WDT set to 32ms for SMCLK = 1MHz; since we are using SMCLK = 25MHz, timer interval is 1.3ms
    SFRIE1 |= WDTIE;        // Enable WDT interrupt
} // end Init_WDT

/* *************************************************************************************** *
 * Call this function to initialize the comparator for load current (L_I) monitoring       *
 *  Since over-discharge of the battery can shorten the expected battery life and lead to  *
 *  severe damage, the load needs to be disabled if overcurrent occurs                     *
 * *************************************************************************************** */
void Init_Comparator(void)
{
    CBCTL0 |= CBIPEN + CBIPSEL_6;   // Enable V+, input channel CB6
    CBCTL1 |= CBPWRMD_1;            // Normal power mode
    switch(Load_I_Limit & 0x03)     // Load current limit = .103125 * (CBREF + 1) / (Amp_Gain * Shunt_Resistance)
    {
        case 0x00:      // 5A -- actual 4.13A
            CBCTL2 = CBRS_1 + CBRSEL + CBREF1_3 + CBREF0_3;
            break;
        case 0x01:      // 10A -- actual 9.28A
            CBCTL2 = CBRS_1 + CBRSEL + CBREF1_8 + CBREF0_8;
            break;
        case 0x02:      // 15A -- actual 14.44A
            CBCTL2 = CBRS_1 + CBRSEL + CBREF1_13 + CBREF0_13;
            break;
        case 0x03:      // 20A -- actual 19.59A
            CBCTL2 = CBRS_1 + CBRSEL + CBREF1_18 + CBREF0_18;
            break;
        default: break;
    } // end switch
    CBCTL3 |= BIT6;             // Input buffer disable at PJ.0/CB6
    __delay_cycles(7500);       // Delay for the reference to settle
    CBINT &= ~(CBIFG + CBIIFG); // Clear any errant interrupts
    CBINT |= CBIE;              // Enable comparator B interrupt on rising edge of CBIFG
    CBCTL1 |= CBON;             // Enable comparator B
} // end Init_Comparator

/* *************************************************************************************** *
 * Call this function to initialize Timer D for PWM generation                             *
 *  PWM signals are used to drive the interleaved buck gates to control the battery        *
 *  charging current                                                                       *
 * *************************************************************************************** */
void Init_Timer(void)
{
    struct s_TLV_Timer_D_Cal_Data * pTD0CAL;    // Structure initialized in tlv.h
    uint8_t bTD0CAL_bytes;
    // Configure TimerD in Hi-Res Free Running mode
    Get_TLV_Info(TLV_TIMER_D_CAL, 0, &bTD0CAL_bytes, (uint16_t **) &pTD0CAL);   // Get TimerD0 cal values (instance 0)
    if(bTD0CAL_bytes == 0x0)
    {
        while(1);                               // loop here since no TimerD free running cal data found
    } // end if
    TD0CTL0 = TDSSEL_2;                         // TDCLK = SMCLK = 25MHz = Hi-Res input clk select

    TD0HCTL1 = pTD0CAL->TDH0CTL1_128;         // Read the 256Mhz TimerD TLV Data

    TD0CTL1 |= TDCLKM_1;                        // Select Hi-res local clock
    TD0HCTL0 = TDHEN + TDHM_0;                  // CALEN = 0 => free running mode; enable Hi-res mode
                                                // TDHM_0 => clk = 8 * TDCLK = 200MHz, TDHM_1 => clk = 16 * TDCLK = 400MHz

    TD0CCR0 = 700;                            // 128M/700 = 182.8Khz is the operating frequency of the buck stage

    TD0CCTL0 |= CCIE;                         // interrupt is enabled to adjust phase
    TD0CCR1 = Duty;
    TD0CCR2 = Duty - DEADBAND;                // Pprovide Dead Band
    TD0CCTL1 |= OUTMOD_2;                     // TD0CCR1, Reset/Set
    TD0CCTL2 |= OUTMOD_6;                     // TD0CCR2, Set/reset

    TD1CCR0 = 700;                            //
    TD1CCR1 = Duty;
    TD1CCR2 = Duty - DEADBAND;
    TD1CCTL1 |= OUTMOD_2;                     // TD1CCR1, Reset/Set
    TD1CCTL2 |= OUTMOD_6;                     // TD1CCR2, Set/reset
    TD1CTL1 = TDCLKM_2;                       // TD1 clock = Auxiliary clock source from master timer instance

    // Syncronize master (TD0) and slave (TD1) timer instances
    TEC1XCTL2 |= TECAXCLREN;            // Enable synchronized clear by enabling Aux clear of slave timer

    TD0CTL0 |= MC_1 + TDCLR;                  // up-mode, clear TDR, Start timer

} // end Init_Timer

/* *************************************************************************************** *
 * Call this function to initialize the I2C bus to communicate with the OPT3001            *
 * *************************************************************************************** */
void Init_I2C(void)
{
    UCB0CTL1 |= UCSWRST;                    // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;          // Use SMCLK, keep SW reset
    UCB0BR0 = 250;                          // fSCL = SMCLK/250 = 100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = SLAVE_ADDR;                 // Slave Address is 044h
    UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation
    UCB0IE |= UCTXIE;                       // Enable interrupt on a receive
} // end Init_I2C

/* *************************************************************************************** *
 * Call this function to initialize the main system clock                                  *
 * *************************************************************************************** */
void Init_Clocks(void)
{
    SetVcoreUp(0x01);
    SetVcoreUp(0x02);
    SetVcoreUp(0x03);

    // Configure DCO = 25Mhz
    UCSCTL3 = SELREF_2;                         // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                          // Set ACLK = REFO
    __bis_SR_register(SCG0);                    // Disable the FLL control loop
    UCSCTL0 = 0x0000;                           // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7;                        // Select DCO range 50MHz operation
    UCSCTL2 = FLLD_1 + 762;                     // Set DCO Multiplier for 25MHz
                                                // (N + 1) * FLLRef = Fdco
                                                // (762 + 1) * 32768 = 25MHz
                                                // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                    // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
    __delay_cycles(782000);

    // Loop until Xt1 & DCO stabilizes - In this case only DCO has to stabilize
    do
    {
        UCSCTL7 &= ~(XT1LFOFFG + XT1HFOFFG + DCOFFG);   // Clear XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                              // Clear fault flags
    }while (SFRIFG1&OFIFG);                             // Test oscillator fault flag
} // end Init_Clocks

void SetVcoreUp(uint16_t level)
{
    PMMCTL0_H = PMMPW_H;                            // Open PMM registers for write
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;    // Set SVS/SVM high side new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;   // Set SVM low side to new level
    while ((PMMIFG & SVSMLDLYIFG) == 0);            // Wait till SVM is settled
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);              // Clear already set flags
    PMMCTL0_L = PMMCOREV0 * level;                  // Set VCore to new level
    if ((PMMIFG & SVMLIFG))                         // Wait till new level reached
    {
        while ((PMMIFG & SVMLVLRIFG) == 0);         // Set SVS/SVM low side to new level
    } // end if
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    PMMCTL0_H = 0x00;                               // Lock PMM registers for write access
} // end SetVcoreUp

/* *************************************************************************************** *
 * Call this function to calculate the maximum power point                                 *
 *  The TIDA-010042 uses the perturb-and-observe method of MPPT                            *
 *  The panel power is calculated each time this function is called and compared to the    *
 *  previous power value, then the power point and PWM duty are adjusted accordingly       *
 *  until the maximum value is found                                                       *
 * *************************************************************************************** */
void MPPT(void)
{
    static uint8_t dutyChange=2;
    Prev_Panel_Power = Panel_Power;                     // Store the previous panel power
    Panel_Power = Panel_Voltage * Panel_Current;        // Calculate current panel power

//    if((Battery_Current < Prev_Battery_Current) || (Panel_Power < Prev_Panel_Power))
    if(Panel_Power < Prev_Panel_Power)
    {
        MPPT_Direction = MPPT_Direction * -1;
        //Check for steady state
        maxFlips+=1;

        if(maxFlips==50)
        {
            dutyChange =1;
        }

    } // end if


    if(MPPT_Direction == 1)
    {

        Duty+=dutyChange;

        if(Duty > BUCK_UPPER_THRESHOLD)
        {
            Duty = BUCK_UPPER_THRESHOLD;
        } // end if
    }
    else
    {

        Duty-=dutyChange;

        if(Duty < BUCK_LOWER_THRESHOLD)
        {
            Duty = BUCK_LOWER_THRESHOLD;
        } // end if
    } // end if-else

    if((Battery_Current >= CC_LIMIT) || (Battery_Voltage >= CC_TO_CV_LIMIT))
    {
        MPP_Loop_Exit_Counter++;
        if(MPP_Loop_Exit_Counter > LOOP_EXIT_LIMIT)
        {
            MPPT_Loop = 0;
            MPP_Loop_Exit_Counter = 0;
            CC_Loop_Exit_Counter = 0;
            if (Battery_Voltage > CC_TO_CV_LIMIT)
                CV_Mode = 1;
            else
                CV_Mode = 0;
        } // end if
    } // end if
    else
        MPP_Loop_Exit_Counter = 0;

} // end MPPT




/* *************************************************************************************** *
 * Call this function to hold the battery in a FLOAT state when the battery has reached    *
 *  its float voltage                                                                      *
 * *************************************************************************************** */
void Battery_Charge_Profiling(void)
{

 static uint8_t delay =0;
 static uint8_t exitCV =0;
 delay+=1;

#if 1
switch (CV_Mode)
{
case 0: //cc mode

    if(Battery_Current>=(CC_LIMIT-5) && Battery_Current<CC_LIMIT+2 ) //dont exit loop
    {
        //
        Duty+=MPPT_Direction;
    }
    else if(Battery_Current>CC_LIMIT+3)
    {
        Duty-=MPPT_Direction;
    }
    else if(Battery_Current<CC_LIMIT-5)
    {
        MPPT_Loop=1; //pump more power
    }

    if(Battery_Voltage>=CC_TO_CV_LIMIT)
    {
        CV_Mode =1;
    }

break;

case 2: //handle soft CV


    break;

case 1: //Full Chargerd
    if(Battery_Voltage>=(CC_TO_CV_LIMIT-5) && Battery_Voltage<CC_TO_CV_LIMIT+2 ) //dont exit loop
      {
          //
          Duty+=MPPT_Direction;
      }
      else if(Battery_Voltage>CC_TO_CV_LIMIT+3)
      {
          Duty-=MPPT_Direction;
      }
      else if(Battery_Voltage<CC_TO_CV_LIMIT-5 && Battery_Current<CC_LIMIT-5)
      {

          MPPT_Loop=1; //pump more power
      }

      if(Battery_Current>=CC_LIMIT+1)
      {
          exitCV+=1;
          if(exitCV==10)
          {
              CV_Mode =0;
          }

      }
      else
      {
          exitCV=0;
      }
    break;
default:
    break;
}

//Sanity
if (Duty > BUCK_UPPER_THRESHOLD)
           {
               Duty = BUCK_UPPER_THRESHOLD;
           }

if (Duty < BUCK_LOWER_THRESHOLD)
            {
                Duty = BUCK_LOWER_THRESHOLD;
            }


#endif
} // end Battery_Charge_Profiling

/* *************************************************************************************** *
 * Call this function to disable the load if the battery voltage is too low                *
 *  (below BATTERY_CUTOFF) to prevent over-discharging of the battery                      *
 *  The battery is reconnected once sufficient charge (above BATTERY_RECONNECT) is reached *
 *  If an load overcurrent is triggered with Comparator B, then the load is disabled until *
 *  the OC_Triggered_Count has reached the OC_TRIGGERED_COUNTER_THRESHOLD                  *
 * *************************************************************************************** */
void Load_Management(void)
{
    if((!Hysteresis_ON) && (Battery_Voltage < BATTERY_CUTOFF) && (!OC_Triggered))
    {
        Cutoff_Counter++;
        if (Cutoff_Counter > CUTOFF_COUNTER_THRESHOLD)
        {
            LOAD_DISABLE;
            Hysteresis_ON = 1;
            Reconnect_Counter = 0;
        } // end if
    }
    else
        Cutoff_Counter = 0; // end if-else

    if((Hysteresis_ON) && (Battery_Voltage > BATTERY_RECONNECT) && (!OC_Triggered))
    {
        Reconnect_Counter++;
        if (Reconnect_Counter > RECONNECT_COUNTER_THRESHOLD)
        {
            LOAD_ENABLE;
            Hysteresis_ON = 0;
            Cutoff_Counter = 0;
        } // end if
    }
    else
        Reconnect_Counter = 0; // end if-else

    if ((OC_Triggered) && (!Hysteresis_ON)) // OC triggered at 8.40 A
    {
        OC_Triggered_Counter++;
        if (OC_Triggered_Counter > OC_TRIGGERED_COUNTER_THRESHOLD)
        {
            LOAD_ENABLE;
            OC_Triggered_Counter = 0;
            OC_Triggered = 0;
        } // end if
    } // end if
} // end Load_Management

/* *************************************************************************************** *
 * Call this function to calculate the average of each ADC channel and convert the ADC     *
 *  values to reflect actual voltage and currents in volts and amps, respectively; the     *
 *  following equations are used and may need to be recalibrated:                          *
 *  * Panel_Voltage = (PV_ADC + 14.021) / 13.985                                           *
 *  * Battery_Voltage = (BV_ADC - 0.0659) / 15.446                                         *
 *  * Panel_Current = (PI_ADC + 1.4595) / 30.581                                           *
 *  * Battery_Current = (BI_ADC + 0.6037) / 30.567                                         *
 * *************************************************************************************** */
void Average_MPPT_ADC_Values()
{
    Avg_MPPT_Counter    = 0;
    Prev_Battery_Current = Battery_Current;

#ifdef USE_FLOAT
    Panel_Voltage       = ((float)((Panel_Voltage_Buffer >> 6) + 15) * (1. / 13.985));
    Battery_Voltage     = ((float)((Battery_Voltage_Buffer >> 6)) * (1. / 15.446));
    Panel_Current       = ((float)((Panel_Current_Buffer >> 6) + 2) * (1. / 30.581));
    Battery_Current     = ((float)((Battery_Current_Buffer >> 6)) * (1. / 30.567));
#else
    Panel_Voltage       = (Panel_Voltage_Buffer >> 3);
    Battery_Voltage     = (Battery_Voltage_Buffer >> 3);
    Panel_Current       = (Panel_Current_Buffer >> 3);
    Battery_Current     = (Battery_Current_Buffer >> 3);
#endif

    Panel_Voltage_Buffer    = 0;
    Battery_Voltage_Buffer  = 0;
    Panel_Current_Buffer    = 0;
    Battery_Current_Buffer  = 0;
} // end Average_MPPT_ADC_Values


void Inst_protection_Values(){

    inst_prot_Counter = 0;

    Panel_Voltage_inst       = (Panel_Voltage_inst_Buffer >> 1);
    Battery_Voltage_inst     = (Battery_Voltage_inst_Buffer >> 1);
    Panel_Current_inst       = (Panel_Current_inst_Buffer >> 1);
    Battery_Current_inst     = (Battery_Current_inst_Buffer >> 1);

    Panel_Voltage_inst_Buffer    = 0;
    Battery_Voltage_inst_Buffer  = 0;
    Panel_Current_inst_Buffer    = 0;
    Battery_Current_inst_Buffer  = 0;
}



//-----------------------------------------------------------------------------------------//
//------------------------------ Interrupt Service Routines -------------------------------//
//-----------------------------------------------------------------------------------------//
#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR(void)
{
    switch(__even_in_range(DMAIV,16))
    {
        case  0: break;                         // No interrupt
        case  2:                                // Sequence of conversions complete
            SFRIE1 &= ~TAIE;                    // Timer Interrupt; Use WDTIE if watch dog is used
            ADC10CTL0 &= ~ADC10ENC;             // Disable conversion

            Panel_Voltage_inst_Buffer += ADC_Readings [P_V];
            Battery_Voltage_inst_Buffer += ADC_Readings [B_V];
            Panel_Current_inst_Buffer += ADC_Readings [P_I];
            Battery_Current_inst_Buffer += ADC_Readings [B_I];

            Panel_Voltage_Buffer += ADC_Readings [P_V];
            Battery_Voltage_Buffer += ADC_Readings [B_V];
            Panel_Current_Buffer += ADC_Readings [P_I];
            Battery_Current_Buffer += ADC_Readings [B_I];

            Avg_MPPT_Counter++;
            inst_prot_Counter++;
            Read_Done = 1;
            SFRIE1 |= TAIE;      // Timer Interrupt; Use WDTIE if watch dog is used
            break;                              // DMA0IFG
        case  4: break;                         // DMA1IFG
        case  6: break;                         // DMA2IFG
        case  8: break;                         // reserved
        case 10: break;                         // reserved
        case 12: break;                         // reserved
        case 14: break;                         // reserved
        case 16: break;                         // reserved
        default: break;
    } // end switch
} // end DMA interrupt


#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
// enable this section (lines 944 and 945) if the interrupt is done through Watch dog

//    __data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &ADC_Readings[0]);
//    ADC10CTL0 |= ADC10ENC + ADC10SC + ADC10ON;        // (ADC10ON) Start ADC sampling and conversion

  //  WDTCTL = WDT_MDLY_32;   // WDT set to 32ms for SMCLK = 1MHz; since we are using SMCLK = 25MHz, timer interval is 1.3ms
    //   SFRIE1 |= WDTIE;
} // end WDT interrupt


#pragma vector=COMP_B_VECTOR
__interrupt void Comp_B_ISR(void)
{
    LOAD_DISABLE;           // Interrupt triggered in event of load overcurrent, disable load
    CBINT &= ~CBIFG;        // Clear Comparator B interrupt flag
    OC_Triggered = 1;
    OC_Triggered_Counter = 0;
} // end Comparator interrupt

#pragma vector=TIMER0_D0_VECTOR
__interrupt void TIMER0_D0_ISR(void)
{
    //TODO - Adjust delay time for 180
    __delay_cycles(55);                     // Delay Adjusted for perfect 180 deg Phase shift
    TD1CTL0 |= TDCLR; //MC_3 +
    TD0CCTL0 &= ~CCIE;
    Phase_Shifting_Done = 1;
} // end Timer D0 interrupt

#pragma vector=TIMER0_D1_VECTOR
__interrupt void TIMER0_D1_ISR(void)
{
    switch(__even_in_range(TD0IV, 30))
    {
        case  0: break;                 // No interrupt
        case  2: break;                 // CCR1 not used
        case  4: break;                 // CCR2 not used
        case  6: break;                 // reserved
        case  8: break;                 // reserved
        case 10: break;                 // reserved
        case 12: break;                 // reserved
        case 14: break;
        case 16: break;
        case 18:                        // Clock fail low
            while(1);                   // Input reference clock frequency too low; trap here
        case 20:                        // Clock fail high
            while(1);                   // Input reference clock frequency too high; trap here
        case 22:                        // Hi-res frequency locked
            P1SEL |= BIT7;                  // P1.7/TD0.1 selected
            P1DIR |= BIT7;                  // P1.7 set as output
            P2SEL |= BIT0 + BIT2 + BIT3;    // P2.0/TD0.2, P2.2/TD1.1, and P2.3/TD1.2 selected
            P2DIR |= BIT0 + BIT2 + BIT3;    // P2.0, P2.2, P2.3 set as output
            __bic_SR_register_on_exit(LPM0_bits + GIE); // Exit LPM0 on return to main
            break;
        case 24: break;                 // Hi-res frequency unlocked
        case 26: break;                 // reserved
        case 28: break;                 // reserved
        case 30: break;                 // reserved
        default: break;
    } // end switch
} // end Timer D1 interrupt

#pragma vector=USCI_B0_VECTOR
__interrupt void I2C_TX_ISR(void)
{
    switch(__even_in_range(UCB0IV, 12))
    {
        case  0:    break;          // No interrupt
        case  2:    break;          // I2C Arbitration Lost not used
        case  4:    break;          // I2C Not-Acknowledge not used
        case  6:    break;          // I2C Start Condition not used
        case  8:    break;          // I2C Stop Condition not used
        case 10:    break;          // I2C Receive
        case 12:    break;          // I2C Transmit
        default:    break;
    }
}

void Mode_Select(int Mode_sel) {
    static unsigned char prev_state = 2;

    if (prev_state != Mode_sel){


        if (Mode_sel == 0) {

            TD0CCTL0 &= ~CCIFG; // wait till the timer completes its current cycle

            while(!(TD0CCTL0 & CCIFG));

            TD0CCR1 = TD1CCR1 = Duty;       // Set the duty cycle for the buck
            TD0CCR2 = TD1CCR2 = Duty - DEADBAND;

            TD0CCTL0 &= ~CCIFG; // wait till the timer completes its current cycle

            while(!(TD0CCTL0 & CCIFG));
            P2OUT &= ~BIT2;
            P1OUT &= ~BIT7;
            P2SEL &= ~BIT2;
            P2DIR |= BIT2;


            P1SEL &= ~BIT7;
            P1DIR |= BIT7;


            P2SEL |= BIT0;
            P2DIR |= BIT0;
            P2OUT |= BIT0;

            P2SEL |= BIT3;
            P2DIR |= BIT3;
            P2OUT |= BIT3;
        }

        else {
            TD0CCTL0 &= ~CCIFG; // wait till the timer completes its current cycle

            while(!(TD0CCTL0 & CCIFG));
            P2SEL |= BIT2;
            P2DIR |= BIT2;
            P2OUT |= BIT2;

            P1SEL |= BIT7;
            P1DIR |= BIT7;
            P1OUT |= BIT7;

            P2SEL |= BIT0;
            P2DIR |= BIT0;
            P2OUT |= BIT0;

            P2SEL |= BIT3;
            P2DIR |= BIT3;
            P2OUT |= BIT3;

            TD0CCTL0 &= ~CCIFG; // wait till the timer completes its current cycle

            while(!(TD0CCTL0 & CCIFG));

            TD0CCR1 = TD1CCR1 = Duty;       // Set the duty cycle for the buck
            TD0CCR2 = TD1CCR2 = Duty - DEADBAND;


        }
        prev_state = Mode_sel;


}

}

void init_TimerA(void) {

    TA0CCTL0 = CCIE;                             // CCR0 interrupt enabled
    TA0CCR0 = 5000;                              // Frequency of ISR (25Mhz/50000) 0.2ms
    TA0CTL = TASSEL_2 + MC_1 +TAIE;              // SMCLK, countmode; Interrupt enable

}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {

    P2DIR |= BIT7; // Sanity check to see if interrupt occurs
    P2OUT ^= BIT7;
    __data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &ADC_Readings[0]);
    ADC10CTL0 |= ADC10ENC + ADC10SC + ADC10ON;        // Start ADC sampling and conversion
    boot_flag = 0;              // Start initially in diode emulation

    boot_count++;

    if (boot_count >= 500)      // Change is count value to control ON time of Bottom FET in Diode Emulation Mode
    {
        boot_count = 0;
        if (Mode_sel == 0){

            boot_flag = 1;     //Flag which sets the turning on instance of FET in DEM
        }

    }
        TA0CCTL0 &= ~CCIFG;     //Clear flag
        TA0CTL &= ~TAIFG;
}
