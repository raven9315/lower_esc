/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//!  EUSCI_A0 External Loopback test using EUSCI_A_UART_init API
//!
//!  Description: This demo connects TX to RX of the MSP430 UART
//!  The example code shows proper initialization of registers
//!  and interrupts to receive and transmit data.
//!
//!  ACLK = BRCLK = 32.768kHz, MCLK = SMCLK = DCO = ~1MHz
//!
//!
//!      Tested on MSP430FR2355
//!             -----------------
//!       RST -|     P4.3/UCA1TXD|----|
//!            |                 |    |
//!            |                 |    |
//!            |     P4.2/UCA1RXD|----|
//!            |                 |
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - UART peripheral
//! - GPIO Port peripheral (for UART pins)
//! - UCA1TXD
//! - UCA1RXD
//! - UCA1CLK/P4.1, UCA1STE/P4.0
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_A1_VECTOR.
//******************************************************************************
#include <msp430.h>
#include "driverlib.h"
#include <main.h>
#include <trxuart0.h>


void initialize(void);
VOID INIT_CS(VOID);
void init_cs1(void);
//void init_cs2(void);
void init_i2c(void);
void init_led(void);
extern EUSCI_A_UART_Error uart_err1;
extern proc_handle_flag handle_flag;
static CS_initFLLParam FLLParam={0};
static uint32_t ACLK_value, MCLK_value, SMCLK_value;
void main(void)
{
    //static proc_handle_flag handle_flag={0};
    // stop watchdog
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();
    // LFXT Setup
    //Set PJ.4 and PJ.5 as Primary Module Function Input.
    /*

    * Select Port J
    * Set Pin 4, 5 to input Primary Module Function, LFXT.
    */


    initialize();

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    //PMM_unlockLPM5();
    __enable_interrupt();
    u1TestSend();
    while (1)
    {

        u1Receive();
        u1Transmit();
        //u0Receive();
    }
}
void initialize(void)
{
    //init_cs();
    init_cs1();
    //init_uart0();
    init_uart1();
    init_i2c();
    init_led();
    handle_flag.uart1_rx_complete_flag=false;
    uart_err1.cnt_int=0;
}
void init_cs(void)
{

}
void init_cs1(void)
{
    //static uint32_t ACLK_value, MCLK_value, SMCLK_value;
    //PMM_setVCore(PMM_CORE_LEVEL_1);
    //CS_setExternalClockSource (32768, 14745600);
    //CS_turnOnLFXTWithTimeout (LFXTDRIVE_3, 100000); // 저주파 모드에서 저주파 크리스탈 시작
    // 고주파 크리스탈 시작
    // # define CS_HFXT_DRIVE_8MHZ_16MHZ (HFXTDRIVE_1)
    //CS_turnOnHFXTWithTimeout (HFXTDRIVE_1, 100000);

    //Set Ratio and Desired MCLK Frequency  and initialize DCO

    FLLParam.csCtl0=0x0000; //0x12fb; //0x0000;
    FLLParam.csCtl1=0x007f; //0x0033; //0x007f;
    FLLParam.fsystem=CS_MCLK_DESIRED_FREQUENCY_IN_KHZ;
    CS_initFLLSettle(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ,CS_MCLK_FLLREF_RATIO);
    //CS_initFLL(CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,CS_SMCLK_FLLREF_RATIO);
    CS_initFLLCalculateTrim(CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,CS_SMCLK_FLLREF_RATIO,&FLLParam);
    CS_initFLLLoadTrim(CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,CS_SMCLK_FLLREF_RATIO,&FLLParam);
    //bool CS_initFLLCalculateTrim(uint16_t fsystem, uint16_t ratio,CS_initFLLParam *param)
    //static void privateCSComputeDCOFTrim(CS_initFLLParam *param)
    //static uint32_t privateCSSourceClockFromDCO(uint16_t FLLRefCLKSource)
    CS_initClockSignal(
                CS_FLLREF,
                CS_REFOCLK_SELECT, //CS_DCOCLKDIV_SELECT,
                CS_CLOCK_DIVIDER_1
                ); //CS_FLLREF

    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_MCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
            );

    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_SMCLK,
            CS_DCOCLKDIV_SELECT,//CS_REFOCLK_SELECT CS_DCOCLKDIV_SELECT
            CS_CLOCK_DIVIDER_1
            );

    //Set ACLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_ACLK,
            CS_REFOCLK_SELECT,
            CS_CLOCK_DIVIDER_1
            );

    ACLK_value = CS_getACLK();
    SMCLK_value = CS_getSMCLK();
    MCLK_value = CS_getMCLK();
    SMCLK_value = CS_getSMCLK();
    //Set DCO frequency to 1 MHz
    //CS_setDCOFreq(CS_DCORSEL_0,CS_DCOFSEL_0);
    //Set external clock frequency to 32.768 KHz
    //CS_setExternalClockSource(32768,0);
    //Set ACLK=LFXT
    //CS_initClockSignal(CS_ACLK,CS_LFXTCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    //CS_initClockSignal(CS_SMCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    //CS_initClockSignal(CS_MCLK,CS_DCOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Start XT1 with no time out
    //CS_turnOnLFXT(CS_LFXT_DRIVE_0);



}
/*void init_cs2(void)
{

}*/
void init_i2c(void)
{
    // Configure Pins for I2C
    /*
    * Select Port 1
    * Set Pin 2, 3 to input with function, (UCB0SIMO/UCB0SDA/P1.2, UCB0SOMI/UCB0SCL/P1.3).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
    );
}
void init_led(void)
{
    //void GPIO_setAsOutputPin(uint8_t selectedPort, uint16_t selectedPins)
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0 + GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5 + GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN6);
}


