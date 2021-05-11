/*
 * uart0.c
 *
 *  Created on: 2021. 3. 9.
 *      Author: raven9315
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "driverlib.h"
#include "trxuart0.h"
#include  "main.h"
//#include <trxuart0.h>
//UARTConfig UART0;
//USCIUARTRegs uartUsciRegs;
//USARTUARTRegs uartUsartRegs;
//extern proc_handle_flag handle_flag;
uint8_t u0RXData, u0TXData;
uint8_t u1RXData, u1TXData;
EUSCI_A_UART_Error uart_err0 = {0};
EUSCI_A_UART_Error uart_err1 = {0};
proc_handle_flag handle_flag={0};
extern uint32_t ACLK_value, MCLK_value, SMCLK_value;
void init_uart0(void)
{
    //extern EUSCI_A_UART_Error uart_err0, uart_err1;
    //extern uint8_t u0RXData, u0TXData, u0check;
    // Configure UART pins
    //Set P2.0 and P2.1 as Secondary Module Function Input.
    /*

    * Select Port 2d
    * Set Pin 0, 1 to input Secondary Module Function, (UCA0TXD/UCA0SIMO, UCA0RXD/UCA0SOMI).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
    );
    // Configure UART
    EUSCI_A_UART_initParam param0 = {0};
    param0.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK; //EUSCI_A_UART_CLOCKSOURCE_ACLK;
    param0.clockPrescalar = 3;
    param0.firstModReg = 0;
    param0.secondModReg = 92;
    param0.parity = EUSCI_A_UART_NO_PARITY;
    param0.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param0.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param0.uartMode = EUSCI_A_UART_MODE;
    param0.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
    uart_err0.cnt_none =0;
    uart_err0.cnt_uart_ucrxifg=0;
    uart_err0.cnt_uart_uctxcptifg=0;
    uart_err0.cnt_uart_uctxifg=0;
    uart_err0.cnt_uart_uscttifg=0;
    u0RXData = 0, u0TXData = 0;


    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param0)) {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
      EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
      EUSCI_A_UART_RECEIVE_INTERRUPT);                     // Enable interrupt

}
void init_uart1(void)
{

    extern EUSCI_A_UART_Error uart_err1;

    //extern uint8_t u1RXData, u1TXData, u1check;
    // Configure UART pins
    //Set P4.3 and P4.2 as Secondary Module Function Input.
    /*

    * Select Port 4
    * Set Pin 3, 2 to input Secondary Module Function, (UCA1TXD/UCA1SIMO, UCA1RXD/UCA1SOMI).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P4,
    GPIO_PIN2,
    GPIO_SECONDARY_MODULE_FUNCTION
    );
    GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P4,
    GPIO_PIN3,
    GPIO_SECONDARY_MODULE_FUNCTION
    );
    /*GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P4,
        GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
    );*/
    /* UCA1CLK P4.1, UCA1STE/P4.0 */
/*    GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P4,
    GPIO_PIN1,
    GPIO_SECONDARY_MODULE_FUNCTION
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P4,
    GPIO_PIN0,
    GPIO_SECONDARY_MODULE_FUNCTION
    );
    */
    // Configure UART
    EUSCI_A_UART_initParam param1 = {0};
    param1.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK; //EUSCI_A_UART_CLOCKSOURCE_ACLK;
    //param1.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_ACLK;
    param1.clockPrescalar = 52; //6; //6; //1M 9600, 52 8M 9600;
    param1.firstModReg = 1; //8; //8; // 1M 9600 1; 8M 9600
    param1.secondModReg = 0; //17; //17; //1M 9600, 0 8M 9600 0; //3;
    param1.parity = EUSCI_A_UART_NO_PARITY;
    param1.msborLsbFirst = EUSCI_A_UART_LSB_FIRST; //EUSCI_A_UART_MSB_FIRST; //EUSCI_A_UART_LSB_FIRST;
    param1.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    //param1.uartMode = EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE; //EUSCI_A_UART_MODE;
    param1.uartMode = EUSCI_A_UART_MODE; //EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE; //EUSCI_A_UART_MODE;
    //param1.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
    param1.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    uart_err1.cnt_none =0; uart_err1.cnt_int=0;
    uart_err1.cnt_uart_ucrxifg=0;
    uart_err1.cnt_uart_uctxcptifg=0;
    uart_err1.cnt_uart_uctxifg=0;
    uart_err1.cnt_uart_uscttifg=0;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &param1)) {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A1_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,
      EUSCI_A_UART_RECEIVE_INTERRUPT|EUSCI_A_UART_TRANSMIT_INTERRUPT);

    // Enable USCI_A1 RX interrupt
    /*EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,
      EUSCI_A_UART_RECEIVE_INTERRUPT+EUSCI_A_UART_TRANSMIT_INTERRUPT+EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT+EUSCI_A_UART_BREAKCHAR_INTERRUPT+EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT); */                    // Enable interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT|EUSCI_A_UART_TRANSMIT_INTERRUPT);                     // Enable interrupt
    //EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT);        // Enable interrupt
    __delay_cycles(10);


}
void u1TestSend(void)
{
    uint8_t i=0,j=0;

    u1RXData = 0, u1TXData = 0;
/*    txdata[1]=atoi("a");
    txdata[2]=atoi("r");
*/
    //printf("uart0\n\r");
    //for(j=0;j<10;j++)
    //{
        for( i=0; i<10;i++)
        {
            EUSCI_A_UART_transmitData(EUSCI_A1_BASE,u1TXData=i+0x31);
            __delay_cycles(100000);
        }
        __delay_cycles(100000);
    //}
    //printf("uart0\n\r");
    for(j=0;j<10;j++)
    {
        for( i=0; i<10;i++)
        {
            EUSCI_A_UART_transmitData(EUSCI_A1_BASE,u1TXData=i+0x31);
            __delay_cycles(10000);
        }
        __delay_cycles(10000);
    }
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    u1TXData=0x41;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData); __delay_cycles(1000);
    __delay_cycles(100);


}
void u0Receive(void)
{
    //extern uint8_t u0RXData, u0TXData, u0check;
    if(handle_flag.uart0_rx_complete_flag==false)return;
    u0TXData = u0TXData+1;                      // Increment TX data
    // Load data onto buffer
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE,
                       u0TXData);
    /*while(u0check != 1);*/

}
void u1Receive(void)
{

    if(handle_flag.uart1_rx_complete_flag==false)return;

    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData);
    handle_flag.uart1_rx_complete_flag=false;
}
void u1Transmit(void)
{
    if(handle_flag.uart1_tx_complete_flag==false)return;
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, u1TXData);
    handle_flag.uart1_tx_complete_flag=false;

}
//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void USCI_A0_ISR(void)
{
    //extern uint8_t u0RXData, u0TXData, u0check;
  //EUSCI_A_UART_Error uart_err0 = {0};

  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: uart_err0.cnt_none++; break;
    case USCI_UART_UCRXIFG:
      u0RXData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
      uart_err0.cnt_uart_ucrxifg++;
      handle_flag.uart0_rx_complete_flag=true;
      break;
    case USCI_UART_UCTXIFG: uart_err0.cnt_uart_uctxifg++;
        handle_flag.uart0_tx_complete_flag=true;
    break;
    case USCI_UART_UCSTTIFG: uart_err0.cnt_uart_uctxcptifg++; break;
    case USCI_UART_UCTXCPTIFG: uart_err0.cnt_uart_uctxcptifg++; break;
  }
}
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void USCI_A1_ISR(void)
{

    //extern uint8_t u1RXData, u1TXData, u1check;
    //uint8_t status;

  //EUSCI_A_UART_Error uart_err1 = {0};
 /*   if(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG) == false)
    {
        uart_err1.cnt_int++;
        return;
    }
*/

  /* status = EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
                                           | EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG
                                           | EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG
                                           | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG ); */
  /*status = EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG
                                           | EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG );*/
  if(EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)==EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
      u1RXData = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
      uart_err1.cnt_uart_ucrxifg++;
      handle_flag.uart1_rx_complete_flag=true;
      u1TXData = u1RXData;
  }
  if(EUSCI_A_UART_getInterruptStatus(EUSCI_A1_BASE,EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)==EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
  {
      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
      uart_err1.cnt_uart_uctxifg++;
      //handle_flag.uart1_tx_complete_flag=true;
  }
  /* if(status & EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG==EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG)
  {
      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG);
      uart_err1.cnt_uart_uscttifg++;
  }
  if(status & EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG==EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG)
  {
      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG);
      uart_err1.cnt_uart_uctxcptifg++;
  }else // if(status&&USCI_NONE)
  {
      EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,USCI_NONE);
      uart_err1.cnt_none++;
  }*/
//#define EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG                             UCRXIFG //0x0001
//#define EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG                            UCTXIFG //0x0002
//#define EUSCI_A_UART_STARTBIT_INTERRUPT_FLAG                           UCSTTIFG //0x0004
//#define EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG                UCTXCPTIFG //0x0008
}
