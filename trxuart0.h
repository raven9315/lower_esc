/*
 * uart0.h
 *
 *  Created on: 2021. 3. 9.
 *      Author: raven9315
 */

#ifndef TRXUART0_H_
#define TRXUART0_H_
#include <stdint.h>
typedef struct EUSCI_A_UART_Error
{
    uint16_t cnt_none;
    uint16_t cnt_uart_ucrxifg;
    uint16_t cnt_uart_uctxifg;
    uint16_t cnt_uart_uscttifg;
    uint16_t cnt_uart_uctxcptifg;
    uint16_t cnt_int;
} EUSCI_A_UART_Error;
typedef struct proc_handle_flag
{
    uint8_t tf_flag;
    uint8_t uart1_rx;
    uint8_t uart1_tx;
    uint8_t uart1_rx_complete_flag;
    uint8_t uart1_tx_complete_flag;
    uint8_t uart0_rx_complete_flag;
    uint8_t uart0_tx_complete_flag;
    //uint8_t cnt_uart_uscttifg;
    //uint8_t cnt_uart_uctxcptifg;
    //uint8_t cnt_int;
} proc_handle_flag;
//extern proc_handle_flag handle_flag;
//extern proc_handle_flag handle_flag;
//extern volatile uint8_t test_val;
#define MAX_CLOCK   1000000
#define baudrate    9600

void init_uart0(void);
void init_uart1(void);
void u1TestSend(void);
void u0Receive(void);
void u1Receive(void);
void u1Transmit(void);
#endif /* TRXUART0_H_ */
