/*
 * lower_esc.h
 *
 *  Created on: 2021. 3. 9.
 *      Author: raven9315
 */

#ifndef MAIN_H_
#define MAIN_H_
//*****************************************************************************
//
//Target frequency for SMCLK in kHz
//
//*****************************************************************************
//#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   1000 //1M
//#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   1048 //1.048M
#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   8000 //8M
//#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   16000 //8M
#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   8000
// Internal, trimmed, low-frequency oscillator with 32768 Hz typical frequency
//
//*****************************************************************************
#define UCS_REFOCLK_FREQUENCY                                             32768
//*****************************************************************************
//
//SMCLK/FLLRef Ratio
//
//*****************************************************************************
//#define CS_SMCLK_FLLREF_RATIO   32 //1M
#define CS_SMCLK_FLLREF_RATIO   CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ / UCS_REFOCLK_FREQUENCY //244 8M
#define CS_MCLK_FLLREF_RATIO   CS_MCLK_DESIRED_FREQUENCY_IN_KHZ / UCS_REFOCLK_FREQUENCY //244 8M
//*****************************************************************************
//
// The following are values that can be passed to the clockSourceDivider
// parameter for functions: UCS_initClockSignal().
//
//*****************************************************************************
#define UCS_CLOCK_DIVIDER_1                                             DIVM__1 //[기본값]
#define UCS_CLOCK_DIVIDER_2                                             DIVM__2
#define UCS_CLOCK_DIVIDER_4                                             DIVM__4
#define UCS_CLOCK_DIVIDER_8                                             DIVM__8
#define UCS_CLOCK_DIVIDER_12                                           DIVM__12 // [ UCS_FLLREF 에만 유효 함]
#define UCS_CLOCK_DIVIDER_16                                           DIVM__16
#define UCS_CLOCK_DIVIDER_32                                           DIVM__32 //[ UCS_FLLREF에 유효하지 않음]

//*****************************************************************************
//
// The following are values that can be passed to the selectedClockSignal
// parameter for functions: UCS_initClockSignal().
//
//*****************************************************************************
#define UCS_ACLK                                                           0x01
#define UCS_MCLK                                                           0x02
#define UCS_SMCLK                                                          0x04
#define UCS_FLLREF                                                         0x08

#define SLAVE_ADDRESS 0x3f


#endif /* MAIN_H_ */
