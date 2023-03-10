/********************************************************************
* FileName:		SymbolTime.h
* Dependencies: ConfigApp.h
* Processor:	PIC18, PIC24F, PIC32, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Hardware:		PICDEM Z, Explorer 16, PIC18 Explorer
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher
*               Microchip C32 v1.02 or higher	
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
*
* Copyright � 2007-2010 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute 
* Software only when embedded on a Microchip microcontroller or digital 
* signal controller and used with a Microchip radio frequency transceiver, 
* which are integrated into your product or third party product (pursuant 
* to the terms in the accompanying license agreement).   
*
* You should refer to the license agreement accompanying this Software for 
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY 
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
* ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
* LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
* TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT 
* NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
*********************************************************************
* File Description:
*
*  This file provides access to all of the time managment functions
*   as well as calculating the timer scaling settings required for
*   accurate symbol time measurement
*
* Change History:
*  Rev   Date         Author    Description
*  0.1   11/09/2006   yfy       Initial revision
*  1.0   01/09/2007   yfy       Initial release
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
*  2.1   06/20/2009   yfy       Add LCD support
*  3.1   5/28/2010    yfy       MiWi DE 3.1
*  4.1   6/3/2011     yfy       MAL v2011-06
********************************************************************/

#ifndef __SYMBOL_TIME_H_
#define __SYMBOL_TIME_H_

/************************ HEADERS **********************************/

#include "Compiler.h"
#include "GenericTypeDefs.h"
#include "system_config.h"
//#include "legacy/timer.h"

/************************ DEFINITIONS ******************************/

#if defined(__PIC32MX__)
    /* this section is based on the Timer 2/3 module of the PIC32MX family */
    #if defined(WIRELESS_EVAL_BOARD)
//        #define INSTR_FREQ              (CLOCK_FREQ/4)//64MHz--->16MHz    //original
//        #define INSTR_FREQ              (CLOCK_FREQ)//64MHz--->16MHz    
        #define INSTR_FREQ                  (GetPeripheralClock())//(CLOCK_FREQ/1)//64MHz--->16MHz
    #else
        #define INSTR_FREQ  (CLOCK_FREQ/4)//64MHz--->16MHz
    #endif

    #if(INSTR_FREQ <= 125000)
        #define CLOCK_DIVIDER 1
        #define CLOCK_DIVIDER_SETTING 0x0000 /* no prescalar */
        #define SYMBOL_TO_TICK_RATE 125000
    #elif(INSTR_FREQ <= 1000000)
        #define CLOCK_DIVIDER 8
        #define CLOCK_DIVIDER_SETTING 0x0030
        #define SYMBOL_TO_TICK_RATE 1000000
    #elif(INSTR_FREQ <= 8000000)
        #define CLOCK_DIVIDER 64
        #define CLOCK_DIVIDER_SETTING 0x0060
        #define SYMBOL_TO_TICK_RATE 8000000
    #elif(INSTR_FREQ <= 16000000)

        #define CLOCK_DIVIDER                       256//original
//        #define CLOCK_DIVIDER_SETTING       0x0070    //original
        #define CLOCK_DIVIDER_SETTING       T2_PS_1_256|T2_SOURCE_INT
        #define SYMBOL_TO_TICK_RATE         INSTR_FREQ  //original
    #else
        #define CLOCK_DIVIDER                           256 //original
//        #define CLOCK_DIVIDER_SETTING           0x70
    #define CLOCK_DIVIDER_SETTING       T2_PS_1_256|T2_SOURCE_INT
        #define SYMBOL_TO_TICK_RATE             INSTR_FREQ
    #endif

//    #define ONE_SECOND                                  (((DWORD)INSTR_FREQ/1000 * 62500) / (SYMBOL_TO_TICK_RATE / 1000) )//original
//    #define ONE_SECOND                                  (((DWORD)INSTR_FREQ/1000 * 39000) / (SYMBOL_TO_TICK_RATE / 1000) * 2 ) // work with CLOCK 40MHz
//    #define ONE_SECOND                                  (((DWORD)INSTR_FREQ/1000 * 39000) / (SYMBOL_TO_TICK_RATE / 1000) * 2 * 4 ) // work with CLOCK 80MHz
//    #define ONE_SECOND                                  ( ( (DWORD)CLOCK_FREQ/1000 ) * 4 ) // work with CLOCK 80MHz, 64MHz, 40MHz
    #define ONE_SECOND                                  ( ( (DWORD)GetPeripheralClock()/1000 ) * 4 ) // work with CLOCK 80MHz, 64MHz, 40MHz
//    #define ONE_SECOND                                  (((DWORD)INSTR_FREQ/1000 * 62500) / (SYMBOL_TO_TICK_RATE / 1000) * 4 ) // work with CLOCK 64MHz
//    #define ONE_SECOND                                  (((DWORD)INSTR_FREQ/1000 * 62500) / (SYMBOL_TO_TICK_RATE / 1000) * 5 )    // work with CLOCK 80MHz

    /* SYMBOLS_TO_TICKS to only be used with input (a) as a constant, otherwise you will blow up the code */
    #define SYMBOLS_TO_TICKS(a)                     (((DWORD)(INSTR_FREQ/100000) * a) / (SYMBOL_TO_TICK_RATE / 100000)) //original
    #define TICKS_TO_SYMBOLS(a)                     (((DWORD)SYMBOL_TO_TICK_RATE/100000) * a / ((DWORD)CLOCK_FREQ/100000)) //original
#else
    #error "Unsupported processor.  New timing definitions required for proper operation"
#endif

#define ONE_MILI_SECOND     (ONE_SECOND/1000)
#define HUNDRED_MILI_SECOND (ONE_SECOND/10)
#define FORTY_MILI_SECOND   (ONE_SECOND/25)
#define TWENTY_MILI_SECOND  (ONE_SECOND/50)
#define TEN_MILI_SECOND     (ONE_SECOND/100)
#define FIVE_MILI_SECOND    (ONE_SECOND/200)
#define TWO_MILI_SECOND     (ONE_SECOND/500)
#define ONE_MINUTE          (ONE_SECOND*60)
#define ONE_HOUR            (ONE_MINUTE*60)

#define MiWi_TickGetDiff(a,b)           (a.Val - b.Val)

/************************ DATA TYPES *******************************/


/******************************************************************
 // Time unit defined based on IEEE 802.15.4 specification.
 // One tick is equal to one symbol time, or 16us. The Tick structure
 // is four bytes in length and is capable of represent time up to
 // about 19 hours.
 *****************************************************************/
typedef union _MIWI_TICK
{
    DWORD Val;
    struct _MIWI_TICK_bytes
    {
        BYTE b0;
        BYTE b1;
        BYTE b2;
        BYTE b3;
    } byte;
    BYTE v[4];
    struct _MIWI_TICK_words
    {
        WORD w0;
        WORD w1;
    } word;
} MIWI_TICK;

void InitSymbolTimer(void);
MIWI_TICK MiWi_TickGet(void);

/************************ VARIABLES ********************************/

extern volatile BYTE timerExtension1,timerExtension2;
#endif
