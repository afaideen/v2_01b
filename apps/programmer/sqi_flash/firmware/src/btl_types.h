/*******************************************************************************
  System Definitions

  File Name:
    btl_types.h

  Summary:
 Bootloader type definitions.

  Description:
    This file contains the type definitions needed for any bootloader on any
    PIC32 platform.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef BTL_TYPES_H
#define	BTL_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* Specify an extension for GCC based compilers */
#if defined(__GNUC__)
#define __EXTENSION __extension__
#else
#define __EXTENSION
#endif

typedef union
{
    uint16_t Val;
    uint8_t v[2];
    struct
    {
        uint8_t LB;
        uint8_t HB;
    } byte;
    struct
    {
        __EXTENSION uint8_t b0:1;
        __EXTENSION uint8_t b1:1;
        __EXTENSION uint8_t b2:1;
        __EXTENSION uint8_t b3:1;
        __EXTENSION uint8_t b4:1;
        __EXTENSION uint8_t b5:1;
        __EXTENSION uint8_t b6:1;
        __EXTENSION uint8_t b7:1;
        __EXTENSION uint8_t b8:1;
        __EXTENSION uint8_t b9:1;
        __EXTENSION uint8_t b10:1;
        __EXTENSION uint8_t b11:1;
        __EXTENSION uint8_t b12:1;
        __EXTENSION uint8_t b13:1;
        __EXTENSION uint8_t b14:1;
        __EXTENSION uint8_t b15:1;
    } bits;
} BTL_WORD_VAL;

typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t v[4];
    struct
    {
        uint16_t LW;
        uint16_t HW;
    } word;
    struct
    {
        uint8_t LB;
        uint8_t HB;
        uint8_t UB;
        uint8_t MB;
    } byte;
    struct
    {
        BTL_WORD_VAL low;
        BTL_WORD_VAL high;
    }wordUnion;
    struct
    {
        __EXTENSION uint8_t b0:1;
        __EXTENSION uint8_t b1:1;
        __EXTENSION uint8_t b2:1;
        __EXTENSION uint8_t b3:1;
        __EXTENSION uint8_t b4:1;
        __EXTENSION uint8_t b5:1;
        __EXTENSION uint8_t b6:1;
        __EXTENSION uint8_t b7:1;
        __EXTENSION uint8_t b8:1;
        __EXTENSION uint8_t b9:1;
        __EXTENSION uint8_t b10:1;
        __EXTENSION uint8_t b11:1;
        __EXTENSION uint8_t b12:1;
        __EXTENSION uint8_t b13:1;
        __EXTENSION uint8_t b14:1;
        __EXTENSION uint8_t b15:1;
        __EXTENSION uint8_t b16:1;
        __EXTENSION uint8_t b17:1;
        __EXTENSION uint8_t b18:1;
        __EXTENSION uint8_t b19:1;
        __EXTENSION uint8_t b20:1;
        __EXTENSION uint8_t b21:1;
        __EXTENSION uint8_t b22:1;
        __EXTENSION uint8_t b23:1;
        __EXTENSION uint8_t b24:1;
        __EXTENSION uint8_t b25:1;
        __EXTENSION uint8_t b26:1;
        __EXTENSION uint8_t b27:1;
        __EXTENSION uint8_t b28:1;
        __EXTENSION uint8_t b29:1;
        __EXTENSION uint8_t b30:1;
        __EXTENSION uint8_t b31:1;
    } bits;
} BTL_DWORD_VAL;


#ifdef	__cplusplus
}
#endif

#endif	/* BTL_TYPES_H */

