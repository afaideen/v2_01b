/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_primitive_local.h

  Summary:
    Primitive Layer Local of the Microchip Graphics Library.

  Description:
    This module implements the primitive rendering routines of the Microchip
    Graphics Library. The routines are independent of the Display Driver Layer
    and should be compatible with any Display Driver that is compliant with
    the requirements of the Display Driver Layer of the Graphics Library.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#ifndef _GFX_PRIMITIVE_LOCAL_H
    #define _GFX_PRIMITIVE_LOCAL_H

#ifdef __cplusplus
    extern "C" {
#endif
        
//////////////////////////////////////////////////////
typedef enum
{
    GFX_SOLID_LINE_IDLE_STATE = 0x0D90,
    GFX_SOLID_LINE_VERHOR_DRAW_STATE,
    GFX_SOLID_LINE_DIAGONAL_INITIALIZE_STATE,
    GFX_SOLID_LINE_UPDATE_DRAW_STATE,
    GFX_SOLID_LINE_DIAGONAL_DRAW_STATE,
    GFX_SOLID_LINE_EXIT_STATE,
} GFX_SOLID_LINE_DRAW_STATE;

typedef struct
{
    uint8_t   state;
    int16_t   deltaX;
    int16_t   deltaY;
    int16_t   error;
    int16_t   stepErrorLT;
    int16_t   stepErrorGE;
    int16_t   stepX;
    int16_t   stepY;
    int16_t   steep;
    int16_t   temp;
    int16_t   style;
    int16_t   type;
    int16_t   x;
    int16_t   y; //used as storage for the x,y data
} GFX_LINE_DRAW;

#ifdef __cplusplus
    }
#endif

#endif // _GFX_PRIMITIVE_LOCAL_H
