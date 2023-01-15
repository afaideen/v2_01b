/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol_local.h

  Summary:
    Graphics Object Layer of the Microchip Graphics Library.

  Description:
    This module implements the common routines for the Graphics Object Layer
    of the Microchip Graphics Library. The routines are independent of the
    Display Driver Layer and should be compatible with any Display Driver
    that is compliant with the requirements of the Display Driver
    Layer of the Graphics Library.
    The module utilizes the Graphics Primitive Layer to render the objects.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _GFX_GOL_LOCAL_H
// DOM-IGNORE-BEGIN
    #define _GFX_GOL_LOCAL_H
// DOM-IGNORE-END

/*********************************************************************
 * Section: Includes
 *********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "gfx/gfx_gol.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
#ifndef GFX_CONFIG_DOUBLE_BUFFERING_DISABLE
#define REDRAW_SCREEN 2
#define REDRAW_AREA 1
#endif

typedef enum {
    RPD_BEGIN,
    ARC1,
    DRAW_EMBOSS1,
    DRAW_EMBOSS2,
    DRAW_EMBOSS3,
    DRAW_EMBOSS4,
    DRAW_FACECOLOR,
    DRAW_FACECOLOR1,
    DRAW_FACECOLOR2,
    #if (GFX_CONFIG_COLOR_DEPTH == 1)
    DRAW_INNERFRAME,
    #endif
    DRAW_FRAME,
    DRAW_IMAGE,
} ROUND_PANEL_DRAW_STATES;

typedef struct
{
    ROUND_PANEL_DRAW_STATES state;
    uint16_t counter;
} PANEL_DRAW;

typedef struct
{
    ROUND_PANEL_DRAW_STATES state;
    uint16_t counter;
} TWO_TONE_PANEL_DRAW;

typedef struct
{
    GFX_GOL_OBJ_HEADER  *pCurrentObj;
    GFX_GOL_OBJ_HEADER *pPrevObj;
    uint8_t displayUpdated;
    uint8_t bufferingActive;
} OBJECT_LIST_DRAW;

typedef struct
{
    bool newList;

    uint8_t redrawPending;

    // Function pointer to the message callback function
    GFX_GOL_MESSAGE_CALLBACK_FUNC    pGfxGOLMessageCallbackFunction;

    // Function pointer to the draw callback function
    GFX_GOL_DRAW_CALLBACK_FUNC       pGfxGOLDrawCallbackFunction;

    // Pointer to the current linked list of objects displayed and receiving messages
    GFX_GOL_OBJ_HEADER               *pGfxGolObjectList;

    // Pointer to the object receiving keyboard input
    GFX_GOL_OBJ_HEADER               *pGfxObjectFocused;

    // Variables for panel drawing. Used by GFX_GOL_PanelDraw
    GOL_PANEL_PARAM                  gfxPanel;

    PANEL_DRAW                       panelDraw;

    TWO_TONE_PANEL_DRAW              twoTonePanelDraw;

    OBJECT_LIST_DRAW                 objectListDraw;
} GFX_GOL_OBJ;


GFX_STATUS GFX_GOL_AlphaBlendTasks(GFX_GOL_OBJ_HEADER *object);

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_GOL_LOCAL_H
