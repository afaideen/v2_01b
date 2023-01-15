/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol_surface.c

  Summary:
    This implements the surface object of the GOL.

  Description:
    Refer to Microchip Graphics Library for complete documentation of the
    Surface Object.
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

#include <stdlib.h>
#include <stdint.h>
#include "system_config.h"
#include "system_definitions.h"

// *****************************************************************************
/*  Function:
    GFX_GOL_SURFACE  *GFX_GOL_SurfaceCreate(
                                SYS_MODULE_INDEX    gfxIndex,
                                uint16_t            ID,
                                uint16_t            left,
                                uint16_t            top,
                                uint16_t            right,
                                uint16_t            bottom,
                                uint16_t            state,
                                GFX_RESOURCE_HDR    *pImage,
                                GFX_XCHAR           *pText,
                                GFX_ALIGNMENT       alignment,
                                GFX_GOL_OBJ_SCHEME  *pScheme)
    Summary:
        This function creates a GFX_GOL_SURFACE object with the parameters
        given. It automatically attaches the new object into a global
        linked list of objects and returns the address of the object.

    Description:
        This function creates a GFX_GOL_SURFACE object with the parameters
        given. It automatically attaches the new object into a global
        linked list of objects and returns the address of the object.

        This function returns the pointer to the newly created object.
        If the object is not successfully created, it returns NULL.

*/
// *****************************************************************************
GFX_GOL_SURFACE  *GFX_GOL_SurfaceCreate(
                                SYS_MODULE_INDEX    gfxIndex,
                                uint16_t            ID,
                                uint16_t            left,
                                uint16_t            top,
                                uint16_t            right,
                                uint16_t            bottom,
                                uint16_t            state,
                                GFX_GOL_SURFACE_CALLBACK_FUNC callback,
                                GFX_RESOURCE_HDR    *pImage,
                                GFX_GOL_OBJ_SCHEME  *pScheme)
{
    GFX_GOL_SURFACE  *pS;

    pS = (GFX_GOL_SURFACE *)GFX_malloc(sizeof(GFX_GOL_SURFACE));
    if(pS == NULL)
        return (pS);

    pS->hdr.gfxIndex = gfxIndex;
    pS->hdr.ID      = ID;
    pS->hdr.pNxtObj = NULL;
    pS->hdr.type    = GFX_GOL_SURFACE_TYPE;
    pS->hdr.left    = left;
    pS->hdr.top     = top;
    pS->hdr.right   = right;
    pS->hdr.bottom  = bottom;
    pS->pImage      = pImage;
    pS->hdr.state   = state;

    pS->hdr.DrawObj = GFX_GOL_SurfaceDraw;       // draw function
    pS->hdr.FreeObj = NULL;                     // free function
    pS->hdr.actionGet = GFX_GOL_SurfaceActionGet;// action get function
    pS->hdr.actionSet = NULL;                   // default action function

    pS->callback = callback;

    // Set the style scheme to be used
    pS->hdr.pGolScheme = pScheme;
 
    GFX_GOL_ObjectAdd(pS->hdr.gfxIndex, (GFX_GOL_OBJ_HEADER *)pS);

    return (pS);
}


// *****************************************************************************
/*  Function:
    GFX_GOL_TRANSLATED_ACTION GFX_GOL_SurfaceActionGet(
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage);

    Summary:
        This function evaluates the message from a user if the
        message will affect the object or not.

    Description:
        This function evaluates the message from a user if the
        message will affect the object or not.
 
*/
// *****************************************************************************
GFX_GOL_TRANSLATED_ACTION GFX_GOL_SurfaceActionGet(
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage)
{
    GFX_GOL_SURFACE *pS;


    pS = (GFX_GOL_SURFACE *)pObject;

    // Evaluate if the message is for the Surface
    // Check if disabled first
    if(GFX_GOL_ObjectStateGet(pS, GFX_GOL_SURFACE_DISABLED_STATE))
        return (GFX_GOL_OBJECT_ACTION_INVALID);

#ifndef GFX_CONFIG_USE_TOUCHSCREEN_DISABLE
    
    uint16_t embossSize;
    
    if(pMessage->type == TYPE_TOUCHSCREEN)
    {

        embossSize = pS->hdr.pGolScheme->EmbossSize;

//        // Check if it falls in the client area
//        if(
//            (pS->hdr.left   + embossSize < pMessage->param1) &&
//            (pS->hdr.right  - embossSize > pMessage->param1) &&
//            (pS->hdr.top    + embossSize +
//                    GOL_OBJECT_SURFACE_TITLE_HEIGHT  < pMessage->param2) &&
//            (pS->hdr.bottom - embossSize > pMessage->param2)
//        )
//        {
//            return (GFX_GOL_SURFACE_ACTION_CLIENT);
//        }
    }

#endif
    return (GFX_GOL_OBJECT_ACTION_INVALID);
}

// *****************************************************************************
/*  Function:
    GFX_STATUS GFX_GOL_SurfaceDraw(void *pObject)

    Summary:
        This function renders the object on the screen based on the
        current state of the object.

    Description:
        This function renders the object on the screen based on the
        current state of the object. Location of the object is
        determined by the left, top, right and bottom parameters.
        The colors used are dependent on the state of the object.
        The font used is determined by the style scheme set.

        The text on the face of the GFX_GOL_SURFACE is drawn on top of
        the bitmap. Text alignment based on the alignment parameter
        set on the object.

        When rendering objects of the same type, each object
        must be rendered completely before the rendering of the
        next object is started. This is to avoid incomplete
        object rendering.

        Normally, application will just call GFX_GOL_ObjectListDraw()
        to allow the Graphics Library to manage all object rendering.
        See GFX_GOL_ObjectListDraw() for more information on object rendering.

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_SurfaceDraw(void *pObject)
{

typedef enum
{
    SURF_DRAW_IDLE,
    SURF_DRAW_HIDE,
    SURF_DRAW_SET_CLIENT_AREA,
    SURF_DRAW_CLIENT_AREA,
    SURF_DRAW_APP_FUNCTION
} GFX_GOL_SURFACE_DRAW_STATES;

//           int16_t         temp;
           GFX_GOL_SURFACE  *pS = (GFX_GOL_SURFACE *)pObject;
    static GFX_GOL_SURFACE_DRAW_STATES state = SURF_DRAW_IDLE;
    static uint16_t        embossSize;
    GFX_PREEMPTION_LEVEL    preemptionLevel =
                                GFX_GOL_PreemptionLevelGet(pS->hdr.gfxIndex);

    while(1)
    {
    
        if (GFX_RenderStatusGet(pS->hdr.gfxIndex) == GFX_STATUS_BUSY_BIT)
            return (GFX_STATUS_FAILURE);

        switch(state)
        {
            case SURF_DRAW_IDLE:
#ifdef GFX_LIB_CFG_USE_BISTABLE_DISPLAY_GOL_AUTO_REFRESH
                GFX_DRIVER_SetupDrawUpdate( pS->hdr.left,
                                            pS->hdr.top,
                                            pS->hdr.right,
                                            pS->hdr.bottom);
#endif
                if(GFX_GOL_ObjectStateGet(
                        pS,
                        GFX_GOL_SURFACE_HIDE_STATE))
                {
                    GFX_ColorSet(pS->hdr.gfxIndex, pS->hdr.pGolScheme->CommonBkColor);
                    state = SURF_DRAW_HIDE;
                    // no break; here so it falls through to SURF_DRAW_HIDE
                }

                if (GFX_GOL_ObjectStateGet(
                        pS,
                        GFX_GOL_SURFACE_DRAW_APP_STATE))
                {
                       state = SURF_DRAW_APP_FUNCTION;
                       break;
                }

                embossSize = pS->hdr.pGolScheme->EmbossSize;
                if(GFX_GOL_ObjectStateGet(
                        pS,
                        GFX_GOL_SURFACE_DRAW_CLIENT_STATE))
                {
                    state = SURF_DRAW_SET_CLIENT_AREA;
                    break;
                }
                else
                {
#ifdef GFX_LIB_CFG_USE_BISTABLE_DISPLAY_GOL_AUTO_REFRESH
                    GFX_DRIVER_CompleteDrawUpdate(  pS->hdr.left,
                                                    pS->hdr.top,
                                                    pS->hdr.right,
                                                    pS->hdr.bottom);
#endif
                    return (GFX_STATUS_SUCCESS);
                }

            case SURF_DRAW_HIDE:

                // Hide the object (remove from screen)
                if (GFX_GOL_ObjectHideDraw(pS->hdr.gfxIndex, &(pS->hdr)) != GFX_STATUS_SUCCESS)
                    return (GFX_STATUS_FAILURE);

                state = SURF_DRAW_IDLE;

#ifdef GFX_LIB_CFG_USE_BISTABLE_DISPLAY_GOL_AUTO_REFRESH
                GFX_DRIVER_CompleteDrawUpdate(   pS->hdr.left,
                                                 pS->hdr.top,
                                                 pS->hdr.right,
                                                 pS->hdr.bottom);
#endif
                return (GFX_STATUS_SUCCESS);
				
            case SURF_DRAW_SET_CLIENT_AREA:
                
                GFX_LineStyleSet(pS->hdr.gfxIndex, GFX_LINE_STYLE_THIN_SOLID);
                
                // set panel parameters
                GFX_GOL_PanelParameterSet(pS->hdr.gfxIndex,
                        pS->hdr.left,
                        pS->hdr.top,
                        pS->hdr.right,
                        pS->hdr.bottom,
                        0,
                        pS->hdr.pGolScheme->CommonBkColor,
                        pS->hdr.pGolScheme->EmbossLtColor,
                        pS->hdr.pGolScheme->EmbossDkColor,
                        pS->hdr.pGolScheme->pCommonBkImage,
                        pS->hdr.pGolScheme->fillStyle,
                        embossSize);

                // set the background information
                GFX_GOL_PanelBackgroundSet(pS->hdr.gfxIndex, &pS->hdr);

                state = SURF_DRAW_CLIENT_AREA;

                if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
                {
                    return GFX_STATUS_FAILURE;
                }
                
            case SURF_DRAW_CLIENT_AREA:

                if(GFX_GOL_PanelDraw(pS->hdr.gfxIndex) != GFX_STATUS_SUCCESS)
                {
                    return (GFX_STATUS_FAILURE);
                }

                else
		{
                    state = SURF_DRAW_APP_FUNCTION;
#ifdef GFX_LIB_CFG_USE_BISTABLE_DISPLAY_GOL_AUTO_REFRESH
                    GFX_DRIVER_CompleteDrawUpdate(  pS->hdr.left,
                                                    pS->hdr.top,
                                                    pS->hdr.right,
                                                    pS->hdr.bottom);
#endif
//                    return (GFX_STATUS_SUCCESS);
                }
                // no break here since the next state is what we want

                if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
                {
                    return GFX_STATUS_FAILURE;
                }

            case SURF_DRAW_APP_FUNCTION:

                pS->callback();
                state = SURF_DRAW_IDLE;
                return (GFX_STATUS_SUCCESS);
                // no break here since the next state is what we want

        }   //end of switch
    }       //end of while
}

