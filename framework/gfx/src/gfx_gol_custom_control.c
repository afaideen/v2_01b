/*******************************************************************************
  Graphics Library Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol_custom_control.c

  Summary:
    This implements the custom control object of the GOL.

  Description:
    Refer to Microchip Graphics Library for complete documentation of the
    CustomControl Object.
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

#include "system_config.h"
#include "system_definitions.h"

/*********************************************************************
 * Function: GFX_GOL_CUSTOMCONTROL* GFX_GOL_CustomControlCreate(
 *                                        SYS_MODULE_INDEX gfxIndex,
 *                                        uint16_t ID,
 *                                        uint16_t left,
 *                                        uint16_t top,
 *                                        uint16_t right,
 *                                        uint16_t bottom,
 *                                        uint16_t state,
 *                                        GFX_GOL_OBJ_SCHEME *pScheme )
 *
 * Overview: creates the object
 *
 * TODO: Documentation needed
 ********************************************************************/
GFX_GOL_CUSTOMCONTROL *GFX_GOL_CustomControlCreate
(
 SYS_MODULE_INDEX           gfxIndex,
 uint16_t ID,
 uint16_t left,
 uint16_t top,
 uint16_t right,
 uint16_t bottom,
 uint16_t state,
 GFX_GOL_OBJ_SCHEME *pScheme
 )
{
    GFX_GOL_CUSTOMCONTROL *pCc = NULL;

    pCc = (GFX_GOL_CUSTOMCONTROL *) malloc(sizeof (GFX_GOL_CUSTOMCONTROL));
    if (pCc == NULL)
        return (pCc);

    //pCc->instance = instance;
    pCc->hdr.gfxIndex = gfxIndex;
    pCc->hdr.ID = ID;
    pCc->hdr.pNxtObj = NULL;
    pCc->hdr.type = GFX_GOL_CUSTOM_TYPE;
    pCc->hdr.left = left;
    pCc->hdr.top = top;
    pCc->hdr.right = right;
    pCc->hdr.bottom = bottom;
    pCc->pos = (pCc->hdr.top + pCc->hdr.bottom) >> 1; // position
    pCc->prevPos = pCc->hdr.bottom - pCc->hdr.pGolScheme->EmbossSize; // previos position
    pCc->hdr.state = state; // set state
    pCc->hdr.DrawObj = GFX_GOL_CustomControlDraw;
    pCc->hdr.actionGet = GFX_GOL_CustomControlActionGet; // message function
    // default message function
    pCc->hdr.actionSet = GFX_GOL_CustomControlActionSet; 
    pCc->hdr.FreeObj = NULL; // default message function

    // Set the style scheme to be used
    pCc->hdr.pGolScheme = (GFX_GOL_OBJ_SCHEME *) pScheme;

    GFX_GOL_ObjectAdd(pCc->hdr.gfxIndex, (GFX_GOL_OBJ_HEADER *) pCc);

    return (pCc);
}

/*********************************************************************
 * Function: uint16_t GFX_GOL_CustomControlActionGet(void *pObj, 
 *                                                    GFX_GOL_MESSAGE *pMsg )
 *
 * Overview: translates the GOL message to this control
 *
 ********************************************************************/
GFX_GOL_TRANSLATED_ACTION GFX_GOL_CustomControlActionGet( void *pObj,
                                                          GFX_GOL_MESSAGE *pMsg)
{

    GFX_GOL_CUSTOMCONTROL *pCc;

    pCc = (GFX_GOL_CUSTOMCONTROL *) pObj;

    // Check if disabled first
    if (GFX_GOL_ObjectStateGet(pCc, GFX_GOL_CUSTOMCONTROL_DISABLED_STATE))
        return (GFX_GOL_OBJECT_ACTION_INVALID);

#ifdef USE_TOUCHSCREEN
    if (pMsg->type == TYPE_TOUCHSCREEN)
    {

        // Check if it falls in the control area
        if
            (
             (pCc->hdr.left + pCc->hdr.pGolScheme->EmbossSize < pMsg->param1) &&
             (pCc->hdr.right - pCc->hdr.pGolScheme->EmbossSize > pMsg->param1)&&
             (pCc->hdr.top + pCc->hdr.pGolScheme->EmbossSize < pMsg->param2)  &&
             (pCc->hdr.bottom - pCc->hdr.pGolScheme->EmbossSize > pMsg->param2)
             )
        {
            return (GFX_GOL_CUSTOMCONTROL_ACTION_SELECTED);
        }
    }

#endif
    return (GFX_GOL_OBJECT_ACTION_INVALID);
}

/*********************************************************************
 * Function: void GFX_GOL_CustomControlActionSet( uint16_t translatedMsg,
                                          void *pObj, GFX_GOL_MESSAGE* pMsg )
 *
 * Overview: changes the state of the object by default
 *
 ********************************************************************/
void GFX_GOL_CustomControlActionSet( GFX_GOL_TRANSLATED_ACTION translatedMsg,
                                     void *pObj, GFX_GOL_MESSAGE *pMsg )
{
    GFX_GOL_CUSTOMCONTROL *pCc;

    pCc = (GFX_GOL_CUSTOMCONTROL *) pObj;

    pCc->pos = pMsg->param2;
    GFX_GOL_ObjectStateSet(pCc, GFX_GOL_CUSTOMCONTROL_DRAW_BAR_STATE);
}

/*********************************************************************
 * Function: uint16_t GFX_GOL_CustomControlDraw(GFX_GOL_CUSTOMCONTROL *pCc)
 *
 * Output: returns the status of the drawing
 *		  0 - not complete
 *         1 - done
 *
 * Overview: draws control
 *
 ********************************************************************/
GFX_STATUS GFX_GOL_CustomControlDraw(void *pObj)
{

    typedef enum
    {
        REMOVE,
        BOX_DRAW,
        RUN_DRAW,
        BAR_DRAW,
        GRID_DRAW,
        WAIT
    } GFX_GOL_CUSTOMCONTROL_DRAW_STATES;

    static GFX_GOL_CUSTOMCONTROL_DRAW_STATES state = REMOVE;
    static short          counter;
    static short          delta;
    static GFX_STATUS     index = GFX_STATUS_SUCCESS;
    GFX_GOL_CUSTOMCONTROL *pCc = (GFX_GOL_CUSTOMCONTROL *) pObj;
    GFX_PREEMPTION_LEVEL  preemptionLevel =
                   GFX_GOL_PreemptionLevelGet(pCc->hdr.gfxIndex);

    switch (state)
    {
        case REMOVE:
            
            if (GFX_GOL_ObjectStateGet(pCc, GFX_GOL_CUSTOMCONTROL_HIDE_STATE))
            {
                GFX_ColorSet( pCc->hdr.gfxIndex,
                              pCc->hdr.pGolScheme->CommonBkColor );
                
                if( GFX_RectangleFillDraw( pCc->hdr.gfxIndex,
                                           pCc->hdr.left,
                                           pCc->hdr.top,
                                           pCc->hdr.right,
                                           pCc->hdr.bottom) !=
                                           GFX_STATUS_SUCCESS )
                {
                    return (GFX_STATUS_FAILURE);
                }

                return (GFX_STATUS_SUCCESS);
            }

            state = BOX_DRAW;
            
            if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
            {
                return GFX_STATUS_FAILURE;
            }

        case BOX_DRAW:
            
            if (GFX_GOL_ObjectStateGet(pCc, GFX_GOL_CUSTOMCONTROL_DRAW_STATE))
            {
                GFX_GOL_PanelParameterSet
                        ( pCc->hdr.gfxIndex,
                         pCc->hdr.left,
                         pCc->hdr.top,
                         pCc->hdr.right,
                         pCc->hdr.bottom,
                         0,
                         pCc->hdr.pGolScheme->Color0,
                         pCc->hdr.pGolScheme->EmbossDkColor,
                         pCc->hdr.pGolScheme->EmbossLtColor,
                         NULL,
                         GFX_FILL_STYLE_COLOR,
                         pCc->hdr.pGolScheme->EmbossSize
                         );

                state = RUN_DRAW;

                case RUN_DRAW:
                if (GFX_GOL_PanelDraw(pCc->hdr.gfxIndex)==GFX_STATUS_FAILURE)
                    return (GFX_STATUS_FAILURE);
            }

            state = BAR_DRAW;

            if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
            {
                return GFX_STATUS_FAILURE;
            }

        case BAR_DRAW:
            
            if (pCc->prevPos > pCc->pos)
            {
                GFX_ColorSet(pCc->hdr.gfxIndex, pCc->hdr.pGolScheme->Color1);
                if ( GFX_RectangleFillDraw( pCc->hdr.gfxIndex,
                                            pCc->hdr.left +
                                            pCc->hdr.pGolScheme->EmbossSize,
                                            pCc->pos,
                                            pCc->hdr.right -
                                            pCc->hdr.pGolScheme->EmbossSize,
                                            pCc->prevPos ) !=
                                            GFX_STATUS_SUCCESS )
                {
                    return GFX_STATUS_FAILURE;
                }
            }
            else
            {
                GFX_ColorSet(pCc->hdr.gfxIndex, pCc->hdr.pGolScheme->Color0);
                if ( GFX_RectangleFillDraw( pCc->hdr.gfxIndex,
                                            pCc->hdr.left +
                                            pCc->hdr.pGolScheme->EmbossSize,
                                            pCc->prevPos,
                                            pCc->hdr.right -
                                            pCc->hdr.pGolScheme->EmbossSize,
                                            pCc->pos ) != GFX_STATUS_SUCCESS )
                {
                    return GFX_STATUS_FAILURE;
                }
            }

            GFX_ColorSet(pCc->hdr.gfxIndex, pCc->hdr.pGolScheme->TextColor0);
            counter = 1;
            delta = ( pCc->hdr.bottom - pCc->hdr.top -
                     (2 * pCc->hdr.pGolScheme->EmbossSize)) >> 3;
            state = GRID_DRAW;

            if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
            {
                return GFX_STATUS_FAILURE;
            }
            
        case GRID_DRAW:
            
            while (counter < 8)
            {
                index =  GFX_RectangleFillDraw
                         (
                          pCc->hdr.gfxIndex,
                          pCc->hdr.left + pCc->hdr.pGolScheme->EmbossSize,
                          pCc->hdr.top + pCc->hdr.pGolScheme->EmbossSize +
                          counter * delta,
                          pCc->hdr.right - pCc->hdr.pGolScheme->EmbossSize,
                          pCc->hdr.top + pCc->hdr.pGolScheme->EmbossSize +
                          counter * delta
                          );
                         
                if(index == GFX_STATUS_FAILURE)
                {
                    return (GFX_STATUS_FAILURE);
                }
                counter++;
            }

            state = WAIT;

            if( GFX_PREEMPTION_LEVEL_2 == preemptionLevel )
            {
                return GFX_STATUS_FAILURE;
            }
            
        case WAIT:
//            if(index->type != DONE) // Wait until renderring is complete
//            {
//                 return(0);
//            }
            pCc->prevPos = pCc->pos;
            state = REMOVE;
            return (GFX_STATUS_SUCCESS);
    }

    return (GFX_STATUS_SUCCESS);
}
