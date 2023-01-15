/*******************************************************************************
  Graphics Library Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol_custom_control.c

  Summary:
    This is the header file for the custom control object of the GOL.

  Description:
    Refer to Microchip Graphics Library for complete documentation of the
    CustomControl Object.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _GFX_GOL_CUSTOM_CONTROL_H
    #define _GFX_GOL_CUSTOM_CONTROL_H
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <gfx/gfx_gol.h>

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* GFX GOL custom control state.

    Typedef:
        GFX_GOL_CUSTOMCONTROL_STATE

    Summary:
        Specifies the different states of the Custom Control object.

    Description:
        This enumeration specifies the different states of the Custom Control
        object used in the library.

        For the Property State bits, more than one of these values may be
        OR'd together to create a complete property state.

        For the Draw State bits, hide draw bit has higher priority than the
        draw bit. Any of these two can be combined with other draw bits to
        create a complete draw state.

        To test a value of any of the state types, the bit of interest must be
        AND'ed with value and checked to see if the result is non-zero.

    Remarks:
        None.

*/
// *****************************************************************************
typedef enum
{
    /* Property bit to indicate object is disabled. */
    GFX_GOL_CUSTOMCONTROL_DISABLED_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0002 /*DOM-IGNORE-END*/,

    /* Draw bit to indicate focus must be redrawn. */
    GFX_GOL_CUSTOMCONTROL_DRAW_BAR_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x2000 /*DOM-IGNORE-END*/,

    /* Draw bit to indicate object must be redrawn. */
    GFX_GOL_CUSTOMCONTROL_DRAW_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x4000 /*DOM-IGNORE-END*/,

    /* Draw bit to indicate object must be removed from screen. */
    GFX_GOL_CUSTOMCONTROL_HIDE_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x8000 /*DOM-IGNORE-END*/,
            
} GFX_GOL_CUSTOMCONTROL_STATE;

// *****************************************************************************
/* GFX GOL custom control.

    Typedef:
        GFX_GOL_CUSTOMCONTROL

    Summary:
        Defines the structure used for the Custom Control object.

    Description:
        Defines the structure used for the Custom Control object.
        Object is drawn with the defined shape parameters and values
        set on the given fields.

    Remarks:
        None.

*/
// *****************************************************************************
typedef struct {

    /* Generic header for all Objects (see GFX_OBJ_HEADER). */
    GFX_GOL_OBJ_HEADER hdr;

    /* instance index */
    uint8_t instance;

    /* instance unique ID */
    uint16_t ID;

    /* a pointer to the next object in the linked list */
    void *pNxtObj;

    /* must be set to OBJ_CUSTOM */
    GFX_GOL_OBJ_TYPE type;

    /* state */
    uint16_t state;

    /* left border */
    uint16_t left;

    /* top border */
    uint16_t top;

    /* right border */
    uint16_t right;

    /* bottom border */
    uint16_t bottom;

    /* the style scheme used */
    GFX_GOL_OBJ_SCHEME *pGolScheme;

    /* custom control draw function */
    DRAW_FUNC draw;

    /* function pointer to the object free function */
    FREE_FUNC FreeObj;

    /* function pointer to the object message function */
    ACTIONGET_FUNC actionGet;

    /* function pointer to the object default message function */
    ACTIONSET_FUNC actionSet;

    /* current position */
    uint16_t pos;

    /* previous position */
    uint16_t prevPos;
    
} GFX_GOL_CUSTOMCONTROL;

// *****************************************************************************
/*  GFX GOL custom control get position

    Function:
        GFX_ALIGNMENT GFX_GOL_CustomControlGetPos(
                                GFX_GOL_CUSTOMCONTROL *pObject)

    Summary:
        This function returns the position of the control.

    Description:
        This function returns the position of the object.

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - pointer to the object.

    Returns:
        The position of the object.

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_CustomControlGetPos(pCc)   pCc->pos

// *****************************************************************************
/*  GFX GOL custom control set position.

    Function:
        void GFX_GOL_CustomControlSetPos(
                                GFX_GOL_CUSTOMCONTROL *pObject, uint16_t pos)

    Summary:
        This function sets the position of the control.

    Description:
        This function sets the position of the object.

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - pointer to the object.

    Returns:
        none

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_CustomControlSetPos(pCc, position) pCc->pos = position

// *****************************************************************************
/*  GFX GOL custom control create.
 
    Function:
        GFX_GOL_CUSTOMCONTROL  *GFX_GOL_CustomControlCreate(
                                SYS_MODULE_INDEX    clientIndex,
                                uint16_t ID,
                                uint16_t left,
                                uint16_t top,
                                uint16_t right,
                                uint16_t bottom,
                                uint16_t state,
                                GFX_GOL_OBJ_SCHEME * pScheme)
    Summary:
        This function creates a GFX_GOL_CUSTOMCONTROL object with the parameters
        given. It automatically attaches the new object into a global
        linked list of objects and returns the address of the object.

    Description:
        This function creates a GFX_GOL_CUSTOMCONTROL object with the parameters
        given. It automatically attaches the new object into a global
        linked list of objects and returns the address of the object.

        This function returns the pointer to the newly created object.
        If the object is not successfully created, it returns NULL.

        The behavior of GFX_GOL_CustomControlCreate() will be undefined if one
        of the following is true:
        - left >= right
        - top >= bottom
        - pScheme is not pointing to a GFX_GOL_OBJ_SCHEME
        - pText is an unterminated string

    Precondition:
        None.

    Parameters:
        clientIndex - client user instance
        ID - Unique user defined ID for the object instance.
        left - Left most position of the object.
        top - Top most position of the object.
        right - Right most position of the object.
        bottom - Bottom most position of the object.
        state - Sets the initial state of the object.
        pText - Pointer to the text of the object.
        alignment - text alignment of the text used in the object.
        pScheme - Pointer to the style scheme used.

    Returns:
        Pointer to the newly created object.

    Example:
        <code>
            GFX_GOL_OBJ_SCHEME  *pScheme;
            GFX_GOL_CUSTOMCONTROL    *pCc[2];

            pCc[0] = GFX_GOL_CustomControlCreate(
                                GFX_INDEX_0,
                                ID_CUSTOM1, // ID
                                30,
                                0,
                                150,
                                120, // dimension
                                // will be displayed after creation
                                GFX_GOL_CUSTOMCONTROL_DRAW_STATE, 
                                pScheme
             );               

        </code>

*/
// *****************************************************************************
GFX_GOL_CUSTOMCONTROL * GFX_GOL_CustomControlCreate(
                                                  SYS_MODULE_INDEX clientIndex,
                                                  uint16_t             ID,
                                                  uint16_t             left,
                                                  uint16_t             top,
                                                  uint16_t             right,
                                                  uint16_t             bottom,
                                                  uint16_t             state,
                                                  GFX_GOL_OBJ_SCHEME * pScheme);

// *****************************************************************************
/*  GFX GOL custom control action get.

    Function:
        GFX_GOL_TRANSLATED_ACTION GFX_GOL_CustomControlActionGet(
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage)

    Summary:
        This function evaluates the message from a user if the
        message will affect the object or not.

    Description:
        This function evaluates the message from a user if the
        message will affect the object or not. The table below enumerates
        the action for each event of the touch screen and keyboard inputs.

	<TABLE>
    	Translated Message                      Input Source    Set/Clear State Bit         Description
     	##################                      ############    #######################     #####################################################################################
        GFX_GOL_CUSTOMCONTROL_ACTION_SELECTED   Touch Screen    EVENT_PRESS, EVENT_RELEASE  If events occurs and the x,y position falls in the area of the Digital Meter.
        GFX_GOL_OBJECT_ACTION_INVALID           Any             Any                         If the message did not affect the object.
         </TABLE>

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - The pointer to the object where the message will be
                   evaluated to check if the message will affect the object.
        pMessage - Pointer to the the message from the user interface.

    Returns:
        - GFX_GOL_CUSTOMCONTROL_ACTION_SELECTED - Object is selected.
        - GFX_GOL_OBJECT_ACTION_INVALID - Object is not affected

    Example:
        None.

*/
// *****************************************************************************
GFX_GOL_TRANSLATED_ACTION GFX_GOL_CustomControlActionGet(void *pObj,
                                                         GFX_GOL_MESSAGE *pMsg);

// *****************************************************************************
/*  GFX GOL custom control action set.

    Function:
        void GFX_GOL_CustomControlActionSet(
                                GFX_GOL_TRANSLATED_ACTION translatedMsg,
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage)

    Summary:
        This function performs the state change of the object based on the
        translated action.

    Description:
        This function performs the state change of the object based on the
        translated action. This change can be overridden by the application
        using the application defined GFX_GOL_MESSAGE_CALLBACK_FUNC.
        When the user message is determined to affect the object, application
        can perform the state change in the message callback function.
        The following state changes are supported:
 	<TABLE>
    	Translated Message                     Input Source	Set/Clear State Bit                         Description
     	##################                     ############	###################                         ###########
        GFX_GOL_CUSTOMCONTROL_ACTION_SELECTED  Touch Screen,   Set GFX_GOL_RADIOBUTTON_DRAW_STATE,         Depending on the current value of RB_CHECKED Check Box will be redrawn.
                                               Keyboard        Set/Clear GFX_GOL_RADIOBUTTON_CHECKED_STATE
	</TABLE>

    Precondition:
        Object must exist in memory.

    Parameters:
        translatedMsg - The action of the object based on the message.
        pObject       - The pointer to the object whose state will be modified.
        pMessage      - The pointer to the original message.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
void GFX_GOL_CustomControlActionSet( GFX_GOL_TRANSLATED_ACTION translatedMsg,
                                     void *pObj,
                                     GFX_GOL_MESSAGE *pMsg );

// *****************************************************************************
/*  GFX GOL custom control draw.

    Function:
        GFX_STATUS GFX_GOL_CustomControlDraw(void *pObject)

    Summary:
        This function renders the object on the screen based on the
        current state of the object.

    Description:
        This function renders the object on the screen based on the
        current state of the object. Location of the object is
        determined by the left, top, right and bottom parameters.
        The colors used are dependent on the state of the object.
        The font used is determined by the style scheme set.

        When rendering objects of the same type, each object
        must be rendered completely before the rendering of the
        next object is started. This is to avoid incomplete
        object rendering.

        Normally, application will just call GFX_GOL_ObjectListDraw()
        to allow the Graphics Library to manage all object rendering.
        See GFX_GOL_ObjectListDraw() for more information on object rendering.

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - Pointer to the object.

    Returns:
        GFX_STATUS_SUCCESS - When the object rendering is finished.
        GFX_STATUS_FAILURE - When the object rendering is not yet finished.
                             Application needs to call this rendering
                             function again to continue the rendering.

    Example:
        None.

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_CustomControlDraw(void *pObj);

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_GOL_CUSTOM_CONTROL_H
