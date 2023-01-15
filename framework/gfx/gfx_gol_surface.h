/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol_surface.h

  Summary:
    This is the header file for the surface object of the GOL.

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

#ifndef _GFX_GOL_SURFACE_H
// DOM-IGNORE-BEGIN
    #define _GFX_GOL_SURFACE_H
// DOM-IGNORE-END

#include <stdint.h>
#include "gfx/gfx_gol.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* 
    <GROUP gol_types>

    Typedef:
        GFX_GOL_SURFACE_STATE

    Summary:
        Specifies the different states of the Surface object.

    Description:
        This enumeration specifies the different states of the Surface
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
    // Property bit for focus state.
    GFX_GOL_SURFACE_FOCUSED_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0001 /*DOM-IGNORE-END*/,
    // Property bit for disabled state.
    GFX_GOL_SURFACE_DISABLED_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0002 /*DOM-IGNORE-END*/,
    // Draw bit to indicate title area must be redrawn.
    GFX_GOL_SURFACE_DRAW_APP_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0200 /*DOM-IGNORE-END*/,
    // Draw bit to indicate client area must be redrawn.
    GFX_GOL_SURFACE_DRAW_CLIENT_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x4000 /*DOM-IGNORE-END*/,
    // Draw bit to indicate object must be redrawn.
    GFX_GOL_SURFACE_DRAW_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x6000 /*DOM-IGNORE-END*/,
    // Draw bit to indicate object must be removed from screen.
    GFX_GOL_SURFACE_HIDE_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x8000 /*DOM-IGNORE-END*/,
} GFX_GOL_SURFACE_STATE;

// *****************************************************************************
/*
    <GROUP gol_objects_rendering>

    Function:
        typedef GFX_STATUS (*GFX_GOL_SURFACE_CALLBACK_FUNC) (void);

    Summary:
        Surface callback function definition. This application defined
        function allows the application to perform application specific
        rendering.

    Description:
        This callback function is implemented by the application.
        This is called inside the GFX_GOL_ObjectListDraw()
        function when the drawing of objects in the active list is
        completed.

        Any application specific rendering must be performed on this
        callback function so no object rendering will be affected by the
        application calls to primitive rendering functions.
        Application setting the drawing color, line style, fill style,
        text string  cursor position and current font will not affect
        the object rendering. This is also the safe place to modify the
        active list.

        When the application has performed its own primitive rendering
        calls, this function must return true to inform
        the GFX_GOL_ObjectListDraw() that it is done rendering and
        checking for object drawing or redrawing can continue.

    Precondition:
        None.

    Parameters:
        None.

    Returns:
        true  - is returned when application rendering is done.
        false - is returned when application rendering is not
                yet finished.

    Example:
        None.

*/
// *****************************************************************************
typedef GFX_STATUS (*GFX_GOL_SURFACE_CALLBACK_FUNC) (void);
    
// *****************************************************************************
/* 
    <GROUP gol_types>

    Typedef:
        GFX_GOL_SURFACE

    Summary:
        Defines the structure used for the Surface object.
        
    Description:
        Defines the parameters required for a Surface object.
        Object is drawn with the defined shape parameters and values 
        set on the given fields. 
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    GFX_GOL_OBJ_HEADER              hdr;        // Generic header for all Objects
                                        // (see GFX_GOL_OBJ_HEADER).
    uint16_t                        textHeight; // Precomputed text height
    GFX_XCHAR                       *pText;     // Pointer to the title text
    GFX_GOL_SURFACE_CALLBACK_FUNC   callback;  // text alignment
    GFX_RESOURCE_HDR                *pImage;    // Pointer to the image for the title bar
} GFX_GOL_SURFACE;

// *****************************************************************************
/*  
    <GROUP gol_surface_object>
    
    Function:
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

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        ID - Unique user defined ID for the object instance.
        left - Left most position of the object.
        top - Top most position of the object.
        right - Right most position of the object.
        bottom - Bottom most position of the object.
        state - Sets the initial state of the object.
        pImage - Pointer to the image used on the face of the object.
        pText - Pointer to the text of the object.
        alignment - text alignment of the text used in the object.
        pScheme - Pointer to the style scheme used.

    Returns:
        Pointer to the newly created object.

    Example:
        <code>
            GFX_GOL_OBJ_SCHEME *pScheme;
            GFX_GOL_SURFACE *pSurface;
            GFX_GOL_SURFACE_STATE state;

            // assume pScheme is initialized to a scheme in memory.
            state = GFX_GOL_SURFACE_DRAW_STATE;


            pSurface  = GFX_GOL_SurfaceCreate(
                   gfxIndex,
                   1,			// ID
                   0,0,GFX_Primitive_MaxXGet(),GFX_Primitive_MaxYGet(),	// whole screen dimension
                   state,					// set state to draw all
                   (char*)myIcon,				// icon
                   "Place Title Here.",		                // text
                   NULL);					// use default GOL scheme

            if (pSurface == NULL)
                return 0;

            return 1;
        </code>

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
                                GFX_GOL_OBJ_SCHEME  *pScheme);

//void GFX_GOL_SurfaceRedrawSet(GFX_GOL_SURFACE *pObject);

// *****************************************************************************
/*  
    <GROUP gol_surface_object>
    
    Function:
        void GFX_GOL_SurfaceImageSet(
                                GFX_GOL_SURFACE *pObject,
                                GFX_RESOURCE_HDR *pImage)

    Summary:
        This function sets the image used in the object.

    Description:
        This function sets the image used in the object.

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - pointer to the object.
        pImage - pointer to the image to be set.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_SurfaceImageSet(pObject, image) \
                                (((GFX_GOL_SURFACE *)pObject)->pImage = image)

// *****************************************************************************
/*  
    <GROUP gol_surface_object>
    
    Function:
        GFX_RESOURCE_HDR *GFX_GOL_SurfaceImageGet(
                                GFX_GOL_SURFACE *pObject)

    Summary:
        This function gets the image used.

    Description:
        This function gets the image used.

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - pointer to the object.

    Returns:
        Pointer to the image resource.

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_SurfaceImageGet(pObject, pImage) \
                                (((GFX_GOL_SURFACE *)pObject)->pImage)

// *****************************************************************************
/*  
    <GROUP gol_window_object>
    
    Function:
        GFX_GOL_TRANSLATED_ACTION GFX_GOL_SurfaceActionGet(
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage);

    Summary:
        This function evaluates the message from a user if the
        message will affect the object or not.

    Description:
        This function evaluates the message from a user if the
        message will affect the object or not. The table below enumerates
        the action for each event of the touch screen and keyboard inputs.

	<TABLE>
    	Translated Message                  Input Source    Events                                      Description
     	##################                  ############    ######                                      ###########
        GFX_GOL_SURFACE_ACTION_TITLE         Touch Screen    EVENT_PRESS, EVENT_RELEASE, EVENT_MOVE      If events occurs and the x,y position falls in the TITLE area of the surface
        GFX_GOL_SURFACE_ACTION_CLIENT        Touch Screen    EVENT_PRESS, EVENT_RELEASE, EVENT_MOVE      If events occurs and the x,y position falls in the CLIENT area of the surface
        GFX_GOL_OBJECT_ACTION_INVALID       Any             Any                                         If the message did not affect the object.
	</TABLE>

    Precondition:
        Object must exist in memory.

    Parameters:
        pObject - The pointer to the object where the message will be
                   evaluated to check if the message will affect the object.
        pMessage - Pointer to the the message from the user interface.

    Returns:
        - GFX_GOL_SURFACE_ACTION_CLIENT - Surface client area selected action ID.
        - GFX_GOL_SURFACE_ACTION_TITLE  - Surface title bar selected action ID.

    Example:
        None.

*/
// *****************************************************************************
GFX_GOL_TRANSLATED_ACTION GFX_GOL_SurfaceActionGet(
                                void *pObject,
                                GFX_GOL_MESSAGE *pMessage);


// *****************************************************************************
/*  
    <GROUP gol_surface_object>
    
    Function:
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
GFX_STATUS GFX_GOL_SurfaceDraw(void *pObject);

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_GOL_SURFACE_H
