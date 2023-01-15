/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_gol.h

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

#ifndef _GFX_GOL_H
// DOM-IGNORE-BEGIN
    #define _GFX_GOL_H
// DOM-IGNORE-END

/*********************************************************************
 * Section: Includes
 *********************************************************************/
#include <stdbool.h>
#include <stdint.h>

//#include "system_config.h"
#include "gfx/gfx_primitive.h"
#include "gfx/gfx_colors.h"
#include "gfx/gfx_gol_scheme.h"
//#include "gfx/gfx_common.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* GFX GOL Machine States

   Summary
    Defines the various states that can be achieved by the GFX GOL operation.

   Description
    This enumeration defines the various states that can be achieved by the
    GOL operation.

   Remarks:
    None.
*/

typedef enum
{
    /*  Module state busy */
    GFX_GOL_STATE_BUSY,

    /*  Module state init */
    GFX_GOL_STATE_INIT,

    /*  Module state initialized */
    GFX_GOL_STATE_INITIALIZED,

} GFX_GOL_STATES;

// *****************************************************************************
/*  GFX GOL Client Status

  Summary:
    Enumerated data type that identifies the  GFX Module Client Status.

  Description:
    This enumeration defines the possible status of the  GFX Module Client.

  Remarks:
    None.
*/

typedef enum
{
     /* Client is closed or the specified handle is invalid */
    GFX_GOL_CLIENT_STATUS_CLOSED
            /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    /* Client is ready */
    GFX_GOL_CLIENT_STATUS_READY
            /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/

} GFX_GOL_CLIENT_STATUS;

// *****************************************************************************
/* GFX GOL task states

  Summary
    Lists the different states that GFX GOL task routine can have.

  Description
    This enumeration lists the different states that GFX GOL task routine can
    have.

  Remarks:
    None.
*/

typedef enum
{
    /* open module for client */
    GFX_GOL_TASK_STATE_OPEN_MODULE,

    /* run module for client */
    GFX_GOL_TASK_STATE_RUNNING,
            
} GFX_GOL_OBJECT_TASK;


// *****************************************************************************
/* GFX GOL object type

    Typedef:
        GFX_GOL_OBJ_TYPE

    Summary:
        Specifies the different object types used in the library.

    Description:
        This enumeration specifies the different object types
        used in the library.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    GFX_GOL_ANALOGCLOCK_TYPE,       // Type defined for Analog Clock Object.
    GFX_GOL_BUTTON_TYPE,            // Type defined for Button Object.
    GFX_GOL_CHART_TYPE,             // Type defined for Chart Object.
    GFX_GOL_CHECKBOX_TYPE,          // Type defined for Check Box Object.
    GFX_GOL_DIGITALMETER_TYPE,      // Type defined for Digital Meter Object.
    GFX_GOL_EDITBOX_TYPE,           // Type defined for Edit Box Object.
    GFX_GOL_GRID_TYPE,              // Type defined for Grid Object.
    GFX_GOL_GROUPBOX_TYPE,          // Type defined for Group Box Object.
    GFX_GOL_LISTBOX_TYPE,           // Type defined for List Box Object.
    GFX_GOL_METER_TYPE,             // Type defined for Meter Object.
    GFX_GOL_PICTURECONTROL_TYPE,    // Type defined for Picture Control Object.
    GFX_GOL_PROGRESSBAR_TYPE,       // Type defined for Progress Bar Object.
    GFX_GOL_RADIOBUTTON_TYPE,       // Type defined for Radio Button Object.
    GFX_GOL_SCROLLBAR_TYPE,         // Type defined for Slider or
                                    // Scroll Bar Object.
    GFX_GOL_STATICTEXT_TYPE,        // Type defined for Static Text Object.
    GFX_GOL_TEXTENTRY_TYPE,         // Type defined for Text-Entry Object.
    GFX_GOL_WINDOW_TYPE,            // Type defined for Window Object.
    GFX_GOL_CUSTOM_TYPE,            // Type defined for Custom Object.
    GFX_GOL_SURFACE_TYPE,
    GFX_GOL_UNKNOWN_TYPE            // Type is undefined and not supported
                                    // by the library.
} GFX_GOL_OBJ_TYPE;

// *****************************************************************************
/* GFX GOL Message

    Typedef:
        GFX_GOL_MESSAGE

    Summary:
        Specifies message structure used in the library.

    Description:
        Specifies message structure used in the library.
        - The types must be one of the INPUT_DEVICE_TYPE:
            - TYPE_UNKNOWN
            - TYPE_KEYBOARD
            - TYPE_TOUCHSCREEN
            - TYPE_MOUSE
        - uiEvent must be one of the INPUT_DEVICE_EVENT.
            - for touch screen:
                - EVENT_INVALID
                - EVENT_MOVE
                - EVENT_PRESS
                - EVENT_STILLPRESS
                - EVENT_RELEASE
            - for keyboard:
                - EVENT_KEYSCAN (param2 contains scan code)
                - EVENT_KEYCODE (param2 contains character code)
        - param1:
            - for touch screen is the x position
            - for keyboard ID of object receiving the message
        - param2
            - for touch screen y position
            - for keyboard scan or key code

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    uint8_t   type;                 // Specifies the type of input device.
    uint8_t   uiEvent;              // An event that occurred in the input
                                    // device.
    int16_t   param1;               // Parameter 1, definition and usage is
                                    // dependent on the type of input device.
    int16_t   param2;               // Parameter 2, definition and usage is
                                    // dependent on the type of input device.
} GFX_GOL_MESSAGE;

// *****************************************************************************
/*  GFX GOL Translated Action

    Typedef:
        GFX_GOL_TRANSLATED_ACTION

    Summary:
        Specifies the different object actions supported in the library.

    Description:
        This enumeration specifies the different object actions
        supported in the library.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    /* Invalid message response. */
    GFX_GOL_OBJECT_ACTION_INVALID = 0x3500,

    /* Passive message response. No change in object needed. */
    GFX_GOL_OBJECT_ACTION_PASSIVE,

    /* Analog Clock Pressed Action */
    GFX_GOL_ANALOGCLOCK_ACTION_PRESSED,

    /* Analog Clock Released Action */
    GFX_GOL_ANALOGCLOCK_ACTION_RELEASED,

    /* Button pressed action ID. */
    GFX_GOL_BUTTON_ACTION_PRESSED,

    /* Button is continuously pressed ID. */
    GFX_GOL_BUTTON_ACTION_STILLPRESSED,

    /* Button released action ID.*/
    GFX_GOL_BUTTON_ACTION_RELEASED,

    /* Button released action ID with button press canceled.*/
    GFX_GOL_BUTTON_ACTION_CANCELPRESS,
            
    /* Chart selected action ID */
    GFX_GOL_CHART_ACTION_SELECTED,

    /* Check Box check action ID. */
    GFX_GOL_CHECKBOX_ACTION_CHECKED,
            
    /* Check Box uncheck action ID. */
    GFX_GOL_CHECKBOX_ACTION_UNCHECKED,

    /* Custom Control selected action ID. */
    GFX_GOL_CUSTOMCONTROL_ACTION_SELECTED,

    /* Digital Meter selected action ID. */
    GFX_GOL_DIGITALMETER_ACTION_SELECTED,

    /* Edit Box insert character action ID. */
    GFX_GOL_EDITBOX_ACTION_ADD_CHAR,

    /* Edit Box remove character action ID. */
    GFX_GOL_EDITBOX_ACTION_DEL_CHAR,
            
    /* Edit Box touchscreen selected action ID. */
    GFX_GOL_EDITBOX_ACTION_TOUCHSCREEN,

    /* Grid item touched action ID. */
    GFX_GOL_GRID_ACTION_TOUCHED,
            
    /* Grid item selected action ID. */
    GFX_GOL_GRID_ACTION_ITEM_SELECTED,

    /* Grid up action ID. */
    GFX_GOL_GRID_ACTION_UP,

    /* Grid down action ID. */
    GFX_GOL_GRID_ACTION_DOWN,
            
    /* Grid left action ID. */
    GFX_GOL_GRID_ACTION_LEFT,
            
    /* Grid right action ID. */
    GFX_GOL_GRID_ACTION_RIGHT,
            
    /* Group Box selected action ID. */
    GFX_GOL_GROUPBOX_ACTION_SELECTED,

    /* List Box item select action ID. */
    GFX_GOL_LISTBOX_ACTION_SELECTED,

    /* List Box item move action ID. */
    GFX_GOL_LISTBOX_ACTION_MOVE,

    /* List Box touchscreen selected action ID. */
    GFX_GOL_LISTBOX_ACTION_TOUCHSCREEN,
            
    /* Meter set value action ID. */
    GFX_GOL_METER_ACTION_SET,
            
    /* Picture selected action ID. */
    GFX_GOL_PICTURECONTROL_ACTION_SELECTED,

    /* Progress Bar selected action ID. */
    GFX_GOL_PROGRESSBAR_ACTION_SELECTED,

    /* Radio Button check action ID. */
    GFX_GOL_RADIOBUTTON_ACTION_CHECKED,

    /* Slider or Scroll Bar increment action ID. */
    GFX_GOL_SCROLLBAR_ACTION_INC,

    /* Slider or Scroll Bar decrement action ID. */
    GFX_GOL_SCROLLBAR_ACTION_DEC,
            
    /* Static Text selected action ID. */
    GFX_GOL_STATICTEXT_ACTION_SELECTED, 
    
    /* TextEntry released action ID */
    GFX_GOL_TEXTENTRY_ACTION_RELEASED,

    /* TextEntry pressed action ID */
    GFX_GOL_TEXTENTRY_ACTION_PRESSED,

    /* TextEntry add character action ID */
    GFX_GOL_TEXTENTRY_ACTION_ADD_CHAR,

    /* TextEntry delete character action ID */
    GFX_GOL_TEXTENTRY_ACTION_DELETE,        
            
    /* TextEntry add space character action ID */
    GFX_GOL_TEXTENTRY_ACTION_SPACE,

    /* TextEntry enter action ID */
    GFX_GOL_TEXTENTRY_ACTION_ENTER,

    /* Window client area selected action ID. */
    GFX_GOL_WINDOW_ACTION_CLIENT,

    /* Window title bar selected action ID. */
    GFX_GOL_WINDOW_ACTION_TITLE,

} GFX_GOL_TRANSLATED_ACTION;

// DOM-IGNORE-BEGIN

// *****************************************************************************
/* Draw Function pointer

    Typedef:
        GFX_STATUS (*DRAW_FUNC)(void *);

    Summary:
        Draw function pointer type definition.
        
    Description:
        This defines the draw function pointer of an object. 
        Each object implements its own draw function and at 
        the object's creation, the pointer assigned to the 
        rendering function of the object is initialized to 
        its own rendering function.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef GFX_STATUS (*DRAW_FUNC)(void *);

// *****************************************************************************
/* Free Function pointer

    Typedef:
        void (*FREE_FUNC)(void *);

    Summary:
        Free function pointer type definition.
        
    Description:
        This defines the free function pointer of an object. 
        Each object implements its own free function and at 
        the object's creation, the pointer assigned to the 
        free function of the object is initialized to 
        its own free function.
       
        The free function is an optional function for an object 
        that requires dynamic memory allocation at run time. This 
        function will make sure that all allocated memory is freed
        when the object is removed from memory.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef void (*FREE_FUNC)(void *);

// *****************************************************************************
/* Action get function pointer

    Typedef:
        GFX_GOL_TRANSLATED_ACTION (*ACTIONGET_FUNC)     
                            (   void *,                 
                                GFX_GOL_MESSAGE *       
                            );

    Summary:
        Object action get function pointer type definition.
        
    Description:
        This defines the object's action get function pointer of an object. 
        Each object implements its own action get function and at 
        the object's creation, the pointer assigned to the action get 
        function of the object is initialized to its own action get function.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef GFX_GOL_TRANSLATED_ACTION (*ACTIONGET_FUNC)     \
                            (   void *,                 \
                                GFX_GOL_MESSAGE *       \
                            );

// *****************************************************************************
/* Action set function pointer

    Typedef:
        void (*ACTIONSET_FUNC)                              
                            (   GFX_GOL_TRANSLATED_ACTION,  
                                void *, GFX_GOL_MESSAGE *   
                            );

    Summary:
        Object action set function pointer type definition.
        
    Description:
        This defines the object's action set function pointer of an object. 
        Each object implements its own action set function and at 
        the object's creation, the pointer assigned to the action set 
        function of the object is initialized to its own action set function.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
/*********************************************************************
 Overview: Object action set function pointer typedef

*********************************************************************/
typedef void (*ACTIONSET_FUNC)                              \
                            (   GFX_GOL_TRANSLATED_ACTION,  \
                                void *, GFX_GOL_MESSAGE *   \
                            );

// DOM-IGNORE-END

// *****************************************************************************
/* GFX GOL object header

    Typedef:
        GFX_GOL_OBJ_HEADER

    Summary:
        Specifies Graphics Object Layer structure used in objects.

    Description:
        This structure defines the Graphics Object Layer header used 
        in all objects in the Graphics Library.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    /* GFX instance index*/
    SYS_MODULE_INDEX    gfxIndex;

    /* Unique id assigned for referencing. */
    uint16_t            ID;

    /* A pointer to the next object. */
    void                *pNxtObj;

    /* Identifies the type of GOL object. */
    GFX_GOL_OBJ_TYPE    type;

    /* State of object. */
    uint16_t            state;

    /* Left position of the Object. */
    uint16_t            left;

    /* Top position of the Object. */
    uint16_t            top;

    /* Right position of the Object. */
    uint16_t            right;

    /* Bottom position of the Object. */
    uint16_t            bottom;

    /* Pointer to the scheme used. */
    GFX_GOL_OBJ_SCHEME  *pGolScheme;

    /* function pointer to the object's draw function. */
    DRAW_FUNC           DrawObj;

    /* function pointer to the object's free function. */
    FREE_FUNC           FreeObj;        

    /* function pointer to the object's action get function. */
    ACTIONGET_FUNC      actionGet;

    /* function pointer to the object's action set function. */
    ACTIONSET_FUNC      actionSet;     
    
    /* GroupNo to keep track of group for radiobutton. */
    uint16_t            groupNo;
                                        
} GFX_GOL_OBJ_HEADER;

// *****************************************************************************
/*  GFX GOL draw callback function pointer

    Function:
        typedef bool (*GFX_GOL_DRAW_CALLBACK_FUNC) (void);

    Summary:
        Draw callback function definition. This application defined
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
typedef bool (*GFX_GOL_DRAW_CALLBACK_FUNC) (void);

// *****************************************************************************
/*  GFX GOL message callback function pointer

    Function:
        typedef bool (*GFX_GOL_MESSAGE_CALLBACK_FUNC)
                        (   GFX_GOL_TRANSLATED_ACTION,
                            GFX_GOL_OBJ_HEADER *,
                            GFX_GOL_MESSAGE *
                        );

    Summary:
        Message callback function definition. This application defined
        function allows the application to perform application specific
        processing of user messages.

    Description:
        This application defined function is called by the
        GFX_GOL_ObjectMessage() function allowing the application
        the opportunity to process the user messages and customize
        object behavior as well as application controlled functions.

        GFX_GOL_ObjectMessage() calls this function when a valid message
        for an object  in the active list is received. Application
        implements any action for the message in this callback function.
        If this callback function returns true, the message for the
        object will be processed using the default action of the object.
        If false is returned, the default action will not be performed.
        In this case, it is assumed that this callback function has
        performed the appropriate changes to the states of the objects.

    Precondition:
        None.

    Parameters:
        GFX_GOL_TRANSLATED_ACTION - Translated message for the object
        GFX_GOL_OBJ_HEADER * - Pointer to the object that processed the
                               message.
        GFX_GOL_MESSAGE * - Pointer to the message from user.

    Returns:
        true  - When true is returned, the object will set its state
                depending on the translated messages.
        false - When false is returned, the object will not process the 
                translated message and will assume the application has
                performed necessary action on the message.

    Example:
        None.

*/
// *****************************************************************************
typedef bool (*GFX_GOL_MESSAGE_CALLBACK_FUNC)       
                    (   GFX_GOL_TRANSLATED_ACTION,  
                        GFX_GOL_OBJ_HEADER *,       
                        GFX_GOL_MESSAGE *           
                    );

// *****************************************************************************
/*  GOL panel parameter

    Typedef:
        GOL_PANEL_PARAM

    Summary:
        Specifies panel parameters.

    Description:
        This structure defines the panel parameters when rendering a panel.

    Remarks:
        None.
*/
// *****************************************************************************
typedef struct
{
    /* Panel left coordinate. */
    int16_t             panelLeft;

    /* Panel top coordinate. */
    int16_t             panelTop;

    /* Panel right coordinate. */
    int16_t             panelRight;

    /* Panel Bottom coordinate. */
    int16_t             panelBottom;

    /* Panel Face color. */
    GFX_COLOR           panelFaceColor;

    /* Panel Emboss light color. */
    GFX_COLOR           panelEmbossLtColor;

    /* Panel Emboss dark color. */
    GFX_COLOR           panelEmbossDkColor;

    /* Panel Emboss size. */
    uint16_t            panelEmbossSize;

    /* Panel radius. */
    uint16_t            panelRadius;

    /* Panel image resource. */
    GFX_RESOURCE_HDR    *pPanelImage;

    /* Panel fill style. */
    GFX_FILL_STYLE      panelFillStyle;

#ifndef GFX_CONFIG_GRADIENT_DISABLE

    /* Panel gradient start color. */
    GFX_COLOR           panelGradientStartColor;

    /* Panel gradient end color .*/
    GFX_COLOR           panelGradientEndColor;
    
#endif

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE

    /* Panel alpha */
    uint16_t            panelAlpha;
    
#endif	

} GOL_PANEL_PARAM;

// *****************************************************************************
/*  GFX GOL common object states

    Typedef:
        Common Object States
        
    Summary:
        Common Object States.

    Description:
        The following macros defines the common Object State bits.

    Remarks:
        None.
*/
// *****************************************************************************
typedef enum
{
    /* Focus state bit */
    GFX_GOL_FOCUSED         /*DOM-IGNORE-BEGIN*/  = 0x0001 
                            /*DOM-IGNORE-END  */,
    /* Disabled state bit. */
    GFX_GOL_DISABLED        /*DOM-IGNORE-BEGIN*/ = 0x0002 
                            /*DOM-IGNORE-END  */,
    /* Object hide state bit. Object will be hidden from the
       screen by drawing over it the common background color. */
    GFX_GOL_HIDE            /*DOM-IGNORE-BEGIN*/  = 0x8000
                            /*DOM-IGNORE-END  */,  
    /* Object redraw state bits. The whole Object must be redrawn. */
    GFX_GOL_DRAW            /*DOM-IGNORE-BEGIN*/  = 0x7C00 
                            /*DOM-IGNORE-END  */,    
    /* Focus redraw state bit. The focus rectangle must be redrawn. */
    GFX_GOL_DRAW_FOCUS      /*DOM-IGNORE-BEGIN*/  = 0x2000 
                            /*DOM-IGNORE-END  */,    
    /* Partial Object redraw state bits. A part
     or parts of the Object must be redrawn to 
     show updated state. */
    GFX_GOL_DRAW_UPDATE     /*DOM-IGNORE-BEGIN*/  = 0x3C00 
                            /*DOM-IGNORE-END  */,    
} GFX_GOL_COMMON_STATE_BITS;


// *****************************************************************************
// *****************************************************************************
// Section: GFX GOL Layer System Interface Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
    Function:
        void GFX_GOL_Initialize ( const SYS_MODULE_INDEX   moduleIndex,
                                  const SYS_MODULE_INIT    * const moduleInit);

    Summary:
        Initialize the GFX GOL Library.

    Description:
        This function initialize the GFX Library GOL layer. The following 
        default settings are set when this function is called.
        1. font -

    Precondition:
        None.

    Parameters:
        moduleIndex - client instance request.

        moduleInit  - initialization data for the instance.

    Returns:
        If successful, returns a valid handle to a device layer object.
        Otherwise, it returns SYS_MODULE_OBJ_INVALID.

    Example:
        <code>
            GFX_INIT gfxInit;
            SYS_MODULE_OBJ gfxObj;

            // GFX Module initialization
            gfxObj = GFX_GOL_Initialize(GFX_INDEX_0, &gfxInit);

            if (SYS_MODULE_OBJ_INVALID == gfxObj)
            {
                // Handle error
            }
            </code>

  Remarks:
    This routine must be called before other GFX library functions.

*/
// *****************************************************************************
SYS_MODULE_OBJ  GFX_GOL_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                               const SYS_MODULE_INIT    * const moduleInit);

// *****************************************************************************
/* Function:
    void GFX_GOL_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the GFX Module.

  Description:
    Deinitializes the specified instance of the GFX Module. All internal data
    structures will be reset.

  Precondition:
    Function GFX_GOL_Initialize should have been called before calling this
    function.

  Parameters:
    object          - GFX layer object handle, returned from the
                      GFX_GOL_Initialize routine

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from GFX_GOL_Initialize
    SYS_STATUS          status;


    GFX_GOL_Deinitialize(object);

    status = GFX_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
*/
void GFX_GOL_Deinitialize          (SYS_MODULE_OBJ moduleObject);

// *****************************************************************************
// *****************************************************************************
// Section: GOL Layer Client Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/*
  Function: GFX_HANDLE GFX_GOL_Open( const SYS_MODULE_INDEX index)

  Summary:
    opens an instance of the GFX GOL Library.
*/
GFX_HANDLE GFX_GOL_Open( const SYS_MODULE_INDEX index);

// *****************************************************************************
/* Function:
    void GFX_GOL_Tasks (SYS_MODULE_INDEX index);

  Summary:
    Maintains the GFX GOL module state machine. It manages the GFX Module object
    list items and responds to GFX Module primitive events.

  Description:
    Maintains the GFX module state machine. It manages the GFX Module object 
    list items and responds to GFX Module primitive events.  This function
    should be called from the SYS_Tasks() function.

  Precondition:
     None.

  Parameters:
    index      - Object index for the specified module instance.

  Returns:
    None.

  Example:
    <code>

    while (true)
    {
        GFX_GOL_Tasks (GFX_INDEX_0);

        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application.
*/
void GFX_GOL_Tasks (SYS_MODULE_INDEX index);


// *****************************************************************************
/*  GFX GOL message call back set.

    Function:
        void GFX_GOL_MessageCallbackSet( SYS_MODULE_INDEX gfxIndex,
                                GFX_GOL_MESSAGE_CALLBACK_FUNC pFunc)
    Summary:
        This function sets the message callback function that the
        application will use to evaluate user inputs that affects
        the objects and application behavior.

    Description:
        This function sets the message callback function that the
        application will use to evaluate user inputs that affects
        the objects and application behavior. The callback function
        location is specified by the function pointer supplied in
        the call. See GFX_GOL_MESSAGE_CALLBACK_FUNC definition for
        details on the message callback function.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pFunc - pointer to the message callback function.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
void GFX_GOL_MessageCallbackSet( SYS_MODULE_INDEX gfxIndex, 
                                 GFX_GOL_MESSAGE_CALLBACK_FUNC pFunc );

// *****************************************************************************
/*  GFX GOL draw call back set.
    
    Function:
        void GFX_GOL_DrawCallbackSet( SYS_MODULE_INDEX gfxIndex,
                                GFX_GOL_DRAW_CALLBACK_FUNC pFunc )
    Summary:
        This function sets the draw callback function that the
        application will use to render application specific rendering.

    Description:
        This function sets the draw callback function that the
        application will use to call primitive function to implement
        application specific shapes.See GFX_GOL_DRAW_CALLBACK_FUNC
        definition for details on the draw callback function.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pFunc - pointer to the draw callback function.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
void GFX_GOL_DrawCallbackSet( SYS_MODULE_INDEX gfxIndex, GFX_GOL_DRAW_CALLBACK_FUNC pFunc);

// *****************************************************************************
/*  GFX GOL object can be focused.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectCanBeFocused( SYS_MODULE_INDEX gfxIndex,
                                                   GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        Checks if the object can be focused.

    Description:
        This function checks if the object can be focused or not.
        If the object can be focused, it returns GFX_STATUS_SUCCESS.
        If it cannot be focused, it returns GFX_STATUS_FAILURE.
        Selected objects have the focus feature. Refer to the object
        documentation for details.

        Objects that do not support focus feature will ignore any focus
        settings.

        If the object is disabled it cannot be set to focused state.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object.

    Returns:
        GFX_STATUS_SUCCESS - when the object can be focused
        GFX_STATUS_FAILURE - when the object cannot be focused or
                             do not support the focus feature.

    Example:
        None.

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectCanBeFocused(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object focus previous get.
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFocusPrevGet( 
                                                     SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function returns the pointer to the previous object
        in the active list of objects which can be focused.

    Description:
        This function returns the pointer to the previous object
        in the active list of objects which can be focused.

        The reference point is the currently focused object. If there
        is no currently focused object, the searched starts from the
        beginning of the active list of objects.

        Objects that can be focused are those objects that can receive
        keyboard inputs.

        If there is no object capable of receiving keyboard
        inputs (i.e. none can be focused) NULL is returned.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        The pointer to the object that can be focused.

    Example:
        None.

*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER  *GFX_GOL_ObjectFocusPrevGet(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object focus next get.
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFocusNextGet( 
                                                    SYS_MODULE_INDEX gfxIndex )

    Summary:
        This function returns the pointer to the next object
        in the active list of objects which can be focused.

    Description:
        This function returns the pointer to the next object
        in the active list of objects which can be focused.

        The reference point is the currently focused object. If there
        is no currently focused object, the searched starts from the
        beginning of the active list of objects.

        Objects that can be focused are those objects that can receive
        keyboard inputs.

        If there is no object capable of receiving keyboard
        inputs (i.e. none can be focused) NULL is returned.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        The pointer to the object that can be focused.

    Example:
        None.
 
*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER  *GFX_GOL_ObjectFocusNextGet(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object focus set.
    
    Function:
        GFX_STATUS *GFX_GOL_ObjectFocusSet( SYS_MODULE_INDEX gfxIndex,
                                            GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function sets the object to be focused.

    Description:
        This function sets the specified object to be focused.

        If the object cannot accept keyboard messages, the object will
        not be set to focused state. If the object can accept keyboard
        messages, then the focus state will be set and will be marked
        to be redrawn to show the focus when the focus feature is enabled.

        Objects that can be focused are those objects that can receive
        keyboard inputs.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object.

    Returns:
        GFX_STATUS_SUCCESS - when the object can be focused
        GFX_STATUS_FAILURE - when the object cannot be focused or
                             do not support the focus feature.

    Example:
        None.

*/
// *****************************************************************************
GFX_STATUS  GFX_GOL_ObjectFocusSet(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object focus get.
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFocusGet(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function returns the pointer to the object that is currently
        receiving keyboard input (or focused).

    Description:
        This function returns the pointer to the object that is currently
        receiving keyboard input (or focused).

        If there are no object that can accept keyboard messages, then the
        function will return NULL.

        Objects that can be focused are those objects that can receive
        keyboard inputs.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        The pointer the currently focused object. Returns NULL if there
        is no object currently set.

    Example:
        None.
 
*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFocusGet(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object type get.
    
    Function:
        GFX_GOL_OBJ_TYPE GFX_GOL_ObjectTypeGet( SYS_MODULE_INDEX gfxIndex, 
                                                GFX_GOL_OBJ_HEADER *pObject )

    Summary:
        This function returns the object type.

    Description:
        This function returns the object type. The object type is one
        of the defined enumerated types of GFX_GOL_OBJ_TYPE.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the object of interest.

    Returns:
        The type of the object. The type is one of the defined
        enumerated types of GFX_GOL_OBJ_TYPE.

    Example:
        None.
 
*/
// *****************************************************************************
GFX_GOL_OBJ_TYPE GFX_GOL_ObjectTypeGet(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object ID get.
    
    Function:
        uint16_t GFX_GOL_ObjectIDGet(GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function returns the object ID.

    Description:
        This function returns the user defined ID assigned to the object. 
 
    Precondition:
        None.

    Parameters:
        Pointer to the object of interest.

    Returns:
        The user defined ID of the object.
 
    Example:
        <code>
            void ExampleUsageOfGettingID(GFX_GOL_OBJ_HEADER *pObject) 
            {
                uint16_t id;
 
                switch(id = GFX_GOL_ObjectIDGet(pObject)) 
                {
                    case ID_WINDOW1:
                        // do something
                    case ID_WINDOW2:
                        // do something else
                    case ID_WINDOW3:
                        // do something else
                    default:
                        // do something else
                }
            }
        </code>

*/
// *****************************************************************************
uint16_t GFX_GOL_ObjectIDGet(GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object next get
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectNextGet( SYS_MODULE_INDEX gfxIndex,
                                                    GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function returns the pointer to next object in the list
        after the specified object.

    Description:
        This function returns the pointer to next object in the list
        after the specified object.
 
    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the object of interest.

    Returns:
        The pointer to the next object in the list.
 
    Example:
        <code>
            void RedrawButtons(void)
            {
            
                GFX_GOL_OBJ_HEADER *pCurr;
                
                // get active list
                pCurr = GFX_GOL_ObjectListGet(gfxIndex);
 
                while(pCurr->pNxtObj != NULL) 
                {
                    // just select button objects and redraw them
                    if (GFX_GOL_ObjectTypeGet(gfxIndex, pCurr) == BUTTON)
                    { 
                        // set to be redrawn
                        pCurr->state = BTN_DRAW;        
                    }
                    pCurr = GFX_GOL_ObjectNextGet(gfxIndex, pCurr);
                }
                // redraw all buttons in the active list
                GFX_GOL_ObjectListDraw(gfxIndex);
            }
        </code>

*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectNextGet(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object find.
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFind(SYS_MODULE_INDEX gfxIndex, uint16_t ID)

    Summary:
        This function returns the pointer to object in the list
        with the user defined ID assigned to it.

    Description:
        This function returns the pointer to object in the list
        with the user defined ID assigned to it.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        User defined ID of the object of interest.

    Returns:
        The pointer to the object in the list with the given ID.

    Example:
        <code>
            void CopyObject(GFX_GOL_OBJ_HEADER *pSrcList, 
                            GFX_GOL_OBJ_HEADER *pDstList,
                            uint16_t ID)
            {
                GFX_GOL_OBJ_HEADER *pTemp;

                // find the object
                pTemp = GFX_GOL_ObjectFind(gfxIndex, ID);

                if (pTemp != NULL)
                {
                    // destination as active list

                    GFX_GOL_ObjectListSet(gfxIndex, pDstList);

                    // add object to active list
                    GFX_GOL_ObjectAdd(gfxIndex, pTemp);
                }
            }
        </code>

*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectFind(SYS_MODULE_INDEX gfxIndex, uint16_t ID);

// *****************************************************************************
/*  GFX GOL object add.
    
    Function:
        void GFX_GOL_ObjectAdd( SYS_MODULE_INDEX gfxIndex,
                                                   GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function adds an object to the tail of the 
        currently active list. 
 
    Description:
        This function adds an object to the tail of the 
        currently active list. The new list tail is set 
        to point to NULL after the new object is added.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the object that will be added to the list.
 
    Returns:
        None.
 
    Example:
        <code>
            void MoveObject(    GFX_GOL_OBJ_HEADER *pSrcList, 
                                GFX_GOL_OBJ_HEADER *pDstList,
                                GFX_GOL_OBJ_HEADER *pObjtoMove) 
            {
                GFX_GOL_OBJ_HEADER *pTemp = pSrcList;
                    
                if(pTemp != pObjtoMove) 
                {
                    while(pTemp->pNxtObj != pObjtoMove)
                        pTemp = pTemp->pNxtObj;
                }

                pTemp->pNxtObj = pObjtoMove->pNxt;  // remove object from list
                GFX_GOL_ObjectListSet(pDstList);    // destination as active list
                GFX_GOL_ObjectAdd(gfxIndex, pObjtoMove);      // add object to active list
	        }
            
        </code>

*/
// *****************************************************************************
void    GFX_GOL_ObjectAdd(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object delete.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectDelete( SYS_MODULE_INDEX gfxIndex,
                                                    GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function removes an object from the currently
        active list.

    Description:
        This function removes an object from the currently 
        active list. Aside from the removal of the object 
        from the list, the RAM resources consumed by the object
        is also freed.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the object that will removed from the list.

    Returns:
        GFX_STATUS_SUCCESS - is returned if the removal was successful.
        GFX_STATUS_FAILURE - is returned if the removal was not successful.

    Example:
        None.

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectDelete(SYS_MODULE_INDEX gfxIndex, GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object by ID delete.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectByIDDelete( SYS_MODULE_INDEX gfxIndex,
                                             uint16_t id)

    Summary:
        This function removes an object with the given user
        defined ID from the currently active list.

    Description:
        This function removes an object with the given user
        defined ID from the currently active list.
        Aside from the removal of the object from the list,
        the RAM resources consumed by the object is also freed.

        If there is no object with the given ID, the function exits
        with GFX_STATUS_FAILURE.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the object that will removed from the list.

    Returns:
        GFX_STATUS_SUCCESS - is returned if the removal was successful.
        GFX_STATUS_FAILURE - is returned if the removal was not successful.

    Example:
        None.

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectByIDDelete(SYS_MODULE_INDEX gfxIndex, uint16_t id);

// *****************************************************************************
/*  GFX GOL object list new
    
    Function:
        GFX_STATUS GFX_GOL_ObjectListNew(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function removes an object with the given user
        defined ID from the currently active list.

    Description:
        This function starts a new linked list of objects and
        resets the keyboard focus to none. This function assigns
        the current active list and current focused object
        (receiving keyboard inputs) object pointers to NULL.
        Any keyboard inputs at this point will be ignored.

        This function does not erase the objects in the previous
        list. Application must save the previous list to
        another pointer if to be referenced later. If not needed
        anymore, memory used by that list should be freed by
        GFX_GOL_ObjectListFree() function. In this case, freeing 
        the list with GFX_GOL_ObjectListFree() function has the same 
        effect as GFX_GOL_ObjectListNew() where the current active 
        list is empty.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        GFX_STATUS_SUCCESS - is returned if the new list start was successful.
        GFX_STATUS_FAILURE - is returned if the new list start was not
                             successful.

    Example:
        <code>
            // assume pointers to objects (pButton, pWindow and pSlider
            // are initialized to objects already created
            // GFX_GOL_OBJ_HEADER *pButton;
            // GFX_GOL_OBJ_HEADER *pWindow;
            // GFX_GOL_OBJ_HEADER *pSlider;

            GFX_GOL_OBJ_HEADER *pSave;

            // save current list
            pSave = GFX_GOL_ObjectListGet(gfxIndex);

            // start the new list, after the start of the list, the
            // current active list is empty.
            GFX_GOL_ObjectListNew(gfxIndex);

            // assume that objects are already created
            // you can now add objects to the new list
            GFX_GOL_ObjectAdd(gfxIndex, pButton);
            GFX_GOL_ObjectAdd(gfxIndex, pWindow);
            GFX_GOL_ObjectAdd(gfxIndex, pSlider);

        </code>

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectListNew(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object list get
    
    Function:
        GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectListGet(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function returns the current active list.

    Description:
        This function returns the pointer to the current active.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        Pointer (type GFX_GOL_OBJ_HEADER) to the current active list.

    Example:
        See GFX_GOL_ObjectLlistNew() for example code.

*/
// *****************************************************************************
GFX_GOL_OBJ_HEADER *GFX_GOL_ObjectListGet(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object list set.
    
    Function:
        GFX_STATUS *GFX_GOL_ObjectListSet( SYS_MODULE_INDEX gfxIndex,
                                                     GFX_GOL_OBJ_HEADER *pList)

    Summary:
        This function sets the active list to the new list.

    Description:
        This function sets the active list to the new list.
        The previous list will still exist in memory. Application
        must save the previous list before the set is called if the
        previous list will be referenced later. If the previous list
        is not needed anymore, then the list must be removed from
        memory by GFX_GOL_ObjectListFree() function.

        Setting the active list to the new list will reset the focused
        pointer object to NULL.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        Pointer to the new list.

    Returns:
        GFX_STATUS_SUCCESS - is returned if the set was successful.
        GFX_STATUS_FAILURE - is returned if the set was not successful.

    Example:
       <code>
            GFX_GOL_OBJ_HEADER *pSave;

            // save current list
            pSave = GFX_GOL_ObjectListSet(gfxIndex);

            // start the new list
            GFX_GOL_ObjectListNew(gfxIndex);

            // you can now add objects to the current list
            // assume that objects are already created
            GFX_GOL_ObjectAdd(gfxIndex, pButton);
            GFX_GOL_ObjectAdd(gfxIndex, pWindow);
            GFX_GOL_ObjectAdd(gfxIndex, pSlider);

            // do something here on the new list

            // return the old list
            GFX_GOL_ObjectListSet(gfxIndex, pSave);
       </code>

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectListSet( SYS_MODULE_INDEX gfxIndex,
                                                     GFX_GOL_OBJ_HEADER *pList);

// *****************************************************************************
/*  GFX GOL Preemption level get
    
    Function:
        GFX_PREEMPTION_LEVEL GFX_GOL_PreemptionLevelGet (
                                                   SYS_MODULE_INDEX gfxIndex )

    Summary:
        This function returns the current preemption level.

    Description:
        This function returns the current preemption level.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        The current preemption level.

*/
// *****************************************************************************
GFX_PREEMPTION_LEVEL GFX_GOL_PreemptionLevelGet ( SYS_MODULE_INDEX gfxIndex );

// *****************************************************************************
/*  GFX GOL Object List free.

    Function:
        GFX_STATUS GFX_GOL_ObjectListFree(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function sets the active list to the new list.

    Description:
        This function frees all the memory used by objects in
        the active list and initializes the active list pointer
        to NULL to start a new empty list. This function must
        be called only inside the GFX_GOL_ObjectDrawCallback()
        function when using GFX_GOL_ObjectListDraw() and
        GFX_GOL_ObjectMessage() functions. This requirement
        assures that primitive rendering settings are not altered
        by the rendering state machines of the objects.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        GFX_STATUS_SUCCESS - is returned if the free was successful.
        GFX_STATUS_FAILURE - is returned if the free was not successful.

    Example:
       <code>
            void DeletePage(GFX_GOL_OBJ_HEADER *pPage)
            {
                GFX_GOL_OBJ_HEADER *pTemp;

                // assuming pPage is different from the current active list
                // save the active list
		pTemp = GFX_GOL_ObjectListGet(gfxIndex);

                // set list as active list
                GFX_GOL_ObjectListSet(gfxIndex, pPage);

                // pPage objects are deleted
		GFX_GOL_ObjectListFree(gfxIndex);

                // restore the active list
		GFX_GOL_ObjectListSet(gfxIndex, pTemp);
            }
       </code>

*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectListFree(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/* GFX GOL object style scheme set.

    Function:
        void GFX_GOL_ObjectStyleSchemeSet(
                                GFX_GOL_OBJ_HEADER *pObject,
                                GFX_GOL_OBJ_SCHEME *pStyle)

    Summary:
        This function sets the style scheme of the object.

    Description:
        This function sets the style scheme of the object to the
        given style scheme. The object and style scheme must exist
        at the time of the assignment. This function do not check
        if the object or the style is valid.

    Precondition:
        None.

    Parameters:
        pObject - pointer to the object.
        pStyle - pointer to the style scheme.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_ObjectStyleSchemeSet(pObject, pStyle)   \
                ((((GFX_GOL_OBJ_HEADER*)pObject)->pGolScheme) = pStyle)

// *****************************************************************************
/*  GFX GOL object style scheme get.

    Function:
        GFX_GOL_OBJ_SCHEME *GFX_GOL_ObjectStyleSchemeGet(
                                GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function returns the style scheme currently set for the object.

    Description:
        This function returns the style scheme currently used by the
        object. The object must exist when this function is called.
        This function do not check if the object is valid.

    Precondition:
        None.

    Parameters:
        pObject - pointer to the object.

    Returns:
        The pointer to the currently set style scheme.

    Example:
        None.

*/
// *****************************************************************************
#define GFX_GOL_ObjectStyleSchemeGet(pObject)   \
                (((GFX_GOL_OBJ_HEADER*)pObject)->pGolScheme)

// *****************************************************************************
/*  GFX GOL object state set.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectStateSet(
                                GFX_GOL_OBJ_HEADER *pObject,
                                uint16_t stateBits);

    Summary:
        This function sets the state bits of the given object.

    Description:
        This function sets the state bits of the given object. Object
        must be redrawn to display the changes. It is possible to set
        several state bits with this function.

    Precondition:
        None.

    Parameters:
        pObject - Pointer to the object.
        stateBits - Defines which state bits are to be  cleared.
                    Please refer to specific objects for object
                    state bits definition for details

    Returns:
        GFX_STATUS_SUCCESS - is returned if the set was successful.
        GFX_STATUS_FAILURE - is returned if the set was not successful.

    Example:
        <code>
            void SetMessage(uint16_t msg, GFX_GOL_BUTTON* pB)
            {
                switch(msg)
                {
                    case GFX_GOL_BUTTON_ACTION_PRESSED:
                        // set pressed and redraw
                        GFX_GOL_ObjectStateSet(pB, BTN_PRESSED|BTN_DRAW);
                        break;
                    case GFX_GOL_BUTTON_ACTION_RELEASED:
                        // reset pressed
                        GFX_GOL_ObjectStateClear(pB, BTN_PRESSED);
                        // redraw
                        GFX_GOL_ObjectStateSet(pB, BTN_DRAW);
                        break;
                    default:
                        break;
                }
            }
        </code>

*/
// *****************************************************************************
#define GFX_GOL_ObjectStateSet(pObject, stateBits)      \
                ((((GFX_GOL_OBJ_HEADER*)pObject)->state) |= stateBits)


// *****************************************************************************
/*  GFX GOL Object state clear.

    Function:
        GFX_STATUS GFX_GOL_ObjectStateClear(
                                GFX_GOL_OBJ_HEADER *pObject,
                                uint16_t stateBits);

    Summary:
        This function clears the state bits of the given object.

    Description:
        This function clears the state bits of the given object. Object
        must be redrawn to display the changes. It is possible to set
        several state bits with this function.

    Precondition:
        None.

    Parameters:
        pObject - Pointer to the object.
        stateBits - Defines which state bits are to be  cleared.
                    Please refer to specific objects for object
                    state bits definition for details
    Returns:
        GFX_STATUS_SUCCESS - is returned if the clear was successful.
        GFX_STATUS_FAILURE - is returned if the clear was not successful.

    Example:
        See GFX_GOL_ObjectStateSet() for code example.

*/
// *****************************************************************************
#define GFX_GOL_ObjectStateClear(pObject, stateBits)        \
                ((((GFX_GOL_OBJ_HEADER*)pObject)->state) &= (~(stateBits)))

// *****************************************************************************
/*  GFX GOL object state get

    Function:
        uint16_t GFX_GOL_ObjectStateGet(
                                GFX_GOL_OBJ_HEADER *pObject,
                                uint16_t stateBits);

    Summary:
        This function retrieves the current value of the state bits of
        an object.

    Description:
        This function retrieves the current value of the state bits of
        an object. It is possible to get several state bits.

    Precondition:
        None.

    Parameters:
        pObject - Pointer to the object.
        stateBits - Defines which state bits are to be retrieved.
                    Please refer to specific objects for object
                    state bits definition for details
    Returns:
        The current status of the specified state bits.

    Example:
        <code>

            #define BTN_HIDE 0x8000

            GFX_GOL_BUTTON *pB;
            // pB is created and initialized
            // do something here to set state

            // Hide the button (remove from screen)
            if (GFX_GOL_ObjectStateGet(pB, GFX_GOL_BUTTON_HIDE_STATE))
            {
                GFX_ColorSet(pB->pGolScheme->CommonBkColor);
                GFX_BarDraw(pB->left, pB->top,pB->right,pB->bottom);
            }
        </code>
*/
// *****************************************************************************
#define GFX_GOL_ObjectStateGet(pObject, stateBits)      \
                (((GFX_GOL_OBJ_HEADER*)pObject)->state & stateBits)

// *****************************************************************************
/*  GFX GOL object list draw.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectListDraw(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function redraws all objects in the current active list
        that has the rendering state bits set.

    Description:
        This function loops through the active list and redraws
        objects that need to be redrawn. Partial redrawing or
        full redraw is performed depending on the drawing states
        of the objects.

        GFX_GOL_ObjectDrawCallback() function is called by
        GFX_GOL_ObjectListDraw() when drawing of objects in the
        active list is completed. GFX_GOL_ObjectDrawCallback()
        is an application implemented function that allows the
        application the opportunity to insert application specific
        rendering using Primitive Layer rendering functions.

        The GFX_GOL_ObjectListDraw() function can return with GFX_STATUS_BUSY.
        In this case, it indicates that the currently redrawn object is
        not able to continue. Application needs to call GFX_GOL_ObjectListDraw()
        again to continue the redraw of the objects in the list.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        GFX_STATUS_SUCCESS - is returned when the active list is completely
                             parsed and redrawn.
        GFX_STATUS_BUSY    - is returned when the active list is not completely
                             parsed and redrawn.

    Example:
        <code>
            // Assume objects are created & states are set to draw objects
            while(1)
            {
                // parse active list and redraw objects
                // that needs to be redrawn
		if( GFX_GOL_ObjectListDraw(gfxIndex) == GFX_STATUS_SUCCESS)
                {
                    // at this point drawing is completed
                    // it is safe to modify objects states and linked list

                    // evaluate messages from touch screen device
                    TouchGetMsg(&msg);

                    // evaluate each object is affected by the message
                    GFX_GOL_ObjectMessage(gfxIndex, &msg);
                }
            }
        </code>
*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectListDraw(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object rectangle redraw.
    
    Function:
        void GFX_GOL_ObjectRectangleRedraw( SYS_MODULE_INDEX gfxIndex,
                                uint16_t left,
                                uint16_t top,
                                uint16_t right,
                                uint16_t bottom)

    Summary:
        This function marks all objects in the active list intersected
        by the given rectangular area to be redrawn.

    Description:
        This function marks all objects in the active list intersected
        by the given rectangular area to be redrawn.

        After calling this function, the next call to GFX_GOL_ObjectListDraw()
        will redraw all objects that are marked for redraw.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        left   - Defines the left most border of the rectangle area.
        top    - Defines the top most border of the rectangle area.
        right  - Defines the right most border of the rectangle area.
        bottom - Defines the bottom most border of the rectangle area.

    Returns:
        None.
        
    Example:
        <code>
            GFX_GOL_OBJ_HEADER *pTemp;
            GFX_GOL_OBJ_HEADER *pAllObjects;

            // assume *pAllObjects points to a list of all existing objects
            // created and initialized

            // mark all objects inside the rectangle to be redrawn
            GFX_GOL_ObjectRectangleRedraw(gfxIndex,10,10,100,100);

            // save the current active list
            pTemp = pAllObjects;

            // reset active list
            GFX_GOL_ObjectListNew(gfxIndex);

            // build the new active list with only those objects that
            // are marked to be redrawn
            while(pTemp->pNxtObj != NULL)
            {
                if (pTemp->state&0x7C00)
                    GFX_GOL_ObjectAdd(gfxIndex, pTemp);

                pTemp = pTemp->pNxtObj;
            }

             // redraw active list
            GFX_GOL_ObjectListDraw(gfxIndex);
        </code>
*/
// *****************************************************************************
void GFX_GOL_ObjectRectangleRedraw( SYS_MODULE_INDEX gfxIndex, uint16_t left,
                                    uint16_t top,
                                    uint16_t right,
                                    uint16_t bottom);

// *****************************************************************************
/*  GFX GOL object list hide.
    
    Function:
        void GFX_GOL_ObjectListHide(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function marks all objects in the active list to be hidden.

    Description:
        This function marks all objects in the active list to be hidden.

        After calling this function, the next call to GFX_GOL_ObjectListDraw()
        will hide all objects.

    Precondition:
        The objects must be using the same background information.

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
void GFX_GOL_ObjectListHide(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object draw enable.
    
    Function:
        void GFX_GOL_ObjectDrawEnable( SYS_MODULE_INDEX gfxIndex, 
                                       GFX_GOL_OBJ_HEADER *pObject )

    Summary:
        This function sets the object to be redraw.

    Description:
        This function sets the object to be redrawn. For the redraw
        to be effective, the object must be in the current active list.
        If not, the redraw action will not be performed until the list
        where the object is currently inserted will be set as the active
        list.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object that will be redrawn.

    Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
void GFX_GOL_ObjectDrawEnable(  SYS_MODULE_INDEX gfxIndex,
                                GFX_GOL_OBJ_HEADER *pObject );

// *****************************************************************************
/*  GFX GOL object draw disable.
    
    Function:
        void GFX_GOL_ObjectDrawDisable( SYS_MODULE_INDEX gfxIndex,
                                                   GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function resets the drawing state bits of the object.

    Description:
        This function resets the drawing state bits of the object.
        This function can be called to cancel any drawing state bits
        that has been set or clears all the drawing state bits after
        the object has been redrawn.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object.

    Returns:
        None.

    Example:
        None.

*/
// *****************************************************************************
void GFX_GOL_ObjectDrawDisable( GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object is redraw set.
    
    Function:
        bool GFX_GOL_ObjectIsRedrawSet( SYS_MODULE_INDEX gfxIndex,
                                GFX_GOL_OBJ_HEADER *pObject)

    Summary:
        This function checks if the object needs to be redrawn or not.

    Description:
        This function checks if the object needs to be redrawn or not.
        The function returns true if it is to be redrawn
        or false if it is not to be redrawn.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object that will be checked.

    Returns:
        true - when the object needs to be redrawn.
        false - when the object does not need to be redrawn.

    Example:
        <code>
            int DrawButtonWindowOnly()
            {
                static GFX_GOL_OBJ_HEADER *pCurrentObj = NULL;
                uint16_t done = 0;

                if (pCurrentObj == NULL)
                {
                    // get current list
                   pCurrentObj = GFX_GOL_ObjectListGet(gfxIndex);
                }

                while(pCurrentObj != NULL)
                {
                    if(GFX_GOL_ObjectIsRedrawSet(gfxIndex, pCurrentObj) == true)
                    {
                        done = pCurrentObj->draw(gfxIndex, pCurrentObj);

                        // reset state of object if done
                        if (done)
                            GOLDrawComplete(pCurrentObj)
                            // Return if not done. This means that Button Draw function
                            // was not able to finish redrawing the object
                            // and must be called again to finish rendering of
                            // objects in the list that have new states.
                        else
                            return 0;
                    }
                    // go to the next object in the list
                    pCurrentObj = pCurrentObj->pNxtObj;
                }
                return 1;
            }
        </code>
*/
// *****************************************************************************
bool GFX_GOL_ObjectIsRedrawSet( GFX_GOL_OBJ_HEADER *pObject);


// *****************************************************************************
/*  GFX GOL object message.

    Function:
        void  GFX_GOL_ObjectMessage(SYS_MODULE_INDEX gfxIndex, GFX_GOL_MESSAGE *pMessage)

    Summary:
        This function process the received message from the user
        to determine the affected objects. Depending on the message
        and the affected objects, object states are modified based
        on the default or user-defined behavior.

    Description:
        This function receives a GFX_GOL_MESSAGE message from user and
        loops through the active list of objects to check which object
        is affected by the message. For affected objects the message
        is translated and GFX_GOL_ObjectMessageCallback() is called.
        In the call back function, user has the ability to implement
        action for the message. If the call back function returns
        non-zero, OBJMsgDefault() is called to process message for
        the object by default.
        If zero is returned OBJMsgDefault() is not called.
        Please refer to GOL Messages section for details.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pMessage - Pointer to the message from user.

    Returns:
        None.

    Example:
        <code>
            // Assume objects are created & states are set to draw objects
            while (1)
            {
                if(GOLDraw())
                {
                    // GOL drawing is completed here
                    // it is safe to change objects

                    // from user interface module
                    TouchGetMsg(&msg);
                    // process the message
                    GFX_GOL_ObjectMessage(gfxIndex, &msg);
                }
            }
        </code>
*/
// *****************************************************************************
void  GFX_GOL_ObjectMessage(SYS_MODULE_INDEX gfxIndex, GFX_GOL_MESSAGE *pMsg);

// *****************************************************************************
/*  GFX GOL panel parameter set.
    
    Function:
        void GFX_GOL_PanelParameterSet(
                                SYS_MODULE_INDEX gfxIndex
                                uint16_t left,
                                uint16_t top,
                                uint16_t right,
                                uint16_t bottom,
                                uint16_t radius,
                                GFX_COLOR faceClr,
                                GFX_COLOR embossLtClr,
                                GFX_COLOR embossDkClr,
                                GFX_RESOURCE_HDR *pBitmap,
                                GFX_FILL_STYLE fillStyle,
                                uint16_t embossSize)

    Summary:
        This function sets the parameters to draw a panel.

    Description:
        This function sets the parameters to draw a panel.
        Panel is not an object. It is a routine to draw a basic
        component of objects. The actual drawing of the panel
        is performed by the GFX_GOL_PanelDraw(). After the parameters
        are set, call GFX_GOL_PanelDraw() to render the panel.
        The panel is drawn using the following:
        1. Panel width is determined by right - left.
        2. Panel height is determined by top - bottom.
        3. Panel radius - specifies if the panel will have a rounded
                          edge. If zero then the panel will have
                          sharp (cornered) edge.
        4. If 2*radius = height = width, the panel is circular.
        5. If the panel is drawn with an image, pBitmap should point
           to an image resource.
        6. If the panel face is drawn with the fill style specified
           by fillStyle. When gradient fill is used, set the gradient
           colors using GFX_GOL_PanelGradientParameterSet(). When
           alpha blending fill is used, set the alpha blending value
           using GFX_GOL_PanelAlphaParameterSet().

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        left - defines the left most pixel of the panel.
        top - defines the top most pixel of the panel.
        right - defines the right most pixel of the panel.
        bottom - defines the bottom most pixel of the panel.
        radius - defines the radius of the rounded corner. A zero value
                 will result in a rectangular panel drawn.
        faceClr - the color used for the face of the panel.
        embossLtClr - the color used for the light emboss color
                      for 3D effect.
        embossDkClr - the color used for the dark emboss color
                      for 3D effect.
        pBitmap - pointer to the image resource of the panel.
        fillStyle - fill style use for the face of the panel.
        embossSize - when this is not zero, the embossLtClr and embossDkClr
                     are used to draw the 3D effect. When this is set
                     to zero, there will be no 3D effect.

    Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
void GFX_GOL_PanelParameterSet(
                                SYS_MODULE_INDEX gfxIndex,
                                uint16_t left,
                                uint16_t top,
                                uint16_t right,
                                uint16_t bottom,
                                uint16_t radius,
                                GFX_COLOR faceClr,
                                GFX_COLOR embossLtClr,
                                GFX_COLOR embossDkClr,
                                GFX_RESOURCE_HDR *pBitmap,
                                GFX_FILL_STYLE fillStyle,
                                uint16_t embossSize);

// *****************************************************************************
/*  GFX GOL panel background set.
    
    Function:
        void GFX_GOL_PanelBackgroundSet( SYS_MODULE_INDEX gfxIndex,
                                            GFX_GOL_OBJ_HEADER *pObjectHeader)

    Summary:
        This function sets panel background information.

    Description:
        This function sets panel background information.
        This is an internal function and should not be called
        by the application.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObjectHeader - the object header of the object that needs to
                        draw a panel with background.
 
     Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
void GFX_GOL_PanelBackgroundSet( SYS_MODULE_INDEX gfxIndex,
                                 GFX_GOL_OBJ_HEADER *pObjectHeader );

// *****************************************************************************
/*  GFX GOL panel gradient parameter set.
    
    Function:
        void GFX_GOL_PanelGradientParameterSet( SYS_MODULE_INDEX gfxIndex,
                                GFX_COLOR startColor,
                                GFX_COLOR endColor)

    Summary:
        This function sets the gradient fill start and end colors
        of a panel.

    Description:
        This function sets the gradient fill start and end colors
        of a panel. This along with the parameters set by the function
        GFX_GOL_PanelParameterSet() will determine how the panel will
        be drawn. The actual drawing of the panel is performed by
        GFX_GOL_PanelDraw().

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        startColor - the gradient fill start color.
        endColor - the gradient fill end color.

    Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
#ifndef GFX_CONFIG_GRADIENT_DISABLE
void GFX_GOL_PanelGradientParameterSet(
                                SYS_MODULE_INDEX gfxIndex,
                                GFX_COLOR startColor,
                                GFX_COLOR endColor);
#endif

// *****************************************************************************
/*  GFX GOL panel alpha parameter set.
    
    Function:
        void GFX_GOL_PanelAlphaParameterSet( SYS_MODULE_INDEX gfxIndex,
                                uint16_t alphaValue)

    Summary:
        This function sets the alpha blending value when using
        alpha blending in panels.

    Description:
        This function sets the alpha blending value when using
        alpha blending in panels. This along with the parameters set
        by the function GFX_GOL_PanelParameterSet() will determine
        how the panel will be drawn. The actual drawing of the panel
        is performed by GFX_GOL_PanelDraw().

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        alphaVlaue - the alpha value used to render alpha blended panels.

    Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
void GFX_GOL_PanelAlphaParameterSet(SYS_MODULE_INDEX gfxIndex,
                                uint16_t alphaValue);
#endif

// *****************************************************************************
/*  GFX GOL panel draw.
    
    Function:
        GFX_STATUS GFX_GOL_PanelDraw(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function renders the panel.

    Description:
        This function renders the panel. Panel parameters are
        set by the GFX_GOL_PanelParameterSet() and
        GFX_GOL_PanelGradientParameterSet() or GFX_GOL_PanelAlphaParameterSet().
        The function returns success (GFX_STATUS_SUCCESS) when the panel
        is rendered. If the function returned not success this function
        must be called again until success is returned.

    Precondition:
        Panel parameters must be set first using GFX_GOL_PanelParameterSet()
        and GFX_GOL_PanelGradientParameterSet() or
        GFX_GOL_PanelAlphaParameterSet().

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        GFX_STATUS_SUCCESS - when the panel rendering is done.
        GFX_STATUS_FAILURE - when the panel rendering is not yet done.

    Example:
        None.
 
*/
// *****************************************************************************
GFX_STATUS GFX_GOL_PanelDraw(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL two tone panel draw.
    
    Function:
        GFX_STATUS GFX_GOL_TwoTonePanelDraw(SYS_MODULE_INDEX gfxIndex)

    Summary:
        This function renders the two-tone panel.

    Description:
        This function renders the two-tone panel. Panel parameters are
        set by the GFX_GOL_PanelParameterSet(). The function returns
        success (GFX_STATUS_SUCCESS) when the panel is rendered. If
        the function returned not success this function must be
        called again until success is returned.

    Precondition:
        Panel parameters must be set first using GFX_GOL_PanelParameterSet().

    Parameters:
        gfxIndex - Object index for the specified module instance.

    Returns:
        GFX_STATUS_SUCCESS - when the panel rendering is done.
        GFX_STATUS_FAILURE - when the panel rendering is not yet done.

    Example:
        None.
*/
// *****************************************************************************
GFX_STATUS GFX_GOL_TwoTonePanelDraw(SYS_MODULE_INDEX gfxIndex);

// *****************************************************************************
/*  GFX GOL object hide draw.
    
    Function:
        GFX_STATUS GFX_GOL_ObjectHideDraw( SYS_MODULE_INDEX gfxIndex,
                                           GFX_GOL_OBJ_HEADER *pObject )

    Summary:
        This function performs the hiding of an object from the screen.

    Description:
        This function performs the hiding of an object from the screen.
        If the object's style scheme is set to have a background, the
        background is taken into account.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObject - pointer to the object that will be checked.

    Returns:
        GFX_STATUS_SUCCESS - when the hiding is done.
        GFX_STATUS_FAILURE - when the hiding is not yet done.

    Example:
        None.
*/
// *****************************************************************************
GFX_STATUS GFX_GOL_ObjectHideDraw( SYS_MODULE_INDEX gfxIndex,
                                   GFX_GOL_OBJ_HEADER *pObject);

// *****************************************************************************
/*  GFX GOL object back ground set.

    Function:
        GFX_STATUS GFX_GOL_ObjectBackGroundSet(SYS_MODULE_INDEX gfxIndex, 
                                              GFX_GOL_OBJ_HEADER *pObjectHeader)

    Summary:
        This function sets the background information.

    Description:
        This function sets the background information. This is an internal
        function and should not be called by the application. This function
        is used by object's drawing functions to set the background
        information.

    Precondition:
        None.

    Parameters:
        gfxIndex - Object index for the specified module instance.
        pObjectHeader - the object header of the object that needs to
                        draw a panel with background.

    Returns:
        None.

    Example:
        None.
*/
// *****************************************************************************
void GFX_GOL_ObjectBackGroundSet( SYS_MODULE_INDEX gfxIndex,
                                  GFX_GOL_OBJ_HEADER *pObjectHeader );

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_GOL_H
