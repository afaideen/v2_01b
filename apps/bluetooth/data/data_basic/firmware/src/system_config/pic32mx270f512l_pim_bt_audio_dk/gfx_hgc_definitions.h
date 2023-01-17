/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    gfx_hgc_definitions.h

  Summary:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version 2.01
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


#ifndef _GFX_HGC_DEFINITIONS_H
#define _GFX_HGC_DEFINITIONS_H

#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

typedef enum
{
    HGC_STATE_INIT = 0,
    HGC_STATE_RUNNING
} HGC_STATES;

typedef enum
{
    HGC_SCREEN_STATE_INIT = 0,
    HGC_SCREEN_STATE_SETUP_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_DRAW_GOL_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_HomeScreen,
    HGC_SCREEN_STATE_DISPLAY_SCREEN_HomeScreen,
	
} HGC_SCREEN_STATES;

/*** GFX GOL Scheme Definitions ***/

#define GFX_SCHEME_NAME_0                       "default"
#define GFX_SCHEME_ALPHA_0                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_0      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_0     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_0          0xFF
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_0        0xFF
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_0         0xFF

#define GFX_SCHEME_SECONDARY_COLOR_RED_0        0x40
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_0      0x40
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_0       0x40

#define GFX_SCHEME_DISABLED_COLOR_RED_0         0xC0
#define GFX_SCHEME_DISABLED_COLOR_GREEN_0       0xC0
#define GFX_SCHEME_DISABLED_COLOR_BLUE_0        0xC0

#define GFX_SCHEME_BACKGROUND_COLOR_RED_0       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_0     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_0      0x00

#define GFX_SCHEME_EMBOSS_SIZE_0                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_0     0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_0   0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_0    0xC0

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_0      0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_0    0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_0     0x40

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_0     0xFF
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_0   0xFF
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_0    0xFF

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_0   0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_0 0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_0  0xFF

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_0    0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_0  0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_0   0x40

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_0    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_0  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_0   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_0    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_0  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_0   0xFF

#define GFX_SCHEME_FILL_STYLE_0                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_0           GFX_BACKGROUND_NONE

		
/*** Screen IDs ***/
#define HomeScreen                 0 
	
/*** GFX GOL Button Definitions ***/
#define GFX_STATICTEXT_LEFT_0            140
#define GFX_STATICTEXT_TOP_0             0
#define GFX_STATICTEXT_RIGHT_0           176
#define GFX_STATICTEXT_BOTTOM_0          15
#define GFX_STATICTEXT_STATE_0          GFX_GOL_STATICTEXT_DRAW_STATE | GFX_GOL_STATICTEXT_NOBACKGROUND_STATE
#define GFX_STATICTEXT_TEXT_0            "0x76, 0x30, 0x2e, 0x30, 0x30, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_0       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_0          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_1            208
#define GFX_STATICTEXT_TOP_1             28
#define GFX_STATICTEXT_RIGHT_1           222
#define GFX_STATICTEXT_BOTTOM_1          43
#define GFX_STATICTEXT_STATE_1          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_1            "0x30, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_1       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_1          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_2            10
#define GFX_STATICTEXT_TOP_2             110
#define GFX_STATICTEXT_RIGHT_2           110
#define GFX_STATICTEXT_BOTTOM_2          128
#define GFX_STATICTEXT_STATE_2          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_2            "0x0"
#define GFX_STATICTEXT_ALIGNMENT_2       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_2          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_3            10
#define GFX_STATICTEXT_TOP_3             93
#define GFX_STATICTEXT_RIGHT_3           110
#define GFX_STATICTEXT_BOTTOM_3          111
#define GFX_STATICTEXT_STATE_3          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_3            "0x0"
#define GFX_STATICTEXT_ALIGNMENT_3       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_3          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_4            10
#define GFX_STATICTEXT_TOP_4             60
#define GFX_STATICTEXT_RIGHT_4           110
#define GFX_STATICTEXT_BOTTOM_4          78
#define GFX_STATICTEXT_STATE_4          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_4            "0x0"
#define GFX_STATICTEXT_ALIGNMENT_4       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_4          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_5            10
#define GFX_STATICTEXT_TOP_5             77
#define GFX_STATICTEXT_RIGHT_5           110
#define GFX_STATICTEXT_BOTTOM_5          95
#define GFX_STATICTEXT_STATE_5          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_5            "0x0"
#define GFX_STATICTEXT_ALIGNMENT_5       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_5          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_6            115
#define GFX_STATICTEXT_TOP_6             40
#define GFX_STATICTEXT_RIGHT_6           221
#define GFX_STATICTEXT_BOTTOM_6          59
#define GFX_STATICTEXT_STATE_6          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_6            "0x4d, 0x41, 0x43, 0x20, 0x41, 0x44, 0x44, 0x52, 0x45, 0x53, 0x53, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_6       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_6          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_7            30
#define GFX_STATICTEXT_TOP_7             40
#define GFX_STATICTEXT_RIGHT_7           130
#define GFX_STATICTEXT_BOTTOM_7          57
#define GFX_STATICTEXT_STATE_7          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_7            "0x42, 0x54, 0x20, 0x4e, 0x61, 0x6d, 0x65, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_7       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_7          gfxScheme0
		
/*** static text IDs ***/
#define HarmonyVersion                  9000 
	
#define BtConnected                  9001 
	
#define Buffer4                  9002 
	
#define Buffer3                  9003 
	
#define Buffer1                  9004 
	
#define Buffer2                  9005 
	
#define MacId                  9006 
	
#define BroadCastName                  9007 
	
/*** Rectangle IDs ***/
#define LED1                 16000 
	
#define RGBLED                 16001 
	
#define LED4                 16002 
	
#define LED5                 16003 
	
#define LED3                 16004 
	
#define LED2                 16005 
	
#define RgbLedOutLine                 16006 
	
#define LedOutLine5                 16007 
	
#define LedOutLine4                 16008 
	
#define LedOutLine3                 16009 
	
#define LedOutLine2                 16010 
	
#define LedOutLine1                 16011 
	
#define BottomTaskBar                 16012 
	
#define TopTaskBar                 16013 
	
/*** Image IDs ***/
#define Image2                 18000 
	
#define MCHP_LOGO_NAME                 18001 
	
/*** Text IDs ***/
#define RGBLED_text                 19000 
	
#define Virtual_LED_Text1                 19001 
	
#define DemoName                 19002 
	



// *****************************************************************************
/* MPLAB Harmony Graphics Composer Objects

  Summary:
    Structure holding the graphics object handles

  Description:
    This structure contains the object handles for all graphics objects 
    generated by the Harmony Graphics Composer

  Remarks:
*/

typedef struct HGC_OBJECTS
{
    HGC_SCREEN_STATES screenState;      // current state of screen state machine
    HGC_SCREEN_STATES prevRefreshState; // previous state of screen state machine

    /*** GOL Objects References ***/
    GFX_GOL_STATICTEXT* pHarmonyVersionObj;
    GFX_GOL_STATICTEXT* pBtConnectedObj;
    GFX_GOL_STATICTEXT* pBuffer4Obj;
    GFX_GOL_STATICTEXT* pBuffer3Obj;
    GFX_GOL_STATICTEXT* pBuffer1Obj;
    GFX_GOL_STATICTEXT* pBuffer2Obj;
    GFX_GOL_STATICTEXT* pMacIdObj;
    GFX_GOL_STATICTEXT* pBroadCastNameObj;
} HGC_OBJECTS;

extern HGC_OBJECTS hgcObj;

/******************************************************************************
  Function:
    void GFX_HGC_Initialize ( void )

  Summary:
    This function initializes the HGC state machine.  
 */
void GFX_HGC_Initialize ( void );


/******************************************************************************
  Function:
    void GFX_HGC_Setup ( void )

  Summary:
    This function sets up the GOL message callback and draw callbacks.  
 */
void GFX_HGC_Setup ( void );


/******************************************************************************
  Function:
    HGC_SCREEN_STATES GFX_HGC_GetScreenState ( void )

  Remarks:
    This function returns the screen state
 */
HGC_SCREEN_STATES GFX_HGC_GetScreenState ( void );


/******************************************************************************
  Function:
    void GFX_HGC_SetScreenState ( HGC_SCREEN_STATES newState )

  Remarks:
    This function sets the screen state machine to a new state
 */
void GFX_HGC_SetScreenState ( HGC_SCREEN_STATES newState );


/******************************************************************************
  Function:
    void GFX_HGC_Tasks (SYS_MODULE_OBJ gfxObject);

  Summary:
    This function is called in SYS_Tasks.  The intent wait until the GFX library
    is initialized before supplying items to draw
*/
void GFX_HGC_Tasks  (SYS_MODULE_OBJ gfxObject);


/******************************************************************************
  Function:
    void GFX_HGC_SchemeCreate ( void )

  Summary:
    Allocates memory for a new scheme
 */
GFX_GOL_OBJ_SCHEME *GFX_HGC_SchemeCreate(void);


/******************************************************************************
  Function:
    void GFX_HGC_InitializeSchemes ( void )

  Summary:
    Initializes all HGC-specified schemes
 */
void GFX_HGC_InitializeSchemes( void );


/******************************************************************************
  Function: 
    bool GFX_HGC_MESSAGECallback(uint16_t objMsg, GFX_GOL_OBJ_HEADER* pObj, GFX_GOL_MESSAGE* pMsg)

  Input: 
    objMsg - translated message for the object,
    pObj - pointer to the object,
    pMsg - pointer to the non-translated, raw GOL message

  Output: 
    If the function returns non-zero the message will be processed by default

  Remarks: 
    GFX_GOL_MESSAGECallback() function calls it each time the valid message for the GOL object is received.
 */
bool GFX_HGC_MessageCallback(GFX_GOL_TRANSLATED_ACTION objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);


/******************************************************************************
  Function: 
    bool GFX_HGC_DrawCallback( void )

  Output: 
    If the function returns non-zero the draw control will be passed to GOL

  Remarks: 
    GFX_GOL_Draw() function calls it each time when GOL objects drawing is 
    completed. HGC-specified draw data is grouped by screens states. It takes 
    two states to complete the drawing of each screen.  GFX GOL objects are drawn
    first and then GFX Primitives are drawn next to ensure primitives are not
    covered by GOL objects.
 */
bool GFX_HGC_DrawCallback( void );
/******************************************************************************
  Function: 
    bool GFX_HGC_DrawScreen_GOL( uint8_t  screenId )

  Output: 
    Returns true if a screenId matches an existing screen identifier

  Remarks: 
    HGC-specified GFX GOL objects are drawn here, grouped by screen.
    The draw order is reverse Z-ordered.
 */
bool GFX_HGC_DrawScreen_GOL(uint8_t screenId);


/******************************************************************************
  Function: 
    bool GFX_HGC_DrawScreen_Primitives( uint8_t  screenId )

  Output:
    Returns true if a screenId matches an existing screen identifier

  Remarks: 
    HGC-specified GFX Primitives are drawn here, grouped by screen.
    GFX Primitives are drawn after GFX GOL Objects to make sure GFX GOL Objects
    do not cover GFX Primitives
 */
bool GFX_HGC_DrawScreen_Primitives(uint8_t screenId);


/******************************************************************************
  Function: 
    void GFX_HGC_SetupScreen( uint8_t  screenId )

  Remarks: 
    Clears current screen and starts a fresh screen with its background color
 */
void GFX_HGC_SetupScreen(uint8_t screenId);


/******************************************************************************
  Function: 
    bool GFX_HGC_HasCurrentScreenDrawn( void )

  Output: 
    Returns true if the current screen is completely drawn

  Remarks: 
    Allows application to know if the current screen is finished drawing
 */
bool GFX_HGC_HasCurrentScreenDrawn( void );


/******************************************************************************
  Function: 
    bool GFX_HGC_ChangeScreen( uint8_t  screenId )

  Output: 
    Returns true if a screenId matches an existing screen identifier

  Remarks: 
    Can be called to initiate GFX Library to draw a new screen
 */
bool GFX_HGC_ChangeScreen(uint8_t screenId);


/******************************************************************************
  Function: 
    bool GFX_HGC_DrawItem(int itemId)

  Output:
    Returns true if a itemId matches an existing item identifier

  Remarks: 
    Every item specified in every screen in HGC is listed in this function
 */
bool GFX_HGC_DrawItem(int itemId);


/******************************************************************************
  Function: 
    bool GFX_HGC_MsgStaticTexts(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Static Text events
 */
bool GFX_HGC_MsgStaticTexts(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _GFX_HGC_DEFINITIONS_H
/*******************************************************************************
 End of File
*/
