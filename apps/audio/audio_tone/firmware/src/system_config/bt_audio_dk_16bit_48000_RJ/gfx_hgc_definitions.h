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
    HGC_SCREEN_STATE_SETUP_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_DRAW_GOL_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_DemoScreen,
    HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen,
	
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

#define GFX_SCHEME_DISABLED_COLOR_RED_0         0x18
#define GFX_SCHEME_DISABLED_COLOR_GREEN_0       0x20
#define GFX_SCHEME_DISABLED_COLOR_BLUE_0        0xF8

#define GFX_SCHEME_BACKGROUND_COLOR_RED_0       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_0     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_0      0x60

#define GFX_SCHEME_EMBOSS_SIZE_0                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_0     0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_0   0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_0    0xC0

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_0      0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_0    0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_0     0x40

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_0     0xF8
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_0   0xFC
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_0    0xF8

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

		

#define GFX_SCHEME_NAME_1                       "ProgressBar"
#define GFX_SCHEME_ALPHA_1                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_1      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_1     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_1          0x98
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_1        0x98
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_1         0x98

#define GFX_SCHEME_SECONDARY_COLOR_RED_1        0x40
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_1      0x40
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_1       0x40

#define GFX_SCHEME_DISABLED_COLOR_RED_1         0xC0
#define GFX_SCHEME_DISABLED_COLOR_GREEN_1       0xC0
#define GFX_SCHEME_DISABLED_COLOR_BLUE_1        0xC0

#define GFX_SCHEME_BACKGROUND_COLOR_RED_1       0x90
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_1     0x94
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_1      0x90

#define GFX_SCHEME_EMBOSS_SIZE_1                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_1     0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_1   0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_1    0xC0

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_1      0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_1    0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_1     0x40

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_1     0x38
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_1   0x3C
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_1    0x38

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_1   0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_1 0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_1  0xFF

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_1    0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_1  0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_1   0x40

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_1    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_1  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_1   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_1    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_1  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_1   0xFF

#define GFX_SCHEME_FILL_STYLE_1                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_1           GFX_BACKGROUND_NONE

		
/*** Screen IDs ***/
#define DemoScreen                 0 
	
/*** GFX GOL Button Definitions ***/
#define GFX_BUTTON_LEFT_0            149
#define GFX_BUTTON_TOP_0             4
#define GFX_BUTTON_RIGHT_0           210
#define GFX_BUTTON_BOTTOM_0          29
#define GFX_BUTTON_TEXT_0            "0x42, 0x75, 0x74, 0x74, 0x6f, 0x6e, 0x0"
#define GFX_BUTTON_ALIGNMENT_0       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_BUTTON_RADIUS_0          0
#define GFX_BUTTON_STATE_0           GFX_GOL_BUTTON_DRAW_STATE
#define GFX_BUTTON_PRESS_IMAGE_0     NULL
#define GFX_BUTTON_RELEASE_IMAGE_0   NULL
#define GFX_BUTTON_SCHEME_0          gfxScheme0
		
/*** Button IDs ***/
#define Button1                  1000 
	
/*** GFX GOL Digital Meter Definitions ***/

#define GFX_DGTL_MTR_LEFT_0            157
#define GFX_DGTL_MTR_TOP_0             144
#define GFX_DGTL_MTR_RIGHT_0           198
#define GFX_DGTL_MTR_BOTTOM_0          169
#define GFX_DGTL_MTR_STATE_0           GFX_GOL_DIGITALMETER_DRAW_STATE
#define GFX_DGTL_MTR_VALUE_0           99999
#define GFX_DGTL_MTR_NO_OF_DIGITS_0    5
#define GFX_DGTL_MTR_DOT_POSITION_0    0
#define GFX_DGTL_MTR_ALIGNMENT_0       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_DGTL_MTR_SCHEME_0          gfxScheme1
		

#define GFX_DGTL_MTR_LEFT_1            68
#define GFX_DGTL_MTR_TOP_1             144
#define GFX_DGTL_MTR_RIGHT_1           109
#define GFX_DGTL_MTR_BOTTOM_1          169
#define GFX_DGTL_MTR_STATE_1           GFX_GOL_DIGITALMETER_DRAW_STATE
#define GFX_DGTL_MTR_VALUE_1           20000
#define GFX_DGTL_MTR_NO_OF_DIGITS_1    5
#define GFX_DGTL_MTR_DOT_POSITION_1    0
#define GFX_DGTL_MTR_ALIGNMENT_1       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_DGTL_MTR_SCHEME_1          gfxScheme1
		

#define GFX_DGTL_MTR_LEFT_2            12
#define GFX_DGTL_MTR_TOP_2             145
#define GFX_DGTL_MTR_RIGHT_2           53
#define GFX_DGTL_MTR_BOTTOM_2          170
#define GFX_DGTL_MTR_STATE_2           GFX_GOL_DIGITALMETER_DRAW_STATE
#define GFX_DGTL_MTR_VALUE_2           400
#define GFX_DGTL_MTR_NO_OF_DIGITS_2    5
#define GFX_DGTL_MTR_DOT_POSITION_2    0
#define GFX_DGTL_MTR_ALIGNMENT_2       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_DGTL_MTR_SCHEME_2          gfxScheme1
		
/*** Digital Meter IDs ***/
#define DigitalMeter1                  2000 
	
#define FreqMeter2                  2001 
	
#define FreqMeter                  2002 
	
/*** GFX GOL Button Definitions ***/
#define GFX_STATICTEXT_LEFT_0            78
#define GFX_STATICTEXT_TOP_0             69
#define GFX_STATICTEXT_RIGHT_0           166
#define GFX_STATICTEXT_BOTTOM_0          88
#define GFX_STATICTEXT_STATE_0          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_0            "0x31, 0x36, 0x20, 0x42, 0x69, 0x74, 0x73, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_0       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_0          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_1            121
#define GFX_STATICTEXT_TOP_1             53
#define GFX_STATICTEXT_RIGHT_1           209
#define GFX_STATICTEXT_BOTTOM_1          75
#define GFX_STATICTEXT_STATE_1          GFX_GOL_STATICTEXT_DRAW_STATE | GFX_GOL_STATICTEXT_NOBACKGROUND_STATE
#define GFX_STATICTEXT_TEXT_1            "0x34, 0x38, 0x30, 0x30, 0x30, 0x20, 0x48, 0x7a, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_1       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_1          gfxScheme0
		
#define GFX_STATICTEXT_LEFT_2            66
#define GFX_STATICTEXT_TOP_2             97
#define GFX_STATICTEXT_RIGHT_2           184
#define GFX_STATICTEXT_BOTTOM_2          113
#define GFX_STATICTEXT_STATE_2          GFX_GOL_STATICTEXT_DRAW_STATE
#define GFX_STATICTEXT_TEXT_2            "0x53, 0x69, 0x6e, 0x65, 0x20, 0x54, 0x6f, 0x6e, 0x65, 0x20, 0x2d, 0x20, 0x4f, 0x4e, 0x0"
#define GFX_STATICTEXT_ALIGNMENT_2       GFX_ALIGN_LEFT | GFX_ALIGN_TOP
#define GFX_STATICTEXT_SCHEME_2          gfxScheme0
		
/*** static text IDs ***/
#define StaticText1                  9000 
	
#define SampleRateTxt                  9001 
	
#define ToneMode                  9002 
	
/*** Rectangle IDs ***/
#define Highlight3                 16000 
	
#define Highlight2                 16001 
	
#define Highlight1                 16002 
	
#define Rectangle1                 16003 
	
/*** Image IDs ***/
#define MchpLogo                 18000 
	
/*** Text IDs ***/
#define Text1                 19000 
	
#define DemoName                 19001 
	
#define DurationText                 19002 
	
#define HzText2                 19003 
	
#define HzText1                 19004 
	
#define Frequency                 19005 
	



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
    GFX_GOL_BUTTON* pButton1Obj;
    GFX_GOL_DIGITALMETER* pDigitalMeter1Obj;
    GFX_GOL_DIGITALMETER* pFreqMeter2Obj;
    GFX_GOL_DIGITALMETER* pFreqMeterObj;
    GFX_GOL_STATICTEXT* pStaticText1Obj;
    GFX_GOL_STATICTEXT* pSampleRateTxtObj;
    GFX_GOL_STATICTEXT* pToneModeObj;
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
    bool GFX_HGC_MsgButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Button events
 */
bool GFX_HGC_MsgButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgDigitalMeters(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Digital Meter events
 */
bool GFX_HGC_MsgDigitalMeters(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);

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
