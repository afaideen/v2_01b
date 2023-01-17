/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    gfx_hgc_definitions.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
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

#include "gfx_hgc_definitions.h"

/*** Generated Asset References ***/
extern const GFX_RESOURCE_HDR MCHP_LOGO;
extern const GFX_RESOURCE_HDR Arial12pt;
extern const GFX_RESOURCE_HDR Arial14pt;
extern const GFX_RESOURCE_HDR Arial20pt;

/*** Generated ASCII Text Labels ***/
/*** HGC Object Global ***/
HGC_OBJECTS hgcObj;
static HGC_STATES hgcState;

/******************************************************************************
  Function:
    HGC_SCREEN_STATES GFX_HGC_GetScreenState ( void )

  Remarks:
    This function returns the screen state
 */
HGC_SCREEN_STATES GFX_HGC_GetScreenState ( void )
{
    return hgcObj.screenState;
}


/******************************************************************************
  Function:
    void GFX_HGC_SetScreenState ( HGC_SCREEN_STATES newState )

  Remarks:
    This function sets the screen state machine to a new state
 */
void GFX_HGC_SetScreenState ( HGC_SCREEN_STATES newState )
{
    hgcObj.prevRefreshState = hgcObj.screenState;
    hgcObj.screenState = newState;
}


/******************************************************************************
  Function:
    void GFX_HGC_Initialize ( void )

  Summary:
    This function initializes the HGC state machine.  
 */
void GFX_HGC_Initialize ( void )
{
    hgcState = HGC_STATE_INIT;
    hgcObj.screenState = HGC_SCREEN_STATE_INIT;
}


/******************************************************************************
  Function:
    void GFX_HGC_Setup ( void )

  Summary:
    This function sets up the GOL message callback and draw callbacks.  
 */
void GFX_HGC_Setup ( void )
{
}

/******************************************************************************
  Function:
    void GFX_HGC_Tasks (SYS_MODULE_OBJ gfxObject);

  Summary:
    This function is called in SYS_Tasks.  The intent wait until the GFX library
    is initialized before supplying items to draw
*/
void GFX_HGC_Tasks  (SYS_MODULE_OBJ gfxObject)
{
    switch ( hgcState )
    {
        case HGC_STATE_INIT:
            if (GFX_Status(gfxObject)==SYS_STATUS_READY)
            {
                GFX_HGC_Setup();
                hgcState = HGC_STATE_RUNNING;
            }
            break;

        case HGC_STATE_RUNNING:
            //Drive screen state machine
            GFX_HGC_DrawScreenTask();
            break;

        default:
            break;
    }
}

/******************************************************************************
  Function: 
    void GFX_HGC_DrawScreenTask( void )

  Remarks: 
    Task loop to drive the HGC generated screen state machine for HGC design
    that has primitives only
 */
void GFX_HGC_DrawScreenTask( void )
{
    switch (hgcObj.screenState)
    {
        case HGC_SCREEN_STATE_INIT:
            //Draw the primary screen as selected in the Composer
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_SETUP_SCREEN_Screen1:
            GFX_HGC_SetupScreen(Screen1);

            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_Screen1:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_Screen1:
            GFX_HGC_DrawScreen_Primitives(Screen1);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_Screen1:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DISPLAY_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_Screen1:
            break;
        default:
            break;
    }
}

/******************************************************************************
  Function: 
    void GFX_HGC_SetupScreen( uint8_t  screenId )

  Remarks: 
    Clears current screen and starts a fresh screen with its background color
 */
void GFX_HGC_SetupScreen(uint8_t screenId)
{
    switch (screenId)
    {
        case Screen1:
            GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0x00,0x00,0x00));
            GFX_ScreenClear(GFX_INDEX_0);
			
            break;
        default:
            break;
    }
}


/******************************************************************************
  Function: 
    bool GFX_HGC_HasCurrentScreenDrawn( void )

  Output: 
    Returns true if the current screen is completely drawn

  Remarks: 
    Allows application to know if the current screen is finished drawing
 */
bool GFX_HGC_HasCurrentScreenDrawn( void )
{
    switch (hgcObj.screenState)
    {
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_Screen1:
            return true;
        default:
            return false;
    }

    return false;
}


/******************************************************************************
  Function: 
    bool GFX_HGC_ChangeScreen( uint8_t  screenId )

  Output: 
    Returns true if a screenId matches an existing screen identifier

  Remarks: 
    Can be called to initiate GFX Library to draw a new screen
 */
bool GFX_HGC_ChangeScreen(uint8_t screenId)
{
    switch (screenId)
    {
        case Screen1:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_Screen1);
            break;
        default:
            return false;
    }

    return true; // release drawing control to GOL
}

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
bool GFX_HGC_DrawScreen_Primitives(uint8_t screenId)
{
    switch (screenId)
    {
        case Screen1:
            GFX_HGC_DrawItem(Rectangle1);
            GFX_HGC_DrawItem(Image1);
            GFX_HGC_DrawItem(DemoName);
            GFX_HGC_DrawItem(Frequency);
            GFX_HGC_DrawItem(Version);
            GFX_HGC_DrawItem(CodecType);
            GFX_HGC_DrawItem(Text1);
            break;
        default:
            return true;
    }

    return true; // release drawing control to GOL
}


/******************************************************************************
  Function: 
    bool GFX_HGC_DrawItem(int itemId)

  Output:
    Returns true if a itemId matches an existing item identifier

  Remarks: 
    Every item specified in every screen in HGC is listed in this function
 */
bool GFX_HGC_DrawItem(int itemId)
{
    switch(itemId)
    {
		case Rectangle1:
		{
			GFX_FillStyleSet(GFX_INDEX_0, GFX_FILL_STYLE_GRADIENT_DOWN);
			GFX_GradientColorSet(GFX_INDEX_0,
			                    GFX_RGBConvert(0xFF, 0x00, 0x00),
			                    GFX_RGBConvert(0x00, 0x00, 0x00));	
			GFX_RectangleRoundFillDraw(GFX_INDEX_0,
									   15, // p1x
									   10, // p1y
									   205, // p2x
									   19, // p2y
									   15); // radius
			
			break;
		}	
		case Image1:
		{
			GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
			GFX_ImageDraw(GFX_INDEX_0,
			              4, // draw point x
						  1, // draw point y
						  (GFX_RESOURCE_HDR*)&MCHP_LOGO); // image
			GFX_TransparentColorDisable(GFX_INDEX_0); // disable transparency
			break;
		}	
		case Text1:
		{
			GFX_XCHAR Text1_text[] = { 0x70, 0x69, 0x63, 0x33, 0x32, 0x4d, 0x58, 0x34, 0x37, 0x30, 0x46, 0x35, 0x31, 0x32, 0x4c, 0x20, 0x2b, 0x20, 0x41, 0x4b, 0x34, 0x36, 0x34, 0x32, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   30, // draw point x
						       43, // draw point y
                               Text1_text, // string
							   0); // length
			break;
		}	
		case CodecType:
		{
			GFX_XCHAR CodecType_text[] = { 0x43, 0x6f, 0x64, 0x65, 0x63, 0x3a, 0x20, 0x41, 0x4b, 0x34, 0x36, 0x34, 0x32, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   31, // draw point x
						       79, // draw point y
                               CodecType_text, // string
							   0); // length
			break;
		}	
		case Version:
		{
			GFX_XCHAR Version_text[] = { 0x55, 0x53, 0x42, 0x20, 0x3a, 0x20, 0x76, 0x31, 0x2e, 0x31, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   31, // draw point x
						       94, // draw point y
                               Version_text, // string
							   0); // length
			break;
		}	
		case Frequency:
		{
			GFX_XCHAR Frequency_text[] = { 0x53, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x46, 0x72, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x63, 0x79, 0x20, 0x3a, 0x20, 0x34, 0x38, 0x4b, 0x48, 0x7a, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   31, // draw point x
						       65, // draw point y
                               Frequency_text, // string
							   0); // length
			break;
		}	
		case DemoName:
		{
			GFX_XCHAR DemoName_text[] = { 0x48, 0x61, 0x72, 0x6d, 0x6f, 0x6e, 0x79, 0x20, 0x55, 0x53, 0x42, 0x20, 0x53, 0x70, 0x65, 0x61, 0x6b, 0x65, 0x72, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFC, 0xFC, 0xFC)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial14pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   30, // draw point x
						       24, // draw point y
                               DemoName_text, // string
							   0); // length
			break;
		}	
	default:
		return false; // process by default
    }

    return true;
}


