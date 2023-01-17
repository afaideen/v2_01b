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

/*** Default GFX GOL Scheme ***/
#define GFX_SCHEMEDEFAULT GOLSchemeDefault
extern GFX_GOL_OBJ_SCHEME GFX_SCHEMEDEFAULT;

/*** HGC-specified GFX GOL Scheme ***/
GFX_GOL_OBJ_SCHEME *gfxScheme0;

/*** Generated Asset References ***/
extern const GFX_RESOURCE_HDR MCHP_LOGO;
extern const GFX_RESOURCE_HDR USB_CONNECTED;
extern const GFX_RESOURCE_HDR USB_NOT_CONNECTED;
extern const GFX_RESOURCE_HDR Mute;
extern const GFX_RESOURCE_HDR VolumeMAX;
extern const GFX_RESOURCE_HDR VolumeMIN;
extern const GFX_RESOURCE_HDR Arial12pt;
extern const GFX_RESOURCE_HDR Arial14pt;
extern const GFX_RESOURCE_HDR Arial20pt;

/*** Generated ASCII Text Labels ***/
static const GFX_XCHAR staticTextStr_0[] = { 0x50, 0x49, 0x43, 0x33, 0x32, 0x4d, 0x5a, 0x32, 0x30, 0x34, 0x38, 0x45, 0x46, 0x48, 0x31, 0x34, 0x34, 0x4c, 0x20, 0x2b, 0x20, 0x50, 0x49, 0x4d, 0x20, 0x2b, 0x20, 0x42, 0x54, 0x41, 0x44, 0x4b, 0x0 }; // PIC32MZ2048EFH144L + PIM + BTADK
static const GFX_XCHAR staticTextStr_1[] = { 0x76, 0x32, 0x2e, 0x30, 0x30, 0x62, 0x0 }; // v2.00b
static const GFX_XCHAR staticTextStr_2[] = { 0x48, 0x61, 0x72, 0x6d, 0x6f, 0x6e, 0x79, 0x20, 0x55, 0x53, 0x42, 0x20, 0x53, 0x70, 0x65, 0x61, 0x6b, 0x65, 0x72, 0x0 }; // Harmony USB Speaker
static const GFX_XCHAR staticTextStr_3[] = { 0x55, 0x53, 0x42, 0x20, 0x3a, 0x20, 0x76, 0x31, 0x2e, 0x31, 0x0 }; // USB : v1.1
static const GFX_XCHAR staticTextStr_4[] = { 0x53, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x46, 0x72, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x63, 0x79, 0x3a, 0x0 }; // Sample Frequency:
static const GFX_XCHAR staticTextStr_5[] = { 0x34, 0x38, 0x4b, 0x48, 0x7a, 0x0 }; // 48KHz
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
    GFX_GOL_MessageCallbackSet(GFX_INDEX_0, &GFX_HGC_MessageCallback);
    GFX_GOL_DrawCallbackSet(GFX_INDEX_0, &GFX_HGC_DrawCallback);

    GFX_HGC_InitializeSchemes();

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
            break;

        default:
            break;
    }
}

/******************************************************************************
  Function:
    void GFX_HGC_SchemeCreate ( void )

  Summary:
    Allocates memory for a new scheme
 */
GFX_GOL_OBJ_SCHEME *GFX_HGC_SchemeCreate(void)
{
    GFX_GOL_OBJ_SCHEME  *pTemp;

    pTemp = (GFX_GOL_OBJ_SCHEME *)GFX_malloc(sizeof(GFX_GOL_OBJ_SCHEME));

    if(pTemp != NULL)
    {
        memcpy(pTemp, &GFX_SCHEMEDEFAULT, sizeof(GFX_GOL_OBJ_SCHEME));
    }

    return (pTemp);
}

/******************************************************************************
  Function:
    void GFX_HGC_InitializeSchemes ( void )

  Summary:
    Initializes all HGC-specified schemes
 */
void GFX_HGC_InitializeSchemes( void )
{
    gfxScheme0 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme0->AlphaValue =        GFX_SCHEME_ALPHA_0;
#endif
    gfxScheme0->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_0, GFX_SCHEME_PRIMARY_COLOR_GREEN_0, GFX_SCHEME_PRIMARY_COLOR_BLUE_0);
    gfxScheme0->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_0, GFX_SCHEME_SECONDARY_COLOR_GREEN_0, GFX_SCHEME_SECONDARY_COLOR_BLUE_0);
    gfxScheme0->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_0, GFX_SCHEME_DISABLED_COLOR_GREEN_0, GFX_SCHEME_DISABLED_COLOR_BLUE_0);
    gfxScheme0->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_0, GFX_SCHEME_BACKGROUND_COLOR_GREEN_0, GFX_SCHEME_BACKGROUND_COLOR_BLUE_0);
	gfxScheme0->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_0;
    gfxScheme0->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_0, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_0, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_0);
    gfxScheme0->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_0, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_0, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_0);
	gfxScheme0->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_0, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_0, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_0);
    gfxScheme0->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_0, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_0, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_0);
    gfxScheme0->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_0, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_0, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_0);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme0->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_0, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_0, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_0);
	gfxScheme0->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_0, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_0, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_0);
#endif
    gfxScheme0->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_0;
    gfxScheme0->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_0;
    gfxScheme0->pFont =             (GFX_RESOURCE_HDR*)&Arial12pt;
    gfxScheme0->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme0->fillStyle =         GFX_SCHEME_FILL_STYLE_0;
	gfxScheme0->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_0;

}

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
bool GFX_HGC_MessageCallback(GFX_GOL_TRANSLATED_ACTION objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    // process messages for demo screens
    switch (hgcObj.screenState)
    {
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_Screen1:
            if (GFX_HGC_MsgStaticTexts(objMsg, pObj, pMsg) == true)
                return true;
            break;
      default:
            // process message by default
            return true;
    }
    return true;
}

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
bool GFX_HGC_DrawCallback( void )
{
    switch (hgcObj.screenState)
    {
        case HGC_SCREEN_STATE_INIT:
            //Draw the primary screen as selected in the Composer
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_Screen1);
            return true;
        case HGC_SCREEN_STATE_SETUP_SCREEN_Screen1:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_Screen1:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_GOL_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_DRAW_GOL_SCREEN_Screen1:
            GFX_HGC_SetupScreen(Screen1);
            GFX_HGC_DrawScreen_GOL(Screen1);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_Screen1);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_Screen1:
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
            return true; // release drawing control to GOL by default
    }

    return true; // release drawing control to GOL
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
			
            GFX_GOL_ObjectListFree(GFX_INDEX_0); // free memory for the objects in the previous linked list and start new list
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
    bool GFX_HGC_DrawScreen_GOL( uint8_t  screenId )

  Output: 
    Returns true if a screenId matches an existing screen identifier

  Remarks: 
    HGC-specified GFX GOL objects are drawn here, grouped by screen.
    The draw order is reverse Z-ordered.
 */
bool GFX_HGC_DrawScreen_GOL(uint8_t screenId)
{
    switch (screenId)
    {
        case Screen1:
            GFX_HGC_DrawItem(SampleFreq);
            GFX_HGC_DrawItem(SampleFreqTxt);
            GFX_HGC_DrawItem(UsbTxt);
            GFX_HGC_DrawItem(TitleBar);
            GFX_HGC_DrawItem(HarmonyVersion);
            GFX_HGC_DrawItem(StaticText1);
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
    case StaticText1:
        hgcObj.pStaticText1Obj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 StaticText1,      // button ID
                 GFX_STATICTEXT_LEFT_0,    // left
                 GFX_STATICTEXT_TOP_0,     // top
                 GFX_STATICTEXT_RIGHT_0,   // right
                 GFX_STATICTEXT_BOTTOM_0,  // bottom
                 GFX_STATICTEXT_STATE_0,   // state
                 (GFX_XCHAR*)staticTextStr_0,    // text
                 GFX_STATICTEXT_ALIGNMENT_0,       //alignment
                 GFX_STATICTEXT_SCHEME_0
                 ); // use alternate scheme
        break;
    case HarmonyVersion:
        hgcObj.pHarmonyVersionObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 HarmonyVersion,      // button ID
                 GFX_STATICTEXT_LEFT_1,    // left
                 GFX_STATICTEXT_TOP_1,     // top
                 GFX_STATICTEXT_RIGHT_1,   // right
                 GFX_STATICTEXT_BOTTOM_1,  // bottom
                 GFX_STATICTEXT_STATE_1,   // state
                 (GFX_XCHAR*)staticTextStr_1,    // text
                 GFX_STATICTEXT_ALIGNMENT_1,       //alignment
                 GFX_STATICTEXT_SCHEME_1
                 ); // use alternate scheme
        break;
    case TitleBar:
        hgcObj.pTitleBarObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 TitleBar,      // button ID
                 GFX_STATICTEXT_LEFT_2,    // left
                 GFX_STATICTEXT_TOP_2,     // top
                 GFX_STATICTEXT_RIGHT_2,   // right
                 GFX_STATICTEXT_BOTTOM_2,  // bottom
                 GFX_STATICTEXT_STATE_2,   // state
                 (GFX_XCHAR*)staticTextStr_2,    // text
                 GFX_STATICTEXT_ALIGNMENT_2,       //alignment
                 GFX_STATICTEXT_SCHEME_2
                 ); // use alternate scheme
        break;
    case UsbTxt:
        hgcObj.pUsbTxtObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 UsbTxt,      // button ID
                 GFX_STATICTEXT_LEFT_3,    // left
                 GFX_STATICTEXT_TOP_3,     // top
                 GFX_STATICTEXT_RIGHT_3,   // right
                 GFX_STATICTEXT_BOTTOM_3,  // bottom
                 GFX_STATICTEXT_STATE_3,   // state
                 (GFX_XCHAR*)staticTextStr_3,    // text
                 GFX_STATICTEXT_ALIGNMENT_3,       //alignment
                 GFX_STATICTEXT_SCHEME_3
                 ); // use alternate scheme
        break;
    case SampleFreqTxt:
        hgcObj.pSampleFreqTxtObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 SampleFreqTxt,      // button ID
                 GFX_STATICTEXT_LEFT_4,    // left
                 GFX_STATICTEXT_TOP_4,     // top
                 GFX_STATICTEXT_RIGHT_4,   // right
                 GFX_STATICTEXT_BOTTOM_4,  // bottom
                 GFX_STATICTEXT_STATE_4,   // state
                 (GFX_XCHAR*)staticTextStr_4,    // text
                 GFX_STATICTEXT_ALIGNMENT_4,       //alignment
                 GFX_STATICTEXT_SCHEME_4
                 ); // use alternate scheme
        break;
    case SampleFreq:
        hgcObj.pSampleFreqObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 SampleFreq,      // button ID
                 GFX_STATICTEXT_LEFT_5,    // left
                 GFX_STATICTEXT_TOP_5,     // top
                 GFX_STATICTEXT_RIGHT_5,   // right
                 GFX_STATICTEXT_BOTTOM_5,  // bottom
                 GFX_STATICTEXT_STATE_5,   // state
                 (GFX_XCHAR*)staticTextStr_5,    // text
                 GFX_STATICTEXT_ALIGNMENT_5,       //alignment
                 GFX_STATICTEXT_SCHEME_5
                 ); // use alternate scheme
        break;
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
			GFX_XCHAR Text1_text[] = { 0x43, 0x6f, 0x64, 0x65, 0x63, 0x3a, 0x20, 0x41, 0x4b, 0x34, 0x33, 0x38, 0x34, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   14, // draw point x
						       93, // draw point y
                               Text1_text, // string
							   0); // length
			break;
		}	
	default:
		return false; // process by default
    }

    return true;
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgStaticTexts(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Static Text events
 */
bool GFX_HGC_MsgStaticTexts(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        default:
            return false; // default false as it is not processed
    }    
}


