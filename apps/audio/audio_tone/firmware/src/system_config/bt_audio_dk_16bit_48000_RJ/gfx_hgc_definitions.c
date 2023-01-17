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
GFX_GOL_OBJ_SCHEME *gfxScheme1;

/*** Generated Asset References ***/
extern const GFX_RESOURCE_HDR MCHP_LOGO;
extern const GFX_RESOURCE_HDR Arial12pt;
extern const GFX_RESOURCE_HDR Arial14pt;
extern const GFX_RESOURCE_HDR Arial20pt;

/*** Generated ASCII Text Labels ***/
static const GFX_XCHAR buttonStr_0[] = { 0x42, 0x75, 0x74, 0x74, 0x6f, 0x6e, 0x0 }; // Button
static const GFX_XCHAR staticTextStr_0[] = { 0x31, 0x36, 0x20, 0x42, 0x69, 0x74, 0x73, 0x0 }; // 16 Bits
static const GFX_XCHAR staticTextStr_1[] = { 0x34, 0x38, 0x30, 0x30, 0x30, 0x20, 0x48, 0x7a, 0x0 }; // 48000 Hz
static const GFX_XCHAR staticTextStr_2[] = { 0x53, 0x69, 0x6e, 0x65, 0x20, 0x54, 0x6f, 0x6e, 0x65, 0x20, 0x2d, 0x20, 0x4f, 0x4e, 0x0 }; // Sine Tone - ON
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

    gfxScheme1 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme1->AlphaValue =        GFX_SCHEME_ALPHA_1;
#endif
    gfxScheme1->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_1, GFX_SCHEME_PRIMARY_COLOR_GREEN_1, GFX_SCHEME_PRIMARY_COLOR_BLUE_1);
    gfxScheme1->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_1, GFX_SCHEME_SECONDARY_COLOR_GREEN_1, GFX_SCHEME_SECONDARY_COLOR_BLUE_1);
    gfxScheme1->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_1, GFX_SCHEME_DISABLED_COLOR_GREEN_1, GFX_SCHEME_DISABLED_COLOR_BLUE_1);
    gfxScheme1->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_1, GFX_SCHEME_BACKGROUND_COLOR_GREEN_1, GFX_SCHEME_BACKGROUND_COLOR_BLUE_1);
	gfxScheme1->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_1;
    gfxScheme1->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_1, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_1, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_1);
    gfxScheme1->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_1, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_1, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_1);
	gfxScheme1->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_1, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_1, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_1);
    gfxScheme1->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_1, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_1, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_1);
    gfxScheme1->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_1, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_1, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_1);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme1->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_1, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_1, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_1);
	gfxScheme1->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_1, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_1, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_1);
#endif
    gfxScheme1->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_1;
    gfxScheme1->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_1;
    gfxScheme1->pFont =             (GFX_RESOURCE_HDR*)&Arial12pt;
    gfxScheme1->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme1->fillStyle =         GFX_SCHEME_FILL_STYLE_1;
	gfxScheme1->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_1;

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
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen:
            if (GFX_HGC_MsgButtons(objMsg, pObj, pMsg) == true)
                return true;
            if (GFX_HGC_MsgDigitalMeters(objMsg, pObj, pMsg) == true)
                return true;
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
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_DemoScreen);
            return true;
        case HGC_SCREEN_STATE_SETUP_SCREEN_DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_GOL_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_DRAW_GOL_SCREEN_DemoScreen:
            GFX_HGC_SetupScreen(DemoScreen);
            GFX_HGC_DrawScreen_GOL(DemoScreen);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_DemoScreen:
            GFX_HGC_DrawScreen_Primitives(DemoScreen);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen);
            break;
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen:
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
        case DemoScreen:
            GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0x00,0x00,0x64));
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
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_DemoScreen:
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
        case DemoScreen:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_DemoScreen);
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
        case DemoScreen:
            GFX_HGC_DrawItem(FreqMeter);
            GFX_HGC_DrawItem(FreqMeter2);
            GFX_HGC_DrawItem(DigitalMeter1);
            GFX_HGC_DrawItem(Button1);
            GFX_HGC_DrawItem(ToneMode);
            GFX_HGC_DrawItem(SampleRateTxt);
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
        case DemoScreen:
            GFX_HGC_DrawItem(Rectangle1);
            GFX_HGC_DrawItem(MchpLogo);
            GFX_HGC_DrawItem(Frequency);
            GFX_HGC_DrawItem(HzText1);
            GFX_HGC_DrawItem(HzText2);
            GFX_HGC_DrawItem(DurationText);
            GFX_HGC_DrawItem(DemoName);
            GFX_HGC_DrawItem(Highlight1);
            GFX_HGC_DrawItem(Highlight2);
            GFX_HGC_DrawItem(Highlight3);
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
    case Button1:
        hgcObj.pButton1Obj = GFX_GOL_ButtonCreate
                (
                 GFX_INDEX_0,
                 Button1,      // button ID
                 GFX_BUTTON_LEFT_0,    // left
                 GFX_BUTTON_TOP_0,     // top
                 GFX_BUTTON_RIGHT_0,   // right
                 GFX_BUTTON_BOTTOM_0,  // bottom
                 GFX_BUTTON_RADIUS_0,  // radius
                 GFX_BUTTON_STATE_0,   // state
                 GFX_BUTTON_PRESS_IMAGE_0,     // press image
                 GFX_BUTTON_RELEASE_IMAGE_0,   // release image
                 (GFX_XCHAR*)buttonStr_0,    // text
                 GFX_BUTTON_ALIGNMENT_0,       //alignment
                 GFX_BUTTON_SCHEME_0
                 ); // use alternate scheme
        break;
    case DigitalMeter1:
        hgcObj.pDigitalMeter1Obj = GFX_GOL_DigitalMeterCreate
                (
                 GFX_INDEX_0,
                 DigitalMeter1,      // digital meter ID
                 GFX_DGTL_MTR_LEFT_0,    // left
                 GFX_DGTL_MTR_TOP_0,     // top
                 GFX_DGTL_MTR_RIGHT_0,   // right
                 GFX_DGTL_MTR_BOTTOM_0,  // bottom             
                 GFX_DGTL_MTR_STATE_0,   // state
                 (uint32_t)GFX_DGTL_MTR_VALUE_0,   // value
                 (uint16_t)GFX_DGTL_MTR_NO_OF_DIGITS_0, // no. of digits
                 (uint16_t)GFX_DGTL_MTR_DOT_POSITION_0,// dot position
                 GFX_DGTL_MTR_ALIGNMENT_0,//alignment
                 GFX_DGTL_MTR_SCHEME_0	//scheme
                 ); // use alternate scheme
        break;
    case FreqMeter2:
        hgcObj.pFreqMeter2Obj = GFX_GOL_DigitalMeterCreate
                (
                 GFX_INDEX_0,
                 FreqMeter2,      // digital meter ID
                 GFX_DGTL_MTR_LEFT_1,    // left
                 GFX_DGTL_MTR_TOP_1,     // top
                 GFX_DGTL_MTR_RIGHT_1,   // right
                 GFX_DGTL_MTR_BOTTOM_1,  // bottom             
                 GFX_DGTL_MTR_STATE_1,   // state
                 (uint32_t)GFX_DGTL_MTR_VALUE_1,   // value
                 (uint16_t)GFX_DGTL_MTR_NO_OF_DIGITS_1, // no. of digits
                 (uint16_t)GFX_DGTL_MTR_DOT_POSITION_1,// dot position
                 GFX_DGTL_MTR_ALIGNMENT_1,//alignment
                 GFX_DGTL_MTR_SCHEME_1	//scheme
                 ); // use alternate scheme
        break;
    case FreqMeter:
        hgcObj.pFreqMeterObj = GFX_GOL_DigitalMeterCreate
                (
                 GFX_INDEX_0,
                 FreqMeter,      // digital meter ID
                 GFX_DGTL_MTR_LEFT_2,    // left
                 GFX_DGTL_MTR_TOP_2,     // top
                 GFX_DGTL_MTR_RIGHT_2,   // right
                 GFX_DGTL_MTR_BOTTOM_2,  // bottom             
                 GFX_DGTL_MTR_STATE_2,   // state
                 (uint32_t)GFX_DGTL_MTR_VALUE_2,   // value
                 (uint16_t)GFX_DGTL_MTR_NO_OF_DIGITS_2, // no. of digits
                 (uint16_t)GFX_DGTL_MTR_DOT_POSITION_2,// dot position
                 GFX_DGTL_MTR_ALIGNMENT_2,//alignment
                 GFX_DGTL_MTR_SCHEME_2	//scheme
                 ); // use alternate scheme
        break;
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
    case SampleRateTxt:
        hgcObj.pSampleRateTxtObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 SampleRateTxt,      // button ID
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
    case ToneMode:
        hgcObj.pToneModeObj = GFX_GOL_StaticTextCreate
                (
                 GFX_INDEX_0,
                 ToneMode,      // button ID
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
		case Highlight3:
		{
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0x00, 0x00));
			GFX_LineStyleSet(GFX_INDEX_0, GFX_LINE_STYLE_THIN_SOLID);
			GFX_RectangleDraw(GFX_INDEX_0,
							  156,  // p1x
							  143,  // p1y
							  198,  // p2x
							  168); // p2y
			
			break;
		}	
		case Highlight2:
		{
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0x00, 0x00));
			GFX_LineStyleSet(GFX_INDEX_0, GFX_LINE_STYLE_THIN_SOLID);
			GFX_RectangleDraw(GFX_INDEX_0,
							  67,  // p1x
							  143,  // p1y
							  108,  // p2x
							  168); // p2y
			
			break;
		}	
		case Highlight1:
		{
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0x00, 0x00));
			GFX_LineStyleSet(GFX_INDEX_0, GFX_LINE_STYLE_THIN_SOLID);
			GFX_RectangleDraw(GFX_INDEX_0,
							  11,  // p1x
							  144,  // p1y
							  52,  // p2x
							  169); // p2y
			
			break;
		}	
		case Rectangle1:
		{
			GFX_FillStyleSet(GFX_INDEX_0, GFX_FILL_STYLE_GRADIENT_DOWN);
			GFX_GradientColorSet(GFX_INDEX_0,
			                    GFX_RGBConvert(0xFF, 0x00, 0x00),
			                    GFX_RGBConvert(0x00, 0x00, 0x00));	
			GFX_RectangleFillDraw(GFX_INDEX_0,
								  0,  // p1x
								  1,  // p1y
								  220,  // p2x
								  32); // p2y
			
			break;
		}	
		case MchpLogo:
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
			GFX_XCHAR Text1_text[] = { 0x52, 0x65, 0x73, 0x6f, 0x6c, 0x75, 0x74, 0x69, 0x6f, 0x6e, 0x3a, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0x00, 0x00)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   6, // draw point x
						       72, // draw point y
                               Text1_text, // string
							   0); // length
			break;
		}	
		case DemoName:
		{
			GFX_XCHAR DemoName_text[] = { 0x61, 0x75, 0x64, 0x69, 0x6f, 0x5f, 0x74, 0x6f, 0x6e, 0x65, 0x20, 0x4d, 0x50, 0x4c, 0x41, 0x42, 0x20, 0x48, 0x61, 0x72, 0x6d, 0x6f, 0x6e, 0x79, 0x20, 0x56, 0x32, 0x2e, 0x30, 0x31, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   8, // draw point x
						       33, // draw point y
                               DemoName_text, // string
							   0); // length
			break;
		}	
		case DurationText:
		{
			GFX_XCHAR DurationText_text[] = { 0x74, 0x20, 0x28, 0x6d, 0x73, 0x29, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   163, // draw point x
						       129, // draw point y
                               DurationText_text, // string
							   0); // length
			break;
		}	
		case HzText2:
		{
			GFX_XCHAR HzText2_text[] = { 0x66, 0x32, 0x20, 0x28, 0x48, 0x7a, 0x29, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   72, // draw point x
						       129, // draw point y
                               HzText2_text, // string
							   0); // length
			break;
		}	
		case HzText1:
		{
			GFX_XCHAR HzText1_text[] = { 0x66, 0x31, 0x20, 0x28, 0x48, 0x7a, 0x29, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0xFC, 0xF8)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   16, // draw point x
						       129, // draw point y
                               HzText1_text, // string
							   0); // length
			break;
		}	
		case Frequency:
		{
			GFX_XCHAR Frequency_text[] = { 0x53, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x46, 0x72, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x63, 0x79, 0x3a, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xF8, 0x00, 0x00)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial12pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   7, // draw point x
						       55, // draw point y
                               Frequency_text, // string
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
    bool GFX_HGC_MsgButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Button events
 */
bool GFX_HGC_MsgButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        default:
            return false; // process by default
    }

    return true;
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgDigitalMeters(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Digital Meter events
 */
bool GFX_HGC_MsgDigitalMeters(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        default:
            return false; // default false as it is not processed
    }    
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


