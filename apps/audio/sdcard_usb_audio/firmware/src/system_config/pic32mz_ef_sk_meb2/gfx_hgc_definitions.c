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
#include "app_display_task.h"

/*** Default GFX GOL Scheme ***/
#define GFX_SCHEMEDEFAULT GOLSchemeDefault
extern GFX_GOL_OBJ_SCHEME GFX_SCHEMEDEFAULT;

/*** HGC-specified GFX GOL Scheme ***/
GFX_GOL_OBJ_SCHEME *gfxScheme0;
GFX_GOL_OBJ_SCHEME *gfxScheme1;
GFX_GOL_OBJ_SCHEME *gfxScheme2;
GFX_GOL_OBJ_SCHEME *gfxScheme3;
GFX_GOL_OBJ_SCHEME *gfxScheme4;
GFX_GOL_OBJ_SCHEME *gfxScheme5;

/*** Generated Asset References ***/
extern const GFX_RESOURCE_HDR MCHP_LOGO;
extern const GFX_RESOURCE_HDR AudioMute16_2;
extern const GFX_RESOURCE_HDR AudioPlay16_2;
extern const GFX_RESOURCE_HDR Arial14pt;
extern const GFX_RESOURCE_HDR Arial18pt;

/*** Generated ASCII Text Labels ***/
static const GFX_XCHAR buttonStr_0[] = { 0x0 }; //  
static const GFX_XCHAR buttonStr_1[] = { 0x76, 0x0 }; // v
static const GFX_XCHAR buttonStr_2[] = { 0x20, 0x0 }; // ^
static const GFX_XCHAR radioButtonStr_0[] = { 0x55, 0x53, 0x42, 0x0 }; // USB
static const GFX_XCHAR radioButtonStr_1[] = { 0x53, 0x44, 0x20, 0x43, 0x61, 0x72, 0x64, 0x0 }; // SD Card

static GFX_XCHAR GroupBox1_text[] = { 0x53, 0x6f, 0x75, 0x72, 0x63, 0x65, 0x0 }; // Source
/*** HGC Object Global ***/
HGC_OBJECTS hgcObj;
static HGC_STATES hgcState;

/*** Message System Service Globals ***/
SYS_MSG_INSTANCE iSysMsg = SYS_MSG_0;
SYS_OBJ_HANDLE hSysMsg, hMsgType, hMailbox;

/******************************************************************************
  Function:
    void GFX_HGC_RegisterAppTouchCallback ( SYS_MSG_RECEIVE_CALLBACK msgCallBackFunction )

  Summary:
    This function allows the application to register a callback to allow access 
    to the current touch behavior
 */
void GFX_HGC_RegisterAppTouchCallback ( SYS_MSG_RECEIVE_CALLBACK msgCallBackFunction )
{
    hgcObj.msgCallBackFunction = msgCallBackFunction;
}


/******************************************************************************
  Function:
    void GFX_HGC_TouchMessageCallback ( SYS_MSG_OBJECT *pMsg )

  Summary:
    This function sets up the Message System Service to route touch callbacks
    to the GOL message system  
 */
void GFX_HGC_TouchMessageCallback ( SYS_MSG_OBJECT *pMsg )
{
    static GFX_GOL_MESSAGE gMsg;

    gMsg.type = pMsg->nMessageTypeID;
    gMsg.uiEvent = pMsg->param0;
    gMsg.param1 = pMsg->param1;
    gMsg.param2 = pMsg->param2;

    GFX_GOL_ObjectMessage(GFX_INDEX_0, &gMsg);

    if (hgcObj.msgCallBackFunction != NULL)
    {
        hgcObj.msgCallBackFunction(pMsg);
    }
}


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
    hgcObj.msgCallBackFunction = NULL;
}


/******************************************************************************
  Function:
    void GFX_HGC_Setup ( void )

  Summary:
    This function sets up the GOL message callback and draw callbacks.  
 */
void GFX_HGC_Setup ( void )
{
    hMsgType = SYS_MSG_TypeCreate(SYS_MSG_0, TYPE_TOUCHSCREEN , 0);
    hMailbox = SYS_MSG_MailboxOpen(SYS_MSG_0, (SYS_MSG_RECEIVE_CALLBACK) &GFX_HGC_TouchMessageCallback );
    SYS_MSG_MailboxMsgAdd( hMailbox, hMsgType);

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
    gfxScheme0->pFont =             (GFX_RESOURCE_HDR*)&Arial14pt;
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
    gfxScheme1->pFont =             (GFX_RESOURCE_HDR*)&Arial14pt;
    gfxScheme1->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme1->fillStyle =         GFX_SCHEME_FILL_STYLE_1;
	gfxScheme1->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_1;

    gfxScheme2 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme2->AlphaValue =        GFX_SCHEME_ALPHA_2;
#endif
    gfxScheme2->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_2, GFX_SCHEME_PRIMARY_COLOR_GREEN_2, GFX_SCHEME_PRIMARY_COLOR_BLUE_2);
    gfxScheme2->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_2, GFX_SCHEME_SECONDARY_COLOR_GREEN_2, GFX_SCHEME_SECONDARY_COLOR_BLUE_2);
    gfxScheme2->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_2, GFX_SCHEME_DISABLED_COLOR_GREEN_2, GFX_SCHEME_DISABLED_COLOR_BLUE_2);
    gfxScheme2->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_2, GFX_SCHEME_BACKGROUND_COLOR_GREEN_2, GFX_SCHEME_BACKGROUND_COLOR_BLUE_2);
	gfxScheme2->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_2;
    gfxScheme2->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_2, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_2, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_2);
    gfxScheme2->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_2, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_2, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_2);
	gfxScheme2->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_2, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_2, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_2);
    gfxScheme2->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_2, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_2, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_2);
    gfxScheme2->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_2, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_2, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_2);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme2->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_2, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_2, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_2);
	gfxScheme2->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_2, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_2, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_2);
#endif
    gfxScheme2->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_2;
    gfxScheme2->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_2;
    gfxScheme2->pFont =             (GFX_RESOURCE_HDR*)&Arial14pt;
    gfxScheme2->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme2->fillStyle =         GFX_SCHEME_FILL_STYLE_2;
	gfxScheme2->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_2;

    gfxScheme3 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme3->AlphaValue =        GFX_SCHEME_ALPHA_3;
#endif
    gfxScheme3->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_3, GFX_SCHEME_PRIMARY_COLOR_GREEN_3, GFX_SCHEME_PRIMARY_COLOR_BLUE_3);
    gfxScheme3->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_3, GFX_SCHEME_SECONDARY_COLOR_GREEN_3, GFX_SCHEME_SECONDARY_COLOR_BLUE_3);
    gfxScheme3->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_3, GFX_SCHEME_DISABLED_COLOR_GREEN_3, GFX_SCHEME_DISABLED_COLOR_BLUE_3);
    gfxScheme3->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_3, GFX_SCHEME_BACKGROUND_COLOR_GREEN_3, GFX_SCHEME_BACKGROUND_COLOR_BLUE_3);
	gfxScheme3->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_3;
    gfxScheme3->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_3, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_3, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_3);
    gfxScheme3->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_3, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_3, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_3);
	gfxScheme3->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_3, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_3, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_3);
    gfxScheme3->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_3, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_3, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_3);
    gfxScheme3->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_3, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_3, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_3);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme3->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_3, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_3, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_3);
	gfxScheme3->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_3, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_3, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_3);
#endif
    gfxScheme3->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_3;
    gfxScheme3->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_3;
    gfxScheme3->pFont =             (GFX_RESOURCE_HDR*)&Arial14pt;
    gfxScheme3->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme3->fillStyle =         GFX_SCHEME_FILL_STYLE_3;
	gfxScheme3->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_3;

    gfxScheme4 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme4->AlphaValue =        GFX_SCHEME_ALPHA_4;
#endif
    gfxScheme4->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_4, GFX_SCHEME_PRIMARY_COLOR_GREEN_4, GFX_SCHEME_PRIMARY_COLOR_BLUE_4);
    gfxScheme4->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_4, GFX_SCHEME_SECONDARY_COLOR_GREEN_4, GFX_SCHEME_SECONDARY_COLOR_BLUE_4);
    gfxScheme4->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_4, GFX_SCHEME_DISABLED_COLOR_GREEN_4, GFX_SCHEME_DISABLED_COLOR_BLUE_4);
    gfxScheme4->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_4, GFX_SCHEME_BACKGROUND_COLOR_GREEN_4, GFX_SCHEME_BACKGROUND_COLOR_BLUE_4);
	gfxScheme4->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_4;
    gfxScheme4->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_4, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_4, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_4);
    gfxScheme4->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_4, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_4, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_4);
	gfxScheme4->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_4, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_4, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_4);
    gfxScheme4->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_4, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_4, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_4);
    gfxScheme4->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_4, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_4, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_4);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme4->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_4, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_4, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_4);
	gfxScheme4->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_4, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_4, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_4);
#endif
    gfxScheme4->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_4;
    gfxScheme4->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_4;
    gfxScheme4->pFont =             (GFX_RESOURCE_HDR*)&Arial18pt;
    gfxScheme4->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme4->fillStyle =         GFX_SCHEME_FILL_STYLE_4;
	gfxScheme4->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_4;

    gfxScheme5 = GFX_HGC_SchemeCreate();

#ifndef GFX_CONFIG_ALPHABLEND_DISABLE
    gfxScheme5->AlphaValue =        GFX_SCHEME_ALPHA_5;
#endif
    gfxScheme5->Color0 =            GFX_RGBConvert(GFX_SCHEME_PRIMARY_COLOR_RED_5, GFX_SCHEME_PRIMARY_COLOR_GREEN_5, GFX_SCHEME_PRIMARY_COLOR_BLUE_5);
    gfxScheme5->Color1 =            GFX_RGBConvert(GFX_SCHEME_SECONDARY_COLOR_RED_5, GFX_SCHEME_SECONDARY_COLOR_GREEN_5, GFX_SCHEME_SECONDARY_COLOR_BLUE_5);
    gfxScheme5->ColorDisabled =     GFX_RGBConvert(GFX_SCHEME_DISABLED_COLOR_RED_5, GFX_SCHEME_DISABLED_COLOR_GREEN_5, GFX_SCHEME_DISABLED_COLOR_BLUE_5);
    gfxScheme5->CommonBkColor =     GFX_RGBConvert(GFX_SCHEME_BACKGROUND_COLOR_RED_5, GFX_SCHEME_BACKGROUND_COLOR_GREEN_5, GFX_SCHEME_BACKGROUND_COLOR_BLUE_5);
	gfxScheme5->EmbossSize =        GFX_SCHEME_EMBOSS_SIZE_5;
    gfxScheme5->EmbossDkColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_DARK_COLOR_RED_5, GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_5, GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_5);
    gfxScheme5->EmbossLtColor =     GFX_RGBConvert(GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_5, GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_5, GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_5);
	gfxScheme5->TextColor0 =        GFX_RGBConvert(GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_5, GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_5, GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_5);
    gfxScheme5->TextColor1 =        GFX_RGBConvert(GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_5, GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_5, GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_5);
    gfxScheme5->TextColorDisabled = GFX_RGBConvert(GFX_SCHEME_TEXT_DISABLED_COLOR_RED_5, GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_5, GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_5);
#ifndef GFX_CONFIG_GRADIENT_DISABLE
	gfxScheme5->gradientStartColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_START_COLOR_RED_5, GFX_SCHEME_GRADIENT_START_COLOR_GREEN_5, GFX_SCHEME_GRADIENT_START_COLOR_BLUE_5);
	gfxScheme5->gradientEndColor = GFX_RGBConvert(GFX_SCHEME_GRADIENT_END_COLOR_RED_5, GFX_SCHEME_GRADIENT_END_COLOR_GREEN_5, GFX_SCHEME_GRADIENT_END_COLOR_BLUE_5);
#endif
    gfxScheme5->CommonBkLeft =      GFX_SCHEME_BACKGROUND_OFFSET_TOP_5;
    gfxScheme5->CommonBkTop =       GFX_SCHEME_BACKGROUND_OFFSET_LEFT_5;
    gfxScheme5->pFont =             (GFX_RESOURCE_HDR*)&Arial14pt;
    gfxScheme5->pCommonBkImage =    (GFX_RESOURCE_HDR*)NULL;
	
	gfxScheme5->fillStyle =         GFX_SCHEME_FILL_STYLE_5;
	gfxScheme5->CommonBkType =      GFX_SCHEME_BACKGROUND_TYPE_5;

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
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_MchpName:
            if (GFX_HGC_MsgButtons(objMsg, pObj, pMsg) == true)
                return true;
            if (GFX_HGC_MsgRadioButtons(objMsg, pObj, pMsg) == true)
                return true;
            if (GFX_HGC_MsgScrollBars(objMsg, pObj, pMsg) == true)
                return true;
            if (GFX_HGC_MsgListBoxes(objMsg, pObj, pMsg) == true)
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
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_MchpName);
            return true;
        case HGC_SCREEN_STATE_SETUP_SCREEN_MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_GOL_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_DRAW_GOL_SCREEN_MchpName:
            GFX_HGC_SetupScreen(MchpName);
            GFX_HGC_DrawScreen_GOL(MchpName);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_MchpName:
            GFX_HGC_DrawScreen_Primitives(MchpName);
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_DISPLAY_SCREEN_MchpName);
            break;
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_MchpName:
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
        case MchpName:
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
        case HGC_SCREEN_STATE_DISPLAY_SCREEN_MchpName:
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
        case MchpName:
            GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_MchpName);
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
        case MchpName:
            GFX_HGC_DrawItem(GroupBox1);
            GFX_HGC_DrawItem(RadioButtonSd);
            GFX_HGC_DrawItem(TrackListBox);
            GFX_HGC_DrawItem(ScrollUpButton);
            GFX_HGC_DrawItem(ScrollDownButton);
            GFX_HGC_DrawItem(VolSlider);
            GFX_HGC_DrawItem(VolButton);
            GFX_HGC_DrawItem(RadioButtonUSB);
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
        case MchpName:
            GFX_HGC_DrawItem(Rectangle1);
            GFX_HGC_DrawItem(Image1);
            GFX_HGC_DrawItem(DemoName);
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
    case VolButton:
        hgcObj.pVolButtonObj = GFX_GOL_ButtonCreate
                (
                 GFX_INDEX_0,
                 VolButton,      // button ID
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
    case ScrollDownButton:
        hgcObj.pScrollDownButtonObj = GFX_GOL_ButtonCreate
                (
                 GFX_INDEX_0,
                 ScrollDownButton,      // button ID
                 GFX_BUTTON_LEFT_1,    // left
                 GFX_BUTTON_TOP_1,     // top
                 GFX_BUTTON_RIGHT_1,   // right
                 GFX_BUTTON_BOTTOM_1,  // bottom
                 GFX_BUTTON_RADIUS_1,  // radius
                 GFX_BUTTON_STATE_1,   // state
                 GFX_BUTTON_PRESS_IMAGE_1,     // press image
                 GFX_BUTTON_RELEASE_IMAGE_1,   // release image
                 (GFX_XCHAR*)buttonStr_1,    // text
                 GFX_BUTTON_ALIGNMENT_1,       //alignment
                 GFX_BUTTON_SCHEME_1
                 ); // use alternate scheme
        break;
    case ScrollUpButton:
        hgcObj.pScrollUpButtonObj = GFX_GOL_ButtonCreate
                (
                 GFX_INDEX_0,
                 ScrollUpButton,      // button ID
                 GFX_BUTTON_LEFT_2,    // left
                 GFX_BUTTON_TOP_2,     // top
                 GFX_BUTTON_RIGHT_2,   // right
                 GFX_BUTTON_BOTTOM_2,  // bottom
                 GFX_BUTTON_RADIUS_2,  // radius
                 GFX_BUTTON_STATE_2,   // state
                 GFX_BUTTON_PRESS_IMAGE_2,     // press image
                 GFX_BUTTON_RELEASE_IMAGE_2,   // release image
                 (GFX_XCHAR*)buttonStr_2,    // text
                 GFX_BUTTON_ALIGNMENT_2,       //alignment
                 GFX_BUTTON_SCHEME_2
                 ); // use alternate scheme
        break;
    case RadioButtonUSB:
        hgcObj.pRadioButtonUSBObj = GFX_GOL_RadioButtonListCreate
                (
                 GFX_INDEX_0,
                 RadioButtonUSB,      // Radio button Box ID
                 GFX_RADIOBUTTON_LEFT_0,    // left
                 GFX_RADIOBUTTON_TOP_0,     // top
                 GFX_RADIOBUTTON_RIGHT_0,   // right
                 GFX_RADIOBUTTON_BOTTOM_0,  // bottom 
                 GFX_RADIOBUTTON_STATE_0,   // state
                 (GFX_XCHAR*)radioButtonStr_0,			 		 
                 GFX_RADIOBUTTON_ALIGNMENT_0,//alignment
                 GFX_RADIOBUTTON_SCHEME_0,	  //scheme
                 GFX_RADIOBUTTON_GROUPNO_0,  //group number
                 GFX_RADIOBUTTON_NUM_GROUPS_ON_SCREEN_0 //number of groups on screen
                 ); // use alternate scheme
        break;
    case RadioButtonSd:
        hgcObj.pRadioButtonSdObj = GFX_GOL_RadioButtonListCreate
                (
                 GFX_INDEX_0,
                 RadioButtonSd,      // Radio button Box ID
                 GFX_RADIOBUTTON_LEFT_1,    // left
                 GFX_RADIOBUTTON_TOP_1,     // top
                 GFX_RADIOBUTTON_RIGHT_1,   // right
                 GFX_RADIOBUTTON_BOTTOM_1,  // bottom 
                 GFX_RADIOBUTTON_STATE_1,   // state
                 (GFX_XCHAR*)radioButtonStr_1,			 		 
                 GFX_RADIOBUTTON_ALIGNMENT_1,//alignment
                 GFX_RADIOBUTTON_SCHEME_1,	  //scheme
                 GFX_RADIOBUTTON_GROUPNO_1,  //group number
                 GFX_RADIOBUTTON_NUM_GROUPS_ON_SCREEN_1 //number of groups on screen
                 ); // use alternate scheme
        break;
    case VolSlider:
        hgcObj.pVolSliderObj = GFX_GOL_ScrollBarCreate
                (
                 GFX_INDEX_0,
                 VolSlider,           // Check Box ID
                 GFX_SCROLLBAR_LEFT_0,        // left
                 GFX_SCROLLBAR_TOP_0,        // top
                 GFX_SCROLLBAR_RIGHT_0,     // right
                 GFX_SCROLLBAR_BOTTOM_0,   // bottom 				 
                 GFX_SCROLLBAR_STATE_0,   //State
                 GFX_SCROLLBAR_RANGE_0,  // range
                 GFX_SCROLLBAR_PAGE_0,  // page 
                 GFX_SCROLLBAR_POS_0,  // pos 
                 GFX_SCROLLBAR_SCHEME_0	//scheme
                 ); // use alternate scheme
        break;
    case TrackListBox:
        hgcObj.pTrackListBoxObj = GFX_GOL_ListBoxCreate(
                GFX_INDEX_0,
                TrackListBox, // id
                GFX_LISTBOX_LEFT_0, // left
                GFX_LISTBOX_TOP_0, // top
                GFX_LISTBOX_RIGHT_0, // right
                GFX_LISTBOX_BOTTOM_0, // bottom
				GFX_LISTBOX_STATE_0, // state
				NULL, // default text
                GFX_LISTBOX_ALIGNMENT_0, //alignment
                GFX_LISTBOX_SCHEME_0); // scheme
		
        break;
    case GroupBox1:
        hgcObj.pGroupBox1Obj = GFX_GOL_GroupboxCreate(
                GFX_INDEX_0,
                GroupBox1, // id
                GFX_GROUPBOX_LEFT_0, // left
                GFX_GROUPBOX_TOP_0, // top
                GFX_GROUPBOX_RIGHT_0, // right
                GFX_GROUPBOX_BOTTOM_0, // bottom
				GFX_GROUPBOX_STATE_0, // state
				GroupBox1_text, // text
                GFX_GROUPBOX_ALIGNMENT_0, //alignment
                GFX_GROUPBOX_SCHEME_0); // scheme
				
		break;
		
		case Rectangle1:
		{
			GFX_FillStyleSet(GFX_INDEX_0, GFX_FILL_STYLE_GRADIENT_DOWN);
			GFX_GradientColorSet(GFX_INDEX_0,
			                    GFX_RGBConvert(0xFF, 0x00, 0x00),
			                    GFX_RGBConvert(0x00, 0x00, 0x00));	
			GFX_RectangleFillDraw(GFX_INDEX_0,
								  0,  // p1x
								  1,  // p1y
								  479,  // p2x
								  45); // p2y
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0x00, 0x00, 0x00));
			GFX_LineStyleSet(GFX_INDEX_0, GFX_LINE_STYLE_THIN_SOLID);
			GFX_RectangleDraw(GFX_INDEX_0,
							  0,  // p1x
							  1,  // p1y
							  479,  // p2x
							  45); // p2y
			
			break;
		}	
		case Image1:
		{
			GFX_TransparentColorEnable(GFX_INDEX_0, GFX_RGBConvert(0x1D, 0x2E, 0x3C)); // enable transparency
			GFX_ImageDraw(GFX_INDEX_0,
			              1, // draw point x
						  2, // draw point y
						  (GFX_RESOURCE_HDR*)&MCHP_LOGO); // image
			GFX_TransparentColorDisable(GFX_INDEX_0); // disable transparency
			break;
		}	
		case DemoName:
		{
			GFX_XCHAR DemoName_text[] = { 0x48, 0x61, 0x72, 0x6d, 0x6f, 0x6e, 0x79, 0x20, 0x51, 0x75, 0x69, 0x63, 0x6b, 0x20, 0x41, 0x75, 0x64, 0x69, 0x6f, 0x20, 0x50, 0x72, 0x6f, 0x74, 0x6f, 0x74, 0x79, 0x70, 0x69, 0x6e, 0x67, 0x0 };
			
			GFX_ColorSet(GFX_INDEX_0, GFX_RGBConvert(0xFF, 0xFF, 0xFF)); // set color
			GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*)&Arial14pt); // set font
			GFX_TextStringDraw(GFX_INDEX_0,
			                   34, // draw point x
						       38, // draw point y
                               DemoName_text, // string
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
        case VolButton:
			if (objMsg == GFX_GOL_BUTTON_ACTION_PRESSED)
            {
				// Button Pressed Event Code
				// CustomAction
				APP_DISPLAY_VolumeMuteChange(1);

            }
            if (objMsg == GFX_GOL_BUTTON_ACTION_RELEASED)
            {
                // Button Release Event Code
				// CustomAction
				APP_DISPLAY_VolumeMuteChange(0);

            }
			
            return true;
        case ScrollDownButton:
            if (objMsg == GFX_GOL_BUTTON_ACTION_RELEASED)
            {
                // Button Release Event Code
				// CustomAction
				GFX_GOL_LISTBOX *pLb = (GFX_GOL_LISTBOX *) GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox);
				GFX_GOL_ListBoxItemFocusSet(pLb, GFX_GOL_ListBoxItemFocusGet(pLb) + 1);
				GFX_GOL_ObjectStateSet(pLb, GFX_GOL_LISTBOX_DRAW_ITEMS_STATE);

            }
			
            return true;
        case ScrollUpButton:
            if (objMsg == GFX_GOL_BUTTON_ACTION_RELEASED)
            {
                // Button Release Event Code
				// CustomAction
				GFX_GOL_LISTBOX *pLb = (GFX_GOL_LISTBOX *) GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox);
				if (GFX_GOL_ListBoxItemFocusGet(pLb) > 0) 
				{
					GFX_GOL_ListBoxItemFocusSet(pLb, GFX_GOL_ListBoxItemFocusGet(pLb) - 1);
					GFX_GOL_ObjectStateSet(pLb, GFX_GOL_LISTBOX_DRAW_ITEMS_STATE);
				}

            }
			
            return true;
        default:
            return false; // process by default
    }

    return true;
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgRadioButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Radio Button events
 */
bool GFX_HGC_MsgRadioButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        case RadioButtonUSB:
			if (objMsg == GFX_GOL_RADIOBUTTON_ACTION_CHECKED)
            {
				// RadioButton Check Event Code
				// CustomAction
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox), GFX_GOL_LISTBOX_DISABLED_STATE|GFX_GOL_LISTBOX_DRAW_STATE);
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollUpButton), GFX_GOL_BUTTON_DISABLED_STATE|GFX_GOL_BUTTON_DRAW_STATE);
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollDownButton), GFX_GOL_BUTTON_DISABLED_STATE|GFX_GOL_BUTTON_DRAW_STATE);
				APP_DISPLAY_StreamSourceSet(1);
				APP_DISPLAY_VolumeMuteSourceSet(1);

            }
			
            return true;
        case RadioButtonSd:
			if (objMsg == GFX_GOL_RADIOBUTTON_ACTION_CHECKED)
            {
				// RadioButton Check Event Code
				// CustomAction
				GFX_GOL_ObjectStateClear(GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox), GFX_GOL_LISTBOX_DISABLED_STATE);
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox),GFX_GOL_LISTBOX_SINGLE_SELECT_STATE|GFX_GOL_LISTBOX_DRAW_STATE);
				GFX_GOL_ListBoxItemFocusSet(hgcObj.pTrackListBoxObj,0);
								
				GFX_GOL_ObjectStateClear(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollUpButton), GFX_GOL_BUTTON_DISABLED_STATE);
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollUpButton), GFX_GOL_BUTTON_DRAW_STATE);
				GFX_GOL_ObjectStateClear(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollDownButton), GFX_GOL_BUTTON_DISABLED_STATE);
				GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, ScrollDownButton), GFX_GOL_BUTTON_DRAW_STATE);
												
				APP_DISPLAY_StreamSourceSet(0);
				APP_DISPLAY_VolumeMuteSourceSet(0);

            }
			
            return true;
        default:
            return false; // default false as it is not processed
    }    
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgScrollBars(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Scroll Bar events
 */
bool GFX_HGC_MsgScrollBars(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        case VolSlider:
			if (objMsg == GFX_GOL_SCROLLBAR_ACTION_INC)
            {
				// Scrollbar Increment Event Code
				// CustomAction
				APP_DISPLAY_VolumeMuteChange(2);

            }
            if (objMsg == GFX_GOL_SCROLLBAR_ACTION_DEC)
            {
                // Scrollbar Decrement Event Code
				// CustomAction
				APP_DISPLAY_VolumeMuteChange(2);

            }
			
            return true;
        default:
            return false; // default false as it is not processed
    }    
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgListBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL List Boxes events
 */
bool GFX_HGC_MsgListBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        case TrackListBox:
			if (objMsg == GFX_GOL_LISTBOX_ACTION_TOUCHSCREEN)
            {
				// ListBox Touch Event Code
				// CustomAction
				APP_SDCARD_AUDIO_RandomTrackSet(GFX_GOL_ListBoxItemFocusGet((GFX_GOL_LISTBOX*)pObj));

            }
			
            return true;
        default:
            return false; // default false as it is not processed
    }    
}

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgGroupBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Group Boxes events
 */
bool GFX_HGC_MsgGroupBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)
{
    switch (GFX_GOL_ObjectIDGet(pObj))
    {
        default:
            return false; // default false as it is not processed
    }    
}


