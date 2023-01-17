/*******************************************************************************
 Display Tasks

  Company:
    Microchip Technology Inc.

  File Name:
   Display_tasks.c

  Summary:
    Contains the functional implementation of display task for

  Description:
    This file contains the functional implementation of data parsing functions
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "gfx_resources.h"
#include "gfx_hgc_definitions.h"

#define LARGE_FONT      &Arial24Pt
#define MEDIUM_FONT     &Arial12PtB
#define SMALL_FONT      &Arial12Pt

//static GFX_INDEX      gfxIndex=0;

static char * modeToneOnTxt      = "  Sine Tone - ON  ";
static char * modeToneOffTxt     = "  Sine Tone - off ";
static char * modeChirpOnTxt     = "    Chirp - ON   ";
static char * modeChirpOffTxt    = "    Chirp - off  ";
static char * modeSilentTxt      = "    Chirp - off  ";

//FIX:  This bypasses the GFX Digital Meter Object function with the incorrect
//      size value argument.  CAL
static void GFX_GOL_DigitalMeterValueSet32( GFX_GOL_DIGITALMETER *pObject,
                                            uint32_t value);

void APP_DisplayInit(GUI_DATA * guiData, AUDIO_GENERATE_STATUS * agStatus)
{
    //Parameters
    GFX_GOL_DigitalMeterValueSet32(hgcObj.pFreqMeterObj, (uint32_t) guiData->f1Hz);
    GFX_GOL_ObjectStateSet(hgcObj.pFreqMeterObj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
    GFX_GOL_DigitalMeterValueSet32(hgcObj.pFreqMeter2Obj, (uint32_t) guiData->f2Hz);
    GFX_GOL_ObjectStateSet(hgcObj.pFreqMeter2Obj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
    if (guiData->timeDeltaMs > 0)
    {
        GFX_GOL_DigitalMeterValueSet32(hgcObj.pDigitalMeter1Obj, (uint32_t) guiData->timeDeltaMs);
        GFX_GOL_ObjectStateSet(hgcObj.pDigitalMeter1Obj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
    }
    else
    {
        GFX_GOL_DigitalMeterValueSet32(hgcObj.pDigitalMeter1Obj, 
		                    (uint32_t) 99999); //inf
        GFX_GOL_ObjectStateSet(hgcObj.pDigitalMeter1Obj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
    }

    //Mode
    if (guiData->mode == MODE_TONE)
    {
        if (guiData->onOff)
        {
            GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeToneOnTxt);
        }
        else
        {
            GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeToneOffTxt);
        }
    }
    else if (guiData->mode == MODE_CHIRP)
    {
        if (guiData->onOff)
        {
            GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeChirpOnTxt);
        }
        else
        {
            GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeChirpOffTxt);
        }
    }
    else
    {
        //Silent
        //hgcObj.pSpeakerImageObj->pImage = (GFX_RESOURCE_HDR*)&Mute;
        GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeSilentTxt);
    }
    GFX_GOL_ObjectStateSet(hgcObj.pToneModeObj, GFX_GOL_STATICTEXT_DRAW_STATE);
            
}

//******************************************************************************
// APP_DisplayTask()
//
// GUI_DATA:
//       bool       displayUpdate;
//       GUI_MODE   mode;     MODE_TONE/MODE_CHIRP
//       GUI_SELECT select;   SELECT_F1/SELECT_F2/SELECT_T/SELECT_DUTY
//       int32_t    f1Hz;
//       int32_t    f2Hz;
//       int32_t    timeDeltaMs; //NOTE <0 implies inf. 
//       //int16_t    dutyCycle;
//       bool       onOff;   //Generated output
//
//******************************************************************************
void APP_DisplayTask(GUI_DATA * guiData)
{
    //int Red, Blue, Green, temp;

    if (guiData->displayUpdate == true)
    {
        //Update the selected parameter value display
        switch (guiData->select)
        {
            case SELECT_F1:
                GFX_GOL_DigitalMeterValueSet32(hgcObj.pFreqMeterObj, (uint32_t) guiData->f1Hz);
                GFX_GOL_ObjectStateSet(hgcObj.pFreqMeterObj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
                break;

            case SELECT_F2:
                GFX_GOL_DigitalMeterValueSet32(hgcObj.pFreqMeter2Obj, (uint32_t) guiData->f2Hz);
                GFX_GOL_ObjectStateSet(hgcObj.pFreqMeter2Obj, GFX_GOL_DIGITALMETER_UPDATE_STATE);
                break;

            case SELECT_T:
                if (guiData->timeDeltaMs > 0)
                {
                    GFX_GOL_DigitalMeterValueSet32(hgcObj.pDigitalMeter1Obj, 
                                                 (uint32_t) guiData->timeDeltaMs);
                    GFX_GOL_ObjectStateSet(hgcObj.pDigitalMeter1Obj, 
                                           GFX_GOL_DIGITALMETER_UPDATE_STATE);
                }
                else
                {
                    //Show infinity
                    GFX_GOL_DigitalMeterValueSet32(hgcObj.pDigitalMeter1Obj, 
                                                (uint32_t) 99999);  //5 Digits
                    GFX_GOL_ObjectStateSet(hgcObj.pDigitalMeter1Obj, 
                                           GFX_GOL_DIGITALMETER_UPDATE_STATE);
                    
                }
                break;

            default: break;
        }


        //Mode Display Change
        if (guiData->changeToneMode)
        {
            //Mode
            if (guiData->mode == MODE_TONE)
            {
                if (guiData->onOff)
                {
                    GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeToneOnTxt);
                }
                else
                {
                    GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeToneOffTxt);
                }
            }
            else if (guiData->mode == MODE_CHIRP)
            {
                if (guiData->onOff)
                {
                    GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeChirpOnTxt);
                }
                else
                {
                    GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeChirpOffTxt);
                }
            }
            else
            {
                //Silent
                //hgcObj.pSpeakerImageObj->pImage = (GFX_RESOURCE_HDR*)&Mute;
                GFX_GOL_StaticTextSet( hgcObj.pToneModeObj, (char *) modeSilentTxt);
            }
            GFX_GOL_ObjectStateSet(hgcObj.pToneModeObj, GFX_GOL_STATICTEXT_DRAW_STATE);

            guiData->changeToneMode = false;
        }

        guiData->displayUpdate = false;
        guiData->changeToneMode= false;

    } //End displayUpdate

} //End APP_DisplayTask()

static void GFX_GOL_DigitalMeterValueSet32( GFX_GOL_DIGITALMETER *pObject,
                                            uint32_t value)
{
    // store the previous and current value to be displayed
    pObject->Pvalue = pObject->Cvalue;
    pObject->Cvalue = value;

    pObject->textHeight = GFX_TextStringHeightGet(
                            pObject->hdr.pGolScheme->pFont);
}
