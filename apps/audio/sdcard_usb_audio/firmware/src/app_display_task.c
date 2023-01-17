/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_display_task.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "app_display_task.h"
#include "app_sdcard_audio_task.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

#define SYS_TIMER_PERIODIC_CALLBACK_RATE_MSEC       5
#define VOLUME_SLIDER_DEBOUNCE_PERIOD               50
/*  A brief description of a section can be given directly below the section
    banner.
 */
APP_DISPLAY_DATA AppDisplayData;
static bool AppDisplayDataIsListPopulated;
static uint16_t scrollBarPosition;
static uint16_t scrollBarDefaultPosition;
bool volumeSliderChangeInProgress = false;
uint16_t volatile volumeSliderDebounceCounter = 0;

/*
 * Forward declarations.
 */
static int8_t *_APP_DISPLAY_GetFileName(const int8_t *path);
static void _APP_DISPLAY_HandleMuteOn(void);
static void _APP_DISPLAY_HandleMuteOff(void);
static uint8_t _APP_DISPLAY_HandleVolumeIncreaseDecrease(void);
static void _APP_DISPLAY_VolumeSlider_DebounceTimerHandler(uintptr_t context, uint32_t currTick);

   
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_DISPAY_Initialize(void)
{  
    AppDisplayData.streamSource = APP_DISPLAY_STREAMING_SOURCE_UNCHANGED;
    
    AppDisplayData.volMuteSource = APP_DISPLAY_VOLUME_MUTE_SRC_SDCARD;    
    
    AppDisplayData.volumeMute = APP_DISPLAY_VOLUME_MUTE_UNCHANGED;            
    
    AppDisplayData.debounceTmrHamdle = SYS_TMR_HANDLE_INVALID;
}



///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_Tasks(void)
{
    uint8_t defaultvolume;
    uint8_t volume;

    if (SYS_TMR_HANDLE_INVALID == AppDisplayData.debounceTmrHamdle)
    {
        AppDisplayData.debounceTmrHamdle = SYS_TMR_CallbackPeriodic ( SYS_TIMER_PERIODIC_CALLBACK_RATE_MSEC, 1, _APP_DISPLAY_VolumeSlider_DebounceTimerHandler );
    }

    if(GFX_HGC_HasCurrentScreenDrawn())
    {                        
        if(true == APP_SDCARD_AUDIO_isSonglistAvailable() && AppDisplayDataIsListPopulated == false)
        {                        
            uint16_t index;
            APP_SDCARD_AUDIO_CARD_FILE_PATH *TablePointer;
            TablePointer = APP_SDCARD_AUDIO_GetFileTablePointer();
            
            hgcObj.pListBoxItem = GFX_GOL_ListBoxItemAdd(
            hgcObj.pTrackListBoxObj,
            NULL,
            (GFX_XCHAR *)_APP_DISPLAY_GetFileName(TablePointer[0].path),
            NULL,
            GFX_GOL_LISTBOX_ITEM_STATUS_REDRAW,
            0);
                        
            for (index = 0; index < (APP_SDCARD_AUDIO_FileCountGet()-1); index++)
            {
                hgcObj.pListBoxItem = GFX_GOL_ListBoxItemAdd(
                hgcObj.pTrackListBoxObj,
                hgcObj.pListBoxItem,
                (GFX_XCHAR *)_APP_DISPLAY_GetFileName(TablePointer[index+1].path),
                NULL,
                GFX_GOL_LISTBOX_ITEM_STATUS_REDRAW,
                0);                                                 
            }                    
   
            AppDisplayData.currentTrackId =  APP_SDCARD_AUDIO_CurrentTrackIdGet();
            GFX_GOL_ListBoxItemFocusSet(hgcObj.pTrackListBoxObj,AppDisplayData.currentTrackId);            
            GFX_GOL_ListBoxDraw(hgcObj.pTrackListBoxObj);                                  
            
            defaultvolume = APP_SDCARD_AUDIO_VolumeGet();
            
            scrollBarPosition = ((GFX_SCROLLBAR_RANGE_0 * defaultvolume)/APP_DISPLAY_VOLUME_MAX);
            GFX_GOL_ScrollBarPositionSet(hgcObj.pVolSliderObj,scrollBarPosition);
            scrollBarDefaultPosition = scrollBarPosition;
                           
			GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox), GFX_GOL_LISTBOX_SINGLE_SELECT_STATE|GFX_GOL_LISTBOX_DRAW_STATE);                                                                   
			GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DRAW_STATE);                                                                                                       
            
            AppDisplayDataIsListPopulated = true;
                    
        }                             
    }     
    
    if(AppDisplayData.streamSource!=APP_DISPLAY_STREAMING_SOURCE_UNCHANGED)
    {
        if(AppDisplayData.streamSource == APP_DISPLAY_STREAMING_SOURCE_CHANGED_SDCARD)
        {
            APP_USB_AUDIO_SuspendStreaming(); 
            APP_SDCARD_AUDIO_Initialize();      
            APP_StreamSourceSet(APP_STREAMING_SOURCE_SDCARD);
        }
        else if(AppDisplayData.streamSource == APP_DISPLAY_STREAMING_SOURCE_CHANGED_USB)
        {
            APP_SDCARD_AUDIO_SuspendStreaming();
            APP_USB_AUDIO_AttachDevice();  
            APP_StreamSourceSet(APP_STREAMING_SOURCE_USB);
        }
        else
        {
            
        }       

        GFX_GOL_ScrollBarPositionSet(hgcObj.pVolSliderObj,scrollBarDefaultPosition);
        GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DRAW_STATE);        
        GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, VolButton), (GFX_GOL_BUTTON_PRESSED_STATE | GFX_GOL_BUTTON_DRAW_STATE));                    
        
        AppDisplayData.streamSource = APP_DISPLAY_STREAMING_SOURCE_UNCHANGED;       
    }
    
    
    
    if(AppDisplayData.volumeMute != APP_DISPLAY_VOLUME_MUTE_UNCHANGED)
    {
        if(AppDisplayData.volMuteSource == APP_DISPLAY_VOLUME_MUTE_SRC_SDCARD)
        {
            if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_ON)
            {
                APP_SDCARD_AUDIO_MuteOn();
                _APP_DISPLAY_HandleMuteOn();                
            }
            else if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_OFF)
            {
                APP_SDCARD_AUDIO_MuteOff(); 
                _APP_DISPLAY_HandleMuteOff();
            }
            else if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_INCREASE_DECREASE)
            {                                                
                volumeSliderDebounceCounter = 0;
                volumeSliderChangeInProgress = true;
            }
            else
            {
            }       
        }
        else if(AppDisplayData.volMuteSource == APP_DISPLAY_VOLUME_MUTE_SRC_USB)
        {
            if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_ON)
            {
                APP_USB_AUDIO_MuteOn();
                _APP_DISPLAY_HandleMuteOn();                
            }
            else if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_OFF)
            {
                APP_USB_AUDIO_MuteOff();
                _APP_DISPLAY_HandleMuteOff();
            }
            else if(AppDisplayData.volumeMute == APP_DISPLAY_VOLUME_MUTE_INCREASE_DECREASE)
            {                                                    
                volumeSliderDebounceCounter = 0;
                volumeSliderChangeInProgress = true;
            }  
            else
            {
                ;
            }
        }
        else
        {
            ;
        }
        
        AppDisplayData.volumeMute = APP_DISPLAY_VOLUME_MUTE_UNCHANGED;               
    
    }   

    if (true == volumeSliderChangeInProgress && volumeSliderDebounceCounter > VOLUME_SLIDER_DEBOUNCE_PERIOD)
    {
        volume = _APP_DISPLAY_HandleVolumeIncreaseDecrease();  
        
        if(AppDisplayData.volMuteSource == APP_DISPLAY_VOLUME_MUTE_SRC_SDCARD)
        {            
            APP_SDCARD_AUDIO_VolumeSet(volume);
        }
        else
        {            
            APP_USB_AUDIO_VolumeSet(volume);
        }
        
        volumeSliderChangeInProgress = false;
        volumeSliderDebounceCounter = 0;        
    }
    
    if(AppDisplayDataIsListPopulated == true)
    {
        if(AppDisplayData.currentTrackId != APP_SDCARD_AUDIO_CurrentTrackIdGet())
        {
            AppDisplayData.currentTrackId = APP_SDCARD_AUDIO_CurrentTrackIdGet();
            GFX_GOL_ListBoxItemFocusSet(hgcObj.pTrackListBoxObj,AppDisplayData.currentTrackId);  
            GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, TrackListBox), GFX_GOL_LISTBOX_DRAW_FOCUS_STATE | GFX_GOL_LISTBOX_DRAW_STATE);                                                               
        }
    }    
}

///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_StreamSourceSet(APP_DISPLAY_STREAMING_SOURCE streamSrc)
{
    AppDisplayData.streamSource = streamSrc;
}

///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_VolumeMuteSourceSet(APP_DISPLAY_VOLUME_MUTE_SRC volMuteSrc)
{
    AppDisplayData.volMuteSource = volMuteSrc;
}


///////////////////////////////////////////////////////////////////////////////
void APP_DISPLAY_VolumeMuteChange(APP_DISPLAY_VOLUME_MUTE volMuteChange)
{
    AppDisplayData.volumeMute = volMuteChange;
}

///////////////////////////////////////////////////////////////////////////////
static int8_t *_APP_DISPLAY_GetFileName(const int8_t *path)
{
    int8_t *filename = (int8_t*)strrchr((const char*)path, '/');
    if (filename == NULL)
        filename = (int8_t *)path;
    else
        filename++;
    return filename;
}

///////////////////////////////////////////////////////////////////////////////
static void _APP_DISPLAY_HandleMuteOn(void)
{
    GFX_GOL_ScrollBarPositionSet(hgcObj.pVolSliderObj, 0);
    GFX_GOL_ObjectStateClear(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DISABLED_STATE);
    GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DRAW_STATE);                    
}

///////////////////////////////////////////////////////////////////////////////
static void _APP_DISPLAY_HandleMuteOff(void)
{
    GFX_GOL_ScrollBarPositionSet(hgcObj.pVolSliderObj, scrollBarPosition);
    GFX_GOL_ObjectStateClear(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DISABLED_STATE);
    GFX_GOL_ObjectStateSet(GFX_GOL_ObjectFind(GFX_INDEX_0, VolSlider), GFX_GOL_SCROLLBAR_DRAW_STATE);                                    
}

///////////////////////////////////////////////////////////////////////////////
static uint8_t _APP_DISPLAY_HandleVolumeIncreaseDecrease(void)
{
    scrollBarPosition = GFX_GOL_ScrollBarPositionGet(hgcObj.pVolSliderObj);
    return (scrollBarPosition * APP_DISPLAY_VOLUME_MAX)/GFX_SCROLLBAR_RANGE_0;  
}

///////////////////////////////////////////////////////////////////////////////
static void _APP_DISPLAY_VolumeSlider_DebounceTimerHandler(uintptr_t context, uint32_t currTick)
{    
    if (true == volumeSliderChangeInProgress)
    {
        volumeSliderDebounceCounter++;
    }
}



/* *****************************************************************************
 End of File
 */
