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
#include "gfx/gfx_primitive.h"
#include "gfx/gfx.h"

#define LARGE_FONT      &Arial20pt
#define MEDIUM_FONT     &Arial14pt
#define SMALL_FONT      &Arial12pt
#define RX_DATA_BASE    65
#define TASK_BAR_COLOR GFX_RGBConvert(29, 46, 60)

#define BACKGROUND_COLOR BLACK
#define TEXT_COLOR1     WHITE
#define TEXT_COLOR2     BRIGHTRED


/************PROGRESS BAR******************/
#define PROGRESS_BAR_PIXEL_WIDTH  130
#define PROGRESS_BAR_LEFT         45
#define PROGRESS_BAR_TOP          149
#define PROGRESS_BAR_RIGHT        (PROGRESS_BAR_LEFT+PROGRESS_BAR_PIXEL_WIDTH)
#define PROGRESS_BAR_BOTTOM       164

/************TRACK DURATION****************/
#define DURATION_TIME_LEFT        10
#define DURATION_TIME_TOP         PROGRESS_BAR_TOP


/************Playback Time*****************/
#define PLAYBACK_TIME_LEFT        (PROGRESS_BAR_RIGHT+5)
#define PLAYBACK_TIME_TOP         PROGRESS_BAR_TOP



static GFX_INDEX            gfxIndex=0;

static uint32_t totalSongs = 0;
//float BarScaleRate = 0.0;
static int BarEnd = 0;
bool REFLECTION = 0;
//GFX_STATUS GFX_TextCharRenderReflect(  SYS_MODULE_INDEX gfxIndex, GFX_XCHAR ch, GFX_FONT_OUTCHAR *pParam);
GFX_STATUS_BIT GFX_RenderStatusCheck(SYS_MODULE_INDEX gfxIndex);
GFX_STATUS GFX_DRV_TextCharRender(GFX_XCHAR ch);
//GFX_STATUS GFX_TextCharRender(  SYS_MODULE_INDEX gfxIndex,GFX_XCHAR ch, GFX_FONT_OUTCHAR *pParam);

void display_tasks(PLAYER_EVENT event, uint32_t data)
{
    uint8_t i=0; //for while loops to fill song time
    int BarCurrent = 0;
    int BarSongCurrent = 0;
    char WorkBuffer[30] = {0};
    char WorkBuffer2[30] = {0};

    GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_COLOR);

    GFX_TransparentColorEnable(GFX_INDEX_0, TASK_BAR_COLOR);

    GFX_ColorSet(GFX_INDEX_0, WHITE);
    GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
    
//******************************************************************************
//   Debug
//******************************************************************************   
#if defined DEBUG
    GFX_TextStringDraw(GFX_INDEX_0, 30, 45, WorkBuffer, 0);
    GFX_TextStringDraw(GFX_INDEX_0, 30, 60, WorkBuffer2, 0);
//******************************************************************************
    uint8_t y=0;
    while(y < 172)// y horizontal crosshairs
    {
        GFX_LineDraw(GFX_INDEX_0, 0, y, 10, y);
        y=y+10;
    }
    y=0;
    while(y < 220) //x horizonatal crosshairs
    {
        GFX_LineDraw(GFX_INDEX_0, y, 0, y, 10);
        y=y+10;
    }
#endif

    uint16_t Arial12Height = GFX_TextStringHeightGet((GFX_RESOURCE_HDR*) &Arial12pt);
    APP_AUDIOPLAYER *appDataPtr = APP_GetAppDataInstance();
    switch(appDataPtr->currentStreamType)
    {
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        case APP_STREAM_AAC:
            
            GFX_CircleFillDraw(GFX_INDEX_0,25,54, 2);
            break;
        case APP_STREAM_MP3:
            GFX_CircleFillDraw(GFX_INDEX_0,57,54, 2);
            break;
        case APP_STREAM_WMA:
            GFX_CircleFillDraw(GFX_INDEX_0,89,54, 2);
            break;
        case APP_STREAM_WAV:
            GFX_CircleFillDraw(GFX_INDEX_0,124,54, 2);
            break;
        ///////////////////////Second line of decoders//////////////////////////
        case APP_STREAM_SPEEX:
            GFX_CircleFillDraw(GFX_INDEX_0,30,60+Arial12Height, 2);
            break;
        case APP_STREAM_ADPCM:
            GFX_CircleFillDraw(GFX_INDEX_0,80,60+Arial12Height, 2);
            break;
        case APP_STREAM_OPUS:
            GFX_CircleFillDraw(GFX_INDEX_0,124,60+Arial12Height, 2);
            break;
// Tobe added
/*
        case APP_STREAM_FLAC:
            GFX_CircleFillDraw(GFX_INDEX_0,30,66+Arial12Height, 2);
            break;
*/          
        default:
            break;
            
    }
    
    ////////////////////////////////////////////////////////////////////////////
    if (appDataPtr->AAC_decoder_enabled == 1)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 37, "AAC", 0);
    }
    else
    {
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 37, "AAC", 0);
    }
    ////////////////////////////////////////////////////////////////////////////
    if (appDataPtr->MP3_decoder_enabled == 1)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 45, 37, "MP3", 0);
    }
    else
    {
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 45, 37, "MP3", 0);
    }
    ////////////////////////////////WMA Decoder/////////////////////////////////
    if (appDataPtr->WMA_decoder_enabled)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 75, 37, "WMA", 0);
    }
    else
    {
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 75, 37, "WMA", 0);
    }
    ////////////////////////////////////////////////////////////////////////////
    if(appDataPtr->WAV_decoder_enabled)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 110, 37, "WAV", 0);
    }else{
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 110, 37, "WAV", 0);
    }
    ////////////////////////////////////////////////////////////////////////////
    
    if(appDataPtr->SPEEX_decoder_enabled)
    {
       
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 45+Arial12Height, "SPEEX", 0);
    }else{
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 45+Arial12Height, "SPEEX", 0);
    }
    
    if(appDataPtr->ADPCM_decoder_enabled)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 60, 45+Arial12Height, "ADPCM", 0);
        
    }else{
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 60, 45+Arial12Height, "ADPCM", 0);
    }
    if(appDataPtr->OPUS_decoder_enabled)
    {
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 110, 45+Arial12Height, "OPUS", 0);
        
    }else{
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 110, 45+Arial12Height, "OPUS", 0);
    }
	// Tobe added
    /*
    if(appDataPtr->FLAC_decoder_enabled)
    {       
        GFX_ColorSet(GFX_INDEX_0, BRIGHTBLUE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 70+Arial12Height, "FLAC", 0);
    }else{
        GFX_ColorSet(GFX_INDEX_0, GRAY128);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);
        GFX_TextStringDraw(GFX_INDEX_0, 15, 70+Arial12Height, "FLAC", 0);
    }
    */

//******************************************************************************
// Lower GFX area
//******************************************************************************
    if (event == PLAYER_EVENT_DISK_REMOVED)
    {
        if(GFX_ImageDraw(gfxIndex, 171, 4, (GFX_RESOURCE_HDR *) &NO_USB_DISK_RED) == GFX_STATUS_FAILURE)
            return;
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 0, 40, 220, 175); // clears black area
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        REFLECTION = 1;
        GFX_TextStringDraw(GFX_INDEX_0, 30, 41, "Please Insert USB Drive", 0);
//        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial12pt);

//        GFX_TextStringDraw(GFX_INDEX_0, 5, 149, "00:00", 0);
        REFLECTION = 0;
    }
    if (event == PLAYER_EVENT_DISK_INSERTED)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 0, 40, 220, 175); // clears black area
        if(GFX_ImageDraw(gfxIndex, 171, 4, (GFX_RESOURCE_HDR *) &USB_DISK_GREEN) == GFX_STATUS_FAILURE)
            return;
    }

//******************************************************************************
// Song length and current time
//******************************************************************************

    if (event == PLAYER_EVENT_TRACK_TOTAL_TIME)
    {
        BarEnd = data; // Total Elapsed time in seconds (for end of the track)

      //BarScaleRate = 130.0/(float) data;
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, DURATION_TIME_LEFT, DURATION_TIME_TOP, PROGRESS_BAR_LEFT, 175);
        
        char mm = data/60;
        char ss = (data - mm*60);
        
        WorkBuffer[0] = mm/10 + 48; // convert first digit of minutes to digit '0'~'9' char, ASCII: '0'~'9' ---> 48~57
        WorkBuffer[1] = mm-(mm/10)*10 + 48;

        WorkBuffer[2] = ':';

        WorkBuffer[3] = ss/10+48;
        WorkBuffer[4] = ss-(ss/10)*10+48;

        GFX_ColorSet(GFX_INDEX_0, WHITE);
        REFLECTION = 1;
        GFX_TextStringDraw(GFX_INDEX_0, DURATION_TIME_LEFT, DURATION_TIME_TOP, WorkBuffer, 0);
        REFLECTION = 0;
        return;
    }
    if (event == PLAYER_EVENT_TRACK_PLAY_TIME && (BarEnd ==0 || data <= BarEnd)) // Make sure the play time not exceed total track time
    {

        BarSongCurrent = data; // Elapsed time into track at this time, in seconds
        GFX_ColorSet(GFX_INDEX_0,  BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, PLAYBACK_TIME_LEFT, PLAYBACK_TIME_TOP, 219, 175); //136

        char mm = data/60;
        char ss = (data - mm*60);

        WorkBuffer[0] = mm/10 + 48; // convert first digit of minutes to digit '0'~'9' char
        WorkBuffer[1] = mm-(mm/10)*10 + 48;

        WorkBuffer[2] = ':';

        WorkBuffer[3] = ss/10+48;
        WorkBuffer[4] = ss-(ss/10)*10+48;

        GFX_ColorSet(GFX_INDEX_0, WHITE);
        REFLECTION = 1;
        GFX_TextStringDraw(GFX_INDEX_0, PLAYBACK_TIME_LEFT, PLAYBACK_TIME_TOP, WorkBuffer, 0);
        REFLECTION = 0;
        
        
        
        //******************************************************************************
        // progress bar
        //******************************************************************************
        // Clear Progress bar
        GFX_ColorSet(GFX_INDEX_0,  BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, PROGRESS_BAR_LEFT, PROGRESS_BAR_TOP, PROGRESS_BAR_RIGHT, PROGRESS_BAR_BOTTOM);

        // Draw Progress bar outline
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_RectangleDraw(GFX_INDEX_0, PROGRESS_BAR_LEFT, PROGRESS_BAR_TOP, PROGRESS_BAR_RIGHT, PROGRESS_BAR_BOTTOM);
        if(BarEnd == 0)
        {    
            BarCurrent = 0;
        }
        else{

            BarCurrent = (PROGRESS_BAR_PIXEL_WIDTH*((uint16_t)BarSongCurrent))/BarEnd;
        }


        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_RectangleFillDraw(GFX_INDEX_0, PROGRESS_BAR_LEFT, PROGRESS_BAR_TOP, (BarCurrent+PROGRESS_BAR_LEFT), PROGRESS_BAR_BOTTOM);
        return;
    }


//******************************************************************************
// Next track/track roll over events
//******************************************************************************
    if(event == PLAYER_EVENT_TRACK_CHANGED)
    {
        BarEnd = 0;
//        BarScaleRate = 0.0;
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
      //GFX_RectangleFillDraw(GFX_INDEX_0, 39, 150, 169, 164);
        GFX_RectangleFillDraw(GFX_INDEX_0, 0, 40, 220, 175); // clears black area

        // Render current song index
        itoa(WorkBuffer2, data ,10);
        while(WorkBuffer2[i] != '\0')
        {
            WorkBuffer[i] = WorkBuffer2[i];
            ++i;
        }
        WorkBuffer[i] = ' ';
        WorkBuffer[i++] = 'o';
        WorkBuffer[i++] = 'f';
        WorkBuffer[i++] = ' ';
        int j = i;
        i = 0;
        itoa(WorkBuffer2, totalSongs ,10);
        while(WorkBuffer2[i] != '\0')
        {
            WorkBuffer[j] = WorkBuffer2[i];
            ++i;
            ++j;
        }
                
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 150, 70, WorkBuffer, 0);
        return;
    }
//******************************************************************************
// upper right
//******************************************************************************

    if (event == PLAYER_EVENT_SAMPLERATE)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 150, 40, 220, 54);
        itoa(WorkBuffer2, data ,10);

        WorkBuffer[0] = 'H';
        WorkBuffer[1] = 'z';
        WorkBuffer[2] = ':';
        WorkBuffer[3] = ' ';

        while(WorkBuffer2[i] != '\0')
        {
            WorkBuffer[4+i] = WorkBuffer2[i];
            ++i;
        }

        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 150, 40, WorkBuffer, 0);
        return;
    }
    if (event == PLAYER_EVENT_BITRATE)
    {

        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 150, 55, 220, 70);
        itoa(WorkBuffer2, data ,10);
        WorkBuffer[0] = 'K';
        WorkBuffer[1] = 'b';
        WorkBuffer[2] = '/';
        WorkBuffer[3] = 's';
        WorkBuffer[4] = ':';
        WorkBuffer[5] = ' ';
        while(WorkBuffer2[i] != '\0')
        {
            WorkBuffer[6+i] = WorkBuffer2[i];
            ++i;
        }
        GFX_ColorSet(GFX_INDEX_0, WHITE);
//        REFLECTION = 1;
        GFX_TextStringDraw(GFX_INDEX_0, 150, 55, WorkBuffer, 0);
//        REFLECTION = 0;
        return;
    }
    if(event == PLAYER_EVENT_READY)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 150, 70, 220, 85);
        
        totalSongs = data;
        // Render current song index
        WorkBuffer[i++] = '1';
        WorkBuffer[i++] = ' ';
        WorkBuffer[i++] = 'o';
        WorkBuffer[i++] = 'f';
        WorkBuffer[i++] = ' ';
        int j = i;
        i = 0;
        itoa(WorkBuffer2, totalSongs ,10);
        while(WorkBuffer2[i] != '\0')
        {
            WorkBuffer[j] = WorkBuffer2[i];
            ++i;
            ++j;
        }
                
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 150, 70, WorkBuffer, 0);
        return;
    }
    
//******************************************************************************
// Album info
//******************************************************************************

    if (event == PLAYER_EVENT_TAG_ARTIST)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 10, 90, 150, 105); //1360
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 10, 90,((char*) data), 0);
        return;
    }

    if (event == PLAYER_EVENT_TAG_TITLE)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 10, 105, 150, 120); //1360
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 10, 105,((char*) data), 0);
        return;
    }
    if (event == PLAYER_EVENT_TAG_ALBUM)
    {
        GFX_ColorSet(GFX_INDEX_0, BACKGROUND_COLOR);
        GFX_RectangleFillDraw(GFX_INDEX_0, 10, 120, 150, 135); //1360
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_TextStringDraw(GFX_INDEX_0, 10, 120,((char*) data), 0);
        return;
    }
    


//******************************************************************************
// Volume Change
//******************************************************************************

    if(event == PLAYER_EVENT_VOLUME_CHANGE){
        return;
    }

//******************************************************************************
// Error Message
//******************************************************************************
    
    
}



void GFX_MENU_DRAW(void)
{

        GFX_TransparentColorEnable(GFX_INDEX_0, TASK_BAR_COLOR);

/*******************************************************************************/
//      Draw Task Bars at Top & Bottom
/*******************************************************************************/
        GFX_ColorSet(gfxIndex, BACKGROUND_COLOR );
        GFX_ScreenClear(gfxIndex);

        GFX_ColorSet(gfxIndex, TASK_BAR_COLOR );

        GFX_GradientColorSet(GFX_INDEX_0, BRIGHTRED, BACKGROUND_COLOR);
        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_GRADIENT_DOWN);
        GFX_RectangleRoundFillDraw(GFX_INDEX_0, 15, 0, 205, 19, 15); // top task bar lowest edge 34 pixels down

//        GFX_GradientColorSet(GFX_INDEX_0, BACKGROUND_COLOR, TASK_BAR_COLOR);
//        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_GRADIENT_UP);
//        GFX_RectangleRoundFillDraw(GFX_INDEX_0, 15, 152, 205, 171, 15);// bottom task bar highest edge 137 pixels

        GFX_FillStyleSet(GFX_INDEX_0,GFX_FILL_STYLE_COLOR);
/*******************************************************************************/
//      Text Section
/*******************************************************************************/
        GFX_ColorSet(GFX_INDEX_0, WHITE);
        GFX_FontSet(GFX_INDEX_0, (GFX_RESOURCE_HDR*) &Arial14pt);
        GFX_TextStringDraw(GFX_INDEX_0, 30, 24, "Universal Audio Decoder", 0);



/*******************************************************************************/
//      Image Icons - Upper Task Bar
/*******************************************************************************/

        /* Mircochip logo*/
        if(GFX_ImageDraw(gfxIndex, 4, 1, (GFX_RESOURCE_HDR *) &MCHP_LOGO) == GFX_STATUS_FAILURE)
            return;
}


//<editor-fold defaultstate="collapsed" desc="Text Reflection">
//GFX_STATUS  GFX_TextCharDraw(SYS_MODULE_INDEX gfxIndex, GFX_XCHAR ch)
//{
//
//    typedef enum
//    {
//        TC_IDLE,
//        TC_GET_INFO,
//        TC_CHECK_RENDER_STATUS,
//        TC_RENDER_CHAR,
//        TC_RENDER_CHAR_DRIVER,
//        TC_RENDER_STATUS_CHECK,
//        TC_RENDER_NEWLINE_CHAR,
//    } TEXTCHAR_RENDER_STATES;
//
//    static TEXTCHAR_RENDER_STATES state = TC_IDLE, renderState;
//    static GFX_FONT_OUTCHAR OutCharParam;
//    static GFX_STATUS status;
//    static GFX_STATUS_BIT statusBit;
//    static GFX_RESOURCE_HDR *pFont;
//
//    while(1)
//    {
//        switch (state)
//        {
//            case TC_IDLE:
//
//                status = GFX_STATUS_SUCCESS_BIT;
//                // check for error conditions
//                // since we cannot predict that the font table is always valid we
//                // cannot allow the program to hang. So intead of returning
//                // GFX_STATUS_FAILURE we will return a GFX_STATUS_SUCCESS.
//                // When the character is not in the table, it will just be
//                // ignored and not printed.
//                pFont = GFX_FontGet(gfxIndex);
//
//                // check if new line character, if yes just go and adjust
//                // the text cursor position.
//                if (ch == (GFX_UXCHAR)0x0A)
//                {
//                   state = TC_RENDER_NEWLINE_CHAR;
//                   break;
//                }
//
//                // make sure characters are printable
//                if((GFX_UXCHAR)ch < (GFX_UXCHAR)pFont->resource.font.header.firstChar)
//                    statusBit = GFX_STATUS_ERROR_BIT;
//                else if((GFX_UXCHAR)ch > (GFX_UXCHAR)pFont->resource.font.header.lastChar)
//                    statusBit = GFX_STATUS_ERROR_BIT;
//#ifdef  GFX_CONFIG_FONT_ANTIALIASED_DISABLE
//                else if(pFont->resource.font.header.bpp > 1)
//                    statusBit = GFX_STATUS_ERROR_BIT;
//#endif
//                if (statusBit == GFX_STATUS_ERROR_BIT)
//                    return GFX_STATUS_SUCCESS;
//                else
//                    state = TC_GET_INFO;
//
//                // no break here since the next state is what we want
//
//            case TC_GET_INFO:
//
//                // initialize variables
//#ifndef GFX_CONFIG_FONT_EXTERNAL_DISABLE
//                OutCharParam.pChImage = NULL;
//#endif
//                OutCharParam.xAdjust = 0;
//                OutCharParam.yAdjust = 0;
//                OutCharParam.xWidthAdjust = 0;
//                OutCharParam.heightOvershoot = 0;
//                state = TC_CHECK_RENDER_STATUS;
//
//                // no break here since the next state is what we want
//
//            case TC_CHECK_RENDER_STATUS:
//
//                if (GFX_RenderStatusCheck(gfxIndex) == GFX_STATUS_BUSY_BIT)
//                {
//                    state = TC_IDLE;
//                    return (GFX_STATUS_FAILURE);
//                }
//
//                // get the character information
//                switch (pFont->type)
//                {
//
//                    case GFX_RESOURCE_FONT_FLASH_NONE:
//                        GFX_TextCharInfoFlashGet(gfxIndex, ch, &OutCharParam);
//                        state = TC_RENDER_CHAR;
//                        renderState = TC_RENDER_CHAR;
//                        break;
//
//                    case GFX_RESOURCE_FONT_EXTERNAL_NONE:
//                        GFX_TextCharInfoExternalGet(gfxIndex, ch, &OutCharParam);
//                        state = TC_RENDER_CHAR;
//                        renderState = TC_RENDER_CHAR;
//                        break;
//
//                    case GFX_RESOURCE_FONT_EDS_NONE:
//                        state = TC_RENDER_CHAR_DRIVER;
//                        renderState = TC_RENDER_CHAR_DRIVER;
//                        break;
//
//                    default:
//                        state = TC_IDLE;
//                        return (GFX_STATUS_SUCCESS);
//                } // end of switch
//
//                break;
//
//            case TC_RENDER_CHAR_DRIVER:
//
//                // this is the case when the driver layer implements
//                // code to render the characters
//
//                status = GFX_DRV_TextCharRender(ch);
//                state = TC_RENDER_STATUS_CHECK;
//                break;
//
//            case TC_RENDER_CHAR:
//
////                if(REFLECTION ==1)
////                {
////                   status = GFX_TextCharRenderReflect(gfxIndex, ch, &OutCharParam);
////                }
////                else
//                {
//                    status = GFX_TextCharRender(gfxIndex, ch, &OutCharParam);
//                }
//                state = TC_RENDER_STATUS_CHECK;
//
//                // no break here since the next state is what we want
//
//            case TC_RENDER_STATUS_CHECK:
//
//                // if the rendering of the character did not return with
//                // GFX_STATUS_SUCCESS, application should call it again
//                // and not re-do the initialization done on the previous
//                // states above. So the state only changes to TC_IDLE
//                // when it is a success or an error.
//                if(status == GFX_STATUS_SUCCESS)
//                {
//                    state = TC_IDLE;
//                }
//                else
//                {
//                    state = renderState;
//                }
//                return (status);
//
//            case TC_RENDER_NEWLINE_CHAR:
//                // since the character is a newline character, just
//                // adjust the text cursor position to the new line
//
//                GFX_TextCursorPositionSet(gfxIndex,
//                                GFX_TextCursorPositionXGet(gfxIndex),
//                                (
//                                    GFX_TextCursorPositionYGet(gfxIndex) +
//                                    pFont->resource.font.header.height
//                                ));
//                status = GFX_STATUS_SUCCESS;
//                state = TC_IDLE;
//                return (status);
//
//            default:
//
//                // this state should never happen
//                state = TC_IDLE;
//                return GFX_STATUS_FAILURE;
//        } // end of switch ...
//    } // end of while(1) ...
//
//}

//GFX_STATUS GFX_TextCharRenderReflect(  SYS_MODULE_INDEX gfxIndex, GFX_XCHAR ch, GFX_FONT_OUTCHAR *pParam)
//{
//    uint8_t     shadowy;
//    GFX_COLOR   shadowfade = 0;
//    uint8_t     shadowsteps;
//    uint8_t     temp = 0;
//    uint8_t     mask;
//    uint8_t     restoremask;
//    int16_t     xCnt, yCnt, x = 0, y, fontHeight;
//
////    GFX_PRIMITIVE_CLIENT_OBJ * client = &primitiveClient[gfxIndex];
//
//    fontHeight = GFX_TextStringHeightGet(GFX_FontGet(gfxIndex));
//    restoremask = 0x01;
//
//    fontHeight = GFX_TextStringHeightGet(GFX_FontGet(gfxIndex));
//    y = GFX_TextCursorPositionYGet(gfxIndex) + pParam->yAdjust;
//    shadowsteps = (255 / fontHeight) >> 1;
//    shadowy = y + (fontHeight<<1);   //Gap
//
//    for(yCnt = 0; yCnt < fontHeight + pParam->heightOvershoot; yCnt++)
//    {
//
//        x = GFX_TextCursorPositionXGet(gfxIndex) + pParam->xAdjust;
//        mask = 0;
//
//        for(xCnt = 0; xCnt < pParam->chGlyphWidth; xCnt++)
//        {
//
//            if(mask == 0)
//            {
//                temp = *(pParam->pChImage)++;
//                mask = restoremask;
//            }
//
//            // check if the pixel is in the text region
//            if (
//                    (   (GFX_TextAreaLeftGet(gfxIndex)   < x) &&
//                        (GFX_TextAreaRightGet(gfxIndex)  > x)
//                    )
//                    &&
//                    (   (GFX_TextAreaTopGet(gfxIndex)    < y) &&
//                        (GFX_TextAreaBottomGet(gfxIndex) > y)
//                    )
//               )
//            {
//
//                if(temp & mask)
//                {
//                    GFX_ColorSet(0, WHITE);
//                    DRV_GFX_PixelPut(x, y);
//                    GFX_ColorSet(0, GFX_RGBConvert(shadowfade, shadowfade, shadowfade));
//                    DRV_GFX_PixelPut(x, shadowy);
//                }
//
//            } // end of if statement to check if in rendering area
//
//            mask <<= 1;
//            x++;
//
//        } // for(xCnt = 0; xCnt < pParam->chGlyphWidth; xCnt++)...
//
//        y++;
//        shadowy--;
//        shadowfade += shadowsteps;
//    } // end of for(yCnt = 0; yCnt...
//
//    // move cursor
//    GFX_TextCursorPositionSet(gfxIndex,
//                                (x - pParam->xAdjust - pParam->xWidthAdjust),
//                                GFX_TextCursorPositionYGet(gfxIndex)
//                             );
//
//    return (GFX_STATUS_SUCCESS);
//
//}
// </editor-fold>
