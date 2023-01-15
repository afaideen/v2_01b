/*******************************************************************************
  Graphic Equalizer Interface Source File

  Company:
    Microchip Technology Inc.

  File Name:
    graphic_equalizer.c

  Summary:
    Supports graphic equalizer display.

  Description:
    Supports graphic equalizer display, with primitives that display
    frequency band signal energy and filter gain.  It is used in conjuction
    with the Audio Equalizer Filtering Library (audio_equalizer.h).

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "math/audio_equalizer/audio_equalizer_fixedpoint.h"
#include "gfx/gfx_gol_equalizer.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

// Graphic Display Label memory
static char *_pLeftLabel  = NULL; // Pointer to left  label
static char *_pRightLabel = NULL; // Pointer to right label
static void *_pLabelFont  = NULL; // Pointer to labels font

// Graphic Display Scaling memory
static libq_q15d16_t _SignalMinValue = -30; //dBFS
static libq_q15d16_t _SignalMaxValue =   0; //dBFS
static libq_q15d16_t _GainMinValue   = -20; //dB
static libq_q15d16_t _GainMaxValue   = +20; //dB

// Band value memory
static int16_t _YtopEnergyOld[3][GFX_EQUAL_MAX_BANDS];
static libq_q0d15_t _energyChannelBand[3][GFX_EQUAL_MAX_BANDS];
static libq_q0d15_t _energyChannelBandOld[3][GFX_EQUAL_MAX_BANDS];
static libq_q0d15_t _gainChannelBand[3][GFX_EQUAL_MAX_BANDS];
static libq_q0d15_t _gainChannelBandOld[3][GFX_EQUAL_MAX_BANDS];

// Band focus memory
static bool _bFocusOn = false;

/*********************************************************************
 * Equalizer draw states
 ********************************************************************/
typedef enum
{
    DRAW1,
    DRAW2,
    DRAW3,
    DRAW4,
    DRAW5,
    DRAW6,
    DRAW7,
    DRAW8,
    DRAW9,
    DRAW10,
    DRAW11,
    DRAW12
} EQUALIZER_DRAW_STATES;

// *****************************************************************************
/* Graphic Equalizer Band Energy Color Lookup Table (LUT)

  Summary:
    Defines the Red/Green/Blue color each bar in a channel/band energy display.

  Description:
    Defines the Red/Green/Blue color each bar in a channel/band energy display.

  Remarks:
    The number of colors needed should match the nBars argument used in the call
    to sGRAPHIC_EQUALIZER_Create.
*/

static GFX_COLOR _BarColorLUT[] = { GFX_RGBConvert(255,0,0),
                                    GFX_RGBConvert(255,63,0),
                                    GFX_RGBConvert(255,127,0),
                                    GFX_RGBConvert(255,191,0),
                                    GFX_RGBConvert(255,255,0),
                                    GFX_RGBConvert(207,255,0),
                                    GFX_RGBConvert(191,255,0),
                                    GFX_RGBConvert(81,255,0),
                                    GFX_RGBConvert(0,255,0),
                                    GFX_RGBConvert(0,255,81),
                                    GFX_RGBConvert(0,255,191),
                                    GFX_RGBConvert(0,255,207),
                                    GFX_RGBConvert(0,255,255),
                                    GFX_RGBConvert(0,207,255),
                                    GFX_RGBConvert(0,81,255),
                                    GFX_RGBConvert(0,0,255)};

#define LOG2_BARCOLORLUT_SIZE      4   // Color LUT has 16 entries

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Helper Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    GFX_STATUS _BandRefresh(GFX_EQUAL_CHANNEL channel, uint8_t iBand, bool bForceRedraw)

  Summary:
    Refreshes display for given channel and band
 */
GFX_STATUS _BandRefresh(GFX_GOL_EQUALIZER *equalizer, GFX_EQUAL_CHANNEL channel, uint8_t iBand)
{
    static EQUALIZER_DRAW_STATES state = DRAW1;
    static int16_t iBar,iLUTColor;
    static int16_t barPixelHeight;
    static int16_t Xstart,Xend,Ygain,Ytop,YtopEnergy,Ybottom;
    static GFX_COLOR bgColor, gainIconColor;

    switch(state)
    {
        case DRAW1:

    // If channel/band is in "Focus"
    if ( _bFocusOn && channel == equalizer->iFocusChannel && iBand == equalizer->iFocusBand)
    {
        bgColor = FOCUSCOLOR;
        gainIconColor = GAINFOCUSCOLOR;
    }
    else
    {
        bgColor = EQ_BACKGROUND_COLOR;
        gainIconColor = GAINCOLOR;
    }

    // Bar left/right limits
    Xstart = equalizer->Xleft1stBar + channel*(equalizer->width + (equalizer->BarWidth>>1));
    Xstart += iBand*(equalizer->BarWidth + (equalizer->BarWidth>>1));
    Xend    = Xstart + equalizer->BarWidth;

    barPixelHeight = (equalizer->barPixelRange * _energyChannelBand[channel][iBand])>>15;
    YtopEnergy = equalizer->YtopBars + equalizer->barPixelRange - barPixelHeight;

    // If band energy bar or filter gain need redrawing...
    if ( equalizer->bForceRedraw ||
         YtopEnergy != _YtopEnergyOld[channel][iBand] ||
         _gainChannelBandOld[channel][iBand] != _gainChannelBand[channel][iBand] )
    {
        if ( YtopEnergy > _YtopEnergyOld[channel][iBand] )
        {
            state = DRAW2;
            // Erase unused band "LED"s
case DRAW2:           GFX_ColorSet(bgColor);
            if(GFX_RectangleFillDraw(Xstart,equalizer->YtopBars,Xend,YtopEnergy) == GFX_STATUS_FAILURE)
                return(GFX_STATUS_FAILURE);
        }
        _YtopEnergyOld[channel][iBand] = YtopEnergy; // Save new old value

    // If filter gain has changed, erase old gain
    if ( _gainChannelBandOld[channel][iBand] != _gainChannelBand[channel][iBand] )
    {
        state = DRAW3;
case DRAW3:
        GFX_ColorSet(bgColor);
        barPixelHeight = (equalizer->barPixelRange * _gainChannelBandOld[channel][iBand])>>15;
        Ygain = equalizer->YtopBars + equalizer->barPixelRange - barPixelHeight;

        if(GFX_RectangleRoundDraw(Xstart + (equalizer->BarWidth>>1)-3, Ygain,
                  Xstart + (equalizer->BarWidth>>1)+3, Ygain,
                  (3*equalizer->BarHeight>>3) ) == GFX_STATUS_FAILURE)
            return(GFX_STATUS_FAILURE);
    }

    for(iBar = equalizer->MaxBars-1; iBar >= 0; iBar--)
    {
        iLUTColor = (iBar<<LOG2_BARCOLORLUT_SIZE)/equalizer->MaxBars;

        Ytop = equalizer->YtopBars + iBar*(equalizer->BarHeight+(equalizer->BarHeight>>1));
        Ybottom = Ytop + equalizer->BarHeight;
        Ytop = Ytop >= YtopEnergy ? Ytop : YtopEnergy;
        if ( Ybottom > YtopEnergy )
        {
            state = DRAW4;
case DRAW4:
            GFX_ColorSet(_BarColorLUT[iLUTColor]);
            if ( GFX_RectangleFillDraw(Xstart,Ytop,Xend,Ybottom) == GFX_STATUS_FAILURE)
                return(GFX_STATUS_FAILURE);
        }

        }//end for(iBar = _MaxBars-1; iBar >= 0; iBar--)

    // Draw filter gain icon
    barPixelHeight = (equalizer->barPixelRange * _gainChannelBand[channel][iBand])>>15;
    Ygain = equalizer->YtopBars + equalizer->barPixelRange - barPixelHeight;
    state = DRAW5;
    case DRAW5:
    GFX_ColorSet( gainIconColor );
    if ( GFX_RectangleRoundDraw(Xstart + (equalizer->BarWidth>>1)-3, Ygain,
              Xstart + (equalizer->BarWidth>>1)+3, Ygain,
              (3*equalizer->BarHeight>>3) ) == GFX_STATUS_FAILURE)
        return(GFX_STATUS_FAILURE);

    }//end if ( bForceRedraw ||
     //         YtopEnergy != _YtopEnergyOld[channel][iBand] ||
     //        _gainChannelBandOld[channel][iBand] != _gainChannelBand[channel][iBand] )
    }
    state = DRAW1;
    return(GFX_STATUS_SUCCESS);
}//_BandRefresh

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerLabelsSet(char *pLeftLabel, char *pRightLabel, void *pLabelFont)

  Summary:
    Sets left/right channel label strings and the font to be used in writing the labels.
 */
void GFX_GOL_EqualizerLabelsSet(char *pLeftLabel, char *pRightLabel, void *pLabelFont)
{
    _pLeftLabel  = pLeftLabel;
    _pRightLabel = pRightLabel;
    _pLabelFont  = pLabelFont;

}//GRAPHIC_EQUALIZER_LabelsSet

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_VALUE_TYPE type,
                                           libq_q15d16_t minValue,
                                           libq_q15d16_t maxValue )

  Summary:
    Sets the display scaling for signal strength or filter gain, allowing the
    conversion of data value into pixels on the display.
 */
void GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_VALUE_TYPE type,
                                       libq_q15d16_t minValue,
                                       libq_q15d16_t maxValue )
{
    if( type == GFX_EQUAL_SIGNAL_STRENGTH )
    {
        _SignalMinValue = minValue;
        _SignalMaxValue = maxValue;
    }
    else if ( type == GFX_EQUAL_FILTER_GAIN ) //GFX_GAIN
    {
        _GainMinValue   = minValue;
        _GainMaxValue   = maxValue;
    }//end if(type == GFX_EQUAL_SIGNAL_STRENGTH)
}

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerCreate(int16_t left, int16_t top, int16_t bWidth, int16_t bHeight, uint8_t nBands, uint8_t nBars)

  Summary:
    Creates a Graphic Equalizer display on the application's LCD screen.
 */

GFX_STATUS GFX_GOL_EqualizerDraw(void *pObject)
{
    static int16_t Xleft, Xright, Ytop, Ybottom, Xstart, Xend;
    static bool enable;
    static int8_t iBand;
    static GFX_EQUAL_CHANNEL channel;
    static EQUALIZER_DRAW_STATES state = DRAW1;
    GFX_GOL_EQUALIZER *equalizer;

    equalizer = (GFX_GOL_EQUALIZER *) pObject;

    switch(state)
    {
        case DRAW1:
    if ( ((void*)_pLeftLabel == NULL) && ((void*)_pRightLabel == NULL) )
    {
        equalizer->YtopBars = equalizer->hdr.top;
    }
    else
    {
        equalizer->YtopBars = equalizer->hdr.top + GFX_TextStringHeightGet(_pLabelFont) + (equalizer->BarHeight>>1);
    }

    equalizer->Xleft1stBar = equalizer->hdr.left + (equalizer->BarWidth>>1);

    // Pixel range of signal energy bar, with nBars of "LED"s and
    // 1/2 of a bar between each bar in both X and Y directions
    equalizer->barPixelRange = equalizer->BarHeight*equalizer->MaxBars + ((equalizer->BarHeight*(equalizer->MaxBars-1))>>1);

    GFX_ColorSet(EQ_BACKGROUND_COLOR);

    // Background for Left Channel
    Xleft   = equalizer->hdr.left;
    Ytop    = equalizer->YtopBars - (equalizer->BarHeight>>1); // back up 1/2 bar
    Xright  = Xleft + equalizer->BarWidth*equalizer->NumBands + ((equalizer->BarWidth*(equalizer->NumBands+1))>>1);
    Ybottom = Ytop  + equalizer->BarHeight*(equalizer->MaxBars+1) + ((equalizer->BarHeight*(equalizer->MaxBars+1))>>1) + 1;
    equalizer->hdr.bottom = Ybottom;
    
    if(GFX_RectangleFillDraw(Xleft,Ytop,Xright,Ybottom)== GFX_STATUS_FAILURE)
    {
        return(GFX_STATUS_FAILURE);
    }
    state = DRAW2;

    case DRAW2:
    equalizer->width = Xright - Xleft; //Width of the equalizer display object

    // Background for Right Channel
    Xleft   = Xright + (equalizer->BarWidth>>1); // 1/2 bar between channel backgrounds
    Xright  = Xleft + equalizer->BarWidth*equalizer->NumBands + ((equalizer->BarWidth*(equalizer->NumBands+1))>>1);
    equalizer->hdr.right = Xright;

    if(GFX_RectangleFillDraw(Xleft,Ytop,Xright,Ybottom) == GFX_STATUS_FAILURE)
    {
        return(GFX_STATUS_FAILURE);
    }

    if ( _pLeftLabel != NULL )
    {
        state = DRAW3;
    
    case DRAW3:
        // Label backgrounds
        GFX_ColorSet(BLACK);
        GFX_FontSet(_pLabelFont);
        if(GFX_TextStringDraw(equalizer->hdr.left, equalizer->hdr.top, _pLeftLabel, strlen(_pLeftLabel))== GFX_STATUS_FAILURE)
        {
            return(GFX_STATUS_FAILURE);
        }
    }
    if ( _pRightLabel != NULL )
    {
         state = DRAW4;
    
     case DRAW4:
        // Label backgrounds
        GFX_ColorSet(BLACK);
        GFX_FontSet(_pLabelFont);
        if(GFX_TextStringDraw(equalizer->hdr.left + (equalizer->BarWidth>>1) + equalizer->width, equalizer->hdr.top, _pRightLabel, strlen(_pRightLabel))
                == GFX_STATUS_FAILURE)
        {
            return(GFX_STATUS_FAILURE);
        }
    }
        state = DRAW5;

        case DRAW5:
            if(GFX_GOL_ObjectStateGet(equalizer, GFX_GOL_EQUALIZER_REMOVE_FOCUS_STATE) ||
                    GFX_GOL_ObjectStateGet(equalizer, GFX_GOL_EQUALIZER_DRAW_FOCUS_STATE))
            {  
                if(GFX_GOL_ObjectStateGet(equalizer, GFX_GOL_EQUALIZER_REMOVE_FOCUS_STATE))
                {
                    enable = false;
                }    
                else
                {
                    enable = true;
                }    
                
                channel = equalizer->channel;
                iBand = equalizer->iBand;
                state = DRAW6;
                return(GFX_STATUS_FAILURE);
            }

            if(GFX_GOL_ObjectStateGet(equalizer, GFX_GOL_EQUALIZER_UPDATE_STATE))
            {
                channel = equalizer->channel;
                iBand = equalizer->iBand;

             if ( channel <= GFX_EQUAL_MAX_CHANNELS &&
         iBand   < equalizer->NumBands                    )
    {
        if ( iBand < 0 ) // Refresh all bands
        {
            for ( iBand = 0; iBand < equalizer->NumBands ; iBand++ )
            {
                // Has anything changed??
                if (  equalizer->bForceRedraw
                     || _energyChannelBandOld[channel][iBand] != _energyChannelBand[channel][iBand]
                     || _gainChannelBandOld[channel][iBand] != _gainChannelBand[channel][iBand]  )
                {
                    state = DRAW10;
case DRAW10:
                    if(_BandRefresh(equalizer, channel,iBand) == GFX_STATUS_FAILURE)
                        return(GFX_STATUS_FAILURE);
                }
            }
        }
        else // Refresh display for just one band, as defined by iBand
        {
            // Has anything changed??
            if (  equalizer->bForceRedraw
                 || _energyChannelBandOld[channel][iBand] != _energyChannelBand[channel][iBand]
                 || _gainChannelBandOld[channel][iBand] != _gainChannelBand[channel][iBand]  )
            {
                state = DRAW11;
case DRAW11:
                if(_BandRefresh(equalizer, channel,iBand) == GFX_STATUS_FAILURE)
                    return(GFX_STATUS_FAILURE);
            }
        }
             }
             GFX_GOL_ObjectStateClear(equalizer, GFX_GOL_EQUALIZER_UPDATE_STATE);
            }
            state = DRAW5;
            break;
    case DRAW6:

         if ( channel <= GFX_EQUAL_MAX_CHANNELS &&
         iBand   < equalizer->NumBands                    )
    {
        _bFocusOn = enable;
        if ( _bFocusOn )
        {
            equalizer->iFocusChannel = channel;
            equalizer->iFocusBand    = iBand;
        }

        // Draw rectangle around band that is in "focus"
        Xstart = equalizer->Xleft1stBar + channel*(equalizer->width + (equalizer->BarWidth>>1));
        Xstart += iBand*(equalizer->BarWidth + (equalizer->BarWidth>>1)) - 1;
        Xend = Xstart + equalizer->BarWidth + 2;
        state = DRAW7;
case DRAW7:
        if(enable == true)
        {
            GFX_ColorSet(FOCUSCOLOR);
        }
        else
        {
            GFX_ColorSet(EQ_BACKGROUND_COLOR);
        }
        if(GFX_RectangleFillDraw(Xstart,equalizer->YtopBars - 1,
            Xend,  equalizer->YtopBars + equalizer->BarHeight*equalizer->MaxBars + ((equalizer->BarHeight*equalizer->MaxBars)>>1) + 4)== GFX_STATUS_FAILURE)
            return(GFX_STATUS_FAILURE);
        state = DRAW8;
case DRAW8:
        // Draw Band Focus Prompt
        if(enable == true)
        {
            GFX_ColorSet(GAINPROMPTCOLOR);
        }

        if(GFX_RectangleFillDraw(Xstart, equalizer->hdr.bottom - equalizer->BarHeight + 1,
            Xend  , equalizer->hdr.bottom -  1 )== GFX_STATUS_FAILURE)
            return(GFX_STATUS_FAILURE);

        // If defocus, force refresh of bar
        if(enable != true)
        {
            state = DRAW9;
case DRAW9:
            if(_BandRefresh(equalizer,channel,iBand) == GFX_STATUS_FAILURE)
                return(GFX_STATUS_FAILURE);
        }
    }
    GFX_GOL_ObjectStateClear(equalizer, GFX_GOL_EQUALIZER_REMOVE_FOCUS_STATE|GFX_GOL_EQUALIZER_DRAW_FOCUS_STATE);
    state = DRAW5;
    default:
        break;
    }
    return(GFX_STATUS_SUCCESS);
}//GFX_GOL_EqualizerCreate

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerCreate(int16_t left, int16_t top, int16_t bWidth, int16_t bHeight, uint8_t nBands, uint8_t nBars)

  Summary:
    Configures a Graphic Equalizer display.
 */
GFX_GOL_EQUALIZER *GFX_GOL_EqualizerCreate(uint16_t ID, int16_t left, int16_t top,
                                 int16_t bWidth, int16_t bHeight,
                                 uint8_t nBands, uint8_t nBars, uint16_t state)
{

    GFX_GOL_EQUALIZER *equalizer = NULL;

    equalizer = (GFX_GOL_EQUALIZER *) GFX_malloc(sizeof (GFX_GOL_EQUALIZER));

    if (equalizer == NULL)
    {
        return (NULL);
    }
    
    equalizer->hdr.ID = ID; // unique id assigned for referencing
    equalizer->hdr.pNxtObj = NULL; // initialize pointer to NULL
    equalizer->hdr.type = GFX_GOL_EQUALIZER_TYPE; // set object type  equalizer->hdr.left = left;
    equalizer->hdr.top  = top;

    if ( ((void*)_pLeftLabel == NULL) && ((void*)_pRightLabel == NULL) )
    {
        equalizer->YtopBars = equalizer->hdr.top;
    }
    else
    {
        equalizer->YtopBars = equalizer->hdr.top + GFX_TextStringHeightGet(_pLabelFont) + (equalizer->BarHeight>>1);
    }

    equalizer->BarWidth  = bWidth;
    equalizer->Xleft1stBar = equalizer->hdr.left + (equalizer->BarWidth>>1);
    equalizer->BarHeight = bHeight;

    equalizer->MaxBars = nBars;
    equalizer->NumBands = nBands;

    equalizer->hdr.DrawObj = GFX_GOL_EqualizerDraw; // draw function
    equalizer->hdr.actionGet = NULL; // message function
    equalizer->hdr.actionSet = NULL; // default message function
    equalizer->hdr.FreeObj = NULL; // free function

    // Pixel range of signal energy bar, with nBars of "LED"s and
    // 1/2 of a bar between each bar in both X and Y directions
    equalizer->barPixelRange = equalizer->BarHeight*equalizer->MaxBars + ((equalizer->BarHeight*(equalizer->MaxBars-1))>>1);

    GFX_GOL_ObjectAdd((GFX_GOL_OBJ_HEADER *) equalizer);

    return(equalizer);
}

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_VALUE_TYPE type, GFX_EQUAL_CHANNEL channel, uint8_t iBand, libq_q15_t amplitude);

  Summary:
    Update channel/band display on the graphic equalizer with new signal energy or filter gain
 */
void GFX_GOL_EqualizerBandValueUpdate(GFX_GOL_EQUALIZER *equalizer, GFX_EQUAL_VALUE_TYPE type, GFX_EQUAL_CHANNEL channel, uint8_t iBand, libq_q15d16_t bandValue)
{
    int16_t bandValueScaled;

    if ( channel < GFX_EQUAL_MAX_CHANNELS &&
         iBand   < equalizer->NumBands                    )
    {
        if      ( type == GFX_EQUAL_SIGNAL_STRENGTH ) // Signal Strength
        {
            _energyChannelBandOld[channel][iBand] =  _energyChannelBand[channel][iBand];
            bandValueScaled = GFX_GOL_EqualizerScale(_SignalMinValue,     0,
                                                      _SignalMaxValue, 32767,
                                                      bandValue             );
            bandValueScaled = ( bandValueScaled < 0 ) ? 0 : bandValueScaled;

            _energyChannelBand[channel][iBand] = bandValueScaled;
        }
        else if ( type == GFX_EQUAL_FILTER_GAIN )    //Filter Gain
        {
            _gainChannelBandOld[channel][iBand] = _gainChannelBand[channel][iBand];
            bandValueScaled = GFX_GOL_EqualizerScale(_GainMinValue,     0,
                                                      _GainMaxValue, 32767,
                                                      bandValue           );
            bandValueScaled = ( bandValueScaled < 0 ) ? 0 : bandValueScaled;
            _gainChannelBand[channel][iBand] = bandValueScaled;
        }//end if(type == GFX_EQUAL_SIGNAL_STRENGTH)
    }
}//GFX_GOL_EqualizerBandValueUpdate

void GFX_GOL_EqualizerBandForceRedrawStateSet(GFX_GOL_EQUALIZER *equalizer, bool bForceRedraw)
{
    equalizer->bForceRedraw = bForceRedraw;
}

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerBandDisplayRefresh(GFX_EQUAL_CHANNEL channel, int8_t iBand)

  Summary:
    Refreshes display for given channel and band
 */
void GFX_GOL_EqualizerBandDisplayRefresh(GFX_GOL_EQUALIZER *equalizer, GFX_EQUAL_CHANNEL channel, int8_t iBand)
{
    equalizer->channel = channel;
    equalizer->iBand = iBand;
    GFX_GOL_ObjectStateSet(equalizer, GFX_GOL_EQUALIZER_UPDATE_STATE|GFX_GOL_EQUALIZER_DRAW_STATE);
} //GFX_GOL_EqualizerBandDisplayRefresh

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerChannelFocus(bool enable, GFX_EQUAL_CHANNEL channel, uint8_t iBand)

  Summary:
    "Focus" graphic equalizer on specified channel/Band.
 */
void GFX_GOL_EqualizerChannelFocus(GFX_GOL_EQUALIZER *equalizer, bool enable, GFX_EQUAL_CHANNEL channel, uint8_t iBand)
{
    equalizer->channel = channel;
    equalizer->iBand = iBand;

    if(enable == true)
    {
        GFX_GOL_ObjectStateSet(equalizer, GFX_GOL_EQUALIZER_DRAW_FOCUS_STATE|GFX_GOL_EQUALIZER_DRAW_STATE);
    }
    else
    {
       GFX_GOL_ObjectStateSet(equalizer, GFX_GOL_EQUALIZER_REMOVE_FOCUS_STATE|GFX_GOL_EQUALIZER_DRAW_STATE);
    }

}//GFX_GOL_EqualizerChannelFocus

// *****************************************************************************
/* Function:
    libq_q0d15_t GFX_GOL_EqualizerScale(libq_q15d16_t Xmin, libq_q0d15_t Ymin,
                                         libq_q15d16_t Xmax, libq_q0d15_t Ymax,
                                         libq_0d15_t XtoScale );

  Summary:
    Apply linear scaling of XtoScale, outputing Q0.15 value for display by
    GRAPHIC_EQUALIZER_BandValueUpdate as band signal energy or band filter gain.

    // Try out scaling primitive
    libq_q0d15_t Youtput;

    // Update filter gain to +5 dB, (-20 dB, 0) to (+20 dB, +1) provides the conversion curve
    Youtput = GFX_GOL_EqualizerScale( -20<<16,    0,  // -20 dB in Q16.15 is -20<<16
                                       +20<<16,32767,  // +20 dB in Q16.15 is +20<<16
                                         5<<16      ); //  +5 dB in Q16.15 is   5<<16
    //Answer: 20479

    // Update filter gain to -5 dB, (-20 dB, 0) to (+20 dB, +1) provides the conversion curve
    Youtput = GFX_GOL_EqualizerScale( -20<<16,    0,  // -20 dB in Q16.15 is -20<<16
                                       +20<<16,32767,  // +20 dB in Q16.15 is +20<<16
                                        -5<<16      ); //  +5 dB in Q16.15 is   5<<16
    // Answer: 12287

    Youtput = GFX_GOL_EqualizerScale( -0.75*32768,     0,
                                       +0.75*32768,32767,
                                        0 );
    // Answer: 16383

    Youtput = GFX_GOL_EqualizerScale( -0.75*32768,    0,
                                       +0.75*32768,32767,
                                        0.25*32768 );
    // Answer: 21844

    Youtput = GFX_GOL_EqualizerScale( -0.75*32768,    0,
                                       +0.75*32768,32767,
                                       -0.25*32768 );
    // Answer: 10922

*/

int16_t GFX_GOL_EqualizerScale(libq_q15d16_t Xmin, libq_q0d15_t Ymin,
                                libq_q15d16_t Xmax, libq_q0d15_t Ymax,
                                libq_q15d16_t XtoScale )
{
    int32_t deltaX;
    int16_t deltaY;
    int16_t scaledYout;

    XtoScale = XtoScale < Xmin ? Xmin : XtoScale;
    XtoScale = Xmax < XtoScale ? Xmax : XtoScale;

    deltaX = Xmax - Xmin;
    deltaY = Ymax - Ymin;

    scaledYout = (deltaY*(int64_t)(XtoScale-Xmin))/deltaX + Ymin;

    return scaledYout;
}

/*******************************************************************************
 End of File
*/
