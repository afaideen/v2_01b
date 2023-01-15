/*******************************************************************************
  Graphic Equalizer Interface Header File

  Company:
    Microchip Technology Inc.

  File Name:
    graphic_equalizer.h

  Summary:
    Supports graphic equalizer display.

  Description:
    Supports graphic equalizer display, with primitives that display
    frequency band signal energy and filter gain.  It is used in conjunction
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

#ifndef _GRAPHIC_EQUALIZER_H    // Guards against multiple inclusion
#define _GRAPHIC_EQUALIZER_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "gfx/gfx_gol.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/*
    <GROUP gol_types>

    Typedef:
        GFX_GOL_EQUALIZER_STATE

    Summary:
        Specifies the different states of the Equalizer object.

    Description:
        This enumeration specifies the different states of the Equalizer
        object used in the library.

        For the Property State bits, more than one of these values may be
        OR'd together to create a complete property state.

        For the Draw State bits, hide draw bit has higher priority than the
        draw bit. Any of these two can be combined with other draw bits to
        create a complete draw state.

        To test a value of any of the state types, the bit of interest must be
        AND'ed with value and checked to see if the result is non-zero.

    Remarks:
        None.

*/
// *****************************************************************************
typedef enum
{
    // Property bit for focus state.
    GFX_GOL_EQUALIZER_FOCUSED_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0001 /*DOM-IGNORE-END*/,
    GFX_GOL_EQUALIZER_DRAW_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x4000 /*DOM-IGNORE-END*/,
    // Property bit to indicate object will have a toggle behavior.
    GFX_GOL_EQUALIZER_REMOVE_FOCUS_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0002 /*DOM-IGNORE-END*/,
    // Property bit for press state.
    GFX_GOL_EQUALIZER_DRAW_FOCUS_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0004 /*DOM-IGNORE-END*/,
    GFX_GOL_EQUALIZER_UPDATE_STATE
        /*DOM-IGNORE-BEGIN*/  = 0x0008 /*DOM-IGNORE-END*/,
    // Draw bit to indicate object must be removed from screen.
    GFX_GOL_EQUALIZER_HIDE_STATE  
        /*DOM-IGNORE-BEGIN*/  = 0x8000 /*DOM-IGNORE-END*/,
} GFX_GOL_EQUALIZER_STATE;

// *****************************************************************************
/* Graphic Equalizer Use Single Bar

  Summary:
    Defines use single bar for band energy instead of stack of "LED"s

  Description:
    Defines use single bar for band energy instead of stack of "LED"s

  Remarks:
    None.
*/
#define GFX_USE_SINGLE_BAR

// DOM-IGNORE-BEGIN
#undef GFX_USE_SINGLE_BAR
// DOM-IGNORE-END


// *****************************************************************************
/* Graphic Equalizer Use "New" Colors

  Summary:
    Defines use of new display colors.

  Description:
    Defines use of new display colors.

  Remarks:
    None.
*/
#define GFX_USE_NEW_COLORS

// DOM-IGNORE-BEGIN
#undef  GFX_USE_NEW_COLORS
// DOM-IGNORE-END

// *****************************************************************************
/* Graphic Equalizer Maximum Number of Bands

  Summary:
    Defines the maximum number of bands supported.

  Description:
    Defines the maximum number of bands supported.

  Remarks:
    None.
*/
#define GFX_EQUAL_MAX_BANDS 8


// *****************************************************************************
/* Graphic Equalizer Background Color

  Summary:
    Defines the Red/Green/Blue background color for the equalizer display.

  Description:
    Defines the Red/Green/Blue background color for the equalizer display.

  Remarks:
    None.
*/
#define EQ_BACKGROUND_COLOR /* DOM-IGNORE-BEGIN */ EQ_BACKGROUND_COLOR_SET
#if defined( GFX_USE_NEW_COLORS )
    #define EQ_BACKGROUND_COLOR_SET GFX_RGBConvert(159,159,159) // Background is medium gray
#else
    #define EQ_BACKGROUND_COLOR_SET GFX_RGBConvert(  0,  0,  0) // Background is black
#endif
/* DOM-IGNORE-END */


// *****************************************************************************
/* Graphic Equalizer Filter Gain Prompt Color

  Summary:
    Defines the Red/Green/Blue color for the filter gain prompt on the display.

  Description:
    Defines the Red/Green/Blue color for the filter gain prompt on the display.

  Remarks:
    None.
*/
#define GAINPROMPTCOLOR /* DOM-IGNORE-BEGIN */ GAINPROMPTCOLOR_SET
#if defined( GFX_USE_NEW_COLORS )
    #define GAINPROMPTCOLOR_SET      GFX_RGBConvert(207,207,207) // Gain prompt same as focus to hide it
#else
    #define GAINPROMPTCOLOR_SET      GFX_RGBConvert(255,  0,255) // Gain prompt color is magenta
#endif
/* DOM-IGNORE-END */


// *****************************************************************************
/* Graphic Equalizer Filter Gain Icon Color

  Summary:
    Defines the Red/Green/Blue color for filter gain icons on the display.

  Description:
    Defines the Red/Green/Blue color for filter gain icons on the display.

  Remarks:
    None.
*/
#define GAINCOLOR /* DOM-IGNORE-BEGIN */ GAINCOLOR_SET
#if defined( GFX_USE_NEW_COLORS )
    #define GAINCOLOR_SET      GFX_RGBConvert(  0,  0,  0) // Gain icon color is black
#else
    #define GAINCOLOR_SET      GFX_RGBConvert(255,  0,255) // Gain icon color is magenta
#endif
/* DOM-IGNORE-END */


// *****************************************************************************
/* Graphic Equalizer Filter Gain Focused Icon Color

  Summary:
    Defines the Red/Green/Blue color for focused filter gain icons on the display.

  Description:
    Defines the Red/Green/Blue color for focused filter gain icons on the display.

  Remarks:
    Gain icons change color from GAINCOLOR to GAINFOCUSCOLOR when the channel/band
    is subject to filter gain adjustments (channel/band is "focused").
*/
#define GAINFOCUSCOLOR /* DOM-IGNORE-BEGIN */ GAINFOCUSCOLOR_SET
#if defined( GFX_USE_NEW_COLORS )
    #define GAINFOCUSCOLOR_SET GFX_RGBConvert(255,  0,255) // Gain icon color is magenta when focused.
#else
    #define GAINFOCUSCOLOR_SET GFX_RGBConvert(255,  0,255) // Gain icon color is magenta when focused.
#endif
/* DOM-IGNORE-END */


// *****************************************************************************
/* Graphic Equalizer Focused Channel/Band Background Color

  Summary:
    Defines the Red/Green/Blue color for the focused channel/band.

  Description:
    Defines the Red/Green/Blue color for the focused channel/band.

  Remarks:
    The channel/band background changes from EQ_BACKGROUND_COLOR to FOCUSCOLOR when the channel/band
    is subject to filter gain adjustments (channel/band is "focused").
*/
#define FOCUSCOLOR FOCUSCOLOR_SET /* DOM-IGNORE-BEGIN */
#if defined( GFX_USE_NEW_COLORS )
    #define FOCUSCOLOR_SET     GFX_RGBConvert(207,207,207) // Medium Gray
#else
    #define FOCUSCOLOR_SET     GFX_RGBConvert(  0,  0,  0) // Black
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Graphic Equalizer Maximum Number of Bands

  Summary:
    Defines the maximum number of bands supported.

  Description:
    Defines the maximum number of bands supported.

  Remarks:
    None.
*/


#define GFX_EQUAL_MAX_BANDS 8
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Graphic Equalizer Channel Enumeration

  Summary:
    Supports audio channels for stereo music.

  Description:
    Supports audio channels for stereo music.

  Remarks:
    None.
*/
typedef enum {
    GFX_EQUAL_CHANNEL_LEFT,  // Left stereo channel
    GFX_EQUAL_CHANNEL_RIGHT, // Right stereo channel
    GFX_EQUAL_CHANNEL_CENTER, // Center (sub-woofer channel, not yet implemented)
    GFX_EQUAL_MAX_CHANNELS
} GFX_EQUAL_CHANNEL;

// *****************************************************************************
/*
    <GROUP gol_types>

    Typedef:
        GFX_GOL_EQUALIZER

    Summary:
        Defines the structure used for the Button object.

    Description:
        Defines the structure used for the Button object.
        1. Width is determined by right - left parameter in
           GFX_GOL_OBJ_HEADER.
        2. Height is determined by top - bottom parameter in
           GFX_GOL_OBJ_HEADER.
        3. radius - specifies if the GFX_GOL_EQUALIZER will have a rounded
                    edge. If zero then the GFX_GOL_EQUALIZER will have
                    sharp (cornered) edge.
        4. If 2*radius = height = width, the GFX_GOL_EQUALIZER is a
           circular GFX_GOL_EQUALIZER.

    Remarks:
        None.

*/
// *****************************************************************************
typedef struct
{
    GFX_GOL_OBJ_HEADER  hdr;            // Generic header for all Objects (see GFX_GOL_OBJ_HEADER).
    int16_t YtopBars;
    uint8_t BarWidth;
    int16_t Xleft1stBar;
    uint8_t BarHeight;
    uint8_t MaxBars;
    uint8_t NumBands;
    int16_t barPixelRange;
    int16_t width;
    GFX_EQUAL_CHANNEL channel;
    int8_t iBand;
    GFX_EQUAL_CHANNEL iFocusChannel;
    uint8_t iFocusBand;
    bool bForceRedraw;
} GFX_GOL_EQUALIZER;

// *****************************************************************************
/* Graphic Equalizer Channel Value Type Enumeration

  Summary:
    Each graphic equalizer band can show signal strength (energy) and filter
    gain values. This enumeration distinguishes between the two in APIs.

  Description:
    Each graphic equalizer band can show signal strength (energy) and filter
    gain values. This enumeration distinguishes between the two in APIs.

  Remarks:
    None.
*/
typedef enum {
     GFX_EQUAL_SIGNAL_STRENGTH,  // Signal strength/energy/amplitude
     GFX_EQUAL_FILTER_GAIN,      // Band filter gain

} GFX_EQUAL_VALUE_TYPE;


// *****************************************************************************
/* Graphic Equalizer Axis Tick Direction Enumeration

  Summary:
    Specifies direction of axis tick marks from vertical signal energy or filter
    gain axis.

  Description:
    Specifies direction of axis tick marks from vertical signal energy or filter
    gain axis.
  Remarks:
    None.
*/
typedef enum {
    GFX_EQUAL_TICKS_LEFT,  // Tick marks point to left  of vertical axis
    GFX_EQUAL_TICKS_RIGHT, // Tick marks point to right of vertical axis

} GFX_EQUAL_TICKS_DIR;


// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************
// NONE

// *****************************************************************************
// *****************************************************************************
// Section: Graphic Equalizer Primitives
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerLabelsSet(char *pLeftLabel, char *pRightLabel, void *pLabelFont)

  Summary:
    Sets left/right channel label strings and the font to be used in writing the labels.

  Description:
    Sets left/right channel label strings and the font to be used in writing the labels.
    To disable labels pass null pointers as arguments for the text labels.

  Precondition:
    None.

  Parameters:
    pLeftLabel  - Character string defining left  channel label.
    pRightLabel - Character string defining right channel label.
    pLabelFont  - Pointer to label font.

  Returns:
    None.

  Example:
  <code>
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)&Prototype_16);
    GFX_GOL_EqualizerCreate(10, 10, //Left,Top
                              8,  4, //BarWidth,BarHeight
                              8, 16);//NumFreqBands,NumBars
    .
    .
    .
    // Disable labels, rest of display stays the same.
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet(NULL,NULL,(void*)&Prototype_16);
    GFX_GOL_EqualizerCreate(10, 10+GetTextHeight((void*)&Prototype_16), //Left,Top
                              8,  4, //BarWidth,BarHeight
                              8, 16);//NumFreqBands,NumBars

  </code>

  Remarks:
    None.
*/
void GFX_GOL_EqualizerLabelsSet(char *pLeftLabel, char *pRightLabel, void *pLabelFont);

void GFX_GOL_EqualizerBandForceRedrawStateSet(GFX_GOL_EQUALIZER *equalizer, bool bForceRedraw);
// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_VALUE_TYPE type,
                                           libq_q15d16_t minValue,
                                           libq_q15d16_t maxValue )

  Summary:
    Sets the display scaling for signal strength or filter gain, allowing the
    conversion of data value into pixels on the display.

  Description:
    Sets the display scaling for signal strength or filter gain, allowing the
    conversion of data value into pixels on the display.  The data range of
    [minValue,maxValue] is translated to a pixel range, where minValue
    corresponds to the bottom of the display maxValue corresponds to the top.

  Precondition:
    None.

  Parameters:
    type  - GFX_EQUAL_SIGNAL_STRENGTH (for signal energy/amplitude) or GFX_EQUAL_FILTER_GAIN (for filter gain)
    minValue - minimum value, corresponding to the bottom of the display
    maxValue - maximum value, corresponding to the top of the display

  Returns:
    None.

  Example:
  <code>
    // Signal strength will be measured in RMS Volts, from 0 to 1
    // +1 as Q16.16 is 1<<16
    GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_SIGNAL_STRENGTH,0,1<<16);

    // Filter gains will range from -10 dB to +10 dB
    // +/-10 as Q15.16 is +/-10<<16
    GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_FILTER_GAIN,-10<<16,10<<16);
  </code>

  Remarks:
    The default scaling for signal strength is -30 dBFs to 0 dBFS.
    The default scaling for filter gain is -20 dB to 20 dB.
    These scalings will be used unless new scaling values are set by calling
    this function.
*/

void GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_VALUE_TYPE type,
                                           libq_q15d16_t minValue,
                                           libq_q15d16_t maxValue );

// *****************************************************************************
/* Function:
    GFX_GOL_EQUALIZER GFX_GOL_EqualizerCreate(int16_t left, int16_t top, int16_t bWidth, int16_t bHeight, uint8_t nBands, uint8_t nBars)

  Summary:
    Creates a Graphic Equalizer display on the application's LCD screen.

  Description:
    Creates a Graphic Equalizer display on the application's LCD screen.

  Precondition:
    InitGraph has been called to initialize the graphics library.
    GRAPHIC_EQUALIZER_LabelsSet has been called to define labels.

  Parameters:
    left - left-most (X) pixel coordinate of the equalizer object
    top  - top-most (Y) pixel coordinate of the equalizer object
    bWidth - equalizer channel bar width, in pixels
    bHeight - equalizer bar height, in pixels
    nBands - number of bands (columns)
    nBars - number of bars (rows) in "LED" display of band signal energy

  Returns:
    None.

  Example:
  <code>
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)&Prototype_16);
    GFX_GOL_EqualizerCreate(10, 10, //Left,Top
                              8,  4, //BarWidth,BarHeight
                              8, 16);//NumFreqBands,NumBars
  </code>
  Or simply
  <code>
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)&Prototype_16);
    GFX_GOL_EqualizerCreate(10, 10, //Left,Top
                              8,  4, //BarWidth,BarHeight
                              8, 16);//NumFreqBands,NumBars
  </code>

  Remarks:
    Default colors are defined in the file graphic_equalizer_config.h:
*/
GFX_GOL_EQUALIZER *GFX_GOL_EqualizerCreate(uint16_t ID, int16_t left,   int16_t top,
                              int16_t bWidth, int16_t bHeight,
                              uint8_t nBands,  uint8_t nBars, uint16_t state );


// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerConfigure(int16_t left, int16_t top, int16_t bWidth, int16_t bHeight, uint8_t nBands, uint8_t nBars)

  Summary:
    Configures a Graphic Equalizer display.

  Description:
    Configures a Graphic Equalizer display.  This routine and GFX_GOL_EqualizerDraw
    do the same job as GFX_GOL_EqualizerCreate.

  Precondition:
    InitGraph has been called to initialize the graphics library.
    GRAPHIC_EQUALIZER_LabelsSet has been called to define labels.

  Parameters:
    left - left-most (X) pixel coordinate of the equalizer object
    top  - top-most (Y) pixel coordinate of the equalizer object
    bWidth - equalizer channel bar width, in pixels
    bHeight - equalizer bar height, in pixels
    nBands - number of bands (columns)
    nBars - number of bars (rows) in "LED" display of band signal energy

  Returns:
    None.

  Example:
  <code>
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)&Prototype_16);
    GFX_GOL_EqualizerConfigure(10, 10, //Left,Top
                                 8,  4, //BarWidth,BarHeight
                                 8, 16);//NumFreqBands,NumBars
    GFX_GOL_EqualizerDraw();
  </code>
  Or simply
  <code>
    #include "../../Graphics/Fonts/prototype/Prototype16pt.h"
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)&Prototype_16);
    GFX_GOL_EqualizerCreate(10, 10, //Left,Top
                              8,  4, //BarWidth,BarHeight
                              8, 16);//NumFreqBands,NumBars
  </code>

  Remarks:
    Default colors are defined in the file graphic_equalizer_config.h:
*/
GFX_STATUS GFX_GOL_EqualizerDraw(void *pObject);

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_VALUE_TYPE type, GFX_EQUAL_CHANNEL channel, uint8_t iBand, libq_q15d16_t bandValue)

  Summary:
    Update left/right channel and frequency band with new signal energy or filter gain

  Description:
    Update left/right channel and frequency band with new signal energy or filter gain

  Precondition:
    GRAPHIC_EQUALIZER_Create has been called

  Parameters:
    type    - GFX_EQUAL_SIGNAL_STRENGTH (for signal energy/amplitude) or GFX_EQUAL_FILTER_GAIN (for filter gain)
    channel - GFX_EQUAL_CHANNEL_LEFT or GFX_EQUAL_CHANNEL_RIGHT
    iBand  - channel band to be updated, 0 <= iBand < nBands used in GRAPHIC_EQUALIZER_Create
    bandValue - Signal strength or filter gain

  Returns:
    None.

  Example:
  <code>
    #include "audio_equalizer.h"
    #include "audio_equalizer_fixedpoint.h"

    #include "myStereoFilters6x2_Q15.h"
    #define  NBANDS          6
    #define  NFILTERS          2

    uint16_t iBand;
    libq_q0d15_t  XinLeft,XinRight,YoutBand, YoutLeft, YoutRight;
    libq_q15d16_t bandEnergyDBFS;

    // Define labels
    GFX_GOL_EqualizerLabelsSet("Left","Right",(void*)LARGE_FONT);

    // Signal strength will be measured in RMS Volts in dB re Full scale, -30 dBFS to 0 dBFS
    GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_SIGNAL_STRENGTH,-30<<16,0<<16);

    // Filter gains will range from -10 dB to +10 dB
    GFX_GOL_EqualizerDisplayScaleSet(GFX_EQUAL_FILTER_GAIN,-10<<16,10<<16);

    // Draw graphic equalizer display
    GFX_GOL_EqualizerCreate( 8, 6, //Xleft,Ytop
                              8, 4, //BarWidth, BarHeight
                              8,16);//nBands, nBars

    // Setup to measure signal energy in dB re Full Scale (dBFS)
    AUDIO_EQUALIZER_BandEnergySumsInit(2*NBANDS,BAND_ENERGY_RMS_DBFS);

    while ( 1 )
    {
        if ( bGotInput() )
        {
            // Get XinLeft,XinRight

            // Execute equalizer filtering and signal strength updates.
            // See Audio Equalizer Filtering Library for example code to implement this.
            FilterInputPlusEnergy(XinLeft,XinRight,&YoutLeft,&YoutRight);
            // Send YoutRight, YoutRight
        }//end if ( bGotInput )

        if ( bUpdateDisplay() )
        {
            // Update Display
            for ( iBand = 0; iBand < NBANDS; iBand++ )
            {
                // Left channel
                bandEnergyDBFS = AUDIO_EQUALIZER_BandEnergyGetQ15(iBand,true);
                GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_SIGNAL_STRENGTH,GFX_EQUAL_CHANNEL_LEFT,iBand,bandEnergyDBFS);

                // Right Channel
                bandEnergyDBFS = AUDIO_EQUALIZER_BandEnergyGetQ15(iBand+NBANDS,true);
                GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_SIGNAL_STRENGTH,GFX_EQUAL_CHANNEL_RIGHT,iBand,bandEnergyDBFS);

            }//end for ( iBand = 0; iBand < NBANDS; iBand++ )
            GFX_GOL_EqualizerBandDisplayRefresh(GFX_EQUAL_CHANNEL_LEFT, -1); // Refresh display for left  channel, all bands
            GFX_GOL_EqualizerBandDisplayRefresh(GFX_EQUAL_CHANNEL_RIGHT,-1); // Refresh display for right channel, all bands

        }//end if ( bUpdateDisplay )

     }//end while ( 1 )
  </code>

  Remarks:
    Call GFX_GOL_EqualizerBandDisplayRefresh to update the display.

    To transform signal energy or filter gain into Q0.15 values see GRAPHIC_EQUALIZER_Scale.
*/

void GFX_GOL_EqualizerBandValueUpdate(GFX_GOL_EQUALIZER *equalizer, GFX_EQUAL_VALUE_TYPE type, GFX_EQUAL_CHANNEL channel, uint8_t iBand, libq_q15d16_t bandValue);

// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerBandDisplayRefresh(GFX_EQUAL_CHANNEL channel, int8_t iBand, bool bForceRedraw)

  Summary:
    Refreshes the display for given channel and band

  Description:
    Refreshes the display for given channel and band.  Redraws the signal strength
    bars and the band filter gain icon when these values have changed, otherwise
    no display changes are made.

  Precondition:
    GRAPHIC_EQUALIZER_Create has been called

  Parameters:
    channel - GFX_EQUAL_CHANNEL_LEFT or GFX_EQUAL_CHANNEL_RIGHT
    iBand  - channel band to be refreshed on the display, 0 <= iBand < nBands used in GRAPHIC_EQUALIZER_Create.
             If iBand < 0 then all frequency bands for the channel are refreshed.
    bForceRedraw - redraw channel/band even if no data has changed

  Returns:
    None.

  Example:
    See GFX_GOL_EqualizerBandValueUpdate.

  Remarks:
    None.
*/
void GFX_GOL_EqualizerBandDisplayRefresh(GFX_GOL_EQUALIZER *equalizer, GFX_EQUAL_CHANNEL channel, int8_t iBand);


// *****************************************************************************
/* Function:
    void GFX_GOL_EqualizerChannelFocus(bool enable, GFX_EQUAL_CHANNEL channel, uint8_t iBand)

  Summary:
    "Focus" graphic equalizer on specified channel/iBand.

  Description:
    "Focus" graphic equalizer on specified channel/iBand.  Next get user commands
    to update filter gain, then call GRAPHIC_EQUALIZER_BandValueUpdate(GFX_EQUAL_FILTER_GAIN,...) to update
    commanded gain displayed on screen.  Finally update corresponding filter gain
    to change filter output.

  Precondition:
    GFX_GOL_EqualizerCreate has been called

  Parameters:
    enable - true to enable channel focus, false to disable it
    channel - GFX_EQUAL_CHANNEL_LEFT or GFX_EQUAL_CHANNEL_RIGHT
    iBand  - band that will have gain adjustments, 0 <= iBand < nBands used in GRAPHIC_EQUALIZER_Create

  Returns:
    None.

  Example:
  <code>
    #include "audio_equalizer.h"
    #include "audio_equalizer_fixedpoint.h"

    #include "myStereoFilters6x2_Q15.h"
    #define  NBANDS          6
    #define  NFILTERS          2

    EQUALIZER_FILTER *pMyFilters;
    GFX_EQUAL_CHANNEL myChannel;
    uint8_t           myBand;
    int16_t           iGainAdj;
    libq_q0d15_t      displayGainAdjQ15;

    while ( !GainAdjDone )
    {
        switch ( gainAdjStateGet() );
        {
            case GET_CHANNEL:
                myChannel = GetUserChannel();
                if ( GFX_EQUAL_CHANNEL_LEFT == myChannel )
                {
                    pMyFilters = myFiltersLeft;
                }
                else
                {
                    pMyFilters = myFiltersRight;
                }
                break;

            case GET_BAND:
                myBand = GetUserBand();
                GFX_GOL_EqualizerChannelFocus(true,myChannel,myBand);
                bandGain = // Base all gain adjustments on this old value
                    AUDIO_EQUALIZER_FilterGainGetQ15(pMyFilters, NBANDS, NFILTERS,
                                                                 myBand, NFILTERS );
                break;

            case APPLY_GAIN_ADJ:
                iGainAdj = GetUserGainAdj(); // -50 dB <= iGainAdj <= +50 dB
                displayGainAdjQ15 = 327*iGainAdj + 16384;
                GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_FILTER_GAIN,channel,myBand,displayGainAdjQ15);

                adjBandGain = AUDIO_EQUALIZER_FilterGainAdjustQ15(bandGain,iGainAdj);
                AUDIO_EQUALIZER_FilterGainSetQ15(pMyFilters, NBANDS, NFILTERS,
                                                             myBand, NFILTERS,
                                                             adjBandGain );

                GFX_GOL_EqualizerBandDisplayRefresh(myChannel,myBand);
                break;

            case GAIN_ADJUST_DONE:
                GFX_GOL_EqualizerChannelFocus(false,myChannel,myBand);
                GainAdjDone = true;
                break;

        }//end switch ( gainAdjStateGet() )

    }//end while ( !GainAdjustDone )
  </code>

  Remarks:
    None.
*/
void GFX_GOL_EqualizerChannelFocus(GFX_GOL_EQUALIZER *equalizer, bool enable, GFX_EQUAL_CHANNEL channel, uint8_t iBand);

/*DOM-IGNORE-BEGIN*/

// *****************************************************************************
/* Function:
    libq_q0d15_t GFX_GOL_EqualizerScale(libq_q15d16_t Xmin, libq_q0d15_t Ymin,
                                         libq_q15d16_t Xmax, libq_q0d15_t Ymax,
                                         libq_016d15_t XtoScale );

  Summary:
    Apply linear scaling of XtoScale, outputting Q0.15 value for display by
    GFX_GOL_EqualizerBandValueUpdate as band signal energy or band filter gain.

  Description:
    Apply linear scaling of XtoScale, outputting Q0.15 value for display by
    GFX_GOL_EqualizerBandValueUpdate as band signal energy or band filter gain.

  Precondition:

  Parameters:
    (Xmin,Ymin) - Left  end of linear transform from Xinput to Youtput
    (Xmax,Ymax) - Right end of linear transform from Xinput to Youtput
    XtoScale    - X value to scale from Q16.15 to Q0.15

  Returns:
    Youtput - scaled Q0.15 value for display.

  Example:
  <code>
    libq_q0d15_t Youtput;

    // Update filter gain to +5 dB, (-20 dB, 0) to (+20 dB, +1) provides the conversion curve
    Youtput = GFX_GOL_EqualizerScale( -20<<16,    0,  // -20 dB in Q16.15 is -20<<16
                                       +20<<16,32767,  // +20 dB in Q16.15 is +20<<16
                                         5<<16      ); //  +5 dB in Q16.15 is   5<<16
    // Youtput = 20479/32768, Desired = 20480/32768 = 0.625
    GFX_GOL_EqualizerBandValueUpdate(GFX_EQUAL_FILTER_GAIN,GFX_EQUAL_CHANNEL_LEFT,iBand,Youtput);

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
  </code>

  Remarks:
    None.
*/

int16_t GFX_GOL_EqualizerScale(libq_q15d16_t Xmin, libq_q0d15_t Ymin,
                                libq_q15d16_t Xmax, libq_q0d15_t Ymax,
                                libq_q15d16_t XtoScale );

/*DOM-IGNORE-END*/

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif // _GRAPHIC_EQUALIZER_H

/*******************************************************************************
 End of File
*/

