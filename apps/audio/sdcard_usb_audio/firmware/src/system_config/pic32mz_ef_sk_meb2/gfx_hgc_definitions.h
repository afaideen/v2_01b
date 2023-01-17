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
    HGC_SCREEN_STATE_SETUP_SCREEN_MchpName,
    HGC_SCREEN_STATE_PRE_DRAW_GOL_SCREEN_MchpName,
    HGC_SCREEN_STATE_DRAW_GOL_SCREEN_MchpName,
    HGC_SCREEN_STATE_POST_DRAW_GOL_SCREEN_MchpName,
    HGC_SCREEN_STATE_PRE_DRAW_PRIMITIVE_SCREEN_MchpName,
    HGC_SCREEN_STATE_DRAW_PRIMITIVE_SCREEN_MchpName,
    HGC_SCREEN_STATE_POST_DRAW_PRIMITIVE_SCREEN_MchpName,
    HGC_SCREEN_STATE_DISPLAY_SCREEN_MchpName,
	
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

#define GFX_SCHEME_DISABLED_COLOR_RED_0         0xC0
#define GFX_SCHEME_DISABLED_COLOR_GREEN_0       0xC0
#define GFX_SCHEME_DISABLED_COLOR_BLUE_0        0xC0

#define GFX_SCHEME_BACKGROUND_COLOR_RED_0       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_0     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_0      0x00

#define GFX_SCHEME_EMBOSS_SIZE_0                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_0     0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_0   0xC0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_0    0xC0

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_0      0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_0    0x40
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_0     0x40

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_0     0x00
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_0   0x00
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_0    0x00

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

		

#define GFX_SCHEME_NAME_1                       "GroupScheme"
#define GFX_SCHEME_ALPHA_1                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_1      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_1     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_1          0x00
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_1        0x00
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_1         0x00

#define GFX_SCHEME_SECONDARY_COLOR_RED_1        0x00
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_1      0x00
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_1       0x70

#define GFX_SCHEME_DISABLED_COLOR_RED_1         0xC0
#define GFX_SCHEME_DISABLED_COLOR_GREEN_1       0xC0
#define GFX_SCHEME_DISABLED_COLOR_BLUE_1        0xC0

#define GFX_SCHEME_BACKGROUND_COLOR_RED_1       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_1     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_1      0x00

#define GFX_SCHEME_EMBOSS_SIZE_1                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_1     0xA0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_1   0x14
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_1    0x10

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_1      0xE8
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_1    0x00
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_1     0x00

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_1     0x18
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_1   0xE0
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_1    0xF0

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

		

#define GFX_SCHEME_NAME_2                       "RadioScheme"
#define GFX_SCHEME_ALPHA_2                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_2      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_2     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_2          0x00
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_2        0x00
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_2         0x00

#define GFX_SCHEME_SECONDARY_COLOR_RED_2        0x00
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_2      0x00
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_2       0x70

#define GFX_SCHEME_DISABLED_COLOR_RED_2         0xC0
#define GFX_SCHEME_DISABLED_COLOR_GREEN_2       0xC0
#define GFX_SCHEME_DISABLED_COLOR_BLUE_2        0xC0

#define GFX_SCHEME_BACKGROUND_COLOR_RED_2       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_2     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_2      0x00

#define GFX_SCHEME_EMBOSS_SIZE_2                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_2     0xF0
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_2   0xF4
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_2    0xF0

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_2      0xF0
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_2    0xF4
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_2     0xF0

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_2     0xE8
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_2   0x90
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_2    0xD8

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_2   0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_2 0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_2  0xFF

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_2    0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_2  0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_2   0x40

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_2    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_2  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_2   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_2    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_2  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_2   0xFF

#define GFX_SCHEME_FILL_STYLE_2                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_2           GFX_BACKGROUND_NONE

		

#define GFX_SCHEME_NAME_3                       "TrackListBox"
#define GFX_SCHEME_ALPHA_3                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_3      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_3     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_3          0x00
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_3        0x00
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_3         0x00

#define GFX_SCHEME_SECONDARY_COLOR_RED_3        0xC8
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_3      0x6C
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_3       0x78

#define GFX_SCHEME_DISABLED_COLOR_RED_3         0x28
#define GFX_SCHEME_DISABLED_COLOR_GREEN_3       0x24
#define GFX_SCHEME_DISABLED_COLOR_BLUE_3        0x20

#define GFX_SCHEME_BACKGROUND_COLOR_RED_3       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_3     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_3      0x00

#define GFX_SCHEME_EMBOSS_SIZE_3                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_3     0xA8
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_3   0x18
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_3    0x18

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_3      0xE8
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_3    0x00
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_3     0x00

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_3     0xB0
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_3   0xF4
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_3    0x90

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_3   0x90
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_3 0xF4
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_3  0xF0

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_3    0x08
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_3  0x10
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_3   0x08

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_3    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_3  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_3   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_3    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_3  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_3   0xFF

#define GFX_SCHEME_FILL_STYLE_3                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_3           GFX_BACKGROUND_NONE

		

#define GFX_SCHEME_NAME_4                       "ScrollUpScheme"
#define GFX_SCHEME_ALPHA_4                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_4      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_4     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_4          0x00
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_4        0x00
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_4         0x00

#define GFX_SCHEME_SECONDARY_COLOR_RED_4        0xC8
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_4      0x6C
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_4       0x78

#define GFX_SCHEME_DISABLED_COLOR_RED_4         0x28
#define GFX_SCHEME_DISABLED_COLOR_GREEN_4       0x24
#define GFX_SCHEME_DISABLED_COLOR_BLUE_4        0x20

#define GFX_SCHEME_BACKGROUND_COLOR_RED_4       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_4     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_4      0x00

#define GFX_SCHEME_EMBOSS_SIZE_4                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_4     0xA8
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_4   0x18
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_4    0x18

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_4      0xE8
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_4    0x00
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_4     0x00

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_4     0xB0
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_4   0xF4
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_4    0x90

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_4   0x90
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_4 0xF4
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_4  0xF0

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_4    0x08
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_4  0x10
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_4   0x08

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_4    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_4  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_4   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_4    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_4  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_4   0xFF

#define GFX_SCHEME_FILL_STYLE_4                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_4           GFX_BACKGROUND_NONE

		

#define GFX_SCHEME_NAME_5                       "VolumeButton"
#define GFX_SCHEME_ALPHA_5                      100

#define GFX_SCHEME_BACKGROUND_OFFSET_TOP_5      0
#define GFX_SCHEME_BACKGROUND_OFFSET_LEFT_5     0

#define GFX_SCHEME_PRIMARY_COLOR_RED_5          0x00
#define GFX_SCHEME_PRIMARY_COLOR_GREEN_5        0x00
#define GFX_SCHEME_PRIMARY_COLOR_BLUE_5         0x00

#define GFX_SCHEME_SECONDARY_COLOR_RED_5        0x00
#define GFX_SCHEME_SECONDARY_COLOR_GREEN_5      0x00
#define GFX_SCHEME_SECONDARY_COLOR_BLUE_5       0x00

#define GFX_SCHEME_DISABLED_COLOR_RED_5         0x00
#define GFX_SCHEME_DISABLED_COLOR_GREEN_5       0x00
#define GFX_SCHEME_DISABLED_COLOR_BLUE_5        0x00

#define GFX_SCHEME_BACKGROUND_COLOR_RED_5       0x00
#define GFX_SCHEME_BACKGROUND_COLOR_GREEN_5     0x00
#define GFX_SCHEME_BACKGROUND_COLOR_BLUE_5      0x00

#define GFX_SCHEME_EMBOSS_SIZE_5                3

#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_RED_5     0x00
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_GREEN_5   0x00
#define GFX_SCHEME_EMBOSS_LIGHT_COLOR_BLUE_5    0x00

#define GFX_SCHEME_EMBOSS_DARK_COLOR_RED_5      0x00
#define GFX_SCHEME_EMBOSS_DARK_COLOR_GREEN_5    0x00
#define GFX_SCHEME_EMBOSS_DARK_COLOR_BLUE_5     0x00

#define GFX_SCHEME_TEXT_PRIMARY_COLOR_RED_5     0x00
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_GREEN_5   0x00
#define GFX_SCHEME_TEXT_PRIMARY_COLOR_BLUE_5    0x00

#define GFX_SCHEME_TEXT_SECONDARY_COLOR_RED_5   0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_GREEN_5 0xFF
#define GFX_SCHEME_TEXT_SECONDARY_COLOR_BLUE_5  0xFF

#define GFX_SCHEME_TEXT_DISABLED_COLOR_RED_5    0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_GREEN_5  0x40
#define GFX_SCHEME_TEXT_DISABLED_COLOR_BLUE_5   0x40

#define GFX_SCHEME_GRADIENT_START_COLOR_RED_5    0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_GREEN_5  0x00
#define GFX_SCHEME_GRADIENT_START_COLOR_BLUE_5   0x00

#define GFX_SCHEME_GRADIENT_END_COLOR_RED_5    0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_GREEN_5  0xFF
#define GFX_SCHEME_GRADIENT_END_COLOR_BLUE_5   0xFF

#define GFX_SCHEME_FILL_STYLE_5                GFX_FILL_STYLE_COLOR
#define GFX_SCHEME_BACKGROUND_TYPE_5           GFX_BACKGROUND_COLOR

		
/*** Screen IDs ***/
#define MchpName                 0 
	
/*** GFX GOL Button Definitions ***/
#define GFX_BUTTON_LEFT_0            401
#define GFX_BUTTON_TOP_0             209
#define GFX_BUTTON_RIGHT_0           460
#define GFX_BUTTON_BOTTOM_0          247
#define GFX_BUTTON_TEXT_0            "0x0"
#define GFX_BUTTON_ALIGNMENT_0       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_BUTTON_RADIUS_0          0
#define GFX_BUTTON_STATE_0           GFX_GOL_BUTTON_DRAW_STATE | GFX_GOL_BUTTON_PRESSED_STATE | GFX_GOL_BUTTON_TOGGLE_STATE
#define GFX_BUTTON_PRESS_IMAGE_0     (GFX_RESOURCE_HDR*)&AudioPlay16_2
#define GFX_BUTTON_RELEASE_IMAGE_0   (GFX_RESOURCE_HDR*)&AudioMute16_2
#define GFX_BUTTON_SCHEME_0          gfxScheme5
		
#define GFX_BUTTON_LEFT_1            324
#define GFX_BUTTON_TOP_1             150
#define GFX_BUTTON_RIGHT_1           362
#define GFX_BUTTON_BOTTOM_1          231
#define GFX_BUTTON_TEXT_1            "0x76, 0x0"
#define GFX_BUTTON_ALIGNMENT_1       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_BUTTON_RADIUS_1          0
#define GFX_BUTTON_STATE_1           GFX_GOL_BUTTON_DRAW_STATE
#define GFX_BUTTON_PRESS_IMAGE_1     NULL
#define GFX_BUTTON_RELEASE_IMAGE_1   NULL
#define GFX_BUTTON_SCHEME_1          gfxScheme3
		
#define GFX_BUTTON_LEFT_2            324
#define GFX_BUTTON_TOP_2             69
#define GFX_BUTTON_RIGHT_2           362
#define GFX_BUTTON_BOTTOM_2          150
#define GFX_BUTTON_TEXT_2            "0x20, 0x0"
#define GFX_BUTTON_ALIGNMENT_2       GFX_ALIGN_HCENTER | GFX_ALIGN_VCENTER
#define GFX_BUTTON_RADIUS_2          0
#define GFX_BUTTON_STATE_2           GFX_GOL_BUTTON_DRAW_STATE
#define GFX_BUTTON_PRESS_IMAGE_2     NULL
#define GFX_BUTTON_RELEASE_IMAGE_2   NULL
#define GFX_BUTTON_SCHEME_2          gfxScheme4
		
/*** Button IDs ***/
#define VolButton                  1000 
	
#define ScrollDownButton                  1001 
	
#define ScrollUpButton                  1002 
	
/*** GFX GOL Radio Button Definitions ***/

#define GFX_RADIOBUTTON_LEFT_0            39
#define GFX_RADIOBUTTON_TOP_0             167
#define GFX_RADIOBUTTON_RIGHT_0           145
#define GFX_RADIOBUTTON_BOTTOM_0          202
#define GFX_RADIOBUTTON_TEXT_0            "0x55, 0x53, 0x42, 0x0"
#define GFX_RADIOBUTTON_STATE_0    		  GFX_GOL_RADIOBUTTON_DRAW_STATE
#define GFX_RADIOBUTTON_ALIGNMENT_0       GFX_ALIGN_LEFT | GFX_ALIGN_VCENTER
#define GFX_RADIOBUTTON_SCHEME_0          gfxScheme2
#define GFX_RADIOBUTTON_GROUPNO_0          0
#define GFX_RADIOBUTTON_NUM_GROUPS_ON_SCREEN_0   1
		

#define GFX_RADIOBUTTON_LEFT_1            39
#define GFX_RADIOBUTTON_TOP_1             105
#define GFX_RADIOBUTTON_RIGHT_1           147
#define GFX_RADIOBUTTON_BOTTOM_1          140
#define GFX_RADIOBUTTON_TEXT_1            "0x53, 0x44, 0x20, 0x43, 0x61, 0x72, 0x64, 0x0"
#define GFX_RADIOBUTTON_STATE_1    		  GFX_GOL_RADIOBUTTON_DRAW_STATE | GFX_GOL_RADIOBUTTON_CHECKED_STATE | GFX_GOL_RADIOBUTTON_DRAW_CHECK_STATE | GFX_GOL_RADIOBUTTON_GROUP_STATE
#define GFX_RADIOBUTTON_ALIGNMENT_1       GFX_ALIGN_LEFT | GFX_ALIGN_VCENTER
#define GFX_RADIOBUTTON_SCHEME_1          gfxScheme2
#define GFX_RADIOBUTTON_GROUPNO_1          0
#define GFX_RADIOBUTTON_NUM_GROUPS_ON_SCREEN_1   1
		
/*** Radio Button IDs ***/
#define RadioButtonUSB                  4000 
	
#define RadioButtonSd                  4001 
	
/*** GFX GOL Scrollbar Definitions ***/
#define GFX_SCROLLBAR_LEFT_0            	412
#define GFX_SCROLLBAR_TOP_0             	69
#define GFX_SCROLLBAR_RIGHT_0           	448
#define GFX_SCROLLBAR_BOTTOM_0          	207
#define GFX_SCROLLBAR_STATE_0            GFX_GOL_SCROLLBAR_DRAW_STATE | GFX_GOL_SCROLLBAR_DRAW_THUMB_STATE | GFX_GOL_SCROLLBAR_SLIDER_MODE_STATE | GFX_GOL_SCROLLBAR_VERTICAL_STATE
#define GFX_SCROLLBAR_RANGE_0            50
#define GFX_SCROLLBAR_PAGE_0             1
#define GFX_SCROLLBAR_POS_0              0
#define GFX_SCROLLBAR_SCHEME_0           gfxScheme3
		
/*** Scrollbar IDs ***/
#define VolSlider                  5000 
	
/*** GFX GOL List box Definitions ***/
#define GFX_LISTBOX_LEFT_0            173
#define GFX_LISTBOX_TOP_0             69
#define GFX_LISTBOX_RIGHT_0           323
#define GFX_LISTBOX_BOTTOM_0          231
#define GFX_LISTBOX_STATE_0           GFX_GOL_LISTBOX_DRAW_STATE
#define GFX_LISTBOX_SCHEME_0          gfxScheme3
#define GFX_LISTBOX_ALIGNMENT_0       GFX_ALIGN_LEFT
#define GFX_LISTBOX_ITEM_COUNT_0      0

		
/*** List Box IDs ***/
#define TrackListBox                  7000 
/*** GFX GOL Group box Definitions ***/
#define GFX_GROUPBOX_LEFT_0            33
#define GFX_GROUPBOX_TOP_0             62
#define GFX_GROUPBOX_RIGHT_0           153
#define GFX_GROUPBOX_BOTTOM_0          232
#define GFX_GROUPBOX_STATE_0           GFX_GOL_GROUPBOX_DRAW_STATE
#define GFX_GROUPBOX_SCHEME_0          gfxScheme1
#define GFX_GROUPBOX_ALIGNMENT_0       GFX_ALIGN_HCENTER

		
/*** Group Box IDs ***/
#define GroupBox1                  8000 
/*** Rectangle IDs ***/
#define Rectangle1                 16000 
	
/*** Image IDs ***/
#define Image1                 18000 
	
/*** Text IDs ***/
#define DemoName                 19000 
	



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

    /*** Touch Event Callback ***/
    SYS_MSG_RECEIVE_CALLBACK msgCallBackFunction;

    /*** GOL Objects References ***/
    GFX_GOL_BUTTON* pVolButtonObj;
    GFX_GOL_BUTTON* pScrollDownButtonObj;
    GFX_GOL_BUTTON* pScrollUpButtonObj;
    GFX_GOL_RADIOBUTTON* pRadioButtonUSBObj;
    GFX_GOL_RADIOBUTTON* pRadioButtonSdObj;
    GFX_GOL_SCROLLBAR* pVolSliderObj;
    GFX_GOL_LISTITEM* pListBoxItem;
    GFX_GOL_LISTBOX* pTrackListBoxObj;
    GFX_GOL_GROUPBOX* pGroupBox1Obj;
} HGC_OBJECTS;

extern HGC_OBJECTS hgcObj;

/******************************************************************************
  Function:
    void GFX_HGC_RegisterAppTouchCallback ( SYS_MSG_RECEIVE_CALLBACK msgCallBackFunction )

  Summary:
    This function allows the application to register a callback to allow access 
    to the current touch behavior
 */
void GFX_HGC_RegisterAppTouchCallback ( SYS_MSG_RECEIVE_CALLBACK msgCallBackFunction );


/******************************************************************************
  Function:
    void GFX_HGC_TouchMessageCallback ( SYS_MSG_OBJECT *pMsg )

  Summary:
    This function sets up the Message System Service to route touch callbacks
    to the GOL message system  
 */
void GFX_HGC_TouchMessageCallback ( SYS_MSG_OBJECT *pMsg );


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
    bool GFX_HGC_MsgRadioButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Radio Button events
 */
bool GFX_HGC_MsgRadioButtons(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgScrollBars(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL Scroll Bar events
 */
bool GFX_HGC_MsgScrollBars(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);

/******************************************************************************
  Function: 
    bool GFX_HGC_MsgListBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg)

  Output:
    Returns true if a Object ID matches an existing item identifier

  Remarks: 
    Handles GFX GOL List Boxes events
 */
bool GFX_HGC_MsgListBoxes(uint16_t objMsg, GFX_GOL_OBJ_HEADER *pObj, GFX_GOL_MESSAGE *pMsg);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _GFX_HGC_DEFINITIONS_H
/*******************************************************************************
 End of File
*/
