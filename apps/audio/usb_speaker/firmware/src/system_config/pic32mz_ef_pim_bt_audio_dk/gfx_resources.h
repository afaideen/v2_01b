/*****************************************************************************
  AUTO-GENERATED CODE:  Graphics Resource Converter version: 4.00.39 BETA

  Company:
      Microchip Technology, Inc.
 
  File Name:
     gfx_resources.h
 
  Summary:
      This file is generated by the Microchip's Graphics Resource Converter.
  Description:
      This file is generated by the Graphics Resource Converter containing
      resources such as images and fonts that can be used by Microchip's
      Graphics Library, which is part of the MLA.
 *****************************************************************************/

// DOM-IGNORE-BEGIN
/*****************************************************************************
  Software License Agreement

  Copyright(c) 2013 Microchip Technology Inc.  All rights reserved.
  Microchip licenses to you the right to use, modify, copy and distribute
  Software only when embedded on a Microchip microcontroller or digital
  signal controller that is integrated into your product or third party
  product (pursuant to the sublicense terms in the accompanying license
  agreement).
 
  You should refer to the license agreement accompanying this Software
  for additional information regarding your rights and obligations.
 
  SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY
  KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
  OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
  PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
  OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
  BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
  DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
  INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
  COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
  CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
  OR OTHER SIMILAR COSTS.
 *****************************************************************************/
// DOM-IGNORE-END

#ifndef GFX_RESOURCES_H_FILE
#define GFX_RESOURCES_H_FILE
/*****************************************************************************
 * SECTION:  Includes
 *****************************************************************************/
#include <gfx/gfx.h>
#include <stdint.h>

/*****************************************************************************
 * SECTION:  Graphics Library Firmware Check
 *****************************************************************************/
#if(GRAPHICS_LIBRARY_VERSION != 0x0400)
#warning "It is suggested to use Graphics Library version 4.00 with this version of GRC."
#endif

/*****************************************************************************
 * SECTION:  Graphics Bitmap Padding Check
 *****************************************************************************/
#ifdef GFX_CONFIG_IMAGE_PADDING_DISABLE
#error These bitmap resources have been converted with padding of the horizontal lines, but GFX_CONFIG_IMAGE_PADDING_DISABLE is defined in system_config.h.
#endif


/*****************************************************************************
 * SECTION: Graphics Configuration
 * Overview:    This defines the size of the buffer used by font functions
 *              to retrieve font data from the external memory. The buffer
 *              size can be increased to accommodate large font sizes.
 *              The user must be aware of the expected glyph sizes of the 
 *              characters stored in the font table. 
 *              To modify the size used, declare this macro in the 
 *              GraphicsConfig.h file with the desired size.
 *
 * #define GFX_EXTERNAL_FONT_RASTER_BUFFER_SIZE     (63)
 *****************************************************************************/

/*****************************************************************************
 * This is an error check for the color depth
 *****************************************************************************/
#if (COLOR_DEPTH > 16)
#error "Color Depth needs to be 16 to correctly use these resources"
#endif



/*****************************************************************************
 * SECTION:  BITMAPS
 *****************************************************************************/

/*********************************
 * Bitmap Structure
 * Label: MCHP_LOGO
 * Description:  122x30 pixels, 16-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR MCHP_LOGO;
#define MCHP_LOGO_WIDTH     (122)
#define MCHP_LOGO_HEIGHT    (30)
#define MCHP_LOGO_SIZE      (7326)
/*********************************
 * Bitmap Structure
 * Label: USB_DISCONNECTED
 * Description:  30x30 pixels, 16-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR USB_DISCONNECTED;
#define USB_DISCONNECTED_WIDTH     (30)
#define USB_DISCONNECTED_HEIGHT    (30)
#define USB_DISCONNECTED_SIZE      (1806)
/*********************************
 * Bitmap Structure
 * Label: USB_NOT_CONNECTED
 * Description:  30x30 pixels, 16-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR USB_NOT_CONNECTED;
#define USB_NOT_CONNECTED_WIDTH     (30)
#define USB_NOT_CONNECTED_HEIGHT    (30)
#define USB_NOT_CONNECTED_SIZE      (1806)
/*********************************
 * Bitmap Structure
 * Label: USB_CONNECTED
 * Description:  30x30 pixels, 16-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR USB_CONNECTED;
#define USB_CONNECTED_WIDTH     (30)
#define USB_CONNECTED_HEIGHT    (30)
#define USB_CONNECTED_SIZE      (1806)
/*********************************
 * Bitmap Structure
 * Label: AudioMute16_2
 * Description:  32x25 pixels, 4-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR AudioMute16_2;
#define AudioMute16_2_WIDTH     (32)
#define AudioMute16_2_HEIGHT    (25)
#define AudioMute16_2_SIZE      (438)
/*********************************
 * Bitmap Structure
 * Label: AudioPlay16_2
 * Description:  29x22 pixels, 4-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR AudioPlay16_2;
#define AudioPlay16_2_WIDTH     (29)
#define AudioPlay16_2_HEIGHT    (22)
#define AudioPlay16_2_SIZE      (368)
/*********************************
 * Bitmap Structure
 * Label: Frequency
 * Description:  16x21 pixels, 16-bits per pixel
 ***********************************/
extern const GFX_RESOURCE_HDR Frequency;
#define Frequency_WIDTH     (16)
#define Frequency_HEIGHT    (21)
#define Frequency_SIZE      (678)
/*****************************************************************************
 * SECTION:  Fonts
 *****************************************************************************/

/*********************************
 * TTF Font File Structure
 * Label: Arial14pt
 * Description:  Height: 17 pixels, 1 bit per pixel, Range: ' ' to ''
 ***********************************/
extern const GFX_RESOURCE_HDR Arial14pt;
#define Arial14pt_SIZE    (2483)
/*********************************
 * TTF Font File Structure
 * Label: Arial12pt
 * Description:  Height: 15 pixels, 1 bit per pixel, Range: ' ' to ''
 ***********************************/
extern const GFX_RESOURCE_HDR Arial12pt;
#define Arial12pt_SIZE    (2072)
/*********************************
 * TTF Font File Structure
 * Label: Arial13pt
 * Description:  Height: 15 pixels, 1 bit per pixel, Range: ' ' to ''
 ***********************************/
extern const GFX_RESOURCE_HDR Arial13pt;
#define Arial13pt_SIZE    (2207)
/*********************************
 * TTF Font File Structure
 * Label: Monotype_Corsiva
 * Description:  Height: 21 pixels, 1 bit per pixel, Range: ' ' to ''
 ***********************************/
extern const GFX_RESOURCE_HDR Monotype_Corsiva;
#define Monotype_Corsiva_SIZE    (4151)
#endif

