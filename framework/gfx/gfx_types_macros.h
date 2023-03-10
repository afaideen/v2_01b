/*******************************************************************************
 Module for Microchip Graphics Library 

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_types_macros.h

  Summary:
    Data types used in Microchip Graphics Library.

  Description:
    This module defines the data types used in the Microchip
    Graphics Library for use of Primitive and Driver Layers.
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

#ifndef _GFX_TYPES_MACROS_H
// DOM-IGNORE-BEGIN
    #define _GFX_TYPES_MACROS_H
// DOM-IGNORE-END

#include <stdint.h>
#include "driver/gfx/gfx_common.h"
#include "gfx/gfx_types_palette.h"
#include "gfx/gfx_types_font.h"
#include "gfx/gfx_types_image.h"
#include "gfx/gfx_types_resource.h"

// DOM-IGNORE-BEGIN

#ifdef __cplusplus
    extern "C" {
#endif
        
/*********************************************************************
 Overview: Data type that defines the coordinate data.
           This type is dependent on the compiler setting.
           - GFX_COORD is type uint16_t if XC16 compiler
           - GFX_COLOR is type uint32_t if XC32 compiler

*********************************************************************/

#if defined(__XC16__)
    typedef uint16_t GFX_COORD;
#else
    typedef uint16_t GFX_COORD;
#endif

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_LINE_STYLE

    Summary:
        Specifies the different line styles.

    Description:
        This enumeration specifies the available line styles
        used in the library.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    /*DOM-IGNORE-BEGIN*/
    GFX_LINE_TYPE_MASK          = 0x0007, // DO NOT document, mask for line type
    GFX_LINE_THICKNESS_MASK     = 0x0010, // DO NOT document, mask for thickness
    /*DOM-IGNORE-END*/
    
    // solid line, 1 pixel wide (default)
    GFX_LINE_STYLE_THIN_SOLID   /*DOM-IGNORE-BEGIN*/ = 0x0A00
                                /*DOM-IGNORE-END*/,  
    // dotted line, 1 pixel wide
    GFX_LINE_STYLE_THIN_DOTTED  /*DOM-IGNORE-BEGIN*/ = 0x0A01
                                /*DOM-IGNORE-END*/,  
    // dashed line, , 1 pixel wide
    GFX_LINE_STYLE_THIN_DASHED  /*DOM-IGNORE-BEGIN*/ = 0x0A04
                                /*DOM-IGNORE-END*/, 
    // solid line, 3 pixel wide                                
    GFX_LINE_STYLE_THICK_SOLID  /*DOM-IGNORE-BEGIN*/ = 0x0A10
                                /*DOM-IGNORE-END*/, 
    // dotted line, 3 pixel wide
    GFX_LINE_STYLE_THICK_DOTTED /*DOM-IGNORE-BEGIN*/ = 0x0A11
                                /*DOM-IGNORE-END*/, 
    // dashed line, 3 pixel wide
    GFX_LINE_STYLE_THICK_DASHED /*DOM-IGNORE-BEGIN*/ = 0x0A14
                                /*DOM-IGNORE-END*/, 
    /*DOM-IGNORE-BEGIN*/
    // anti-aliased line (future feature)
    GFX_LINE_STYLE_ANTI_ALIASED = 0x0A20, 
    /*DOM-IGNORE-END*/
} GFX_LINE_STYLE;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_FILL_STYLE

    Summary:
        Specifies the available fill styles.

    Description:
        This enumeration specifies the available fill styles
        used in the library.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // no fill
    GFX_FILL_STYLE_NONE   /*DOM-IGNORE-BEGIN*/ = 0x0B00
                          /*DOM-IGNORE-END*/,  
    // color fill
    GFX_FILL_STYLE_COLOR,                 
    // color fill with alpha blending
    GFX_FILL_STYLE_ALPHA_COLOR,           
    // gradient, vertical-down direction
    GFX_FILL_STYLE_GRADIENT_DOWN,         
    // gradient, vertical-up direction
    GFX_FILL_STYLE_GRADIENT_UP,           
    // gradient, horizontal-right direction
    GFX_FILL_STYLE_GRADIENT_RIGHT,        
    // gradient, horizontal-left direction
    GFX_FILL_STYLE_GRADIENT_LEFT,
    // gradient, vertical-up/down direction
    GFX_FILL_STYLE_GRADIENT_DOUBLE_VER, 
    // gradient, horizontal-left/right direction
    GFX_FILL_STYLE_GRADIENT_DOUBLE_HOR,   
} GFX_FILL_STYLE;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_FILL_STYLE

    Summary:
        Specifies the different background fill types.

    Description:
        Defines the types of background information.
        Knowing the background information allows easier refresh
        of an area on the display buffer.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // No background information set.
    GFX_BACKGROUND_NONE             /*DOM-IGNORE-BEGIN*/ = 0x0000
                                    /*DOM-IGNORE-END*/,      
    // Background is set to a color.
    GFX_BACKGROUND_COLOR            /*DOM-IGNORE-BEGIN*/ = 0x0001
                                    /*DOM-IGNORE-END*/, 
    // Background is set to an image.
    GFX_BACKGROUND_IMAGE            /*DOM-IGNORE-BEGIN*/ = 0x0002
                                    /*DOM-IGNORE-END*/,       
    // Background is set to the current
    // content of the display buffer.
    // This requires support of
    // GFX_PixelArrayGet().    
    GFX_BACKGROUND_DISPLAY_BUFFER   /*DOM-IGNORE-BEGIN*/ = 0x0003
                                    /*DOM-IGNORE-END*/, 
    
} GFX_BACKGROUND_TYPE;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_FEATURE_STATUS

    Summary:
        States of a feature that can be enabled/disabled at run time.
        
    Description:
        Defines the states of a Graphics Library feature that
        can be enabled/disabled at run time.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // The feature is paused.
    GFX_FEATURE_PAUSED     /*DOM-IGNORE-BEGIN*/ = 0x0002
                            /*DOM-IGNORE-END*/, 
    // The feature is enabled.
    GFX_FEATURE_ENABLED     /*DOM-IGNORE-BEGIN*/ = 0x0001
                            /*DOM-IGNORE-END*/, 
    // The feature is disabled.                        
    GFX_FEATURE_DISABLED    /*DOM-IGNORE-BEGIN*/ = 0x0000
                            /*DOM-IGNORE-END*/, 
} GFX_FEATURE_STATUS;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_ALIGNMENT

    Summary:
        Summary of the different text alignment supported in the library.
        
    Description:
        This lists all the different text alignment supported in the library.
        The text alignment is used in GFX_TextStringBoxDraw().

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // left aligned text
    GFX_ALIGN_LEFT      /*DOM-IGNORE-BEGIN*/ = 0x0C01
                        /*DOM-IGNORE-END*/,
    // horizontal center aligned text
    GFX_ALIGN_HCENTER   /*DOM-IGNORE-BEGIN*/ = 0x0C02
                        /*DOM-IGNORE-END*/,          
    // right aligned text
    GFX_ALIGN_RIGHT     /*DOM-IGNORE-BEGIN*/ = 0x0C04
                        /*DOM-IGNORE-END*/,
    // top aligned text
    GFX_ALIGN_TOP       /*DOM-IGNORE-BEGIN*/ = 0x0C10
                        /*DOM-IGNORE-END*/,
    // vertical center aligned text
    GFX_ALIGN_VCENTER   /*DOM-IGNORE-BEGIN*/ = 0x0C20
                        /*DOM-IGNORE-END*/,          
    // bottom aligned text
    GFX_ALIGN_BOTTOM    /*DOM-IGNORE-BEGIN*/ = 0x0C40
                        /*DOM-IGNORE-END*/,          
    // vertical & horizontal center aligned text
    GFX_ALIGN_CENTER    /*DOM-IGNORE-BEGIN*/ = 0x0C22
                        /*DOM-IGNORE-END*/,          
    
    /*DOM-IGNORE-BEGIN*/
    GFX_ALIGN_HORIZONTAL_MASK  = 0x0C07,   // horizontal mask, DO
                                           // NOT document this item
    GFX_ALIGN_VERTICAL_MASK    = 0x0C70,   // vertical mask, DO
                                           // NOT document this item
    /*DOM-IGNORE-END*/
            
} GFX_ALIGNMENT;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_FONT_ANTIALIAS_TYPE

    Summary:
        Summary of the transparency types in text anti-aliasing.
        
    Description:
        Transparency type when rendering characters with anti-aliasing.

    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // Mid colors are calculated only once while rendering each 
    // character. This is ideal for rendering text over a constant
    // background.
    GFX_FONT_ANTIALIAS_OPAQUE       /*DOM-IGNORE-BEGIN*/ = 0x0000
                                    /*DOM-IGNORE-END*/,
    // Mid colors are calculated for every necessary pixel. This
    // feature is useful when rendering text over an image or 
    // when the background is not one flat color.
    GFX_FONT_ANTIALIAS_TRANSLUCENT  /*DOM-IGNORE-BEGIN*/ = 0x0001
                                    /*DOM-IGNORE-END*/,    
    
} GFX_FONT_ANTIALIAS_TYPE;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
      GFX_TEXT_CURSOR_TYPE

    Summary:
      Enumeration defines types of text cursor supported.
        
    Description:
      This enumeration defines types of cursor can be drawn.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    /* Text cursor of type none */
    GFX_TEXT_CURSOR_TYPE_NONE       /*DOM-IGNORE-BEGIN*/ = 0, /*DOM-IGNORE-END*/
    
    /* Text cursor of type bar */
    GFX_TEXT_CURSOR_TYPE_BAR        /*DOM-IGNORE-BEGIN*/ = 1, /*DOM-IGNORE-END*/
            
    /* Text cursor of type block */
    GFX_TEXT_CURSOR_TYPE_BLOCK      /*DOM-IGNORE-BEGIN*/ = 2, /*DOM-IGNORE-END*/
            
    /* Text cursor of type underscore */
    GFX_TEXT_CURSOR_TYPE_UNDERSCORE /*DOM-IGNORE-BEGIN*/ = 3, /*DOM-IGNORE-END*/
            
    /* DOM-IGNORE-BEGIN*/ GFX_TEXT_CURSOR_TYPE_MAX, /* DOM-IGNORE-END */
            
} GFX_TEXT_CURSOR_TYPE;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_STATUS

    Summary:
        Rendering status.
        
    Description:
        When rendering into the frame buffer, it is sometimes necessary
        to return the status of the rendering. This is especially true
        when using hardware to render primitive shapes or when 
        a FIFO is used to render primitive shapes.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // Rendering is not yet performed, or has started but not yet finished.
    GFX_STATUS_FAILURE      /*DOM-IGNORE-BEGIN*/ = 0x0000
                            /*DOM-IGNORE-END*/, 
    // Rendering has been performed.
    GFX_STATUS_SUCCESS      /*DOM-IGNORE-BEGIN*/ = 0x0001
                            /*DOM-IGNORE-END*/, 
} GFX_STATUS;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_STATUS_BIT

    Summary:
        Additional rendering status.
        
    Description:
        The following rendering status types are available for a detailed
        description of the status of rendering.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef enum
{
    // Rendering failed. Depending on the driver used, 
    // hardware may or may not recover.
    GFX_STATUS_FAILURE_BIT  /*DOM-IGNORE-BEGIN*/ = 0x0000
                            /*DOM-IGNORE-END*/, 
    // Rendering is successful.
    GFX_STATUS_SUCCESS_BIT  /*DOM-IGNORE-BEGIN*/ = 0x0001
                            /*DOM-IGNORE-END*/,
    // Rendering resulted in an error and cannot recover.
    GFX_STATUS_ERROR_BIT    /*DOM-IGNORE-BEGIN*/ = 0x0002
                            /*DOM-IGNORE-END*/,
    // Rendering cannot proceed due to a common resource 
    // is busy.
    GFX_STATUS_BUSY_BIT     /*DOM-IGNORE-BEGIN*/ = 0x0004
                            /*DOM-IGNORE-END*/,
    // Rendering can proceed.
    GFX_STATUS_READY_BIT    /*DOM-IGNORE-BEGIN*/ = 0x0008
                            /*DOM-IGNORE-END*/, 
} GFX_STATUS_BIT;

// DOM-IGNORE-BEGIN
/*********************************************************************
 Overview: Internal macro that returns the graphics GFX_STATUS.

*********************************************************************/
#define BITS_2_GFX_STATUS(mask) (mask & GFX_STATUS_SUCCESS)

/*********************************************************************
* Overview: Bevel shapes can be rendered in full or half size. 
            The following types defines the full, top and bottom 
            halves.
*********************************************************************/
typedef enum
{
    GFX_DRAW_FULLBEVEL          = 0xFF,   // render full bevel shape.
    GFX_DRAW_TOPBEVEL           = 0xF0,   // render top bevel shape.
    GFX_DRAW_BOTTOMBEVEL        = 0x0F    // render bottom half bevel shape.
} GFX_BEVEL_RENDER_TYPE;

// DOM-IGNORE-END

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_BACKGROUND

    Summary:
        Background information structure.
        
    Description:
        Structure describing the background information. Useful in refreshing 
        the whole screen or an area of the screen. The background information, 
        when used, can be one of the enumerated types in GFX_BACKGROUND_TYPE.
        - pBackGroundImage - pointer to a GFX_RESOURCE_HEADER with
                             the type GFX_RESOURCE_TYPE_MCHP_MBITMAP.
        - backGroundColor - value of the background color used when
                            pBackgroundImage is NULL.
        The pBackGroundImage has the high priority and will assume an image 
        is used as a background when the pointer is initialized. If this is 
        NULL the backgroundColor is assumed to be the background.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    uint16_t            left;     // Horizontal starting position of the background.
    uint16_t            top;      // Horizontal starting position of the background.
    GFX_BACKGROUND_TYPE type;     // Specifies the type of background to use.
    GFX_RESOURCE_HDR    *pImage;  // Pointer to the background image used. Set this
                                  // to NULL if not used.
    GFX_COLOR           color;    // If the background image is NULL, background is
                                  // assumed to be this color.

} GFX_BACKGROUND;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_PARTIAL_IMAGE_PARAM

    Summary:
        Partial Image information structure.
        
    Description:
        Structure describing the partial image area to be rendered.
       
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    uint16_t   width;       // Partial Image width.
    uint16_t   height;      // Partial Image height.
    uint16_t   xoffset;     // xoffset of the partial image
                            // (with respect to the image horizontal origin).
    uint16_t   yoffset;     // yoffset of the partial image 
                            // (with respect to the image vertical origin).
} GFX_PARTIAL_IMAGE_PARAM;

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_RECTANGULAR_AREA

    Summary:
        A generic rectangular area structure.
        
    Description:
        Structure describing a generic rectangular area defined by the left,top 
        and right,bottom points on the frame buffer.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    uint16_t left;          // left most pixel of the area.
    uint16_t top;           // top most pixel of the area.
    uint16_t right;         // right most pixel of the area.
    uint16_t bottom;        // bottom most pixel of the area.
} GFX_RECTANGULAR_AREA;

// DOM-IGNORE-BEGIN
/*********************************************************************
* Overview: Defines the maximum number of invalidated areas when
            double buffering is enabled.
*********************************************************************/
#ifndef GFX_MAX_INVALIDATE_AREAS
#define GFX_MAX_INVALIDATE_AREAS 5
#endif
// DOM-IGNORE-END

// *****************************************************************************
/* 
    <GROUP primitive_types>

    Typedef:
        GFX_RECTANGULAR_AREA

    Summary:
        Structure used for double buffering management.
        
    Description:
        Structure describing the double buffering states of the frame and 
        draw buffer. This saves the current states of the feature for 
        management by the graphics library. Double buffering is only available
        to selected configurations.
        
    Remarks:
        None.
        
*/
// *****************************************************************************
typedef struct
{
    // The status of the double buffering feature.
    GFX_FEATURE_STATUS      gfxDoubleBufferFeature;
    // The status of the double buffering feature.
    GFX_FEATURE_STATUS      gfxDoubleBufferFeatureRecentStatus;
    // The status of the request for synchronization.
    GFX_FEATURE_STATUS      gfxDoubleBufferRequestSync;
    // The status of the full synchronization request.
    GFX_FEATURE_STATUS      gfxDoubleBufferFullSync;
    // The array of rectangular areas that needs synchronization.
    GFX_RECTANGULAR_AREA    gfxDoubleBufferAreas[GFX_MAX_INVALIDATE_AREAS];
    // The current count of unsynchronized areas.
    uint16_t                gfxUnsyncedAreaCount;
} GFX_DOUBLE_BUFFERING_MODE;

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_TYPES_MACROS_H
