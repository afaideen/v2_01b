<#--
/*******************************************************************************
  GFX Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    gfx.h.ftl

  Summary:
    GFX Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->


/*** GFX Library Configuration ***/

<#if CONFIG_USE_GFX_STACK == true>
#define GFX_INSTANCES_NUMBER                            ${CONFIG_GFX_MAX_LAYERS}
#define GFX_SELF_PREEMPTION_LEVEL                       ${CONFIG_GFX_PREEMPTION_LEVEL}
#define GFX_CONFIG_COLOR_DEPTH                          ${CONFIG_GFX_COLOR_DEPTH}

<#if CONFIG_USE_GFX_FONTS == true>
#define GFX_CONFIG_FONT_CHAR_SIZE                       ${CONFIG_GFX_FONT_CHAR_SIZE}
<#if CONFIG_GFX_FONT_FLASH_DISABLE == false>
#define GFX_CONFIG_FONT_FLASH_DISABLE
</#if>
<#if CONFIG_GFX_FONT_EXTERNAL_DISABLE == false>
#define GFX_CONFIG_FONT_EXTERNAL_DISABLE
<#else>
#define GFX_EXTERNAL_FONT_RASTER_BUFFER_SIZE            ${CONFIG_GFX_EXTERNAL_FONT_RASTER_BUFFER_SIZE}
</#if>
<#if CONFIG_GFX_FONT_RAM_DISABLE == false>
#define GFX_CONFIG_FONT_RAM_DISABLE
</#if>
<#else>
#define GFX_CONFIG_FONT_FLASH_DISABLE
#define GFX_CONFIG_FONT_EXTERNAL_DISABLE
#define GFX_CONFIG_FONT_RAM_DISABLE
</#if>
<#if CONFIG_USE_GFX_IMAGES == true>
<#if CONFIG_GFX_IMAGE_FLASH_DISABLE == false>
#define GFX_CONFIG_IMAGE_FLASH_DISABLE
</#if>
<#if CONFIG_GFX_IMAGE_EXTERNAL_DISABLE == false>
#define GFX_CONFIG_IMAGE_EXTERNAL_DISABLE
</#if>
<#if CONFIG_GFX_IMAGE_RAM_DISABLE == false>
#define GFX_CONFIG_IMAGE_RAM_DISABLE
</#if>
<#if CONFIG_GFX_RLE_DECODE_DISABLE == false>
#define GFX_CONFIG_RLE_DECODE_DISABLE
</#if>
<#if CONFIG_GFX_IPU_DECODE_DISABLE == false>
#define GFX_CONFIG_IPU_DECODE_DISABLE
</#if>
<#if CONFIG_USE_GFX_JPEG_SUPPORT == true>
#define IMG_SUPPORT_JPEG
</#if>
<#if CONFIG_USE_GFX_BMP_SUPPORT == true>
#define IMG_SUPPORT_BMP
</#if>
<#if CONFIG_USE_GFX_GIF_SUPPORT == true>
#define IMG_SUPPORT_GIF
</#if>
<#if CONFIG_USE_GFX_JPEG_SUPPORT == true 
	|| CONFIG_USE_GFX_GIF_SUPPORT == true
	|| CONFIG_USE_GFX_BMP_SUPPORT == true>
#define IMG_SUPPORT_IMAGE_DECODER_LOOP_CALLBACK
</#if>
<#if CONFIG_GFX_IMAGE_PADDING_DISABLE == false>
#define GFX_CONFIG_IMAGE_PADDING_DISABLE
</#if>
<#else>
#define GFX_CONFIG_IMAGE_FLASH_DISABLE
#define GFX_CONFIG_IMAGE_EXTERNAL_DISABLE
#define GFX_CONFIG_IMAGE_RAM_DISABLE
#define GFX_CONFIG_RLE_DECODE_DISABLE
#define GFX_CONFIG_IMAGE_PADDING_DISABLE
</#if>
<#if CONFIG_USE_GFX_TECHNIQUES == true>
<#if CONFIG_GFX_ALPHABLEND_DISABLE == false>
#define GFX_CONFIG_ALPHABLEND_DISABLE
</#if>
<#if CONFIG_GFX_GRADIENT_DISABLE == false>
#define GFX_CONFIG_GRADIENT_DISABLE
</#if>
<#if CONFIG_GFX_PALETTE_DISABLE == false>
#define GFX_CONFIG_PALETTE_DISABLE
</#if>
<#if CONFIG_GFX_FONT_ANTIALIASED_DISABLE == false>
#define GFX_CONFIG_FONT_ANTIALIASED_DISABLE
</#if>
<#if CONFIG_GFX_TRANSPARENT_COLOR_DISABLE == false>
#define GFX_CONFIG_TRANSPARENT_COLOR_DISABLE
</#if>
<#if CONFIG_GFX_PALETTE_EXTERNAL_DISABLE == false>
#define GFX_CONFIG_PALETTE_EXTERNAL_DISABLE
</#if>
<#if CONFIG_GFX_DOUBLE_BUFFERING_DISABLE == false>
#define GFX_CONFIG_DOUBLE_BUFFERING_DISABLE
</#if>
<#if CONFIG_GFX_VSYNC_ENABLE == true>
#define GFX_CONFIG_VSYNC_ENABLE
</#if>
<#if CONFIG_GFX_PIXEL_ARRAY_NO_SWAP_DISABLE == true>
#define GFX_CONFIG_PIXEL_ARRAY_NO_SWAP_DISABLE
</#if>
<#else>
#define GFX_CONFIG_ALPHABLEND_DISABLE
#define GFX_CONFIG_GRADIENT_DISABLE
#define GFX_CONFIG_PALETTE_DISABLE
#define GFX_CONFIG_FONT_ANTIALIASED_DISABLE
#define GFX_CONFIG_TRANSPARENT_COLOR_DISABLE
#define GFX_CONFIG_PALETTE_EXTERNAL_DISABLE
#define GFX_CONFIG_DOUBLE_BUFFERING_DISABLE
</#if>
<#if CONFIG_USE_GFX_INPUT_DEVICE == true>
<#if CONFIG_GFX_USE_KEYBOARD_DISABLE == false>
#define GFX_CONFIG_USE_KEYBOARD_DISABLE
</#if>
<#if CONFIG_GFX_USE_TOUCHSCREEN == false>
#define GFX_CONFIG_USE_TOUCHSCREEN_DISABLE
</#if>
<#else>
#define GFX_CONFIG_USE_KEYBOARD_DISABLE
#define GFX_CONFIG_USE_TOUCHSCREEN_DISABLE
</#if>
<#if CONFIG_USE_GFX_GOL == true>
<#if CONFIG_USE_GFX_GOL_RADIO_BTN == true>
<#if CONFIG_GFX_HGC_NUM_RADIOBUTTON_GROUPS?has_content>
#define GFX_GOL_NUM_RADIOBUTTON_GROUPS  ${CONFIG_GFX_HGC_NUM_RADIOBUTTON_GROUPS}
<#else>
#define GFX_GOL_NUM_RADIOBUTTON_GROUPS  1
</#if>
</#if>
<#if CONFIG_GFX_FOCUS_DISABLE == false>
#define GFX_CONFIG_FOCUS_DISABLE
</#if>
#define GFX_malloc(size)                                    malloc(size)
#define GFX_free(pObj)                                      free(pObj)
<#else>
#define GFX_CONFIG_GOL_DISABLE
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->

