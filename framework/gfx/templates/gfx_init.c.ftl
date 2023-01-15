<#--
/*******************************************************************************
  GFX Initialization File

  File Name:
    gfx_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

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
 -->

    sysObj.gfxObject0 = GFX_Initialize(GFX_INDEX_0, (SYS_MODULE_INIT *)&gfxInit0);
    <#if (CONFIG_GFX_INST_1 = true)>
    sysObj.gfxObject1 = GFX_Initialize(GFX_INDEX_1, (SYS_MODULE_INIT *)&gfxInit0);
    </#if>
    <#if (CONFIG_GFX_INST_2 = true)>
    sysObj.gfxObject2 = GFX_Initialize(GFX_INDEX_2, (SYS_MODULE_INIT *)&gfxInit0);
    </#if>

<#if CONFIG_USE_GFX_JPEG_SUPPORT == true 
	|| CONFIG_USE_GFX_GIF_SUPPORT == true
	|| CONFIG_USE_GFX_BMP_SUPPORT == true>
    ImageDecoderInit();
</#if>
<#if CONFIG_USE_GFX_JPEG_SUPPORT == true>
    JPEGInit();
</#if>
<#if CONFIG_USE_GFX_BMP_SUPPORT == true>
    BMPInit();
</#if>
<#if CONFIG_USE_GFX_GIF_SUPPORT == true>
    GIFInit();
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
