<#--
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

<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true>
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"
<#if CONFIG_PIC32MZ == true>
#include "driver/usb/usbhs/drv_usbhs.h"
<#elseif ((CONFIG_PIC32MX == true) || (CONFIG_PIC32WK == true))>
#include "driver/usb/usbfs/drv_usbfs.h"
</#if>
</#if>

<#if CONFIG_DRV_USB_HOST_SUPPORT == true>
#include "usb/usb_chapter_9.h"
#include "usb/usb_host.h"
<#if CONFIG_PIC32MZ == true>
#include "driver/usb/usbhs/drv_usbhs.h"
<#elseif ((CONFIG_PIC32MX == true) || (CONFIG_PIC32WK == true))>
#include "driver/usb/usbfs/drv_usbfs.h"
</#if>
</#if>

<#if CONFIG_DRV_USB_BETA_SW_HOST_SUPPORT == true>
#include "usb/beta_sw/usb_chapter_9.h"
#include "usb/beta_sw/usb_host.h"
<#if CONFIG_PIC32MZ == true>
#include "driver/usb/beta_sw/usbhs/drv_usbhs.h"
<#elseif ((CONFIG_PIC32MX == true) || (CONFIG_PIC32WK == true))>
#include "driver/usb/beta_sw/usbfs/drv_usbfs.h"
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
