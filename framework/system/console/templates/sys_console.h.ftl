<#--
/*******************************************************************************
  Clock System Service Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    sys_console.h.ftl

  Summary:
    Clock System Service Freemarker Template File

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
<#if CONFIG_USE_SYS_CONSOLE == true>
<#if CONFIG_USE_SYS_CONSOLE == true && CONFIG_SYS_CONSOLE_MODE == "STATIC">
#define SYS_CONSOLE_DEVICE_MAX_INSTANCES        1
#define SYS_CONSOLE_INSTANCES_NUMBER            1
</#if>
<#if CONFIG_USE_SYS_CONSOLE == true && CONFIG_SYS_CONSOLE_MODE == "DYNAMIC">

/*** Console System Service Configuration ***/

<#if CONFIG_SYS_CONSOLE_OVERRIDE_STDIO == true>
#define SYS_CONSOLE_OVERRIDE_STDIO
</#if>
#define SYS_CONSOLE_DEVICE_MAX_INSTANCES        2
#define SYS_CONSOLE_INSTANCES_NUMBER            ${CONFIG_SYS_CONSOLE_INSTANCES_NUMBER}
<#if CONFIG_SYS_CONSOLE_APPIO_RD_QUEUE_DEPTH?has_content>
#define SYS_CONSOLE_APPIO_RD_QUEUE_DEPTH    ${CONFIG_SYS_CONSOLE_APPIO_RD_QUEUE_DEPTH}
#define SYS_CONSOLE_APPIO_WR_QUEUE_DEPTH    ${CONFIG_SYS_CONSOLE_APPIO_WR_QUEUE_DEPTH}
</#if>
<#if CONFIG_SYS_CONSOLE_UART_RD_QUEUE_DEPTH?has_content>
#define SYS_CONSOLE_UART_IDX               DRV_USART_INDEX_${CONFIG_SYS_CONSOLE_USE_UART_INSTANCE}
#define SYS_CONSOLE_UART_RD_QUEUE_DEPTH    ${CONFIG_SYS_CONSOLE_UART_RD_QUEUE_DEPTH}
#define SYS_CONSOLE_UART_WR_QUEUE_DEPTH    ${CONFIG_SYS_CONSOLE_UART_WR_QUEUE_DEPTH}
</#if>
<#if CONFIG_SYS_CONSOLE_USB_CDC_RD_QUEUE_DEPTH?has_content>
<#if CONFIG_USB_DEVICE_FUNCTION_7_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_7_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_7}
<#elseif CONFIG_USB_DEVICE_FUNCTION_6_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_6_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_6}
<#elseif CONFIG_USB_DEVICE_FUNCTION_5_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_5_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_5}
<#elseif CONFIG_USB_DEVICE_FUNCTION_4_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_4_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_4}
<#elseif CONFIG_USB_DEVICE_FUNCTION_3_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_3_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_3}
<#elseif CONFIG_USB_DEVICE_FUNCTION_2_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_2_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_2}
<#elseif CONFIG_USB_DEVICE_FUNCTION_1_DEVICE_CLASS_CDC_IDX0?has_content && CONFIG_USB_DEVICE_FUNCTION_1_DEVICE_CLASS_CDC_IDX0 == true>
#define SYS_CONSOLE_USB_CDC_INSTANCE       ${CONFIG_SYS_CONSOLE_USB_CDC_INSTANCE_1}
</#if>
#define SYS_CONSOLE_USB_CDC_COMM_BAUD_RATE ${CONFIG_SYS_CONSOLE_USB_CDC_COMM_BAUD_RATE}
#define SYS_CONSOLE_USB_CDC_RD_QUEUE_DEPTH ${CONFIG_SYS_CONSOLE_USB_CDC_RD_QUEUE_DEPTH}
#define SYS_CONSOLE_USB_CDC_WR_QUEUE_DEPTH ${CONFIG_SYS_CONSOLE_USB_CDC_WR_QUEUE_DEPTH}
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true>
#define SYS_CONSOLE_USB_CDC_READ_BUFFER_SIZE   512
<#else>
#define SYS_CONSOLE_USB_CDC_READ_BUFFER_SIZE   64
</#if>
</#if>
<#if (CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true) && CONFIG_USB_DEVICE_USE_CDC_NEEDED == true>
#define SYS_CONSOLE_BUFFER_DMA_READY        __attribute__((coherent)) __attribute__((aligned(16)))
<#else>
#define SYS_CONSOLE_BUFFER_DMA_READY
</#if>

</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
