<#--
/*******************************************************************************
  SDHC Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc.h.ftl

  Summary:
    SDHC Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

/*** SDHC Driver Configuration ***/
<#if CONFIG_USE_DRV_SDHC == true>
<#-- Driver Instances -->
#define DRV_SDHC_QUEUE_POOL_SIZE      ${CONFIG_DRV_SDHC_QUEUE_POOL_SIZE}
#define DRV_SDHC_INSTANCES_NUMBER     ${CONFIG_DRV_SDHC_INSTANCES_NUMBER}
#define DRV_SDHC_CLIENTS_NUMBER       ${CONFIG_DRV_SDHC_CLIENTS_NUMBER}
<#if CONFIG_DRV_SDHC_SDHC_BUS_SPEED == "HIGH_SPEED">
#define DRV_SDHC_BUS_SPEED            SDR12_MODE
<#else>
#define DRV_SDHC_BUS_SPEED            SDR25_MODE
</#if>
<#if CONFIG_DRV_SDHC_TRANSFER_MODE == "ADMA">
#define DRV_SDHC_USE_DMA              true
<#else>
#define DRV_SDHC_USE_DMA              false
</#if>
#define DRV_SDHC_BUS_WIDTH            BIT_${CONFIG_DRV_SDHC_BUS_WIDTH}
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
