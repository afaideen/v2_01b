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

/*** SRAM Driver Initialization Data ***/
<#if CONFIG_USE_DRV_SRAM == true>

SYS_FS_MEDIA_REGION_GEOMETRY SRAMGeometryTable[3] = 
{
    {
        .blockSize = 512,
        .numBlocks = (DRV_SRAM_MEDIA_SIZE * (1024/512)),
    },
    {
       .blockSize = 512,
       .numBlocks = (DRV_SRAM_MEDIA_SIZE * (1024/512)),
    },
    {
       .blockSize = 512,
       .numBlocks = (DRV_SRAM_MEDIA_SIZE * (1024/512))
    }
};

const SYS_FS_MEDIA_GEOMETRY SRAMGeometry = 
{
    .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING,
    .numReadRegions = 1,
    .numWriteRegions = 1,
    .numEraseRegions = 1,
    .geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&SRAMGeometryTable
};

const DRV_SRAM_INIT drvSramInit =
{
<#if CONFIG_DRV_SRAM_MEDIA_START_ADDRESS?has_content>
    .mediaStartAddress = ${CONFIG_DRV_SRAM_MEDIA_START_ADDRESS},
</#if>
    .sramMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)&SRAMGeometry
};

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
