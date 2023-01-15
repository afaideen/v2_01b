/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_config.h

  Summary:
    SD Host Controller Driver System Configuration

  Description:
    The SD Host Controller driver provides a simple interface to manage the SD 
    Host Controller peripheral.  This file defines the interface definitions 
    and prototypes for the SD Host Controller driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _DRV_SDHC_CONFIG_H
#define _DRV_SDHC_CONFIG_H

/* Transfer block size : 512 bytes*/
#define XFER_BLK_SIZE 512 

/*Transfer block count : 1*/
#define XFER_BLK_COUNT 100

#define HOST_BASE_ADDRESS    0xBF8EC000

/*BOOT MESSAGE FOR SDIO DEVICE*/

#define SDIO_BOOT_MESSAGE 			ROM_DBG_COMM_ENABLED
#define HOST_MSG_FOR_BOOT_DATA		0xAA

//Disable The MMC driver to decrease code space to allow for additional debug messages to be printed.
#define DISABLE_MMC  1   

#endif	
