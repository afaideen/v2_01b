/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_host.h

  Summary:
    SD Host Controller Driver System Host Definitions

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

#ifndef _DRV_SDHC_HOST_H
#define _DRV_SDHC_HOST_H

#include "system_definitions.h"
#include "drv_sdhc_sd.h"
#include "drv_sdhc_error_types.h"

#define HOST_BASE_CLOCK_FROM_FUSE	(1 << 0)
#define HOST_USE_DMA			  	(1 << 1)	
#define HOST_SUPPORT_DMA 		  	(1 << 2)
#define HOST_BOOT_FROM_MMC			(1 << 3)
#define HOST_BOOT_FROM_SD			(1 << 4)
#define HOST_BOOT_FROM_SDIO			(1 << 5)
#define HOST_BOOT_FROM_EMMC			(1 << 6)
#define HOST_BOOT_FROM_ESD			(1 << 7)
#define HOST_BOOT_FROM_ESDIO		(1 << 8)
#define HOST_USE_REMOVABLE_SLOT  	(1 << 9)
#define HOST_USE_SHAREDBUS_SLOT  	(1 << 10)
#define	HOST_BOOT_BUS_WIDTH_1		(1 << 11)
#define	HOST_BOOT_BUS_WIDTH_4		(1 << 12)
#define	HOST_BOOT_BUS_WIDTH_8		(1 << 13)
#define HOST_USE_SDR12_MODE         (1 << 14)
#define HOST_USE_SDR25_MODE		  	(1 << 15)
#define	HOST_USE_SDR50_MODE			(1 << 16)
#define	HOST_USE_SDR104_MODE		(1 << 17)
#define	HOST_USE_DDR50_MODE			(1 << 18)
#define HOST_ENABLE_1_8V_SIGNAL		(1 << 19)
#define HOST_ENABLE_1_2V_SIGNAL		(1 << 20)

#define HOST_TIMEOUT_CLOCK_FROM_FUSE (1 << 18)


#define HOST_CLOCK_V3_BASE_MASK		0x0000FF00
#define HOST_CLOCK_BASE_MASK			0x00003F00
#define HOST_TIMEOUT_CLK_MASK  		0x0000003F
#define HOST_TIMEOUT_CLK_UNIT 		0x00000080
#define HOST_CAN_DO_HISPD			0x00200000
#define HOST_CAN_8BIT_SUPPORT       0X00040000


#define HOST_CLOCK_BASE_SHIFT		8
#define HOST_TIMEOUT_CLK_SHIFT 		0
#define HOST_MAX_DIV_SPEC_200	256
#define HOST_MAX_DIV_SPEC_300	2046
#define HOST_DIV_MASK	0xFF
#define  HOST_DIVIDER_SHIFT	8
#define  HOST_DIVIDER_HI_SHIFT	6
#define  HOST_DIV_MASK	0xFF
#define  HOST_DIV_MASK_LEN	8
#define  HOST_DIV_HI_MASK	0x300
#define  HOST_CLOCK_CARD_EN	0x0004
#define  HOST_CLOCK_INT_STABLE	0x0002
#define  HOST_CLOCK_INT_EN	0x0001

#define HOST_CAN_VDD_330    0x01000000
#define HOST_CAN_VDD_300	0x02000000
#define HOST_CAN_VDD_180	0x04000000
#define HOST_CAN_SDR50		0X00000001
#define HOST_CAN_SDR104		0X00000002
#define HOST_CAN_DDR50		0X00000004





#define  HOST_RESET_ALL	0x01

#define  HOST_INT_RESPONSE	0x00000001
#define  HOST_INT_DATA_END	0x00000002
#define  HOST_INT_DMA_END	0x00000008
#define  HOST_INT_SPACE_AVAIL	0x00000010
#define  HOST_INT_DATA_AVAIL	0x00000020
#define  HOST_INT_CARD_INSERT	0x00000040
#define  HOST_INT_CARD_REMOVE	0x00000080
#define  HOST_INT_CARD_INT	0x00000100
#define  HOST_INT_ERROR	0x00008000
#define  HOST_INT_TIMEOUT	0x00010000
#define  HOST_INT_CRC		0x00020000
#define  HOST_INT_END_BIT	0x00040000
#define  HOST_INT_INDEX	0x00080000
#define  HOST_INT_DATA_TIMEOUT	0x00100000
#define  HOST_INT_DATA_CRC	0x00200000
#define  HOST_INT_DATA_END_BIT	0x00400000
#define  HOST_INT_CUR_TIMEOUT_ERR 0x00800000
#define  HOST_INT_ACMD12_ERR	0x01000000
#define  HOST_INT_ADMA_ERR	0x02000000
#define  HOST_INT_VENDOR_SPECIFIC_ERR 0xE0000000
#define  HOST_BUS_ERROR		0x80000000
#define  INVALID_BUS_MODE_SEL_ERR 0x40000000


#define CMD_INT_MASK   (HOST_INT_TIMEOUT | HOST_INT_RESPONSE | HOST_INT_CRC | HOST_INT_END_BIT | HOST_INT_INDEX )
#define DATA_INT_MASK  (HOST_INT_DATA_END | HOST_INT_SPACE_AVAIL | HOST_INT_DATA_AVAIL | HOST_INT_DATA_TIMEOUT | HOST_INT_DATA_CRC | HOST_INT_DATA_END_BIT)

#define DMA_CMD_INT_MASK   (HOST_INT_TIMEOUT | HOST_INT_RESPONSE | HOST_INT_CRC | HOST_INT_END_BIT | HOST_INT_INDEX ) 
#define DMA_DATA_INT_MASK  (HOST_INT_VENDOR_SPECIFIC_ERR | HOST_INT_CUR_TIMEOUT_ERR | HOST_INT_SPACE_AVAIL | HOST_INT_DATA_AVAIL | HOST_INT_ADMA_ERR | HOST_INT_ACMD12_ERR | HOST_INT_DATA_END |  HOST_INT_DMA_END | HOST_INT_DATA_TIMEOUT | HOST_INT_DATA_CRC | HOST_INT_DATA_END_BIT)



#define CMD_COMPL_FLAG       0x01
#define BUFF_WR_READY_FLAG   0x10
#define BUFF_RD_READY_FLAG   0x20
#define DATA_COMPLETE        0x02
#define DMA_INTERRUPT        0x08


#define   HOST_SPEC_100	0
#define   HOST_SPEC_200	1
#define   HOST_SPEC_300	2


#define HOST_INT_ALL_MASK	((unsigned int)-1)

bool _drv_sdhc_is_write_protected(drv_sdhc_sd_handle_t *);
DRV_SDHC_RESPONSE _drv_sdhc_host_init(drv_sdhc_sd_host_t *,drv_sdhc_adma_descriptor *);
DRV_SDHC_RESPONSE _drv_sdhc_init(const drv_sdhc_fuse_settings_t *,drv_sdhc_adma_descriptor *,sd_cmd_t *);
DRV_SDHC_RESPONSE _drv_sdhc_host_master_init(const drv_sdhc_fuse_settings_t *, drv_sdhc_sd_host_t *,drv_sdhc_adma_descriptor *);
DRV_SDHC_RESPONSE _drv_sdhc_capture_response( drv_sdhc_sd_handle_t *);
DRV_SDHC_RESPONSE _drv_sdhc_sd_cmd_send(drv_sdhc_sd_handle_t *);
DRV_SDHC_RESPONSE _drv_sdhc_handle_int(drv_sdhc_sd_handle_t *);
void _drv_sdhc_host_irq(drv_sdhc_sd_handle_t *);
void _drv_sdhc_set_bus_width(bus_width_t busWidth);
void _drv_sdhc_set_adma_descripter_table(drv_sdhc_adma_descriptor *descrTable);
void _drv_sdhc_set_transfer_block_size(uint16_t blk_size);
drv_sdhc_sd_handle_t* _drv_sdhc_check_card_inserted();
void _drv_sdhc_reset_error(drv_sdhc_sd_host_t *, uint8_t);
uint16_t _drv_sdhc_transfer_mode(drv_sdhc_sd_handle_t *card);
void _drv_sdhc_set_block_count(uint16_t blk_count);
uint32_t _drv_sdhc_read_buffer_data();
void _drv_sdhc_write_buffer_data(uint32_t data);

#endif









