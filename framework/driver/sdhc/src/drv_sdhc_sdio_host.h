/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_sdio_host.h

  Summary:
    SD Host Controller Driver SDIO Host Definitions

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

#ifndef _DRV_SDHC_SDIO_HOST_H
#define _DRV_SDHC_SDIO_HOST_H

/*SDIO Host Controller Registers*/
#define SDIO3_SDHC_BLK_SIZE         0x004
#define SDIO3_SDHC_BLK_CNT          0x006
#define SDIO3_SDHC_ARG_LOW          0x008
#define SDIO3_SDHC_ARG_HIGH         0x00A
#define SDIO3_SDHC_TM               0x00C
#define SDIO3_SDHC_CMD              0x00E
#define SDIO3_SDHC_RSP0             0x010
#define SDIO3_SDHC_RSP1             0x012
#define SDIO3_SDHC_RSP2             0x014
#define SDIO3_SDHC_RSP3             0x016
#define SDIO3_SDHC_RSP4             0x018
#define SDIO3_SDHC_RSP5             0x01A
#define SDIO3_SDHC_RSP6             0x01C
#define SDIO3_SDHC_RSP7             0x01E

#define SDIO3_SDHC_BDP_0            0x020
#define SDIO3_SDHC_BDP_1            0x022
#define SDIO3_SDHC_PSTATE           0x024
#define SDIO3_SDHC_HCTL_1           0x028
#define SDIO3_SDHC_PCTL             0x029
#define SDIO3_SDHC_BGCTL            0x02A
#define SDIO3_SDHC_WCTL             0x02B
#define SDIO3_SDHC_CLKCTL           0x02C
#define SDIO3_SDHC_TCTL             0x02E
#define SDIO3_SDHC_SFT_RST          0x02F

#define SDIO3_SDHC_INT_STS          0x030
#define SDIO3_SDHC_INT_STS_EN       0x034
#define SDIO3_SDHC_INT_SIG_EN       0x038

#define SDIO3_SDHC_ERR_INT_STS      0x032
#define SDIO3_SDHC_ERR_INT_STS_EN   0x036
#define SDIO3_SDHC_ERR_INT_SIG_EN   0x03A

#define SDIO3_SDHC_AUTO_CMD_ESTS    0x03C
#define SDIO3_SDHC_HCTL_2           0x03E
#define SDIO3_SDHC_CAP1             0x040
#define SDIO3_SDHC_CAP2             0x044
#define SDIO3_SDHC_MCCAP            0x048
#define SDIO3_SDHC_FEVENT_ESTS      0x050
#define SDIO3_SDHC_FEVENT_EINTSTS   0x052
#define SDIO3_SDHC_ADMA_SYS_ADDR    0x058
#define SDIO3_SDHC_PRESET_VAL       0x060
#define SDIO3_SDHC_SHARED_BUSCTL    0x0E0
#define SDIO3_SDHC_EMMC_CONFIG		0x0E4
#define SDIO3_SDHC_SLOT_INTSTS      0x0FC
#define SDIO3_SDHC_HCTL_VER         0x0FE

/*End of host controller registers*/

/*software reset register masks at offset (0x02F)*/

#define  SDHCI_RESET_ALL	0x01
#define  SDHCI_RESET_CMD	0x02
#define  SDHCI_RESET_DATA	0x04



/* Response Types defined at Command register offset(0x0E)*/
#define SD_CMD_RSP_NONE              0x0
#define SD_CMD_RSP_LEN_136           0x1
#define SD_CMD_RSP_LEN_48            0x2
#define SD_CMD_RSP_LEN_48BUSY        0x3

/* Block Gap Control Register Masks offset (0x2A)*/
#define SD_BLOCK_GAP_STOP_REQ (1<<0)


/*software reset bits at offset(0x2F)*/
#define  SFT_RST_ALL   0x01
#define  SFT_RST_CMD   0x02
#define  SFT_RST_DATA  0x04

/*power control register bits at offset(0x29)*/
#define  SD_BUS_PWRON     0x01
#define  SD_POWER_1_8     0x0A
#define  SD_POWER_3_0     0x0C
#define  SD_POWER_3_3     0x0E

/*clock control register bits at offset(0x2C)*/
#define  SDCLK_FREQ_SEL  (0x80 << 8)  /*440 khz*/
#define  INT_CLK_EN       1<<0
#define  INT_CLK_STABLE   1<<1
#define  SD_CLK_EN        1<<2

/*wakeup control register bits at offset(0x2B)*/
#define  WAKE_ON_INT      0x01
#define  WAKE_ON_INSERT   0x02
#define  WAKE_ON_REMOVE   0x04

/*Host control1 register bits at offset(0x28)*/
#define  BITBUS1_CTRL      0x00
#define  LED_CTRL          0x01
#define  BITBUS4_CTRL      0x02
#define  HISPD_EN_CTRL     0x04
#define  DMA_EN_CTRL       0x10
#define  EXT_8BITBUS_CTRL  0x20
#define  CARD_DET_TEST_LVL 0x40
#define  CARD_DET_SIG_SEL  0x80

/*normal interrupt status register bits at offset(0x30)*/
#define  SD_CMD_COMPLETE_BIT     0x01
#define  SD_ERROR_INTERRUPT_BIT  0x80

/*error interrupt status register bits at offset(0x32)*/
#define  SD_CMD_TIMEOUT_ERR_BIT  0x01
#define  SD_CMD_CRC_ERR_BIT      0x02

/*Host control2 register bits at offset (0x3E)*/
#define  UHS_MASK		     0x0007
#define  UHS_SDR12		     0x0000
#define  UHS_SDR25		     0x0001
#define  UHS_SDR50		     0x0002
#define  UHS_SDR104		     0x0003
#define  UHS_DDR50		     0x0004
#define  SIGNAL_ENABLE_1_8V  0x0008

#define  VDD_180		0x0008

#define  DRV_TYPE_MASK	0x0030
#define  DRV_TYPE_B		0x0000
#define  DRV_TYPE_A		0x0010
#define  DRV_TYPE_C		0x0020
#define  DRV_TYPE_D		0x0030

#define  EXEC_TUNING	0x0040
#define  TUNED_CLK		0x0080
#define  PRESET_VAL_ENABLE	0x8000


/*present state register bits at offset(0x24)*/
#define  CMD_INHIBIT         1<<1
#define  DATA_INHIBIT        1<<2
#define  WRITE_XFER_ACTIVE   1<<8
#define  READ_XFER_ACTIVE    1<<9
#define  BUFF_WR_EN          1<<10
#define  BUFF_RD_EN          1<<11
#define  CARD_INSERTED       1<<16
#define  CARD_STATE_STABLE   1<<17
#define  CARD_DETECT         1<<18
#define  WRITE_PROTECT       1<<19

/* Capablities register bits at offset(0x40)*/

#define  TIMEOUT_CLK_FRQ	      0x0000003F
#define  TIMEOUT_CLK_UNIT	      0x00000080
#define  BASE_CLK_FRQ   	      0x00003F00
#define  CLOCK_V3_BASE_FRQ	      0x0000FF00
#define  MAX_BLOCK_LEN	              0x00030000
#define  CAN_SUPPORT_8BIT             0x00040000
#define  CAN_SUPPORT_HISPD            0x00200000
#define  CAN_SUPPORT_VDD_3_3          0x01000000
#define  CAN_SUPPORT_VDD_3_0	      0x02000000
#define  CAN_SUPPORT_VDD_1_8          0x04000000
#define  CAN_SUPPORT_64BIT_BUS        0x10000000

/*Transfer mode register bits at offset(0X0c) */

#define SD_XFER_MODE_DMA_EN_MASK        (1<<0)
#define SD_XFER_MODE_BLK_CNT_MASK       (1<<1)
#define SD_XFER_MODE_ACMD12_EN_MASK     (1<<2)
#define SD_XFER_MODE_TO_HOST_DIR_MASK   (1<<4)
#define SD_XFER_MODE_MULTI_BLK_SEL_MASK (1<<5)


/*emmc configuration register bits*/

#define EMMC_ENABLE_PUSHPULL	(1 << 0)
#define EMMC_ENABLE_DDR_12V		(1 << 1)

#define SDHCI_MAX_DIV_SPEC_200	256
#define SDHCI_MAX_DIV_SPEC_300	2046

#endif
