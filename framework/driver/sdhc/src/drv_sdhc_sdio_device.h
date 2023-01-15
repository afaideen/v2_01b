/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_sdio_device.h

  Summary:
    SD Host Controller Driver SDIO Device Definitions

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

#ifndef _DRV_SDHC_SDIO_DEVICE_H
#define _DRV_SDHC_SDIO_DEVICE_H

#include "peripheral/peripheral_common_32bit.h"

/*
**********************************************************************************************
 			AHB SLAVE REGISTERS Offsets

**********************************************************************************************
*/

#define SDIO_RCA 	 		    	0x00
#define SDIO_VERSION  		 		0x04
#define SDIO_FUNC_MEMORY  			0x08
#define SDIO_CAPS  		    		0x0c
#define SDIO_SMPC_SHS  				0x10
#define SDIO_FUNCTION_SPS			0x14
#define SDIO_S8B         			0x18      /*Support 8 Bit Interface for Embedded SDIO */
#define SDIO_DRIVER_TYPE			0x1c      /* Support Driver Type.. A,C,D */
#define SDIO_FUNCTION_SAI      		0x20      /* Support for Asynchronous Interrupt */
#define SDIO_DEVICE_MSG_CODE        0x24
#define SDIO_INTERRUPT_STATUS       0x28      /* Interrupt status*/
#define SDIO_DATA_PORT              0x2c	  /*Data Port*/	
#define SDIO_DMA_LENGTH             0x30	  /* DMA length*/
#define SDIO_INTERRUPT_ENABLE       0x34      /* Interrupt enable*/
	




/*
**********************************************************************************************
		AHB SLAVE REGISTERS BIT DEFINITIONS		     
**********************************************************************************************
*/



#define  SDIO_CCCR_MEMORY_PRESENT	(1 << 11)
#define  SDIO_CCCR_4BLS  		(1 << 14)
#define  SDIO_CCCR_LSC			(1 << 13)
#define  SDIO_CCCR_S4MI			(1 << 12)
#define  SDIO_CCCR_SBS			(1 << 11)
#define  SDIO_CCCR_SRW			(1 << 10)
#define  SDIO_CCCR_SMB			(1 << 9)
#define  SDIO_CCCR_SDC			(1 << 8)
#define  SDIO_CCCR_SCSI			(1 << 0)


#define  SDIO_CCCR_SHS			(1 << 17)
#define  SDIO_CCCR_SMPC			(1 << 16)


#define  SDIO_CCCR_CSA_SUPPORT		(1 << 7)
#define  SDIO_CCCR_SPS			(1 << 6)


#define  SDIO_CCCR_8BIT_SUPPORT   (1 << 0)                                                                                   // ------------------
#define  SDIO_CCCR_SDR50_SUPPORT  (1 << 8)
#define  SDIO_CCCR_SDR104_SUPPORT (1 << 9)
#define  SDIO_CCCR_DDR50_SUPPORT  (1 << 10)

/**/
#define CCCR_DRIVER_TYPE
#define  SDIO_CCCR_SDTA           (1 << 0)                                                                                    // -----------------
#define  SDIO_CCCR_SDTC           (1 << 1)
#define  SDIO_CCCR_SDTD           (1 << 2)

#define  SDIO_SAI                 (1 << 0)                                                                                    // -------------------


#define  SDIO_INTERRUPT_OUT_ENABLE		(1 << 0)
#define  SDIO_INTERRUPT_OUT_STATUS	    (1 << 0)
#define  SDIO_INTERRUPT_OUT_MASK        (1 << 0)                                                                    // -------------------

#define SDIO_HOSTMSG_INTERRUPT_ENABLE    (1<<0)
#define SDIO_BLKSIZE_INTERRUPT_ENABLE    (1<<1)
#define SDIO_XFERCOMP_INTERRUPT_ENABLE   (1<<2)

#define SDIO_DAT1_INTERRUPT_SELECT       (1<<1)
#define SDIO_INT_N_INTERRUPT_SELECT       (0<<1)

#define SDIO_HOSTMSG_INTERRUPT_STATUS    (1<<0)
#define SDIO_BLKSIZE_INTERRUPT_STATUS	 (1<<1)
#define SDIO_XFERCOMP_INTERRUPT_STATUS   (1<<2)







/*
********************************************************************************************************
		internal I/O input/output macro 		 

********************************************************************************************************
*/
#define get_value(n)    (*((volatile unsigned char *)(n)))          /* byte input */
#define put_value(n,c)  (*((volatile unsigned char *)(n)) = (c))    /* byte output */
#define get_hvalue(n)   (*((volatile unsigned short *)(n)))         /* half word input */
#define put_hvalue(n,c) (*((volatile unsigned short *)(n)) = (c))   /* half word output */
#define get_wvalue(n)   (*((volatile unsigned int *)(n)))          /* word input */
#define put_wvalue(n,c) (*((volatile unsigned int *)(n)) = (c))    /* word output */
#define set_bit(n,c)    (*((volatile unsigned char *)(n))|= (c))    /* byte bit set */
#define clr_bit(n,c)    (*((volatile unsigned char *)(n))&=~(c))    /* byte bit clear */
#define set_hbit(n,c)   (*((volatile unsigned short *)(n))|= (c))   /* half word bit set */
#define clr_hbit(n,c)   (*((volatile unsigned short *)(n))&=~(c))   /* half word bit clear */
#define set_wbit(n,c)   (*((volatile unsigned int *)(n))|= (c))    /* word bit set */
#define clr_wbit(n,c)   (*((volatile unsigned int *)(n))&=~(c))    /* word bit clear */ 


/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void WriteReg(unsigned int , unsigned int);
unsigned int ReadReg(unsigned int);
void SDIODevice_Init(int);
void sdioProgramBootWord(uint8_t );
void enable_interrupt(uint8_t );
void generate_card_interrupt_dat1(void);
void generate_card_interrupt_int_n(void);
void write_device_msg_code(uint8_t );
uint8_t read_host_msg_code(void);
uint32_t read_dma_length(void);
uint32_t read_data_port(void);
void write_data_port(uint32_t );

#endif
