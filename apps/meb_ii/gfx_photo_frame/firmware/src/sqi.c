/*******************************************************************************
  SQI functions

  File Name:
    sqi.c

  Summary:
    SQI functions

  Description:
    This is the sample driver that implements all the SQI functions.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "sys/kmem.h"
#include "sqi.h"
#include "peripheral/sqi/plib_sqi.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************

/* buffers in non cached memory */
uint8_t __attribute__((coherent)) sqiCmdBuffer[32];
sqiDMADesc __attribute__((coherent)) sqiDescCommand1;
sqiDMADesc __attribute__((coherent)) sqiDescCommand2;
sqiDMADesc __attribute__((coherent)) sqiDescBuffer[SQI_NUM_BUFFER_DESC + 1]; /* allow for end descriptor */

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Read
// *****************************************************************************
// *****************************************************************************

static uint32_t APP_ReadCoreTimer()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return (timer);
}

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Sart
// *****************************************************************************
// *****************************************************************************

static void APP_StartCoreTimer(uint32_t period)
{
    /* Reset the coutner */
    volatile uint32_t loadZero = 0;

    asm volatile("mtc0   %0, $9" : "+r"(loadZero));
    asm volatile("mtc0   %0, $11" : "+r" (period));
}
// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Delay
// *****************************************************************************
// *****************************************************************************

static void APP_CoreTimer_Delay(uint32_t delayValue)
{
    while ((APP_ReadCoreTimer() <= delayValue))
        asm("nop");
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI DMA Read
// *****************************************************************************
// *****************************************************************************
int SQI_DMA_PageRead(uint32_t deviceAddress, uint32_t count, uint8_t* destAddr)
{
    uint32_t pageCount, pageIndex;
    uint32_t i;
    uint8_t* baseAddress;

    baseAddress = destAddr;

    memset(sqiCmdBuffer, 0, sizeof(sqiCmdBuffer));
    sqiCmdBuffer[0] = (uint8_t) SST26VF_FAST_READ;
    sqiCmdBuffer[1] = (uint8_t) (deviceAddress >> 16);
    sqiCmdBuffer[2] = (uint8_t) (deviceAddress >> 8);
    sqiCmdBuffer[3] = (uint8_t) (deviceAddress);
    sqiCmdBuffer[4] = 0;                            /* dummy byte */

    /************************************************************************************/
    /* bounds check the count value, only integer multiples of 256 work */
    if ((count % 256) != 0)
    {
        return -1;
    }

    pageCount = count / 256;
    if (pageCount > SQI_NUM_BUFFER_DESC)
    {
        pageCount = SQI_NUM_BUFFER_DESC;
    }

    /************************************************************************************/
    /* Prepare the buffer descriptors */

    // Command Descriptor 1 - BD1 (to send fast read command and address)
    sqiDescCommand1.BDCon = 0x90800004;
    sqiDescCommand1.BDStat = 0;
    sqiDescCommand1.BDAddr = (unsigned int*) KVA_TO_PA(&sqiCmdBuffer[0]);
    sqiDescCommand1.nextBDAddr = (struct sqiDMADesc *) KVA_TO_PA(&sqiDescCommand2);

    // Command Descriptor 2 - BD2 (to send dummy cycles)
    sqiDescCommand2.BDCon = 0x90900001;
    sqiDescCommand2.BDStat = 0;
    sqiDescCommand2.BDAddr = (unsigned int*) KVA_TO_PA(&sqiCmdBuffer[4]);
    sqiDescCommand2.nextBDAddr = (struct sqiDMADesc *) KVA_TO_PA(&sqiDescBuffer[0]);

    memset(sqiDescBuffer, 0, sizeof(sqiDescBuffer));
    for(pageIndex = 0; pageIndex < pageCount - 1; pageIndex++)
    {
        sqiDescBuffer[pageIndex].BDCon = 0x90900100;
        sqiDescBuffer[pageIndex].BDStat = 0;
        sqiDescBuffer[pageIndex].BDAddr = (unsigned int*) KVA_TO_PA(destAddr);
        sqiDescBuffer[pageIndex].nextBDAddr = (struct sqiDMADesc*) KVA_TO_PA(&sqiDescBuffer[pageIndex + 1]);
        destAddr += 256;
    }

    // Last data descriptor
    sqiDescBuffer[pageIndex].BDCon = 0xD09D0100;
    sqiDescBuffer[pageIndex].BDStat = 0;
    sqiDescBuffer[pageIndex].BDAddr = (unsigned int*) KVA_TO_PA(destAddr);
    sqiDescBuffer[pageIndex].nextBDAddr = (struct sqiDMADesc*) KVA_TO_PA(&sqiDescBuffer[pageIndex + 1]);

    /* Dummy descriptor */
    sqiDescBuffer[pageIndex+1].BDCon = 0x50000000;
    sqiDescBuffer[pageIndex+1].nextBDAddr = 0x00000000;

    // Setup SQI transfer thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 24);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 24);

    //BD_BUFFER1_ADDR address pointer (Base buffer descriptor)
    PLIB_SQI_DMABDBaseAddressSet(SQI_ID_0, (void *) (KVA_TO_PA(&sqiDescCommand1)));

    //Configure DMA mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_DMA);

    // Enable and start DMA
    PLIB_SQI_DMAEnable(SQI_ID_0);
    PLIB_SQI_DMABDFetchStart(SQI_ID_0);

    PLIB_SQI_DMADisable(SQI_ID_0);
 
    /* wait for the process to complete...*/
    for(i = 0; i < pageCount; i++)
    {
        APP_StartCoreTimer(0);
        APP_CoreTimer_Delay(2600);
    }

    SYS_DEVCON_DataCacheInvalidate((uint32_t) baseAddress, count);

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI PIO Write
// *****************************************************************************
// *****************************************************************************
void SQI_PIO_PageWrite(uint32_t devAddress, uint8_t* pageBuffer)
{
    uint32_t writeLoop, bufLoop;
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t* wordBuffer;
    uint32_t txWord;

    Nop();
    
    wordBuffer = (uint32_t*) pageBuffer;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (devAddress >> 16);
    tempAddress2 = (uint8_t) (devAddress >> 8);
    tempAddress3 = (uint8_t) devAddress;
    devAddress = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);

    /* write the control words and WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* WEN */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_WEN << 24) | 0x00000000);      // wen = 0x06

    /* write the command and address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  /* Page write */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 3);  /* address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 0x100);  /* data */
    
    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, devAddress << 8 | SST26VF_PAGE_WRITE);   // page_write = 0x02

    for(writeLoop = 0; writeLoop < 16; writeLoop++)
    {
        APP_StartCoreTimer(0);
        APP_CoreTimer_Delay(400);
        while (!(PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_TXTHR)));
        for(bufLoop = 0; bufLoop < 4; bufLoop++)
        {
            txWord = *wordBuffer++;
            PLIB_SQI_TransmitData(SQI_ID_0, txWord);
        }
    }
    
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(180000);   // was 10000000 
}

/* write a single page of data using DMA */
void SQI_DMA_PageWrite(uint32_t deviceAddress, uint8_t* pageBuffer)
{
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    Nop();

    /************************************************************************************/
    /* Prepare the buffer descriptors */
    memset(sqiCmdBuffer, 0, sizeof(sqiCmdBuffer));

    sqiDescBuffer[0].BDCon = 0xD08D0100;
    sqiDescBuffer[0].BDStat = 0;
    sqiDescBuffer[0].BDAddr = (unsigned int*) KVA_TO_PA(pageBuffer);
    sqiDescBuffer[0].nextBDAddr = 0x00000000;

    /* output the read address and fast read command in PIO mode */
    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (deviceAddress >> 16);
    tempAddress2 = (uint8_t) (deviceAddress >> 8);
    tempAddress3 = (uint8_t) deviceAddress;
    deviceAddress = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    /* turn on an output pin to indicate the start of the write operation */
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);

    /* write the control words and WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1); /* WEN */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_WEN << 24) | 0x00000000);

    /* write the command and address */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 4);  /* page_write + device address */

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, (deviceAddress << 8) | SST26VF_PAGE_WRITE);

    // Enable BDDONE flag
    PLIB_SQI_InterruptEnable(SQI_ID_0, SQI_BDDONE); // writes to SQI1INTEN

    // Setup SQI transfer thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 24);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 24);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 24);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 24);

    //BD_BUFFER1_ADDR address pointer (Base buffer descriptor)
    PLIB_SQI_DMABDBaseAddressSet(SQI_ID_0, (void *) (KVA_TO_PA(&sqiDescBuffer[0])));

    //Configure DMA mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_DMA);

    // Enable and start DMA
    PLIB_SQI_DMAEnable(SQI_ID_0);
    PLIB_SQI_DMABDFetchStart(SQI_ID_0);

    //Check to see if BD is finished
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_BDDONE) == false);

    PLIB_SQI_DMADisable(SQI_ID_0);

    /* delay 1.6ms to allow write to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(160000);

    /* turn off an output pin to indicate the end of the write operation */
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash Setup
// *****************************************************************************
// *****************************************************************************
int SQI_FlashID_Read(void)
{
    uint32_t JedecID;
    uint32_t b1;

    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    PLIB_SQI_InterruptSignalEnable(SQI_ID_0, SQI_RXTHR);

    // Configure to send the Read Quad ID
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_RECEIVE, 4);

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_QJID << 24) | 0x00000000);

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(40000);

    /* wait until transmission finished */
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);
    b1 = PLIB_SQI_ReceiveData(SQI_ID_0);
    /* align data */
    JedecID = b1 & 0x00FFFFFF;

    PLIB_SQI_InterruptSignalDisable(SQI_ID_0, SQI_RXTHR);

    if (JedecID == SST26VF_JEDECID)
        return true;
    else
        return false;
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash Setup
// *****************************************************************************
// *****************************************************************************
void SQI_Flash_Setup(void)
{
    Nop();
    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    // Reset the SQI
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_TransmitData(SQI_ID_0, 0xFFFFFFFF);

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000); // delay 50ms
    Nop();

    // configure the SQI for quad mode
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_EQIO << 24) | 0x00000000); // 0x38

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(400000); // delay 40ms
    Nop();

    /* send block unprotect command */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* BLKUP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 8);     /* block unprotect bits */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_BLKUP << 24) | (SST26VF_WEN << 16) | 0x00000000);        // BLKUP = 0x42
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);

    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 2);     /* 2 more bytes of data */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
}

/* unprotect and erase the entire device */
void SQI_Flash_Erase(void)
{
    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);
    
    /* set thresholds */
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    /* send commands */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* ERASE */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_ERASE << 24) | (SST26VF_WEN << 16) | 0x00000000); // WEN = 0x06, ERASE = 0xC7

    /* wait for 50ms for the erase to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000); // delay 50ms

    /* send block unprotect command */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* BLKUP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 8);     /* block unprotect bits */

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_BLKUP << 24) | (SST26VF_WEN << 16) | 0x00000000);        // BLKUP = 0x42
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);

    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 2);     /* 2 more bytes of data */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
}

uint8_t SQI_Flash_ReadStatus(void)
{
    uint32_t res;

    /* turn on an output pin to indicate the start of the operation */
    //PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);

    /* configure for PIO mode */
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_PIO);

    // Setup SQI FIFO Thresholds
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    // Configure to send the Read Quad ID
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // NOP
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // EQIO
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);  // RDSR
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_RECEIVE, 4);

    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF_RDSR << 24) | (SST26VF_EQIO << 16) | 0x00000000);

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(40000);

    /* wait until transmission finished */
    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);
    res = PLIB_SQI_ReceiveData(SQI_ID_0);

    /* turn off an output pin to indicate the end of the operation */
    //PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_0);

    return ((uint8_t) res);
}

/*******************************************************************************
End of File
 */
