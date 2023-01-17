/*******************************************************************************
  SQI functions

  File Name:
    sqi_sst26vf032b.c

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
#include "app.h"
#include "sqi.h"
#include "peripheral/sqi/plib_sqi.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************
// Aligned(16) attribute on the JedecID variable is added to make sure it is allocated
// at the CACHE line boundary. If adding a variable, make sure JedecIDGolden variable
// is also updated with aligned(16) attribute to prevent cache issues.
uint32_t __attribute__((coherent, aligned(16))) JedecID;
uint32_t __attribute__((coherent)) JedecIDGolden;
uint32_t sqiOps = 0;

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

    return(timer);
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
// Section: SQI PIO Read
// *****************************************************************************
// *****************************************************************************
int SQI_PIO_Read_test (uint32_t address)
{
    uint32_t readLoop, bufLoop, checkLoop;
    uint32_t *readBuf = (uint32_t *) PIO_READ_BUF_ADDR;
    uint32_t * writeBuf = (uint32_t *) WRITE_BUF_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t errCount = 1;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    // SQI Transfer Configuration
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send FAST READ and address<15:0>
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,3);
    // Setup control word to send address<23:16> and 3 dummy cycles
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,4);

    //Write the command to the transfer buffer {WEN,NOP,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, tempAddress1 << 16 | tempAddress2 << 24 | (SST26VF032B_FASTREAD << 8) | 0x00);

    // Write address and dummy bytes {DUMMY, ADDRESS[3:0]}
    PLIB_SQI_TransmitData(SQI_ID_0, (0x000000 << 8) | tempAddress3);

    // Setup control word to read 256 bytes
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_RECEIVE,0x100);

    // Setup receive buffer threshold
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0,0x18);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0,0x18);

    for (readLoop=0; readLoop < 8; readLoop++)
    {
        while ((PLIB_SQI_RxBufferThresholdGet(SQI_ID_0)& 0xFF) != 0x18);
        for (bufLoop=0; bufLoop < MAX_READ_BUF_DEPTH; bufLoop++){
            *readBuf++ = PLIB_SQI_ReceiveData(SQI_ID_0);
        }
    }

    readBuf = (uint32_t *) PIO_READ_BUF_ADDR;

    for (checkLoop=0; checkLoop <64; checkLoop++){
        if (*readBuf++ != *writeBuf++)
            errCount++;
    }

    return errCount;
}

int SQI_PIO_Read (uint32_t address)
{
    uint32_t readLoop, bufLoop;
    uint32_t *readBuf = (uint32_t *) PIO_READ_BUF_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    // SQI Transfer Configuration
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send FAST READ and address<15:0>
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,3);
    // Setup control word to send address<23:16> and 3 dummy cycles
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,4);

    //Write the command to the transfer buffer {WEN,NOP,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, tempAddress1 << 16 | tempAddress2 << 24 | (SST26VF032B_FASTREAD << 8) | 0x00);

    // Write address and dummy bytes {DUMMY, ADDRESS[3:0]}
    PLIB_SQI_TransmitData(SQI_ID_0, (0x000000 << 8) | tempAddress3);

    // Setup control word to read 256 bytes
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_RECEIVE,0x100);

    // Setup receive buffer threshold
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0,0x18);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0,0x18);

    for (readLoop=0; readLoop < 8; readLoop++)
    {
        while ((PLIB_SQI_RxBufferThresholdGet(SQI_ID_0)& 0xFF) != 0x18);
        for (bufLoop=0; bufLoop < MAX_READ_BUF_DEPTH; bufLoop++){
            *readBuf++ = PLIB_SQI_ReceiveData(SQI_ID_0);
        }
    }

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI PIO Write
// *****************************************************************************
// *****************************************************************************
void SQI_PIO_PageWrite_test (uint32_t address)
{
    uint32_t writeLoop, bufLoop;
    uint8_t writeLoopChar = 0;
    uint8_t * writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;
    uint8_t * txBufChar  = (uint8_t *) SQI_TXBUF_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    // Setup transmit data
    for (writeLoop=0;writeLoop < 256; writeLoop++)
        *writeBufAddrChar++= writeLoopChar++;

    writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    //Setup SQI transmit buffer thresholds
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0,4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0,4);

    // SQI Transfer Configuration
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send WEN command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);

    //Write the command to the transfer buffer {WEN,NOP,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_WEN << 24) | 0x00000000);

    //Start Write
    // Setup control word to send PAGE WRITE command
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send ADDRESS
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,3);
    // Setup control word to send 256 bytes of DATA
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,0x100);

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, address << 8 | SST26VF032B_PAGEWRITE);

    //Write the data to flash
    for (writeLoop=0; writeLoop < 16; writeLoop++){
        while (!(PLIB_SQI_InterruptFlagGet(SQI_ID_0,SQI_TXTHR)));
        for (bufLoop=0; bufLoop < MAX_WRITE_BUF_DEPTH; bufLoop++){
            *txBufChar = *writeBufAddrChar++;          // Next byte of write data
        }
    }

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(180000);
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI PIO Write
// *****************************************************************************
// *****************************************************************************
void SQI_PIO_PageWrite (uint32_t address)
{
    uint32_t writeLoop, bufLoop;
//    uint8_t writeLoopChar = 0;
    uint8_t * writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;
    uint8_t * txBufChar  = (uint8_t *) SQI_TXBUF_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    // Transmit data should already be in buffer at WRITE_BUF_ADDR
    //for (writeLoop=0;writeLoop < 256; writeLoop++)
    //    *writeBufAddrChar++= writeLoopChar++;

    //writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    //Setup SQI transmit buffer thresholds
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0,4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0,4);

    // SQI Transfer Configuration
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send WEN command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);

    //Write the command to the transfer buffer {WEN,NOP,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_WEN << 24) | 0x00000000);

    //Start Write
    // Setup control word to send PAGE WRITE command
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send ADDRESS
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,3);
    // Setup control word to send 256 bytes of DATA
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,0x100);

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, address << 8 | SST26VF032B_PAGEWRITE);

    //Write the data to flash
    for (writeLoop=0; writeLoop < 16; writeLoop++){
        while (!(PLIB_SQI_InterruptFlagGet(SQI_ID_0,SQI_TXTHR)));
        for (bufLoop=0; bufLoop < MAX_WRITE_BUF_DEPTH; bufLoop++){
            *txBufChar = *writeBufAddrChar++;          // Next byte of write data
        }
    }

    sqiOps++;
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(180000);
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash ID Read
// *****************************************************************************
// *****************************************************************************
int SQI_FlashID_Read (void)
{
    JedecIDGolden = SST26VF032B_JEDECID;

    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send QJID command
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,2);
    // Setup control word to read the JEDEC ID
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_RECEIVE,4);

    //Write the command to the transfer buffer {WEN,EQIO,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_QJID << 16) | 0x00000000);

    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);

    JedecID = PLIB_SQI_ReceiveData(SQI_ID_0);

    if (JedecID != JedecIDGolden)
        return false;
    else
        return true;

}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash Setup
// *****************************************************************************
// *****************************************************************************
void SQI_Flash_Setup (void)
{
    /* Setup CON, TX and RX buffer thresholds */
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    /* Setup control buffer to send block un-protect command */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);   /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);   /* EQIO */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* GBLKUP */

    /* Send necessary commands to un-protect the flash blocks*/
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_GLOBAL_BLKUP << 24) | (SST26VF032B_WEN << 16) | (SST26VF032B_EQIO << 8) | 0x00000000);

    /* Setup control buffer to set SQI Flash in quad lane mode and  send erase the flash command  */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* EQIO */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* NOP */

    /* Send necessary commands to erase the flash */
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_WEN << 16) |
                         0x00000000);

    /* Wait for 50ms for the erase to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(10000000);
}

void SQI_Flash_Erase(void)
{
    /* Setup control buffer to set SQI Flash in quad lane mode and  send erase the flash command  */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* EQIO */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* WEN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);   /* ERASE */

    /* Send necessary commands to erase the flash */
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032B_ERASE << 24) |
                         (SST26VF032B_WEN << 16) |
                         0x00000000);

    /* Wait for 50ms for the erase to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(10000000);
}

void SQI_XIP_Mode(void)
{
    // Configure XIP Control 1 SFR (quad lane fast read)
    PLIB_SQI_XIPControlWord1Set(SQI_ID_0,DUMMY_BYTES_3,ADDR_BYTES_3,
                                SST26VF032B_FASTREAD,SQI_LANE_QUAD,
                                SQI_LANE_QUAD,SQI_LANE_QUAD,
                                SQI_LANE_QUAD,SQI_LANE_QUAD);

    // Configure XIP Control 2 SFR (chip select 1)
    PLIB_SQI_XIPControlWord2Set(SQI_ID_0,SQI_CS_1,MODE_BYTES_0,0);

    // Set to XIP mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_XIP);
}

bool BootloaderProgramExistsCheck(void)     // Overriding bootloader version
{
    return true;
}

/*******************************************************************************
 End of File
*/
