/*******************************************************************************
  SQI definitions and functions

  File Name:
    sqi.h

  Summary:
    SQI definitions and prototypes

  Description:
    This file contains the SQI definitions and prototypes.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#include "app.h"

#ifndef _SQI_HEADER_H
#define _SQI_HEADER_H

#define MAX_WRITE_BUF_DEPTH             16
#define MAX_READ_BUF_DEPTH              8
#define WRITE_BUF_ADDR                  0xA0060000
#define PIO_READ_BUF_ADDR               0xA0060200
#define SQI_TXBUF_ADDR                  0xBF8E2024
#define SQI_RXBUF_ADDR                  0xBF8E2028
#define MAX_BUF_DEPTH                   8

#define FLASH_PAGE_ADDR                 0x00100000

#define SST26VF032_ERASE                0xC7
#define SST26VF032_EQIO                 0x38
#define SST26VF032_WEN                  0x06
#define SST26VF032_NOP                  0
#define SST26VF032_BLKUP                0x42
#define SST26VF032_PAGEWRITE            0x02
#define SST26VF032_FASTREAD             0x0B
#define SST26VF032_QJID                 0xAF
#define SST26VF032_JEDECID              0xBF0226BF
#define SST26VF032B_ERASE               0xC7
#define SST26VF032B_EQIO                0x38
#define SST26VF032B_WEN                 0x06
#define SST26VF032B_NOP                 0
#define SST26VF032B_BLKUP               0x42
#define SST26VF032B_PAGEWRITE           0x02
#define SST26VF032B_FASTREAD            0x0B
#define SST26VF032B_QJID                0xAF
#define SST26VF032B_JEDECID             0xBF4226BF
#define SST26VF032B_GLOBAL_BLKUP        0x98

// *****************************************************************************
/* SQI DMA descriptor

  Summary:
    Holds DMA descriptor data

  Description:
    This structure holds the DMA descriptor data.

  Remarks:
    None.
 */
typedef struct
{
    // Buffer Descriptor Control Word
    unsigned int BDCon;

    // Buffer Descriptor Status Word - reserved.
    unsigned int BDStat;

    // Buffer Address.
    unsigned int *BDAddr;

    // Next Buffer Descriptor Address Pointer
    struct sqiDMADesc *nextBDAddr;

} sqiDMADesc;

// *****************************************************************************
/* USART Client-Specific Driver Status

  Summary:
    Defines the client-specific status of the USART driver.

  Description:
    This enumeration defines the client-specific status codes of the USART
    driver.

  Remarks:
    Returned by the DRV_USART_ClientStatus function.
*/

typedef enum
{
    /* An error has occurred.*/
    SQI_STATUS_FAULIRE = 0,

    /* No errors occured*/
    SQI_STATUS_SUCCESS = 1,

} SQI_STATUS;

/*******************************************************************************
  Function:
    void SQI_Flash_Setup ( void )

  Summary:
    Serial Flash (in this case SST26VF032 plugged in to Starter Kit) setup routine

  Description:
    This routine Setsup SQI Flash for transactions.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_Flash_Setup();
    </code>

  Remarks:
    None.
*/
void SQI_Flash_Setup (void);
void SQI_Flash_Erase (void);

/*******************************************************************************
  Function:
    int SQI_FlashID_Read (void)

  Summary:
    Serial Flash (in this case SST26VF032 plugged in to Starter Kit) ID read routine

  Description:
    This routine reads Serial Flash ID.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_Flash_Setup();
    </code>

  Remarks:
    None.
*/
int SQI_FlashID_Read (void);

/*******************************************************************************
  Function:
    void SQI_PIO_PageWrite ( uint32_t pageAddress )

  Summary:
    Serial Flash PIO mode page write routine

  Description:
    This routine is used to execute serial flash page writes in PIO (CPU assisted)
    mode.

  Precondition:
    SQI_Initialize and SQI_SerialFlash_Setup routines must be called before
    calling this routine.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_PIO_PageWrite(address);
    </code>

  Remarks:
    None.
*/
void SQI_PIO_PageWrite (uint32_t address);
void SQI_PIO_PageWrite_test (uint32_t address);

/*******************************************************************************
  Function:
    void SQI_PIO_Read( uint32_t address )
 
  Summary:
    Serial Flash PIO mode page read routine

  Description:
    This routine is used to execute serial flash reads in PIO mode

  Precondition:
    SQI_Initialize and SQI_SerialFlash_Setup routines must be called before
    calling this routine.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_PIO_Read(address);
    </code>

  Remarks:
    Make sure Flash has valid content before read.
*/
int SQI_PIO_Read (uint32_t address);
int SQI_PIO_Read_test (uint32_t address);

/*******************************************************************************
  Function:
    void SQI_XIP_Read( uint32_t address )

  Summary:
    Serial Flash XIP mode page read routine

  Description:
    This routine is used to execute serial flash reads in XIP mode

  Precondition:
    SQI_Initialize and SQI_SerialFlash_Setup routines must be called before
    calling this routine.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_XIP_Read(address);
    </code>

  Remarks:
    Make sure Flash has valid content before read.
*/
int SQI_XIP_Read (uint32_t address);

/*******************************************************************************
  Function:
    void SQI_DMA_Read( uint32_t address )

  Summary:
    Serial Flash DMA mode page read routine

  Description:
    This routine is used to execute serial flash reads in DMA mode

  Precondition:
    SQI_Initialize and SQI_SerialFlash_Setup routines must be called before
    calling this routine.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    SQI_DMA_Read(address);
    </code>

  Remarks:
    Make sure Flash has valid content before read.
*/
int SQI_DMA_Read (uint32_t address);

void SQI_Flash_Erase(void);
void SQI_XIP_Mode(void);

#include <stdint.h>

#define DATA_RECORD         0
#define END_OF_FILE_RECORD  1
#define EXT_SEG_ADRS_RECORD 2
#define EXT_LIN_ADRS_RECORD 4

char APP_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen);
void APP_FlashErase(void);
void APP_NVMClearError(void);

/* NVM Flash Memory programming
 * key 1*/

#define NVM_PROGRAM_UNLOCK_KEY1     0xAA996655

/* NVM Driver Flash Memory programming
 * key 1*/

#define NVM_PROGRAM_UNLOCK_KEY2     0x556699AA

#endif /* _SQI_HEADER_H */
/*******************************************************************************
 End of File
*/