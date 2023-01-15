/*******************************************************************************
  SD Host Controller Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_local.h

  Summary:
    SD Host Controller driver local declarations, structures and function prototypes.

  Description:
    This file contains the SD Host Controller driver's local declarations and definitions.
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

#ifndef _DRV_SDHC_LOCAL_H
#define _DRV_SDHC_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/sdhc/drv_sdhc.h"
#include "driver/sdhc/src/drv_sdhc_sd.h"
#include "driver/sdhc/src/drv_sdhc_sdio_host.h"
#include "driver/sdhc/src/drv_sdhc_error_types.h"
#include "driver/sdhc/src/drv_sdhc_host.h"
#include "driver/sdhc/src/drv_sdhc_sdio_device.h"
#include "driver/sdhc/src/drv_sdhc_config.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************
/* Helper macros for the driver */

// *****************************************************************************
/* SD Host Controller instance address

  Summary:
    Gets the SD card instance address from its index.

  Description:
    This macro gets the SD card instance address from its index.

  Remarks:
    None.
*/


#define _DRV_SDHC_INSTANCE_GET(object)        &gDrvSDHCObj[object]

// *****************************************************************************
/* SDHC Driver Buffer Handle Macros

  Summary:
    SDHC driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SDHC driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define _DRV_SDHC_BUF_TOKEN_MAX         (0xFFFF)
#define _DRV_SDHC_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define _DRV_SDHC_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == _DRV_SDHC_BUF_TOKEN_MAX) ? 0: (token); \
}

#define _DRV_SDHC_FLOATING_BUS_TIMEOUT            (1000)
#define _DRV_SDHC_R1B_RESP_TIMEOUT                (100)

#define _DRV_SDHC_INITIALIZATION_RETRY_WAIT_IN_MS (100)
#define _DRV_SDHC_READ_WAIT_TIME_IN_MS            (250)
#define _DRV_SDHC_WRITE_WAIT_TIME_IN_MS           (250)

#define _DRV_SDHC_APP_CMD_RESP_TIMEOUT_IN_MS      (1000)
#define _DRV_SDHC_CSD_TOKEN_TIMEOUT_IN_MS         (1000)

#define _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE   (_DRV_SDHC_MEDIA_BLOCK_SIZE)
#define _DSV_SDHC_DESCRIPTOR_TABLE_SIZE           (1024)

// *****************************************************************************
/* SDHC Read/Write/Erase Region Index Numbers

  Summary:
    SDHC Geometry Table Index definitions.

  Description:
    These constants provide SDHC Geometry Table index definitions.

  Remarks:
    None
*/
#define GEOMETRY_TABLE_READ_ENTRY   (0)
#define GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define GEOMETRY_TABLE_ERASE_ENTRY  (2)

// *****************************************************************************
// *****************************************************************************
// Section: SD Host Controller constants
// *****************************************************************************
// *****************************************************************************
/* Constants used by SD card driver */

// *****************************************************************************
/* No of bytes to be read for SD card CSD.

  Summary:
    Number of bytes to be read to get the SD card CSD.

  Description:
    This macro holds number of bytes to be read to get the SD card CSD.


  Remarks:
    None.
*/

#define _DRV_SDHC_CSD_READ_SIZE                                            19


// *****************************************************************************
/* SD card V2 device type.

  Summary:
    Holds the value to be checked against to state the device type is V2.

  Description:
    This macro holds value to be checked against to state the device type is V2.

  Remarks:
    None.
*/

#define _DRV_SDHC_CHECK_V2_DEVICE                                          0x0040


// *****************************************************************************
/* SD card sector size.

  Summary:
    Holds the SD card sector size.

  Description:
    This macro holds the SD card sector size.

  Remarks:
    None.
*/

#define _DRV_SDHC_MEDIA_BLOCK_SIZE                                         512


// *****************************************************************************
/* SD card transmit bit.

  Summary:
    Holds the SD card transmit bit position.

  Description:
    This macro holds the SD card transmit bit position.

  Remarks:
    None.
*/

#define DRV_SDHC_TRANSMIT_SET												0x40


// *****************************************************************************
/* Write response token bit mask.

  Summary:
    Bit mask to AND with the write token response byte from the media,

  Description:
    This macro holds the bit mask to AND with the write token response byte from
    the media to clear the don't care bits.

  Remarks:
    None.
*/

#define DRV_SDHC_WRITE_RESPONSE_TOKEN_MASK                                0x1F


// *****************************************************************************
/* SD card bad response

  Summary:
    This macro represents a bad SD card response byte.

  Description:
   	This macro represents a bad SD card response byte.

  Remarks:
    None.
*/

#define DRV_SDHC_MMC_BAD_RESPONSE    DRV_SDHC_MMC_FLOATING_BUS


// *****************************************************************************
/* SD card start single block

  Summary:
    This macro represents an SD card start single data block token (used for
    single block writes).

  Description:
   	This macro represents an SD card start single data block token (used for
   	single block writes).

  Remarks:
    None.
*/

#define DRV_SDHC_DATA_START_TOKEN                     0xFE


// *****************************************************************************
/* SD card start multiple blocks

  Summary:
    This macro represents an SD card start multi-block data token (used for
    multi-block writes).

  Description:
   	This macro represents an SD card start multi-block data token (used for
   	multi-block writes).

  Remarks:
    None.
*/

#define DRV_SDHC_DATA_START_MULTI_BLOCK_TOKEN         0xFC


// *****************************************************************************
/* SD card stop transmission

  Summary:
    This macro represents an SD card stop transmission token.  This is used when
    finishing a multi block write sequence.

  Description:
   	This macro represents an SD card stop transmission token.  This is used when
   	finishing a multi block write sequence.

  Remarks:
    None.
*/

#define DRV_SDHC_DATA_STOP_TRAN_TOKEN                 0xFD


// *****************************************************************************
/* SD card data accepted token

  Summary:
    This macro represents an SD card data accepted token.

  Description:
   	This macro represents an SD card data accepted token.

  Remarks:
    None.
*/

#define DRV_SDHC_DATA_ACCEPTED                        0x05


// *****************************************************************************
/* SD card R1 response end bit

  Summary:
    This macro holds a value to check R1 response end bit.

  Description:
   	This macro holds a value to check R1 response end bit.

  Remarks:
    None.
*/

#define CMD_R1_END_BIT_SET          					0x01


// *****************************************************************************
/* SD card initial packet size

  Summary:
    This macro holds SD card initial packet size.

  Description:
   	This macro holds SD card initial packet size.

  Remarks:
    None.
*/

#define DRV_SDHC_PACKET_SIZE      					6


// *****************************************************************************
/* SD card Media initialization array size

  Summary:
    This macro holds SD card Media initialization array size.

  Description:
   	This macro holds SD card Media initialization array size.

  Remarks:
    None.
*/

#define MEDIA_INIT_ARRAY_SIZE       					10


// *****************************************************************************
// *****************************************************************************
// Section: SD Host Controller Enumerations
// *****************************************************************************
// *****************************************************************************

typedef enum {
    TASK_STATE_IDLE,
    TASK_STATE_CARD_STATUS,
    TASK_STATE_CARD_COMMAND
} _DRV_SDHC_TASK_STATE;

// *****************************************************************************
/* SD Host Controller Driver read task Status

  Summary:
    Lists different states of the read task of the SD card driver

  Description:
    This enumeration lists different states of the read task of the SD card
    driver

  Remarks:
    None.
*/

typedef enum
{       
    /* Check if device is attached */
    DRV_SDHC_TASK_CHECK_DEVICE,

    /* If the card is attached, initialize */
    DRV_SDHC_TASK_MEDIA_INIT

} DRV_SDHC_TASK_STATES;

// *****************************************************************************
// *****************************************************************************
// Section: SD Host Controller Enumerations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SD Host Controller Driver read task Status

  Summary:
    Lists different states of the read task of the SD card driver

  Description:
    This enumeration lists different states of the read task of the SD card
    driver

  Remarks:
    None.
*/

typedef enum
{
    /* A device is attached */
    DRV_SDHC_BUFFER_IO_CHECK_DEVICE,

    /* Process queue for Read and write */
    DRV_SDHC_TASK_PROCESS_QUEUE,

	/* Send stop transmission at the end of multiple
	block read command */
    DRV_SDHC_TASK_STOP_TRANSMISSION,

    /* Need to seed CMD55 before sending ACMD6 */
    DRV_SDHC_TASK_PRE_SET_BUS_WIDTH,

    /* ACMD6 sets the card's bus width between 1-bit and 4-bit, only available when the card is unlocked */
    DRV_SDHC_TASK_SET_BUS_WIDTH,

    /* Sets the block length, 
      some what redundant with High Capacity cards because the block size is fixed at 512 bytes */
    DRV_SDHC_TASK_SET_BLOCK_LENGTH,
            
    /* Select card, send card to transfer state */
    DRV_SDHC_TASK_SELECT_CARD,
            
    /* Process read */
    DRV_SDHC_TASK_PROCESS_READ,

    /* Process write */
    DRV_SDHC_TASK_PROCESS_WRITE,

    /* Get the status of the SD card */
    DRV_SDHC_TASK_READ_TRANSFER_BUSY_STATE,

    /* Wait for the start token */
    DRV_SDHC_TASK_READ_NEXT_BUFFER,

    /* Wait for the start token */
    DRV_SDHC_TASK_READ_TRANSFER_COMPLETE,

    /* Get the status of the SD card */
    DRV_SDHC_TASK_WRITE_TRANSFER_BUSY_STATE,

    /* Wait for the start token */
    DRV_SDHC_TASK_WRITE_NEXT_BUFFER,

    /* Wait for the start token */
    DRV_SDHC_TASK_WRITE_TRANSFER_COMPLETE,

    /* Check for the data read completion */
    DRV_SDHC_TASK_SEND_CARD_TO_STANDBY,

    /* SD card write is complete */
    DRV_SDHC_PROCESS_NEXT,

    /* Read aborted, handle it */
    DRV_SDHC_TASK_READ_ABORT,

    /* Something went wrong on write */
    DRV_SDHC_TASK_WRITE_ABORT

} DRV_SDHC_BUFFER_IO_TASK_STATES;
// *****************************************************************************
/* SD Host Controller connection states

  Summary:
    Lists SD card physical connection states

  Description:
    This enumeration lists different SD card physical connection states.

  Remarks:
    None.
*/

typedef enum
{
    /* SD Host Controller is attached from the system */
    DRV_SDHC_IS_DETACHED,

    /* SD Host Controller is attached to the system */
    DRV_SDHC_IS_ATTACHED

}DRV_SDHC_ATTACH;

// *****************************************************************************
/* SD Host Controller transfer type

  Summary:
    Specifies whether the SD card transfer is read or write.

  Description:
    This enumeration holds constants to specify whether the SD card transfer
    is read or write.

  Remarks:
    None.
*/

typedef enum
{
    /* SD card transfer is read from the card */
    DRV_SDHC_TRANSFER_READ,

   /* SD card transfer is write to the card */
    DRV_SDHC_TRANSFER_WRITE,

}DRV_SDHC_TRANSFER_TYPE;

// *****************************************************************************
/* SD Host Controller commands

  Summary:
    Lists different commands supported by the SD card.

  Description:
    This enumeration lists different commands supported by the SD card

  Remarks:
    None.
*/

typedef enum
{
    /* Command code to reset the SD card */
    CMD_VALUE_GO_IDLE_STATE = 0,

    /* Command code to initialize the SD card */
    CMD_VALUE_SEND_OP_COND  = 1,

    /* Broadcast command code to get all card IDs */
    CMD_VALUE_ALL_SEND_CID  = 2,

    /* Command card to respond with its RCA, tells it to go to standby state */
    CMD_VALUE_SEND_RCA  = 3,

    /* ACMD6 sets the card's bus width between 1-bit and 4-bit, only available when the card is unlocked */
    CMD_VALUE_SET_BUS_WIDTH  = 6,

    /* Select/Deselect card message, sends the card to transfer state */
    CMD_VALUE_SELECT_CARD  = 7,

    /* This macro defined the command code to check for sector addressing */
    CMD_VALUE_SEND_IF_COND  = 8,

    /* Command code to get the Card Specific Data */
    CMD_VALUE_SEND_CSD      = 9,

    /* Command code to get the Card Information */
    CMD_VALUE_SEND_CID      = 10,

    /* Command code to stop transmission during a multi-block read */
    CMD_VALUE_STOP_TRANSMISSION = 12,

    /* Command code to get the card status information */
    CMD_VALUE_SEND_STATUS       = 13,

    /* Command code to set the block length of the card */
    CMD_VALUE_SET_BLOCKLEN      = 16,

    /* Command code to read one block from the card */
    CMD_VALUE_READ_SINGLE_BLOCK  = 17,

    /* Command code to read multiple blocks from the card */
    CMD_VALUE_READ_MULTI_BLOCK   = 18,

    /* Command code to tell the media how many blocks to pre-erase
     (for faster multi-block writes to follow)
     Note: This is an "application specific" command.  This tells the media how
     many blocks to pre-erase for the subsequent WRITE_MULTI_BLOCK */
    CMD_VALUE_SET_WR_BLK_ERASE_COUNT =  23,

    /* Command code to write one block to the card */
    CMD_VALUE_WRITE_SINGLE_BLOCK  = 24,

    /* Command code to write multiple blocks to the card */
    CMD_VALUE_WRITE_MULTI_BLOCK   = 25,

    /* Command code to set the address of the start of an erase operation */
    CMD_VALUE_TAG_SECTOR_START    = 32,

    /* Command code to set the address of the end of an erase operation */
    CMD_VALUE_TAG_SECTOR_END      = 33,

    /* Command code to erase all previously selected blocks */
    CMD_VALUE_ERASE              =  38,

    /* Command code to initialise an SD card and provide the CSD register value.
    Note: this is an "application specific" command (specific to SD cards)
    and must be preceded by CMD_APP_CMD */
    CMD_VALUE_SD_SEND_OP_COND     = 41,

    /* Command code to begin application specific command inputs */
    CMD_VALUE_APP_CMD             = 55,

    /* Command code to get the OCR register information from the card */
    CMD_VALUE_READ_OCR            = 58,

    /* Command code to disable CRC checking */
    CMD_VALUE_CRC_ON_OFF          = 59,

}DRV_SDHC_COMMAND_VALUE;


// *****************************************************************************
/* SD Host Controller Responses

  Summary:
    Lists different responses to commands by the SD card.

  Description:
    This enumeration lists different command responses by the SD card

  Remarks:
    None.
*/

typedef enum
{
    NO_RESPONSE = 0,
    
    /* R1 type response */
    RESPONSE_R1 = sd_response_r1,

    /* R1b type response */
    RESPONSE_R1b = sd_response_r1b,

    /* R2 type response */
    RESPONSE_R2 = sd_response_r2,

    /* R3 type response */
    RESPONSE_R3 = sd_response_r3,

    /* R4 type response */
    RESPONSE_R4 = sd_response_r4,

    /* R5 type response */
    RESPONSE_R5 = sd_response_r5,

    /* R6 type response */
    RESPONSE_R6 = sd_response_r6,

    /* R7 type response */
    RESPONSE_R7 = sd_response_r7

}DRV_SDHC_RESPONSES;


// *****************************************************************************
/* SD Host Controller data response

  Summary:
    Holds the flags to set whether command is expecting data or not.

  Description:
    This enumeration holds the flags to set whether command is expecting data
    or not.

  Remarks:
    None.
*/

typedef enum
{
    /* No data is expected from the SD Card */
    DRV_SDHC_GET_NODATA,

    /* Data is expected from the SD Card */
    DRV_SDHC_GET_MOREDATA

}DRV_SDHC_DATA_GET;


// *****************************************************************************
/* SYS FS Media information

  Summary:
   TODO - move this to file system common header.

  Description:
    TODO

  Remarks:
    None.
*/

typedef enum
{
    /* No errors */
    SYS_FS_MEDIA_NO_ERROR,

    /* The requested device is not present */
    SYS_FS_MEDIA_DEVICE_NOT_PRESENT,

    /* Cannot initialize media */
    SYS_FS_MEDIA_CANNOT_INITIALIZE

} SYS_FS_MEDIA_ERRORS;


// *****************************************************************************
/* SD Host Controller command indices

  Summary:
    Lists the indices of the commands in the command array.

  Description:
    This enumeration lists the indices of the commands in the command array.
    This useful in retrieving commands from the command array.

  Remarks:
    None.
*/

typedef enum
{
    /* Index of in the CMD_GO_IDLE_STATE command 'Command array' */
    DRV_SDHC_GO_IDLE_STATE,

    /* Index of in the CMD_SEND_OP_COND command 'Command array' */
    DRV_SDHC_SEND_OP_COND,

    /* Index of in the CMD_SEND_IF_COND command 'Command array' */
    DRV_SDHC_SEND_IF_COND,

    /* Index of in the CMD_SEND_CSD command 'Command array' */
    DRV_SDHC_SEND_CSD,

    /* Index of in the CMD_SEND_CID command 'Command array' */
    DRV_SDHC_SEND_CID,

    /* Index of in the CMD_STOP_TRANSMISSION command 'Command array' */
    DRV_SDHC_STOP_TRANSMISSION,

    /* Index of in the CMD_SEND_STATUS command 'Command array' */
    DRV_SDHC_SEND_STATUS,

    /* Index of in the CMD_SET_BLOCKLEN command 'Command array' */
    DRV_SDHC_SET_BLOCKLEN,

    /* Index of in the CMD_READ_SINGLE_BLOCK command 'Command array' */
    DRV_SDHC_READ_SINGLE_BLOCK,

    /* Index of in the CMD_READ_MULTI_BLOCK command 'Command array' */
    DRV_SDHC_READ_MULTI_BLOCK,

    /* Index of in the CMD_WRITE_SINGLE_BLOCK command 'Command array' */
    DRV_SDHC_WRITE_SINGLE_BLOCK,

    /* Index of in the CMD_WRITE_MULTI_BLOCK command 'Command array' */
    DRV_SDHC_WRITE_MULTI_BLOCK,

    /* Index of in the CMD_TAG_SECTOR_START command 'Command array' */
    DRV_SDHC_TAG_SECTOR_START,

    /* Index of in the CMD_TAG_SECTOR_END command 'Command array' */
    DRV_SDHC_TAG_SECTOR_END,

    /* Index of in the CMD_ERASE command 'Command array' */
    DRV_SDHC_ERASE,

    /* Index of in the CMD_APP_CMD command 'Command array' */
    DRV_SDHC_APP_CMD,

    /* Index of in the CMD_READ_OCR command 'Command array' */
    DRV_SDHC_READ_OCR,

    /* Index of in the CMD_CRC_ON_OFF command 'Command array' */
    DRV_SDHC_CRC_ON_OFF,

    /* Index of in the CMD_SD_SEND_OP_COND command 'Command array' */
    DRV_SDHC_SD_SEND_OP_COND,

    /* Index of in the CMD_SET_WR_BLK_ERASE_COUNT command 'Command array' */
    DRV_SDHC_SET_WR_BLK_ERASE_COUNT,

    /* Broadcast command for all cards to send their CIDs */
    DRV_SDHC_ALL_SEND_CID,

    /* Command card to response with its RCA, tell the card to go to standby */
    DRV_SDHC_SEND_RCA,
            
    /* Command card to go to transfer state */
    DRV_SDHC_SELECT_CARD,

    /* ACMD6 sets the card's bus width between 1-bit and 4-bit, only available when the card is unlocked */
    DRV_SDHC_SET_BUS_WIDTH

}DRV_SDHC_COMMANDS;


// *****************************************************************************
/* SD Host Controller type

  Summary:
    Lists different types of SD cards.

  Description:
    This enumeration lists different types of SD cards.

  Remarks:
    None.
*/

typedef enum
{
    /* Normal SD Host Controller */
    DRV_SDHC_MODE_NORMAL,

    /* SDHC type Card */
    DRV_SDHC_MODE_HC

}DRV_SDHC_TYPE;


// *****************************************************************************
/* SD Host Controller initialization states

  Summary:
    Lists various initialization stages of SD card.

  Description:
    This enumeration lists various initialization stages of SD card.

  Remarks:
    None.
*/

typedef enum
{
    /* Initial state */
    DRV_SDHC_CMD_DETECT_START_INIT,

    /* Send the CMD0 command so the card will go to idle */
    DRV_SDHC_CMD_DETECT_GO_TO_IDLE,

    /* Wait some time after CMD0 to wait for the card to go idle */
    DRV_SDHC_CMD_DETECT_WAIT_FOR_IDLE,

    /* Card detected, in idle state */
    DRV_SDHC_CMD_DETECT_IDLE,

    /* Dummy read/write to put the card in the known state */
    DRV_SDHC_CMD_DETECT_READ_DATA,

    /* State to check if the dummy read/write is complete */
    DRV_SDHC_CMD_DETECT_READ_DATA_DONE,

    /* Reset the SD card */
    DRV_SDHC_CMD_DETECT_RESET_SDHC,

    /* Placeholder state until the MediaInitialize function indicates if the card
    is really present or not. */
    DRV_SDHC_CMD_DETECT_INIT_DUMMY_STATE,

    /* Check whether the card has been detached. */
    DRV_SDHC_CMD_DETECT_CHECK_FOR_DETACH

} DRV_SDHC_CMD_DETECT_STATES;


// *****************************************************************************
/* SD Host Controller command stages

  Summary:
    Lists various stages while sending a command to SD card.

  Description:
    This enumeration lists various stages while sending a command to SD card.

  Remarks:
    None.
*/

typedef enum
{
    DRV_SDHC_CMD_FRAME_PACKET,
    
    /* Send the framed command packet. */
    DRV_SDHC_CMD_SEND_PACKET,

    /* Check whether the command processing is complete */
    DRV_SDHC_CMD_CHECK_TRANSFER_COMPLETE,

    /* Command execution is complete */
    DRV_SDHC_CMD_EXEC_IS_COMPLETE,

    /* Command retry */
    DRV_SDHC_CMD_RETRY
            
}DRV_SDHC_CMD_STATES;


// *****************************************************************************
/* SD Host Controller initialization stages

  Summary:
    Lists various stages while initializing the SD card.

  Description:
    This enumeration lists various stages while initializing the SD card.

  Remarks:
    None.
*/

typedef enum
{
    DRV_SDHC_INIT_MEDIA,

    /* Reset the SD Host Controller using CMD0 */
    DRV_SDHC_INIT_RESET_SDHC,

    /* Check the card's interface condition */
    DRV_SDHC_INIT_CHK_IFACE_CONDITION,

    /* Send APP CMD */
    DRV_SDHC_INIT_SEND_APP_CMD,

    /* Send APP CMD ACMD41 to check if the card's internal init is done */
    DRV_SDHC_INIT_SEND_ACMD41,

    /* Retry ACMD41 while waiting for card to go to ready state */
    DRV_SDHC_INIT_ACMD41_RETRY_IDLE,
            
    /* Read the Card's Operating Conditions Register */
    DRV_SDHC_INIT_ALL_SEND_CID,

    /* Issue command to response with its RCA, tell the card to go to standby */
    DRV_SDHC_INIT_SEND_RCA,

    /* Idle state to wait for card to go to standby */
    DRV_SDHC_INIT_WAIT_FOR_CARD_TO_STANDBY,
            
    /* Issue command to read the card's Card Specific Data register */
    DRV_SDHC_INIT_READ_CSD,

    /* SD Host Controller Init is done */
    DRV_SDHC_INIT_SD_INIT_DONE,

    /* Error initializing the SD card */
    DRV_SDHC_INIT_ERROR

} DRV_SDHC_INIT_STATE;

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

typedef uintptr_t 									DRV_SDHC_CLIENT_OBJ_HANDLE;
typedef uintptr_t 									DRV_SDHC_OBJ_HANDLE;
typedef uintptr_t 									DRV_SDHC_QUEUE_HANDLE;


// *****************************************************************************
// *****************************************************************************
// Section: Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SD Host Controller transfer object

  Summary:
    This structure holds the elements of the SD card transfer queue.

  Description:
    This structure holds the elements of the SD card transfer queue. The queue
    will be processed by the SD card task and the elements to the queue will be
    added from 'sector read' or 'sector write' functions. So this queue must hold
    all the information about the request.

  Remarks:
    None.
*/

typedef struct
{
    bool                        inUse;

    /* Client that owns this buffer */
    DRV_HANDLE                  hClient;

    /* Present status of this command */
    DRV_SDHC_COMMAND_STATUS   status;

    /* Current command handle of this buffer object */
    DRV_SDHC_COMMAND_HANDLE   commandHandle;
    
    /* Buffer to which the data to written or read from */
    uint8_t 				*buffer;
    
	/* read/write flag */
    DRV_SDHC_TRANSFER_TYPE 	readWrite;

	/* Sector start address */
    uint32_t 					sectorAddress;

	/* Number of sectors  */
	uint32_t 					sectorCount;

    uint16_t                    blockIncr;

    uint8_t                     command;
    
    uint16_t                    bufferIncr;
}DRV_SDHC_XFER_OBJECT;


// *****************************************************************************
/* SD Host Controller queue object

  Summary:
    SD Host Controller queue elements.

  Description:
   	This structure holds the variables necessary to handle the SD card Queue.

  Remarks:
    None.
*/

typedef struct
{
	/* Queue start index */
	unsigned char                	startIndex;

	/* Queue end index */
	unsigned char                   endIndex;

	/* Queue Link to the actual buffers */
	DRV_SDHC_XFER_OBJECT          *bufferPool;

	/* Size of the queue */
	unsigned char                	size;

}DRV_SDHC_QUEUE_OBJECT;


// *****************************************************************************
/* SD Host Controller Command data structure

  Summary:
   Holds different commands are the expected response for those commands.

  Description:
    This enumeration the commands supported by the SD card, its CRC and response
    corresponding to each command.

  Remarks:
    None.
*/

typedef struct
{
    /* Command code */
    DRV_SDHC_COMMAND_VALUE      commandCode;

    /* Response type */
    DRV_SDHC_RESPONSES    responseType;

    /* Whether more data is expected or not */
    DRV_SDHC_DATA_GET    moredataexpected;

} DRV_SDHC_CMD_OBJ;


// *****************************************************************************
/* SD Host Controller Command data structure

  Summary:
   Holds different commands ard the expected response for those commands.

  Description:
    This enumeration the commands supported by the SD card, its CRC and response
    corresponding to each command.

  Remarks:
    None.
*/

typedef struct
{
    uint8_t *buffer;

    DRV_SDHC_TRANSFER_TYPE readWrite;

    uint32_t sectorSize;

    uint32_t sectorAddress;

    uint32_t sectorCount;

    uint32_t bytesRemaining;

    uint32_t bytesToRead;

    uint16_t blockIncr;

    uint8_t command;

    DRV_SDHC_COMMAND_STATUS *status;

}DRV_SDHC_TASK_OPERATIONS;


// *****************************************************************************
/* Sys FS media information

  Summary:
   Sys FS media information.

  Description:
    Sys FS media information.

  Remarks:
    None.
*/

typedef struct
{
    uint8_t    errorCode;
    union
    {
        uint8_t    value;
        struct
        {
            uint8_t    sectorSize  : 1;
            uint8_t    maxLun      : 1;
        }   bits;
    } validityFlags;

    uint16_t    sectorSize;
    uint8_t    maxLun;

} SYS_FS_MEDIA_INFORMATION;


// *****************************************************************************
/* SD Host Controller Command packet

  Summary:
    Different commands ard the expected response for those commands.

  Description:
    This union represents different ways to access an SD card command packet.

  Remarks:
    None.
*/

typedef union
{
    /* This structure allows array-style access of command bytes */
    struct
    {
        uint8_t field[7];
    };

    /* This structure allows byte-wise access of packet command bytes */
    struct
    {
        /* The CRC byte */
        uint8_t crc;

        /* Filler space (since bitwise declarations can't
         cross a uint32_t boundary) */
        uint8_t c32filler[3];

        /* Address byte 0 */
        uint8_t address0;

        /* Address byte 1 */
        uint8_t address1;

        /* Address byte 2 */
        uint8_t address2;

        /* Address byte 3 */
        uint8_t address3;

        /* Command code byte */
        uint8_t cmd;
    };

    /* This structure allows bitwise access to elements of the command bytes */
    struct
    {
        /* Packet end bit */
        uint8_t  endBit:1;

        /* CRC value */
        uint8_t  crc7:7;

        /* Address */
        uint32_t    address;

        /* Command code */
        uint8_t  cmdIndex:6;

        /* Transmit bit */
        uint8_t  transmitBit:1;

        /* Packet start bit */
        uint8_t  startBit:1;
    };
} DRV_SDHC_CMD_PACKET;


// *****************************************************************************
/* SD Host Controller R1 type response format

  Summary:
    Different ways to access R1 type response packet.

  Description:
    This union represents different ways to access an SD card R1 type response
    packet.

  Remarks:
    None.
*/

typedef struct
{
    union
    {
        uint16_t word0;

        /* This structure allows bitwise access of the response */
        struct // 0 : 15
        {
            unsigned reserved0 : 1; //reserved for SDIO
            unsigned reserved1 : 1; //reserved for SDIO
            unsigned reserved2 : 1; //reserved for SDIO

            unsigned akeSeqErr : 1;

            unsigned reserved4 : 1; //reserved for SDIO

            unsigned appCmdEnable : 1;

            unsigned reserved6 : 1;
            unsigned reserved7 : 1;

            unsigned readyForData : 1;

            unsigned cardState0 : 1;
            unsigned cardState1 : 1;
            unsigned cardState2 : 1;
            unsigned cardState3 : 1;

            unsigned eraseReset : 1;
            unsigned cardEccDisable : 1;
            unsigned wpEraseSkip : 1;
        };

    };

    
    union
    {
        uint16_t word1;

        /* This structure allows bitwise access of the response */
        struct // 16 : 32
        {
            unsigned csdOverwrite : 1; //bit 16

            unsigned reserved17 : 1;
            unsigned reserved18 : 1;

            unsigned generalErr : 1; //bit 19
            unsigned ccErr : 1;
            unsigned cardECCFailed : 1;
            unsigned illegalCmdErr: 1; //bit 22 of Card Status
            unsigned cmdCRCErr : 1; //bit 23 of Card Status
            unsigned lockUnlockFail : 1;  
            unsigned cardIsLocked : 1;
            unsigned wpViolation : 1;

            unsigned eraseParam : 1;
            unsigned eraseSeqErr : 1;

            unsigned blockLenErr : 1;
            unsigned addressErr : 1;
            unsigned outOfRange : 1;
        };
    };

} DRV_SDHC_RESPONSE_1;


// *****************************************************************************
/* SD Host Controller R2 type response format

  Summary:
    Different ways to access R2 type response packet.

  Description:
    This union represents different ways to access an SD card R2 type response
    packet.

  Remarks:
    None.
*/

typedef struct
{
    uint16_t word0;
    uint16_t word1;
    uint16_t word2;
    uint16_t word3;
    uint16_t word4;
    uint16_t word5;
    uint16_t word6;
    uint16_t word7;    
} DRV_SDHC_RESPONSE_2;


// *****************************************************************************
/* SD Host Controller R6 type response format

  Summary:
    Different ways to access R6 type response packet.

  Description:
    This struct represents different ways to access an SD card R6 type response
    packet.

  Remarks:
    None.
*/

typedef struct
{
    union
    {
        uint16_t word0;
        
        struct// 0  : 15
        {
            unsigned reserved0 : 1; //reserved for SDIO
            unsigned reserved1 : 1; //reserved for SDIO
            unsigned reserved2 : 1; //reserved for SDIO
            unsigned reserved3 : 1; //reserved for SDIO
            unsigned reserved4 : 1; //reserved for SDIO
            unsigned appCmdEnable : 1;
            unsigned reserved6 : 1;
            unsigned reserved7 : 1;
            unsigned readyForData : 1;
            unsigned cardState0 : 1;
            unsigned cardState1 : 1;
            unsigned cardState2 : 1;
            unsigned cardState3 : 1;
            unsigned generalErr : 1; //bit 19
            unsigned illegalCmdErr: 1; //bit 22 of Card Status
            unsigned cmdCRCErr : 1; //bit 23 of Card Status
        };        
    };
    
    uint16_t rca; // 16 : 31
    
} DRV_SDHC_RESPONSE_6;

// *****************************************************************************
/* SD Host Controller R7 and R3 type response format

  Summary:
    Different ways to access R3/R7 type response packet.

  Description:
    This union represents different ways to access an SD card R3/R7 type
    response packet.

  Remarks:
    Note: The SD card argument response field is 32-bit, big endian format.
    However, the C compiler stores 32-bit values in little endian in RAM. When
    writing to the _returnVal/argument bytes, make sure to byte swap the order
    from which it arrived from the SD card.
*/

typedef struct
{
    union
    {
        uint16_t word0;

        struct
        {
            uint8_t echoback;
            uint8_t voltages;
        };
    };

    uint16_t word1;
    uint16_t word2;
    
} DRV_SDHC_RESPONSE_7;


// *****************************************************************************
/* SD Host Controller Responses

  Summary:
    Different SD card response packets.

  Description:
    This an union of SD card response packets.

  Remarks:
    None.
*/

typedef union
{
    /* SD Host Controller response 1 */
    DRV_SDHC_RESPONSE_1  response1;

    /* SD Host Controller response 6 */
    DRV_SDHC_RESPONSE_6  response6;

    /* SD Host Controller response 2 */
    DRV_SDHC_RESPONSE_2  response2;

    /* SD Host Controller response 7 */
    DRV_SDHC_RESPONSE_7  response7;

}DRV_SDHC_RESPONSE_PACKETS;


// *****************************************************************************
/* SD Host Controller data register

  Summary:
    A description of the card specific data register.

  Description:
    This union represents different ways to access information in a packet with
    SD card CSD information. For more information on the CSD register, refer
    SD card user's manual.

  Remarks:
    None.
*/

typedef union
{
    struct
    {
        /* Access 32 bit format 1st */
        uint32_t access32_0;

        /* Access 32 bit format 2nd */
        uint32_t access32_1;

        /* Access 32 bit format 3rd */
        uint32_t access32_2;

        /* Access 32 bit format 4th */
        uint32_t access32_3;
    };
    struct
    {
        /* Access as bytes */
        uint8_t byte[16];
    };

    struct
    {
        /* Un used bit */
        unsigned unUsed           :1;

        /* Crc */
        unsigned crc                :7;

        /* Ecc */
        unsigned ecc                :2;

        /* File format */
        unsigned fileFormat        :2;

        /* temporary write protection */
        unsigned tempWriteProtect  :1;

        /* Permanent write protection */
        unsigned permanantWriteProtect :1;

        /* Copy flag */
        unsigned copyFlag           :1;

        /* File format group */
        unsigned fileFormatGroup    :1;

        /* Reserved bits */
        unsigned reserved_1         :5;

        /* Partial blocks for write allowed */
        unsigned writeBlockPartial   :1;

        /* Max. write data block length high */
        unsigned writeBlockLenHigh    :2;

        /* Max. write data block length low */
        unsigned writeBlockLenLow     :2;

        /* write speed factor */
        unsigned writeSpeedFactor     :3;

        /* default Ecc */
        unsigned defaultEcc            :2;

        /* Write protect group enable */
        unsigned writeProtectGrpEnable :1;

        /* Write protect group size */
        unsigned writeProtectGrpSize   :5;

        /* Erase sector size low */
        unsigned eraseSectorSizeLow   :3;

        /* Erase sector size high */
        unsigned eraseSectorSizeHigh   :2;

        /* Sector size */
        unsigned sectorSize        :5;

        /* TODO  erase single block enable??*/
        /* Device size multiplier low */
        unsigned deviceSizeMultiLow      :1;

        /* Device size multiplier high */
        unsigned deviceSizeMultiHigh      :2;

        /* Max. write current @VDD max */
        unsigned vddWriteCurrentMax     :3;

        /* max. write current @VDD min */
        unsigned vddWriteCurrentMin      :3;

        /* max. read current @VDD max */
        unsigned vddReadCurrentMax     :3;

        /* max. read current @VDD min */
        unsigned vddReadCurrentMin     :3;

        /* device size low */
        unsigned deviceSizeLow         :2;

        /* device size high */
        unsigned deviceSizeHigh        :8;

        /* device size upper */
        unsigned deviceSizeUp          :2;

        /* reserved bits */
        unsigned reserved2             :2;

        /* DSR implemented */
        unsigned dsrImplemented        :1;

        /* Read block misalignment */
        unsigned readBlockMissAllign  :1;

        /* Write block misalignment */
        unsigned writeBlockMissAllign :1;

        /* Partial blocks for read allowed */
        unsigned partialBlockRead     :1;

        /* Max. read data block length */
        unsigned readDataBlockMax     :4;

        /* Card command classes low */
        unsigned cardCmdClassesLow    :4;

        /* Card command classes high */
        unsigned cardCmdClassesHigh   :8;

        /* max. data transfer rate */
        unsigned maxDataTrasferRate   :8;

        /* Data read access-time-2 in clock cycles (NSAC*100) */
        unsigned dataReadTime2Clocks   :8;

        /* data read access-time-1 */
        unsigned dataReadTime1Clocks   :8;

        /* Reserved bits */
        unsigned reserved3             :2;

        /* Specification version */
        unsigned specVersion           :4;

        /* CSD structure */
        unsigned csdStructure         :2;
    };

} DRV_SDHC_CSD;


// *****************************************************************************
/* SD Host Controller information register

  Summary:
    A description of the card specific information register.

  Description:
    This union represents different ways to access information in a packet with
    SD card CID information. For more information on the CSD register, refer
    SD card user's manual.

  Remarks:
    None.
*/

typedef union
{
    struct
    {
        /* Access 32 bit format 1st */
        uint32_t access32_0;

        /* Access 32 bit format 2nd */
        uint32_t access32_1;

        /* Access 32 bit format 3rd */
        uint32_t access32_2;

        /* Access 32 bit format 4th */
        uint32_t access32_3;
    };
    struct
    {
        uint8_t byte[16];
    };
    struct
    {
        /* Unused bit */
        unsigned    notUsed            :1;

        /* Crc */
        unsigned    crc                 :7;

        /* Manufacturing date */
        unsigned    mnufacturingDate    :8;

        /* Product serial number */
        uint32_t       serialNumber;

        /* Product revision */
        unsigned    revision            :8;

        /* Product name */
        char        name[6];

        /* OEM Application ID */
        uint16_t        oemId;

        /* Manufacturer ID */
        unsigned    manufacturerId     :8;
    };
} DRV_SDHC_CID;


// *****************************************************************************
/* SD Host Controller Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance

  Description:
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None
*/

typedef struct _DRV_SDHC_OBJ_STRUCT
{
    /* Index of the driver */
    SYS_MODULE_INDEX                drvIndex;

    /* The status of the driver */
    SYS_STATUS                      status;

    /* Flag to indicate in use  */
    bool                            inUse;

    /* Flag to indicate that SD Host Controller is used in exclusive access mode */
    bool                            isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                         numClients;

    /* Other members to support driver operations */
    /* Sector size of the SD Host Controller */
    uint32_t                        mediaSectorSize;

    /* Number of sectors in the SD card */
    uint32_t                        discCapacity;

    /* Device is attached */
    DRV_SDHC_ATTACH               isAttached;

    /* Device is attached last status */
    DRV_SDHC_ATTACH               isAttachedLastStatus;

    /* SD card type, will be updated by initialization function */
    DRV_SDHC_TYPE                 sdCardType;

    /* Write protect state */
    char                            isWriteProtected;

    /* Card detect pin setting */
    PORTS_CHANNEL                   cardDetectPort;

    /* Position of the bit in the port selected for card detection */
    PORTS_BIT_POS                   cardDetectBitPosition;

    /* Update in initialization */
    DRV_SDHC_QUEUE_HANDLE         queueHandle;

    /* Write protect pin setting */
    PORTS_CHANNEL                   writeProtectPort;

    /* Position of the bit in the port selected for Write protection */
    PORTS_BIT_POS                   writeProtectBitPosition;

    /* Chip select pin setting */
    PORTS_CHANNEL                   chipSelectPort;

    /* Position of the bit in the port selected for chip selection */
    PORTS_BIT_POS                   chipSelectBitPosition;

    /* This variable holds the current state of the DRV_SDHC_Task */
    DRV_SDHC_TASK_STATES          taskState;
    /* This variable holds the current state of the DRV_SDHC_Task */
    DRV_SDHC_BUFFER_IO_TASK_STATES taskBufferIOState;
    /* Different stages of initialization */
    DRV_SDHC_CMD_DETECT_STATES     cmdDetectState;

    /* The command response will be updated here */
    DRV_SDHC_RESPONSE_PACKETS     cmdResponse;

    /* Different states in sending a command */
    DRV_SDHC_CMD_STATES           cmdState;

    /* Different stages in media initialization */
    DRV_SDHC_INIT_STATE           mediaInitState;

    SYS_FS_MEDIA_HANDLE           mediaHandle;
    
    /* Information about the SD card, Will be updated on initialization */
    SYS_FS_MEDIA_INFORMATION        mediaInformation;

    /* Variables used to hadle the operations in the task */
    DRV_SDHC_XFER_OBJECT         *localTaskObj;

	/* Status of the operation */
	SYS_FS_MEDIA_COMMAND_STATUS    *bufferStatus;

    /* Packet which is currently being send ( +1 to handle
     DRV_SDHC_STOP_TRANSMISSION command ) */
    uint8_t                         packetArray [ DRV_SDHC_PACKET_SIZE +1 ];

    /* Index in the packet array */
    uint8_t                         cmdIndex;

    /* Media index , used by mediaInitialize function */
    uint8_t                         mediaInitArray [ MEDIA_INIT_ARRAY_SIZE ];

    /* Data about the disc */
    uint8_t                         mediaInitReadCsd [ _DRV_SDHC_CSD_READ_SIZE ];

    /* Keeps track of the CSD byte read */
    uint8_t                         mediaInitindex;

	/* The disk number allocated by Media Manager */
    int								disk;
    SDHC_DETECTION_LOGIC          sdCardDetectionLogic;
    /* Status of the device */
    SYS_FS_MEDIA_STATUS             mediaState;

    /* System Timer Handle */
    SYS_TMR_HANDLE          sysTmrHandle;

    uint32_t sdHcHost;

    _DRV_SDHC_TASK_STATE  sdState;
        
    SDHC_CARD_STATE     cardState;
    
    bool    isCardLocked;
    
    /* Mutex */
    OSAL_MUTEX_DECLARE(mutex);
    
    drv_sdhc_fuse_settings_t  sdhcFuseSettings;
    
    drv_sdhc_sd_handle_t*  cardHandle;
    
    sd_cmd_t command;

} DRV_SDHC_OBJ;

// *****************************************************************************
/* SD Host Controller Driver Client Object

  Summary:
    Defines the object required for the maintenance of the software clients

  Description:
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None
*/

typedef struct _DRV_SDHC_CLIENT_OBJ_STRUCT
{
    /* Hardware instance index associated with the client */
    DRV_SDHC_OBJ_HANDLE                          driverObject;

    /* Save the index while opening the driver */
    SYS_MODULE_INDEX                               drvIndex;
    /* Flag to indicate in use  */
    bool                                           inUse;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                                  intent;

    /* Client specific event handler */
    DRV_SDHC_EVENT_HANDLER                       eventHandler;

    /* Client specific context */
    uintptr_t                                      context;

} DRV_SDHC_CLIENT_OBJ;

/*****************************************************************************
 * If the SD card needs to be controlled by media manager, then declare the
 * following as 1. Otherwise, declare as 0.
 *
 *****************************************************************************/
#define DRV_SDHC_MEDIA_MANAGER_USE                1

// *****************************************************************************
// *****************************************************************************
// Section: Extern data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*  Hardware Objects for the dynamic driver
*/

extern DRV_SDHC_OBJ            gDrvSDHCObj[];


// *****************************************************************************
/*  Client Objects for the multi-client driver
*/

extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj[];


// *****************************************************************************
/*  Hardware Objects for the static driver
*/

extern DRV_SDHC_OBJ            gDrvSDHCObj0;
extern DRV_SDHC_OBJ            gDrvSDHCObj1;
extern DRV_SDHC_OBJ            gDrvSDHCObj2;
extern DRV_SDHC_OBJ            gDrvSDHCObj3;
extern DRV_SDHC_OBJ            gDrvSDHCObj4;
extern DRV_SDHC_OBJ            gDrvSDHCObj5;
extern DRV_SDHC_OBJ            gDrvSDHCObj6;


// *****************************************************************************
/*  Client Objects for the single-client driver
*/

extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj0;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj1;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj2;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj3;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj4;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj5;
extern DRV_SDHC_CLIENT_OBJ     gDrvSDHCClientObj6;

// *****************************************************************************
// *****************************************************************************
// Section: Function prototypes of local functions
// *****************************************************************************
// *****************************************************************************

void _DRV_SDHC_CommandSend ( SYS_MODULE_OBJ object, DRV_SDHC_COMMANDS command,
                                uint32_t address );
void _DRV_SDHC_MediaInitialize ( SYS_MODULE_OBJ object );

bool _DRV_SDHC_RequestCardSpecificData ( SYS_MODULE_OBJ object );

DRV_SDHC_QUEUE_HANDLE _DRV_SDHC_QueueInitialize( const SYS_MODULE_INDEX    drvIndex );

DRV_SDHC_COMMAND_HANDLE _DRV_SDHC_AddToQueue 
( 
    DRV_SDHC_QUEUE_HANDLE handle,
    DRV_SDHC_TRANSFER_TYPE readWrite,
    uint8_t *buffer,
    uint32_t blockStart,
    uint32_t nBlock
);


bool _DRV_SDHC_MediaCommandDetect 
(
    SYS_MODULE_OBJ object
);

DRV_SDHC_XFER_OBJECT* _DRV_SDHC_ReadFromQueue ( DRV_SDHC_QUEUE_HANDLE handle );

DRV_SDHC_RESPONSE _DRV_SDHC_PrepareAdmaDescriptorTableForTransfer( DRV_SDHC_OBJ *dObj );

#endif //#ifndef _DRV_SDHC_LOCAL_H

/*******************************************************************************
 End of File
*/

