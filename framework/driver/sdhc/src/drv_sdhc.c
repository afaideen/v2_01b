/*******************************************************************************
  SD Host Controller Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc.c

  Summary:
    SD Host Controller Device Driver Dynamic Implementation

  Description:
    The SD Host Controller device driver provides a simple interface to manage the 
    SD Host Controller modules on Microchip microcontrollers.  This file Implements 
    the core interface routines for the SD Host Controller driver.

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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <string.h>
#include <sys/kmem.h>

#include "system_config.h"
#include "system_definitions.h"
#include "osal/osal.h"
#include "driver/sdhc/src/drv_sdhc_local.h"
#include "driver/sdhc/drv_sdhc.h"
#include "driver/sdhc/src/drv_sdhc_sd.h"
#include "driver/sdhc/src/drv_sdhc_sdio_host.h"
#include "driver/sdhc/src/drv_sdhc_error_types.h"
#include "driver/sdhc/src/drv_sdhc_host.h"
#include "driver/sdhc/src/drv_sdhc_sdio_device.h"
#include "driver/sdhc/src/drv_sdhc_config.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

uint8_t gDrvSDHCInitCount = 0;
drv_sdhc_adma_descriptor admaDescriptorTable[_DSV_SDHC_DESCRIPTOR_TABLE_SIZE]  __attribute__((coherent, aligned(32)));
OSAL_MUTEX_DECLARE(gDrvSDHCClientMutex);
volatile bool data_xfer_completed;

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/
uint16_t gDrvSDHCBufferToken = 0;

/*************************************
 * SDHC driver geometry object
 ************************************/
SYS_FS_MEDIA_GEOMETRY gDrvSDHCMediaGeometry;

/*****************************************
 * Media Geomtery Table.
 ****************************************/
SYS_FS_MEDIA_REGION_GEOMETRY gDrvSDHCGeometryTable[3];

// *****************************************************************************
/* SD Host Controller command table

  Summary:
    Defines the a command table for SD card.

  Description:
    This data structure makes a command table for the SD Host Controller with the command,
    its CRC, expected response and a flag indicating whether the driver expects
    more data or not. This makes the SD card commands easier to handle.

  Remarks:
    The actual response for the command 'CMD_SD_SEND_OP_COND'is R3, but it has
    same number of bytes as R7. So R7 is used in the table.
*/

const DRV_SDHC_CMD_OBJ gDrvSDHCCmdTable[] =
{
    /* Command                             response       more data */
    {CMD_VALUE_GO_IDLE_STATE,              NO_RESPONSE,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_OP_COND,               RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_IF_COND,               RESPONSE_R7,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_CSD,                   RESPONSE_R2,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_CID,                   RESPONSE_R2,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_STOP_TRANSMISSION,          RESPONSE_R1b,        DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_STATUS,                RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SET_BLOCKLEN,               RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_READ_SINGLE_BLOCK,          RESPONSE_R1,         DRV_SDHC_GET_MOREDATA},
    {CMD_VALUE_READ_MULTI_BLOCK,           RESPONSE_R1,         DRV_SDHC_GET_MOREDATA},
    {CMD_VALUE_WRITE_SINGLE_BLOCK,         RESPONSE_R1,         DRV_SDHC_GET_MOREDATA},
    {CMD_VALUE_WRITE_MULTI_BLOCK,          RESPONSE_R1,         DRV_SDHC_GET_MOREDATA},
    {CMD_VALUE_TAG_SECTOR_START,           RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_TAG_SECTOR_END,             RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_ERASE,                      RESPONSE_R1b,        DRV_SDHC_GET_NODATA},
    {CMD_VALUE_APP_CMD,                    RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_READ_OCR,                   RESPONSE_R7,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_CRC_ON_OFF,                 RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SD_SEND_OP_COND,            RESPONSE_R3,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SET_WR_BLK_ERASE_COUNT,     RESPONSE_R1,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_ALL_SEND_CID,               RESPONSE_R2,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SEND_RCA,                   RESPONSE_R6,         DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SELECT_CARD,                RESPONSE_R1b,        DRV_SDHC_GET_NODATA},
    {CMD_VALUE_SET_BUS_WIDTH,              RESPONSE_R1,         DRV_SDHC_GET_NODATA},
};

// *****************************************************************************
/* SD card media functions.

  Summary:
    These functions are used by the 'media manager' to access the SD card.

  Description:
	These functions are used by the 'media manager' to access the SD card. The
	call will be by using a function pointer. So SD card driver must attach these
	functions to the media manager on initialize.

  Remarks:
    None.
*/

// *****************************************************************************
/* SD card media functions.

  Summary:
    These functions are used by the 'media manager' to access the SD card.

  Description:
	These functions are used by the 'media manager' to access the SD card. The
	call will be by using a function pointer. So SD card driver must attach these
	functions to the media manager on initialize.

  Remarks:
    None.
*/

const SYS_FS_MEDIA_FUNCTIONS sdhcMediaFunctions =
{
    .mediaStatusGet     = DRV_SDHC_IsAttached,
    .mediaGeometryGet   = DRV_SDHC_GeometryGet,
    .sectorRead         = DRV_SDHC_Read,
    .sectorWrite        = DRV_SDHC_Write,
    .eventHandlerset    = DRV_SDHC_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_SDHC_CommandStatus,
    .open               = DRV_SDHC_Open,
    .close              = DRV_SDHC_Close,
    .tasks              = DRV_SDHC_Tasks
};
// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

DRV_SDHC_OBJ              gDrvSDHCObj[DRV_SDHC_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

DRV_SDHC_CLIENT_OBJ       gDrvSDHCClientObj[DRV_SDHC_CLIENTS_NUMBER * DRV_SDHC_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver buffer objects.

  Summary:
    Transfer objects for the driver queue.

  Description:
    This instance of the structure is used as transfer objects for the driver
    queue.

  Remarks:
    None
*/

DRV_SDHC_XFER_OBJECT      gDrvSDHCTransferObj[DRV_SDHC_INSTANCES_NUMBER][ DRV_SDHC_QUEUE_POOL_SIZE ];


// *****************************************************************************
/* Driver queue object

  Summary:
    Variables to handle the queue.

  Description:
    This instance of the structure holds the variables to handle the queue.

  Remarks:
    None
*/

DRV_SDHC_QUEUE_OBJECT   	gDrvSDHCQueueObj[DRV_SDHC_INSTANCES_NUMBER];


// *****************************************************************************
/* Macro: _DRV_SDHC_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SDHC_CLIENT_OBJ(obj,mem)    gDrvSDHCClientObj[obj].mem


// *****************************************************************************
/* Macro: _DRV_SDHC_CLIENT_OBJ_GET(obj)

  Summary:
    Returns the appropriate client instance

  Description:
    Either return the static object or returns the indexed dynamic object.
    This macro has variations for dynamic or static driver.
*/

#define _DRV_SDHC_CLIENT_OBJ_GET(obj)    &gDrvSDHCClientObj[obj]


// *****************************************************************************
/* Macro: _DRV_SDHC_INDEX_GET(drvIndex)

  Summary:
    Returns the appropriate driver id for the configuration

  Description:
    Either return the statically declared driver id or returns the dynamic index
    passed into the macro. This macro has variations for dynamic or static
    driver

*/

#define _DRV_SDHC_INDEX_GET(drvIndex)                            (drvIndex)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

void _DRV_SDHC_UpdateGeometry
(
    DRV_SDHC_OBJ *dObj
)
{
    uint8_t i = 0;

    /* Update the Media Geometry Table */
    for (i = 0; i <= GEOMETRY_TABLE_ERASE_ENTRY; i++) 
    {
        gDrvSDHCGeometryTable[i].blockSize = _DRV_SDHC_MEDIA_BLOCK_SIZE;
        gDrvSDHCGeometryTable[i].numBlocks = dObj->discCapacity;
    }

    /* Update the Media Geometry Main Structure */
    gDrvSDHCMediaGeometry.mediaProperty = (SYS_FS_MEDIA_READ_IS_BLOCKING | SYS_FS_MEDIA_WRITE_IS_BLOCKING),

    /* Number of read, write and erase entries in the table */
    gDrvSDHCMediaGeometry.numReadRegions = 1,
    gDrvSDHCMediaGeometry.numWriteRegions = 1,
    gDrvSDHCMediaGeometry.numEraseRegions = 1,
    gDrvSDHCMediaGeometry.geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&gDrvSDHCGeometryTable;
}

void _DRV_SDHC_RemoveQueuedRequests 
(
    DRV_SDHC_OBJ *dObj
)
{
    /* The media was earlier attached */
    uint8_t i = 0;
    DRV_SDHC_QUEUE_OBJECT *qObj;
    DRV_SDHC_CLIENT_OBJ   *clientObj;

    if (dObj->mediaState != SYS_FS_MEDIA_ATTACHED)
    {
        return;
    }

    qObj = (DRV_SDHC_QUEUE_OBJECT *)dObj->queueHandle;
    dObj->mediaState = SYS_FS_MEDIA_DETACHED;

    if (!qObj)
    {
        return;
    }

    /* Reset the indexes */
    qObj->startIndex = qObj->endIndex;

    /* The media was attached earlier. */
    for (i = 0; i < qObj->size; i++)
    {
        if (qObj->bufferPool[i].status == DRV_SDHC_COMMAND_QUEUED)
        {
            qObj->bufferPool[i].status = DRV_SDHC_COMMAND_ERROR_UNKNOWN;
            qObj->bufferPool[i].inUse = false;

            clientObj = (DRV_SDHC_CLIENT_OBJ*)_DRV_SDHC_CLIENT_OBJ_GET(qObj->bufferPool[i].hClient);
            if(clientObj->eventHandler != NULL)
            {
                /* Call the event handler */
                clientObj->eventHandler(DRV_SDHC_EVENT_COMMAND_ERROR,
                        (DRV_SDHC_COMMAND_HANDLE)qObj->bufferPool[i].commandHandle, clientObj->context);
            }
        }
    }
}

void _DRV_SDHC_CheckWriteProtectStatus 
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDHC_OBJ *dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    /* Read from the pin */
    dObj->isWriteProtected = _drv_sdhc_is_write_protected(dObj->cardHandle);
}

// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SDHC_Initialize 
     (
         const SYS_MODULE_INDEX index,
         const SYS_MODULE_INIT  * const init
     );

  Summary:
    Initializes the SD Host Controller driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine initializes the SD Host Controller driver, making it ready for clients to
    open and use the driver.

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver. This pointer may be null if no
                      data is required because static overrides have been
                      provided.

  Returns:
    If successful, returns a valid handle to a driver object. Otherwise, it
    returns SYS_MODULE_OBJ_INVALID.

  Remarks:
    This routine must be called before any other SD Host Controller routine is called.

    This routine should only be called once during system initialization
    unless DRV_SDHC_Deinitialize is called to deinitialize the driver instance.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to reinitialize, it will be reported by the
    DRV_SDHC_Status operation. The system must use DRV_SDHC_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this routine.
*/

SYS_MODULE_OBJ DRV_SDHC_Initialize 
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT  * const init
)
{
    DRV_SDHC_OBJ  *dObj 		= NULL;
    uint8_t          index      = 0;
    drv_sdhc_adma_descriptor *descTble = 0;
    const DRV_SDHC_INIT * const sdhcInit = (const DRV_SDHC_INIT * const)init;

    /* Validate the driver index */
    if ( _DRV_SDHC_INDEX_GET(drvIndex ) >= DRV_SDHC_INDEX_COUNT)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = _DRV_SDHC_INSTANCE_GET(drvIndex);
    if (dObj->inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if (init == NULL)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if (gDrvSDHCInitCount == 0)
    {
        /* Create the Client Object mutex */
        if (OSAL_MUTEX_Create(&gDrvSDHCClientMutex) != OSAL_RESULT_TRUE)
        {
            return SYS_MODULE_OBJ_INVALID;
        }

    }
    gDrvSDHCInitCount++;

    /* Initialize the driver object's structure members */
    memset (dObj, 0, sizeof(DRV_SDHC_OBJ));

    if (OSAL_MUTEX_Create(&dObj->mutex) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Set that this instance of the driver is in use */
    dObj->inUse                     = true;
    dObj->taskState                 = DRV_SDHC_TASK_CHECK_DEVICE;
    dObj->drvIndex                  = drvIndex;

    dObj->taskBufferIOState         = DRV_SDHC_BUFFER_IO_CHECK_DEVICE ;
    dObj->sdState                   = TASK_STATE_IDLE;

	/* Initialize the SD Host Controller queue */
    dObj->queueHandle               = _DRV_SDHC_QueueInitialize (drvIndex);

    for (index = 0; index < MEDIA_INIT_ARRAY_SIZE; index++)
        dObj->mediaInitArray[index] = 0xFF;
    if(DRV_SDHC_MEDIA_MANAGER_USE)
    {
        dObj->mediaHandle = SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)drvIndex,  (SYS_MODULE_INDEX)drvIndex, 
                                    &sdhcMediaFunctions, SYS_FS_MEDIA_TYPE_SD_CARD);
    }
    
    if (dObj->mediaHandle == SYS_FS_MEDIA_HANDLE_INVALID)
        return DRV_SDHC_FAIL;
    
    dObj->sdhcFuseSettings.ip_mode = HOST_SD;
    dObj->sdhcFuseSettings.card_type = TYPE_SD;
    dObj->sdhcFuseSettings.slot_type = REMOVABLE;
    
    dObj->sdhcFuseSettings.bus_mode = DRV_SDHC_BUS_SPEED;
    dObj->sdhcFuseSettings.bus_width = DRV_SDHC_BUS_WIDTH;
    dObj->sdhcFuseSettings.use_dma = DRV_SDHC_USE_DMA;

    CFGCON2bits.SDRDFTHR = 0x200;
    CFGCON2bits.SDWRFTHR = 0x200;
    
    if (sdhcInit->sdCardDetectEnable)     
        CFGCON2bits.SDCDEN = 0x1;
    else
        CFGCON2bits.SDCDEN = 0x0;
                
    if (&sdhcInit->sdWriteProtectEnable)      
        CFGCON2bits.SDWPEN = 0x1;
    else
        CFGCON2bits.SDWPEN = 0x0;
            
    descTble = (drv_sdhc_adma_descriptor*)(KVA_TO_PA(admaDescriptorTable));
    
    if (_drv_sdhc_init(&dObj->sdhcFuseSettings, descTble, &dObj->command) == DRV_SDHC_OK)
    {        
        /* Update the status */
        dObj->status = SYS_STATUS_READY;
        
        /* Return the object structure */
        return ((SYS_MODULE_OBJ)drvIndex);
    }

    return DRV_SDHC_FAIL;
} /* DRV_SDHC_Initialize */

// *****************************************************************************
/* Function:
    void DRV_SDHC_Reinitialize 
     (
         SYS_MODULE_OBJ          object,
         const SYS_MODULE_INIT * const init
     );

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine reinitializes the driver and refreshes any associated hardware
    settings using the given initialization data, but it will not interrupt any
    ongoing operations.

  Precondition:
    Function DRV_SDHC_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_SDHC_Initialize
                      routine

    init            - Pointer to the initialization data structure

  Returns:
    None

  Remarks:
    This function can be called multiple times to reinitialize the module.

    This operation can be used to refresh any supported hardware registers as
    specified by the initialization data or to change the power state of the
    module.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to reinitialize, it will be reported by the
    DRV_SDHC_Status operation. The system must use DRV_SDHC_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this routine.
*/

void DRV_SDHC_Reinitialize( SYS_MODULE_OBJ        object ,
                              const SYS_MODULE_INIT * const init )
{
    DRV_SDHC_OBJ           	*dObj 		= ( DRV_SDHC_OBJ* ) NULL;

    /* Validate the driver object */
    SYS_ASSERT ( object != SYS_MODULE_OBJ_INVALID, "Invalid system object handle" );

    /* get the driver object */
    dObj = ( DRV_SDHC_OBJ* ) _DRV_SDHC_INSTANCE_GET( object );

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Lock failed");}

	/* Set the current driver state */
    ( dObj->status ) = SYS_STATUS_UNINITIALIZED;

    /* Update the status */
    ( dObj->status ) = SYS_STATUS_READY;
    if (OSAL_MUTEX_Unlock(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Unlock failed");}

} /* DRV_SDHC_Reinitialize */


// *****************************************************************************
/* Function:
    void DRV_SDHC_Deinitialize 
     ( 
         SYS_MODULE_OBJ object 
     );

  Summary:
    Deinitializes the specified instance of the SD Host Controller driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the SD Host Controller driver module, disabling
    its operation (and any hardware). Invalidates all the internal data.

  Precondition:
    Function DRV_SDHC_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the
					  DRV_SDHC_Initialize routine.

  Returns:
    None.

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This routine will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by
    the DRV_SDHC_Status operation.  The system has to use DRV_SDHC_Status to 
	check if the de-initialization is complete.
*/

void DRV_SDHC_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDHC_OBJ *dObj;

    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    dObj->inUse  = false;
    dObj->status = SYS_STATUS_UNINITIALIZED;

    if (OSAL_MUTEX_Delete(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Delete failed");}
    gDrvSDHCInitCount--;
    if (gDrvSDHCInitCount == 0)
    {
        if (OSAL_MUTEX_Delete(&gDrvSDHCClientMutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Delete failed");}
    }

} /* DRV_SDHC_Deinitialize */


// *****************************************************************************
/* Function:
    void DRV_SDHC_InterruptServiceRoutine 
     ( 
         SYS_MODULE_OBJ object 
     );

  Summary:
    Services interrupts from the SDHC Module
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Services interrupts from the SDHC Module

  Precondition:
    Function DRV_SDHC_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the
					  DRV_SDHC_Initialize routine.

  Returns:
    None.

  Remarks:
    None.
*/

void DRV_SDHC_InterruptServiceRoutine
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDHC_OBJ *dObj;

    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    if (dObj->status != SYS_STATUS_READY)
        return;
    
    _drv_sdhc_host_irq(dObj->cardHandle);
} /* DRV_SDHC_InterruptServiceRoutine */

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SDHC_Status 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Provides the current status of the SD Host Controller driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine provides the current status of the SD Host Controller driver module.

  Precondition:
    Function DRV_SDHC_Initialize must have been called before calling this
    function

  Parameters:
    object                    - Driver object handle, returned from the
                                DRV_SDHC_Initialize routine

  Returns:
    SYS_STATUS_READY          - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

                                Note Any value greater than SYS_STATUS_READY is
                                also a normal running state in which the driver
                                is ready to accept new operations.

    SYS_STATUS_BUSY           - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_ERROR          - Indicates that the driver is in an error state

                                Note:  Any value less than SYS_STATUS_ERROR is
                                also an error state.

    SYS_MODULE_DEINITIALIZED  - Indicates that the driver has been deinitialized

                                Note:  This value is less than SYS_STATUS_ERROR

  Remarks:
    This operation can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, then a previous operation
    has not yet completed. If the status operation returns SYS_STATUS_READY,
    then it indicates that all previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1).  Any value less than that is
    also an error state.

    This routine will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be cleared by
    calling the reinitialize operation. If that fails, the deinitialize
    operation will need to be called, followed by the initialize operation to
    return to normal operations.
*/

SYS_STATUS DRV_SDHC_Status
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDHC_OBJ *dObj;

    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    /* Return the driver status */
    return dObj->status;
} /* DRV_SDHC_Status */


// *****************************************************************************
/* Function:
    void _DRV_SDHC_AttachDetachTasks 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Maintains the driver's state machine.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine is used to maintain the driver's internal state machine.

  Precondition:
    The DRV_SDHC_Initialize routine must have been called for the specified
    SDHC driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDHC_Initialize)

  Returns:
    None

  Remarks:
*/

void _DRV_SDHC_AttachDetachTasks 
( 
    SYS_MODULE_OBJ object 
)
{
    DRV_SDHC_OBJ *dObj;
    OSAL_RESULT retVal;

    /* Get the driver object */
    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");
    }

    /* Check what state we are in, to decide what to do */
    switch ( dObj->taskState )
    {
        case DRV_SDHC_TASK_CHECK_DEVICE:
            {
                /* Check for device attach */
                dObj->isAttached = _DRV_SDHC_MediaCommandDetect (object);
                if (dObj->isAttachedLastStatus != dObj->isAttached)
                {
                    dObj->isAttachedLastStatus = dObj->isAttached;
                    /* We should call a function on device attach and detach */
                    if (DRV_SDHC_IS_ATTACHED == dObj->isAttached)
                    {
                        /* An SD card seems to be present. Initiate a full card initialization. */
                        dObj->taskState = DRV_SDHC_TASK_MEDIA_INIT;
                    }
                    else
                    {
                        _DRV_SDHC_RemoveQueuedRequests (dObj);
                        dObj->mediaState = SYS_FS_MEDIA_DETACHED;
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_MEDIA_INIT:
            {
                /* Update the card details to the internal data structure */
                _DRV_SDHC_MediaInitialize (object);

                /* Once the initialization is complete, move to the next stage */
                if (dObj->mediaInitState == DRV_SDHC_INIT_SD_INIT_DONE)
                {
                    /* Check and update the card's write protected status */
                    _DRV_SDHC_CheckWriteProtectStatus (object);

                    /* Update the Media Geometry structure */
                    _DRV_SDHC_UpdateGeometry (dObj);

                    dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_CHECK_FOR_DETACH;
                    /* State that the device is attached. */
                    dObj->mediaState = SYS_FS_MEDIA_ATTACHED;
                    dObj->taskState = DRV_SDHC_TASK_CHECK_DEVICE;
                }
                else if (dObj->mediaInitState == DRV_SDHC_INIT_ERROR)
                {
                    dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_GO_TO_IDLE;
                    dObj->taskState = DRV_SDHC_TASK_CHECK_DEVICE;
                }
            }
            break;
    }

    retVal = OSAL_MUTEX_Unlock(&dObj->mutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");
    }
} /* _DRV_SDHC_AttachDetachTasks */

// *****************************************************************************
/* Function:
    void _DRV_SDHC_BufferIOTasks ( SYS_MODULE_OBJ object )

  Summary:
    Maintains the Buffer I/O state machine.

  Description:
    This routine is used to maintain the Buffer I/O internal state machine.

  Precondition:
    The DRV_SDHC_Initialize routine must have been called for the specified
    SDHC driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDHC_Initialize)

  Returns:
    None

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the File system media manager.
*/

void _DRV_SDHC_BufferIOTasks
( 
    SYS_MODULE_OBJ object
)
{
    uint32_t                  argument = 0;
    uint32_t                  *bufferPtr;
    DRV_SDHC_OBJ              *dObj;
    DRV_SDHC_XFER_OBJECT      *lObj;
    OSAL_RESULT                 retVal;
//    uint32_t                spiRetVal;    
    DRV_SDHC_CLIENT_OBJ       *clientObj;

    DRV_SDHC_EVENT            evtStatus = DRV_SDHC_EVENT_COMMAND_COMPLETE;

    /* Get the driver object */
    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");
    }

    /* Assign a local pointer for faster operation */
    lObj = dObj->localTaskObj;

    /* Check what state we are in, to decide what to do */
    switch (dObj->taskBufferIOState)
    {
        case DRV_SDHC_BUFFER_IO_CHECK_DEVICE:
            if (dObj->mediaState != SYS_FS_MEDIA_ATTACHED)
            {
                break;
            }

            /* Process reads/writes only if the media is present.
               Intentional fallthrough.
               */

        case DRV_SDHC_TASK_PROCESS_QUEUE:
            {
                if (dObj->sdState != TASK_STATE_IDLE)
                    break;

                /* Get the first in element from the queue */
                dObj->localTaskObj = _DRV_SDHC_ReadFromQueue (dObj->queueHandle);
                if (dObj->localTaskObj == NULL)
                {
                    /* If there are no read queued, check for device attach/detach */
                    dObj->taskBufferIOState = DRV_SDHC_BUFFER_IO_CHECK_DEVICE;
                    break;
                }

                dObj->sdState = TASK_STATE_CARD_COMMAND;

                lObj = dObj->localTaskObj;
                lObj->status = DRV_SDHC_COMMAND_IN_PROGRESS;

                _drv_sdhc_set_transfer_block_size(lObj->sectorCount * _DRV_SDHC_MEDIA_BLOCK_SIZE);

                if (lObj->sectorCount == 1)
                {
                    _drv_sdhc_set_block_count(0);                    
                }
                else
                {
                    _drv_sdhc_set_block_count(lObj->sectorCount);                    
                }
                
                if (dObj->sdhcFuseSettings.use_dma)
                {
                    //Setup the descriptor table
                    if (_DRV_SDHC_PrepareAdmaDescriptorTableForTransfer(dObj) == DRV_SDHC_FAIL)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_READ_ABORT;
                        break;
                    }
                }
                
                dObj->taskBufferIOState = DRV_SDHC_TASK_SELECT_CARD;
            }
            break;

        case DRV_SDHC_TASK_SELECT_CARD:
            {                
                argument = (uint32_t)(dObj->cardHandle->rca << 16);
                        
                /* Send select card command so that card goes to transfer state */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SELECT_CARD, argument);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //Check if the card is locked, if so, we are at 1-bit bus width
                    if (DRV_SDHC_BUS_WIDTH != BIT_1 && !dObj->isCardLocked)
                    {
                        _drv_sdhc_set_bus_width(DRV_SDHC_BUS_WIDTH);

                        dObj->taskBufferIOState = DRV_SDHC_TASK_PRE_SET_BUS_WIDTH;                                                                            
                    }
                    else
                    {
                        _drv_sdhc_set_bus_width(BIT_1);
                        
                        dObj->taskBufferIOState = DRV_SDHC_TASK_SET_BLOCK_LENGTH;                        
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_PRE_SET_BUS_WIDTH:
            {
                argument = (uint32_t)(dObj->cardHandle->rca << 16);

                /* Send CMD55 (lets SD card know that the next command is application
                   specific (going to be ACMD6)) */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_APP_CMD, argument);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->taskBufferIOState = DRV_SDHC_TASK_SET_BUS_WIDTH;
                }

            }
            break;
            
        case DRV_SDHC_TASK_SET_BUS_WIDTH:
            {
                argument = (DRV_SDHC_BUS_WIDTH == BIT_4) ? 0x2 : 0;
                
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SET_BUS_WIDTH, argument);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->taskBufferIOState = DRV_SDHC_TASK_SET_BLOCK_LENGTH;
                }
            }
            break;

        case DRV_SDHC_TASK_SET_BLOCK_LENGTH:
            {
                /* Now set the block length to media sector size. It
                   should be already set to this. */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SET_BLOCKLEN, _DRV_SDHC_MEDIA_BLOCK_SIZE);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //Confirm card is in transfer state
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        /* Navigate to different cases based on read/write flags */
                        if (lObj->readWrite == DRV_SDHC_TRANSFER_READ)
                        {
                            if (lObj->sectorCount == 1)
                            {
                                lObj->command = DRV_SDHC_READ_SINGLE_BLOCK;
                            }
                            else
                            {
                                lObj->command = DRV_SDHC_READ_MULTI_BLOCK;
                            }

                            dObj->taskBufferIOState = DRV_SDHC_TASK_PROCESS_READ;
                        }
                        else
                        {
                            if (lObj->sectorCount == 1)
                            {
                                lObj->command = DRV_SDHC_WRITE_SINGLE_BLOCK;
                            }
                            else
                            {
                                lObj->command = DRV_SDHC_WRITE_MULTI_BLOCK;
                            }

                            dObj->taskBufferIOState = DRV_SDHC_TASK_PROCESS_WRITE;
                        }
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_PROCESS_READ:
            {
                _DRV_SDHC_CommandSend (object, lObj->command, lObj->sectorAddress);
                
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //Confirm card is in transfer state, it really should be in data state, 
                    //but the card typically returns the previous state
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_READ_TRANSFER_BUSY_STATE;
                    }
                    else
                    {
                        /* Perhaps the card isn't initialized or present */
                        dObj->taskBufferIOState = DRV_SDHC_TASK_READ_ABORT;
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_READ_TRANSFER_BUSY_STATE:
            {
                if (dObj->command.data_completed)
                {
                    dObj->taskBufferIOState = DRV_SDHC_TASK_READ_TRANSFER_COMPLETE;
                    break;
                }
                
                //If we are using Programming I/O (PIO) data transfer
                if (!dObj->sdhcFuseSettings.use_dma && lObj->blockIncr < lObj->sectorCount)
                {
                    if (dObj->command.buffer_ready)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_READ_NEXT_BUFFER;
                        lObj->bufferIncr = 0;
                    }
                }
            }
            break;
            
        case DRV_SDHC_TASK_READ_NEXT_BUFFER:
            {
                for (lObj->bufferIncr = 0; lObj->bufferIncr < _DRV_SDHC_MEDIA_BLOCK_SIZE; lObj->bufferIncr++)
                {
                    bufferPtr = (void*)(lObj->buffer + lObj->bufferIncr * 4);
                    bufferPtr = (void*)PA_TO_KVA1((_paddr_t)bufferPtr);
                    *bufferPtr = _drv_sdhc_read_buffer_data();                    
                }

                //Reset flag, wait for next buffer
                dObj->command.buffer_ready = false;
                lObj->blockIncr++;
                
                dObj->taskBufferIOState = DRV_SDHC_TASK_READ_TRANSFER_BUSY_STATE;
            }
            break;
            
        case DRV_SDHC_TASK_READ_TRANSFER_COMPLETE:
            {
                _DRV_SDHC_CommandSend (object, DRV_SDHC_SEND_STATUS, (dObj->cardHandle->rca << 16));
                
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //The card will automatically go to transfer state on a single block read
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_SEND_CARD_TO_STANDBY;
                    }
                    else
                    {
                        //Need to send stop transmission on multi-block reads
                        dObj->taskBufferIOState = DRV_SDHC_TASK_STOP_TRANSMISSION;
                    }
                }                
            }
            break;

        case DRV_SDHC_TASK_PROCESS_WRITE:
            {
                /* Send the write single or write multi command, with the LBA or byte
                   address (depending upon SDHC or standard capacity card) */
                _DRV_SDHC_CommandSend (object, lObj->command, lObj->sectorAddress);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //Confirm card is in transfer state, it really should be in data state, 
                    //but the card typically returns the previous state
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_WRITE_TRANSFER_BUSY_STATE;
                    }
                    else
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_WRITE_ABORT;
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_WRITE_TRANSFER_BUSY_STATE:
            {
                if (dObj->command.data_completed)
                {
                    dObj->taskBufferIOState = DRV_SDHC_TASK_WRITE_TRANSFER_COMPLETE;
                    break;
                }
                
                //If we are using Programming I/O (PIO) data transfer
                if (!dObj->sdhcFuseSettings.use_dma && lObj->blockIncr < lObj->sectorCount)
                {
                    if (dObj->command.buffer_ready)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_WRITE_NEXT_BUFFER;
                        lObj->bufferIncr = 0;
                    }
                }
            }
            break;
            
        case DRV_SDHC_TASK_WRITE_NEXT_BUFFER:
            {
                for (lObj->bufferIncr = 0; lObj->bufferIncr < _DRV_SDHC_MEDIA_BLOCK_SIZE; lObj->bufferIncr++)
                {
                    bufferPtr = (void*)(lObj->buffer + lObj->bufferIncr * 4);
                    bufferPtr = (void*)PA_TO_KVA1((_paddr_t)bufferPtr);
                    _drv_sdhc_write_buffer_data(*bufferPtr);
                }

                //Reset flag, wait for next buffer
                dObj->command.buffer_ready = false;
                lObj->blockIncr++;
                
                dObj->taskBufferIOState = DRV_SDHC_TASK_WRITE_TRANSFER_BUSY_STATE;
            }
            break;
            
        case DRV_SDHC_TASK_WRITE_TRANSFER_COMPLETE:
            {
                _DRV_SDHC_CommandSend (object, DRV_SDHC_SEND_STATUS, (dObj->cardHandle->rca << 16));
                
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //The card will automatically go to transfer state on a single block read
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_SEND_CARD_TO_STANDBY;
                    }
                    else
                    {
                        //Need to send stop transmission on multi-block reads
                        dObj->taskBufferIOState = DRV_SDHC_TASK_STOP_TRANSMISSION;
                    }
                }                
            }
            break;

        case DRV_SDHC_TASK_STOP_TRANSMISSION:
            {
                _DRV_SDHC_CommandSend (object, DRV_SDHC_STOP_TRANSMISSION, 0);
                
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cardState == SDHC_CARD_STATE_TRANSFER)
                    {
                        dObj->taskBufferIOState = DRV_SDHC_TASK_SEND_CARD_TO_STANDBY;                        
                    }
                }
            }
            break;

        case DRV_SDHC_TASK_SEND_CARD_TO_STANDBY:
            {
                //De-select card to bring card back to standby card state
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SELECT_CARD, 0);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    //On un-select a card, we are not going to receive a response from the card, just proceed to the next state
                    dObj->taskBufferIOState = DRV_SDHC_PROCESS_NEXT;
                }
            }
            break;

        case DRV_SDHC_PROCESS_NEXT:
        case DRV_SDHC_TASK_READ_ABORT:
        case DRV_SDHC_TASK_WRITE_ABORT:
            {
                if (dObj->taskBufferIOState == DRV_SDHC_PROCESS_NEXT)
                {
                    lObj->status = DRV_SDHC_COMMAND_COMPLETED;
                    evtStatus = DRV_SDHC_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    lObj->status = DRV_SDHC_COMMAND_ERROR_UNKNOWN;
                    evtStatus = DRV_SDHC_EVENT_COMMAND_ERROR;
                }

                /* Mark the buffer object as free */
                lObj->inUse = false;

                clientObj = (DRV_SDHC_CLIENT_OBJ*)_DRV_SDHC_CLIENT_OBJ_GET(lObj->hClient);
                if(clientObj->eventHandler != NULL)
                {
                    /* Call the event handler */
                    clientObj->eventHandler(evtStatus,
                            (DRV_SDHC_COMMAND_HANDLE)lObj->commandHandle, clientObj->context);
                }

                dObj->sdState = TASK_STATE_IDLE;
                dObj->taskBufferIOState = DRV_SDHC_BUFFER_IO_CHECK_DEVICE;
                break;
            }
        default:
            break;
    }

    retVal = OSAL_MUTEX_Unlock(&dObj->mutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");
    }

} /* DRV_SDHC_BufferIOTasks */


DRV_SDHC_RESPONSE _DRV_SDHC_PrepareAdmaDescriptorTableForTransfer
(
    DRV_SDHC_OBJ *dObj
)
{
    DRV_SDHC_XFER_OBJECT *lObj = dObj->localTaskObj;
    int32_t transferSize = lObj->sectorCount * _DRV_SDHC_MEDIA_BLOCK_SIZE;
    
    //First: Round up the transfer size to full block size
    int32_t numberOfBlocks = (transferSize > _DRV_SDHC_MEDIA_BLOCK_SIZE) ? transferSize / _DRV_SDHC_MEDIA_BLOCK_SIZE : 1;
    int32_t residual = (transferSize > _DRV_SDHC_MEDIA_BLOCK_SIZE) ? transferSize - (numberOfBlocks * _DRV_SDHC_MEDIA_BLOCK_SIZE) : 0;
    transferSize = (residual > 0) ? (numberOfBlocks + 1) * _DRV_SDHC_MEDIA_BLOCK_SIZE : numberOfBlocks * _DRV_SDHC_MEDIA_BLOCK_SIZE;
            
    //Second: Figure out how many descriptor lines are required
    int32_t divisor = transferSize / _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE;
    int32_t numberOfDescriptorLines = (transferSize >= _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE) ? divisor : 1;
    int32_t remainder = (transferSize > _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE) ? transferSize - (divisor * _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE) : 0;    
    int32_t lineCounter = 0;
    uint32_t addressOffset = (uint32_t)lObj->buffer;
    int32_t bytesLeftToTransfer = transferSize;
    numberOfDescriptorLines = (remainder > 0) ? numberOfDescriptorLines + 1 : numberOfDescriptorLines;
            
    //Reserve the last descriptor table line for ending the transmission
    if (numberOfDescriptorLines > (_DSV_SDHC_DESCRIPTOR_TABLE_SIZE - 1))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"DRV_SDHC: The transfer size is greater than the supported ADMA descriptor table size.");
        return DRV_SDHC_FAIL;
    }
    
    if (numberOfDescriptorLines <= 0)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"DRV_SDHC: The transfer size for ADMA descriptor table size is 0.");
        return DRV_SDHC_FAIL;
    }

    //This assumes the memory buffer is contiguous
    for (lineCounter = 0; lineCounter < numberOfDescriptorLines; ++lineCounter)
    {
        addressOffset += lineCounter * _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE;
        
        admaDescriptorTable[lineCounter].address = addressOffset;
        
        if (bytesLeftToTransfer > _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE)
        {
            admaDescriptorTable[lineCounter].length = (_DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE >= 65536) ? 0 : _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE;
            bytesLeftToTransfer -= _DSV_SDHC_NUM_BYTES_PER_DESCRIPTOR_LINE;
        }
        else
        {
            admaDescriptorTable[lineCounter].length = bytesLeftToTransfer;            
        }
        
        if (lineCounter == numberOfDescriptorLines - 1)
        {
            admaDescriptorTable[lineCounter].attribute = DRV_SDHC_ADMA_DESC_TBL_ATT_ACT12_TRANFER_ONE_LINE;            
            admaDescriptorTable[lineCounter].attribute |= DRV_SDHC_ADMA_DESC_TBL_ATT_INT_EN_BIT | DRV_SDHC_ADMA_DESC_TBL_ATT_LAST_LINE_BIT | DRV_SDHC_ADMA_DESC_TBL_ATT_VALID_BIT;             
        }
        else
        {
            admaDescriptorTable[lineCounter].attribute = DRV_SDHC_ADMA_DESC_TBL_ATT_ACT12_TRANFER_ONE_LINE;
            admaDescriptorTable[lineCounter].attribute |= DRV_SDHC_ADMA_DESC_TBL_ATT_INT_EN_BIT | DRV_SDHC_ADMA_DESC_TBL_ATT_VALID_BIT;             
        }
    }
    
    _drv_sdhc_set_adma_descripter_table((drv_sdhc_adma_descriptor*)KVA_TO_PA(&admaDescriptorTable));
        
    return DRV_SDHC_OK;
    
} /* DRV_SDHC_PrepareDescriptorTableForTransfer */

// *****************************************************************************
/* Function:
    void DRV_SDHC_Tasks 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Maintains the driver's state machine.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine is used to maintain the driver's internal state machine.

  Precondition:
    The DRV_SDHC_Initialize routine must have been called for the specified
    SDHC driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDHC_Initialize)

  Returns:
    None

  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SDHC_Tasks 
( 
    SYS_MODULE_OBJ object 
)
{
    _DRV_SDHC_AttachDetachTasks (object);
    _DRV_SDHC_BufferIOTasks (object);
}


DRV_SDHC_CLIENT_OBJ * _DRV_SDHC_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_SDHC_CLIENT_OBJ * clientObj;
    DRV_SDHC_OBJ * dObj;

    if (handle < 0 || handle > (DRV_SDHC_CLIENTS_NUMBER * DRV_SDHC_INSTANCES_NUMBER))
        return NULL;

    /* See if the client has been opened */
    clientObj = _DRV_SDHC_CLIENT_OBJ_GET(handle);
    if (!clientObj->inUse)
        return NULL;

    /* Check if the driver is ready for operation */
    dObj = (DRV_SDHC_OBJ *)clientObj->driverObject;
    if (dObj->status != SYS_STATUS_READY)
        return NULL;

    return clientObj;
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_SDHC_Open 
     (
         const SYS_MODULE_INDEX drvIndex,
         const DRV_IO_INTENT    intent
     );

  Summary:
    Opens the specified SD Host Controller driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine opens the specified SD Card driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_SDHC_Initialize must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    intent      - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID.

  Remarks:
    The handle returned is valid until the DRV_SDHC_Close routine is called.

    This routine will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.

    If the requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_SDHC_Open 
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    uint32_t iClient;
    DRV_SDHC_CLIENT_OBJ *clientObj; 
    DRV_SDHC_OBJ *dObj;
    OSAL_RESULT retVal;
   
    /* Validate the driver index */
    if (drvIndex >= DRV_SDHC_INDEX_COUNT)
    {
        return DRV_HANDLE_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(drvIndex);

    /* Acquire the driver object mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return DRV_HANDLE_INVALID;
    }

    if (dObj->status != SYS_STATUS_READY)
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
        return DRV_HANDLE_INVALID;
    }

    /* The SDHC driver doesn't support blocking intent. Flag error. */
    /* Flag error if max number of clients are already open.
     * Flag error if driver was already opened exclusively.
     * Flag error if the client is to trying to open the driver exclusively
     * when it is already by other clients in non-exclusive mode.
     * */
    if ((DRV_IO_ISBLOCKING(ioIntent)) ||
        (dObj->numClients == DRV_SDHC_CLIENTS_NUMBER) ||
        (dObj->isExclusive) || 
        ((dObj->numClients > 0) && DRV_IO_ISEXCLUSIVE(ioIntent)))
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
        return DRV_HANDLE_INVALID;
    }

    retVal = OSAL_MUTEX_Lock(&gDrvSDHCClientMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
		return DRV_HANDLE_INVALID;
    }

    /* Find available slot in array of client objects */
    clientObj = (DRV_SDHC_CLIENT_OBJ *)gDrvSDHCClientObj;
    for (iClient = 0; iClient < (DRV_SDHC_CLIENTS_NUMBER * DRV_SDHC_INSTANCES_NUMBER); iClient++)
    {
        if (!clientObj->inUse)
        {
            clientObj->inUse        = true;
            clientObj->eventHandler = NULL;
            clientObj->context      = 0;
            clientObj->drvIndex     = drvIndex;
            clientObj->driverObject = (DRV_SDHC_OBJ_HANDLE)dObj;
            clientObj->intent       = ioIntent;

            dObj->numClients++;

            if (DRV_IO_ISEXCLUSIVE(ioIntent))
            {
                dObj->isExclusive = true;
            }
            
            OSAL_MUTEX_Unlock(&gDrvSDHCClientMutex);
            OSAL_MUTEX_Unlock(&dObj->mutex);

            return ((DRV_HANDLE)iClient);
        }

        clientObj++;
    }

    OSAL_MUTEX_Unlock(&gDrvSDHCClientMutex);
    OSAL_MUTEX_Unlock(&dObj->mutex);

    return DRV_HANDLE_INVALID;
} /* DRV_SDHC_Open */


// *****************************************************************************
/* Function:
    void DRV_SDHC_Close
     (
         DRV_HANDLE handle
     );

  Summary:
    Closes an opened-instance of the SD Card driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine closes an opened-instance of the SD Card driver, invalidating
    the handle.

  Precondition:
    The DRV_SDHC_Initialize routine must have been called for the specified
    SD Card driver instance.

    DRV_SDHC_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SDHC_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_SDHC_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/
void DRV_SDHC_Close( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SDHC_CLIENT_OBJ   *clientObj =
        ( DRV_SDHC_CLIENT_OBJ* ) _DRV_SDHC_CLIENT_OBJ_GET ( handle );
     DRV_SDHC_OBJ* dObj = ( DRV_SDHC_OBJ* )clientObj->driverObject;
     if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Lock failed");}

    /* Update the Client Status */
    clientObj->inUse = false;

    dObj->numClients--;
    if (OSAL_MUTEX_Unlock(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDHC Driver: OSAL_MUTEX_Unlock failed");}

} /* DRV_SDHC_Close */

//******************************************************************************
/* Function:
    bool _DRV_SDHC_MediaCommandDetect ( SYS_MODULE_OBJ object )

  Summary:
    Determines whether an SD card is present using a command response method.

  Description:
    This routine determines whether an SD card is present using a command
    response method. If it is a Micro SD card(doesn't has a card detect pin)
    calling this API is the only option detect the presence of the card. This API
    could be called directly or DRV_SDHC_MediaDetect function will call this
    API based on the configuration.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    true - The Card is present.
    false - The Card is not present.
*/

uint16_t resetTimeout = 0;

bool _DRV_SDHC_MediaCommandDetect 
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDHC_OBJ *dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);
    uint32_t argument = 0;
    
    switch (dObj->cmdDetectState)
    {
        case DRV_SDHC_CMD_DETECT_START_INIT:
            {
                dObj->cardHandle = _drv_sdhc_check_card_inserted();
                
                if (dObj->cardHandle != NULL)
                {
                    dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_RESET_SDHC;
                }
            }
            break;
        case DRV_SDHC_CMD_DETECT_RESET_SDHC:
            {
                _DRV_SDHC_CommandSend (object, DRV_SDHC_GO_IDLE_STATE, 0x0);

                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    /* Wait for the card to go to idle */
                    dObj->sysTmrHandle = SYS_TMR_DelayMS( _DRV_SDHC_INITIALIZATION_RETRY_WAIT_IN_MS ); 
                    
                    if (dObj->sysTmrHandle != SYS_TMR_HANDLE_INVALID)
                    {
                        /* Card is present.  Idle until otherwise*/
                        dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_WAIT_FOR_IDLE;
                    }
                }
            }
            break;
        case DRV_SDHC_CMD_DETECT_WAIT_FOR_IDLE:            
            if(SYS_TMR_DelayStatusGet(dObj->sysTmrHandle) == true)
            {
                dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_IDLE;
            }
            break;
        case DRV_SDHC_CMD_DETECT_IDLE:
            {
                /* Card is present and idle */
                return true;
            }
            break;
        case DRV_SDHC_CMD_DETECT_CHECK_FOR_DETACH:
            {
                if (dObj->sdState == TASK_STATE_CARD_COMMAND)
                {
                    return true;
                }

                dObj->sdState = TASK_STATE_CARD_STATUS;

                argument = dObj->cardHandle->rca << 16;
                
                _DRV_SDHC_CommandSend (object, DRV_SDHC_SEND_STATUS, argument);
                
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    /* For status command SD card will respond with R2 type packet */
                    dObj->sdState = TASK_STATE_IDLE;
                    
                    if ((dObj->cmdResponse.response2.word0 & 0xEC0C) != 0x0000)
                    {
                        return (_drv_sdhc_check_card_inserted() != NULL);
                    }
                    else
                    {
                        dObj->cmdDetectState = DRV_SDHC_CMD_DETECT_CHECK_FOR_DETACH;
                    }
                }
            }
            return true;
        default:
            break;
    }

    return false;

} /* _DRV_SDHC_MediaCommandDetect */

//******************************************************************************
/* Function:
    void _DRV_SDHC_CommandSend ( DRV_HANDLE handle, DRV_SDHC_COMMANDS command,
                                uint32_t address )

  Summary:
    Sends a command.

  Description:
    This routine sends a command to the SD card. The response will be updated in
    the driver instance object. The code is written in an event driven method.
    The user is required to call this API multiple times till the status becomes
    'complete'.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

    command     - Command to send.

    address     - If there is an address associated with the command. If there
                    is no address associated, then pass '0'.

  Returns:
    None

  Remarks:
    It is expected to call this routine continuously until we get the status as
    'execution successful'. The caller should not execute this function again
    after the execution of the command. It will cause to execute the same command
    again.
*/

void _DRV_SDHC_CommandSend
(
    SYS_MODULE_OBJ object,
    DRV_SDHC_COMMANDS command,
    uint32_t argument
)
{
    DRV_SDHC_OBJ *dObj = (DRV_SDHC_OBJ*)_DRV_SDHC_INSTANCE_GET(object);

    SYS_ASSERT(dObj->command != NULL,"Missing command object");
                
    switch (dObj->cmdState)
    {
        case DRV_SDHC_CMD_EXEC_IS_COMPLETE:
            // Fall-through to frame packet, as we are ready for the next command
        case DRV_SDHC_CMD_FRAME_PACKET:
            {
                dObj->command.opcode = gDrvSDHCCmdTable[command].commandCode;
                dObj->command.resp_type = gDrvSDHCCmdTable[command].responseType;
                dObj->command.argument = argument;
                dObj->command.data_present = gDrvSDHCCmdTable[command].moredataexpected;
                dObj->command.xfer_mode = _drv_sdhc_transfer_mode(dObj->cardHandle);
                dObj->command.cmd_completed = false;
                dObj->command.buffer_ready = false;
                dObj->command.data_completed = false;
                dObj->command.cmd_error_flag = 0; //Clear errors
                
                dObj->cmdState = DRV_SDHC_CMD_SEND_PACKET;
                break;
            }
        case DRV_SDHC_CMD_RETRY:
            // Fall-through to send packet, as we are retrying the command
        case DRV_SDHC_CMD_SEND_PACKET:
            {
                if (_drv_sdhc_sd_cmd_send(dObj->cardHandle) == DRV_SDHC_OK)
                {
                    dObj->cmdState = DRV_SDHC_CMD_CHECK_TRANSFER_COMPLETE;
                }
            }
            break;
        case DRV_SDHC_CMD_CHECK_TRANSFER_COMPLETE:
            {
                if (dObj->command.cmd_error_flag != 0)
                {
                    //Check if command timeout
                    if(dObj->command.cmd_error_flag & HOST_INT_TIMEOUT)
                    {
                        //Re-send the command
                        dObj->cmdState = DRV_SDHC_CMD_RETRY;
                        dObj->command.cmd_error_flag = 0;
                        return;
                    }
                }
                    
                if (dObj->command.cmd_completed == false)
                {
                    return;
                }

                switch (gDrvSDHCCmdTable[command].responseType)
                {
                    case RESPONSE_R1:
                    case RESPONSE_R1b:
                        {
                            dObj->cmdResponse.response1.word0 = dObj->command.response[0];
                            dObj->cmdResponse.response1.word1 = dObj->command.response[1];

                            dObj->cardState = (dObj->cmdResponse.response1.cardState3 << 3 |
                                    dObj->cmdResponse.response1.cardState2 << 2 |
                                    dObj->cmdResponse.response1.cardState1 << 1 |
                                    dObj->cmdResponse.response1.cardState0);

                            dObj->isCardLocked = dObj->cmdResponse.response1.cardIsLocked;                            
                        }
                        break;

                    case RESPONSE_R2:
                    case RESPONSE_R4:
                    case RESPONSE_R5:
                    case RESPONSE_R3:
                    case RESPONSE_R6:
                    case RESPONSE_R7:
                        {
                            dObj->cmdResponse.response2.word0 = dObj->command.response[0];
                            dObj->cmdResponse.response2.word1 = dObj->command.response[1];
                            dObj->cmdResponse.response2.word2 = dObj->command.response[2];
                            dObj->cmdResponse.response2.word3 = dObj->command.response[3];
                            dObj->cmdResponse.response2.word4 = dObj->command.response[4];
                            dObj->cmdResponse.response2.word5 = dObj->command.response[5];
                            dObj->cmdResponse.response2.word6 = dObj->command.response[6];
                            dObj->cmdResponse.response2.word7 = dObj->command.response[7];
                            
                            if (gDrvSDHCCmdTable[command].responseType == RESPONSE_R6)
                            {
                                dObj->cardState = (dObj->cmdResponse.response6.cardState3 << 3 |
                                        dObj->cmdResponse.response6.cardState2 << 2 |
                                        dObj->cmdResponse.response6.cardState1 << 1 |
                                        dObj->cmdResponse.response6.cardState0);                                            
                            }                            
                        }
                        break;
                    default:
                        break;
                }

                dObj->cmdState = DRV_SDHC_CMD_EXEC_IS_COMPLETE;
        }
        break;
    }
} /* End of _DRV_SDHC_CommandSend */

//******************************************************************************
/* Function:
    void _DRV_SDHC_RequestCardSpecificData ( SYS_MODULE_OBJ object )

  Summary:
    Handle the CMD9 Send CSD message with the card.

  Description:
    The function updates CSD structure.  This function should be called
    from within a state machine structure

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

  Returns:
    None
*/

bool _DRV_SDHC_RequestCardSpecificData ( SYS_MODULE_OBJ object )
{
    DRV_SDHC_OBJ *dObj = ( DRV_SDHC_OBJ* ) _DRV_SDHC_INSTANCE_GET ( object );
    uint8_t cSizeMultiplier;
    uint16_t blockLength;
    uint32_t cSize;
    uint8_t *csdPtr;
    uint32_t mult;
    
    /* CMD9: Read CSD data structure */
    _DRV_SDHC_CommandSend (object, DRV_SDHC_SEND_CSD, (dObj->cardHandle->rca << 16));

    /* Change from this state only on completion of command execution */
    if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
    {
        /* Gather CSD data */
        dObj->mediaSectorSize = _DRV_SDHC_MEDIA_BLOCK_SIZE;
        dObj->mediaInformation.sectorSize = _DRV_SDHC_MEDIA_BLOCK_SIZE;
        dObj->mediaInformation.validityFlags.bits.sectorSize = true;
        /* Extract some fields from the response for computing the card
        capacity. */
        /* Note: The structure format depends on if it is a CSD V1 or V2 device.
        Therefore, need to first determine version of the specs that the card
        is designed for, before interpreting the individual fields.
        */
        /* Calculate the MDD_SDSPI_finalLBA (see SD card physical layer
        simplified spec 2.0, section 5.3.2).
        In USB mass storage applications, we will need this information
        to correctly respond to SCSI get capacity requests.  Note: method
        of computing MDD_SDSPI_finalLBA TODO depends on CSD structure spec
        version (either v1 or v2).
        */
        csdPtr = ((uint8_t*)&dObj->cmdResponse.response2.word0) + 1;
        if (dObj->cmdResponse.response2.word0 & _DRV_SDHC_CHECK_V2_DEVICE)
        {
            /* Check CSD_STRUCTURE field for v2+ struct device */
            /* Must be a v2 device (or a reserved higher version, that
            doesn't currently exist) */
            /* Extract the C_SIZE field from the response.  It is a 22-bit
            number in bit position 69:48.  This is different from v1.
            It spans bytes 7, 8, and 9 of the response.
            */
            cSize = (uint32_t)((dObj->cmdResponse.response2.word4 & 0x003F) << 16) | (uint32_t)dObj->cmdResponse.response2.word5;

            // Per SD Simplified Spec V2.00 Section 5.3.3, the value is in block size of 512
            dObj->discCapacity = ((uint32_t)(cSize + 1) * (uint32_t)(1024));
        }
        else /* Not a V2 device, Must be a V1 device */
        {
            /* Must be a v1 device. Extract the C_SIZE field from the
            response.  It is a 12-bit number in bit position 73:62.
            Although it is only a 12-bit number, it spans bytes 6, 7,
            and 8, since it isn't byte aligned.
            */
            cSize = csdPtr[6] & 0x3;
            cSize <<= 8;
            cSize |= csdPtr[7];
            cSize <<= 2;
            cSize |= (csdPtr[8] >> 6);
            /* Extract the C_SIZE_MULT field from the response.  It is a
            3-bit number in bit position 49:47 */
            cSizeMultiplier = (csdPtr[9] & 0x03) << 1;
            cSizeMultiplier |= ((csdPtr[10] & 0x80) >> 7);

            /* Extract the BLOCK_LEN field from the response. It is a
            4-bit number in bit position 83:80
            */
            blockLength = csdPtr[5] & 0x0F;
            blockLength = 1 << (blockLength - 9);

            /* Calculate the capacity (see SD card physical layer simplified
            spec 2.0, section 5.3.2). In USB mass storage applications,
            we will need this information to correctly respond to SCSI get
            capacity requests (which will cause MDD_SDSPI_ReadCapacity()
            to get called).
            */

            mult = 1 << (cSizeMultiplier + 2);
            dObj->discCapacity = (((uint32_t)(cSize + 1) * mult) * blockLength);
        }

        return true;
    }

    return false;
} /* End of _DRV_SDHC_RequestCardSpecificData */

//******************************************************************************
/* Function:
    void _DRV_SDHC_MediaInitialize ( SYS_MODULE_OBJ object )

  Summary:
    Initializes the SD card.

  Description:
    The function updates MEDIA_INFORMATION structure.  The
    errorCode member may contain the following values:
        * MEDIA_NO_ERROR - The media initialized successfully
        * MEDIA_CANNOT_INITIALIZE - Cannot initialize the media.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

  Returns:
    None
*/

void _DRV_SDHC_MediaInitialize ( SYS_MODULE_OBJ object )
{
    /* Get the driver object */
    DRV_SDHC_OBJ *dObj = ( DRV_SDHC_OBJ* ) _DRV_SDHC_INSTANCE_GET ( object );
    uint16_t response = 0;

    /* Check what state we are in, to decide what to do */
    switch (dObj->mediaInitState)
    {
        case DRV_SDHC_INIT_MEDIA:
            {
                dObj->mediaInformation.errorCode = SYS_FS_MEDIA_NO_ERROR;
                dObj->mediaInformation.validityFlags.value = 0;
                dObj->discCapacity = 0;
                dObj->sdCardType = DRV_SDHC_MODE_NORMAL;

                dObj->mediaInitState = DRV_SDHC_INIT_CHK_IFACE_CONDITION;
            }
            break;

        case DRV_SDHC_INIT_CHK_IFACE_CONDITION:
            {
                /* Send CMD8 (SEND_IF_COND) to specify/request the SD card interface
                   condition (ex: indicate what voltage the host runs at).
                   0x000001AA --> VHS = 0001b = 2.7V to 3.6V.  The 0xAA LSB is the check
                   pattern, and is arbitrary, but 0xAA is recommended (good blend of 0's
                   and '1's). The SD card has to echo back the check pattern correctly
                   however, in the R7 response. If the SD card doesn't support the
                   operating voltage range of the host, then it may not respond. If it
                   does support the range, it will respond with a type R7 response packet
                   (6 bytes/48 bits). Additionally, if the SD card is MMC or SD card
                   v1.x spec device, then it may respond with invalid command.  If it is
                   a v2.0 spec SD card, then it is mandatory that the card respond to CMD8
                 */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SEND_IF_COND, 0x1AA);

                /* Note: CRC value in the table is set for value "0x1AA", it should
                   be changed if a different value is passed. */

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    if ((dObj->cmdResponse.response7.voltages > 0)
                            && (dObj->cmdResponse.response7.echoback == 0xAA))
                    {
                        dObj->sdHcHost = 1;
                    }
                    else
                    {
                        dObj->sdHcHost = 0;
                    }

                    dObj->mediaInitState = DRV_SDHC_INIT_SEND_APP_CMD;
                }
            }
            break;

        case DRV_SDHC_INIT_SEND_APP_CMD:
            {
                /* Send CMD55 (lets SD card know that the next command is application
                   specific (going to be ACMD41)) */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_APP_CMD, 0x0);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->mediaInitState = DRV_SDHC_INIT_SEND_ACMD41;
                }
            }
            break;

        case DRV_SDHC_INIT_SEND_ACMD41:
            {
                /* Send ACMD41.  This is to check if the SD card is finished booting
                   up/ready for full frequency and all further commands.  Response is
                   R3 type (6 bytes/48 bits, middle four bytes contain potentially useful
                   data). */
                /* Note: When sending ACMD41, the HCS bit is bit 30, and must be = 1 to
                   tell SD card the host supports SDHC
                 */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SD_SEND_OP_COND, 
                        dObj->sdHcHost << 30 | 0x180000);
                
                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    response = dObj->cmdResponse.response2.word1;
                    
                    if (response & 0x4000)
                    {
                        dObj->sdCardType = DRV_SDHC_MODE_HC;
                    }
                    else
                    {
                        dObj->sdCardType = DRV_SDHC_MODE_NORMAL;
                    }
                    
                    //Check if card is initialized, if not, tell the card go idle and try again
                    if (response & 0x8000)
                    {
                        dObj->mediaInitState = DRV_SDHC_INIT_ALL_SEND_CID;
                    }
                    else
                    {                        
                        dObj->sysTmrHandle = SYS_TMR_DelayMS( _DRV_SDHC_INITIALIZATION_RETRY_WAIT_IN_MS );
                        dObj->mediaInitState = DRV_SDHC_INIT_ACMD41_RETRY_IDLE;
                    }
                }
            }
            break;
            
        case DRV_SDHC_INIT_ACMD41_RETRY_IDLE:
            /* Wait until timer expires then retry the CMD55-ACMD41 combo */
            if (SYS_TMR_DelayStatusGet(dObj->sysTmrHandle) == true)
            {
                dObj->mediaInitState = DRV_SDHC_INIT_SEND_APP_CMD;
            }
            break;

        case DRV_SDHC_INIT_ALL_SEND_CID:
            {
                /* Now send CMD2(All cards send CID over CMD line) 
                 */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_ALL_SEND_CID, 0x0);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    /* Card initialization is complete, switch to normal operation */
                    dObj->mediaInitState = DRV_SDHC_INIT_SEND_RCA;
                }
            }
            break;
        case DRV_SDHC_INIT_SEND_RCA:
            {
                /* Issue command to response with its RCA, tell the card to go to standby */
                _DRV_SDHC_CommandSend(object, DRV_SDHC_SEND_RCA, 0x0);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDHC_CMD_EXEC_IS_COMPLETE)
                {
                    if ((dObj->cmdResponse.response6.cmdCRCErr 
                            | dObj->cmdResponse.response6.illegalCmdErr
                            | dObj->cmdResponse.response6.generalErr) == 1)
                    {
                        dObj->mediaInitState = DRV_SDHC_INIT_ERROR;
                    }
                    else if (dObj->cardState == SDHC_CARD_STATE_STANDBY)
                    {
                        dObj->cardHandle->rca = dObj->cmdResponse.response2.word1;
                        
                        /* Card initialization is complete, request the first CSD */
                        dObj->sysTmrHandle = SYS_TMR_DelayMS( _DRV_SDHC_INITIALIZATION_RETRY_WAIT_IN_MS );
                        dObj->mediaInitState = DRV_SDHC_INIT_WAIT_FOR_CARD_TO_STANDBY;
                    }
                }
            }
            break;
        case DRV_SDHC_INIT_WAIT_FOR_CARD_TO_STANDBY:
            {
                if (SYS_TMR_DelayStatusGet(dObj->sysTmrHandle)==true)
                {
                    dObj->mediaInitState = DRV_SDHC_INIT_READ_CSD;
                }
            }
            break;
        case DRV_SDHC_INIT_READ_CSD:
            {
                if(_DRV_SDHC_RequestCardSpecificData( object ) == true)
                {
                    dObj->mediaInitState = DRV_SDHC_INIT_SD_INIT_DONE;
                }
            }
            break;

        case DRV_SDHC_INIT_SD_INIT_DONE:
            /* Coming for the first time */
            dObj->mediaInitState = DRV_SDHC_INIT_MEDIA;
            break;

        case DRV_SDHC_INIT_ERROR:
            dObj->mediaInitState = DRV_SDHC_INIT_MEDIA;
            dObj->mediaInformation.errorCode = SYS_FS_MEDIA_CANNOT_INITIALIZE;
            break;

        default:
            break;
    }
}

// *****************************************************************************
/* Function:
    void DRV_SDHC_Read
    (
        const DRV_HANDLE handle,
        DRV_SDHC_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified block address of the SD Card.

  Description:
    This function schedules a non-blocking read operation for reading blocks
    of data from the SD Card. The function returns with a valid buffer handle
    in the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SDHC_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SDHC_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SDHC_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SDHC_Initialize routine must have been called for the specified SDHC 
    driver instance.

    DRV_SDHC_Open must have been called with DRV_IO_INTENT_READ or 
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    targetBuffer  - Buffer into which the data read from the SD Card will be placed

    blockStart    - Start block address of the SD Card from where the read should begin.

    nBlock        - Total number of blocks to be read.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SDHC_COMMAND_HANDLE_INVALID if the request was not successful.

  Remarks:
    None.
*/

void DRV_SDHC_Read 
(
    const DRV_HANDLE handle,
    DRV_SDHC_COMMAND_HANDLE *commandHandle,
    void *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SDHC_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SDHC_CLIENT_OBJ     *clientObj;
    DRV_SDHC_OBJ            *dObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SDHC_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return;

    if (!(clientObj->intent & DRV_IO_INTENT_READ))
        return;

    if ((targetBuffer == NULL) || (nBlock == 0))
        return;
    
    targetBuffer = (void*)KVA_TO_PA(targetBuffer);

    if (((blockStart + nBlock) > gDrvSDHCGeometryTable[GEOMETRY_TABLE_READ_ENTRY].numBlocks))
        return;

    dObj = (DRV_SDHC_OBJ*)clientObj->driverObject;

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) 
        return;

    /* Add to the queue specifying the type as READ */
    *tempHandle1 = _DRV_SDHC_AddToQueue (handle, DRV_SDHC_TRANSFER_READ, targetBuffer, blockStart, nBlock);

    OSAL_MUTEX_Unlock(&dObj->mutex);
}


// *****************************************************************************
/* Function:
    void DRV_SDHC_Write
    (
        const DRV_HANDLE handle,
        DRV_SDHC_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting at the specified address of the SD Card.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data to the SD Card. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SDHC_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the number
      of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SDHC_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SDHC_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SDHC_Initialize() routine must have been called for the specified
    SDHC driver instance.

    DRV_SDHC_Open() routine must have been called to obtain a valid opened device
    handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    sourceBuffer  - The source buffer containing data to be programmed to the SD Card.

    blockStart    - Start block address of SD Card where the writes should begin. 

    nBlock        - Total number of blocks to be written. 

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SDHC_COMMAND_HANDLE_INVALID if the request was not successful.

  Remarks:
    None.
*/

void DRV_SDHC_Write
(
    const DRV_HANDLE handle,
    DRV_SDHC_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SDHC_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SDHC_CLIENT_OBJ     *clientObj;
    DRV_SDHC_OBJ            *dObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SDHC_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return;

    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
        return;

    if ((sourceBuffer == NULL) || (nBlock == 0))
        return;

    if (((blockStart + nBlock) > gDrvSDHCGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
        return;

    dObj = (DRV_SDHC_OBJ*)clientObj->driverObject;

    /* Return error if the card is write protected */
    if (dObj->isWriteProtected)
        return;

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        return;

    sourceBuffer = (void*)KVA_TO_PA(sourceBuffer);

    /* Add to the queue specifying the type as WRITE */
    *tempHandle1 = _DRV_SDHC_AddToQueue (handle, DRV_SDHC_TRANSFER_WRITE, sourceBuffer, blockStart, nBlock);

    OSAL_MUTEX_Unlock(&dObj->mutex);
}


// *****************************************************************************
/* Function:
    DRV_SDHC_COMMAND_STATUS DRV_SDHC_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SDHC_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must use
    this routine where the status of a scheduled command needs to be polled on. The
    function may return DRV_SDHC_COMMAND_HANDLE_INVALID in a case where the command
    handle has expired. A command handle expires when the internal buffer object
    is re-assigned to another read or write request. It is recommended that this
    function be called regularly in order to track the command status correctly.

    The application can alternatively register an event handler to receive read or
    write operation completion events.

  Preconditions:
    The DRV_SDHC_Initialize() routine must have been called.

    The DRV_SDHC_Open() must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SDHC_COMMAND_STATUS value describing the current status of the command.
    Returns DRV_SDHC_COMMAND_HANDLE_INVALID if the client handle or the command
    handle is not valid.

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_SDHC_COMMAND_STATUS DRV_SDHC_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_SDHC_COMMAND_HANDLE commandHandle
)
{
    uint16_t index;
    DRV_SDHC_CLIENT_OBJ *clientObj;
    DRV_SDHC_QUEUE_OBJECT *qObj;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj == NULL)
    {
        return DRV_SDHC_COMMAND_ERROR_UNKNOWN;
    }

    qObj = (DRV_SDHC_QUEUE_OBJECT *)&gDrvSDHCQueueObj[clientObj->drvIndex];

    index = commandHandle & 0xFFFF;

    if (qObj->bufferPool[index].commandHandle != commandHandle)
    {
        return DRV_SDHC_COMMAND_COMPLETED;
    }

    return qObj->bufferPool[index].status;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SDHC_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SD Card.
    - Media Property
    - Number of Read/Write/Erase regions in the SD Card
    - Number of Blocks and their size in each region of the device

  Precondition:
    The DRV_SDHC_Initialize() routine must have been called for the
    specified SDHC driver instance.

    The DRV_SDHC_Open() routine must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry information.

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SDHC_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_SDHC_CLIENT_OBJ *clientObj;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj != NULL)
    {
        return (&gDrvSDHCMediaGeometry);
    }

    return NULL;
}

// *****************************************************************************
/* Function:
    void DRV_SDHC_EventHandlerSet
    (
        const DRV_HANDLE handle,
        const void * eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed.  
    When a client queues a request for a read or a write operation, it is provided 
    with a handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the queued operation has completed.
    
    The event handler should be set before the client performs any read or write
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_SDHC_Initialize() routine must have been called for the specified
    SDHC driver instance.

    The DRV_SDHC_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    eventHandler - Pointer to the event handler function implemented by the user
    
    context      - The value of parameter will be passed back to the client 
                   unchanged, when the eventHandler function is called. It can
                   be used to identify any client specific data object that 
                   identifies the instance of the client module (for example, 
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Remarks:
    If the client does not want to be notified when the queued operation
    has completed, it does not need to register a callback.
*/

void DRV_SDHC_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SDHC_CLIENT_OBJ *clientObj;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj != NULL)
    {
        /* Set the event handler */
        clientObj->eventHandler = eventHandler;
        clientObj->context = context;
    }
}


// *****************************************************************************
/* Function:
    bool DRV_SDHC_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the SD Card.

  Description:
    This function returns the physical attach status of the SD Card.

  Precondition:
    The DRV_SDHC_Initialize() routine must have been called for the specified 
    SDHC driver instance.

    The DRV_SDHC_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns false if the handle is invalid otherwise returns the attach status
    of the SD Card. Returns true if the SD Card is attached and initialized by the
    SDHC driver otherwise returns false.

  Remarks:
    None.
*/

bool DRV_SDHC_IsAttached
(
    const DRV_HANDLE handle
)
{
    DRV_SDHC_CLIENT_OBJ *clientObj;
    DRV_SDHC_OBJ *dObj;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return false;

    dObj = (DRV_SDHC_OBJ*)clientObj->driverObject;
    return dObj->mediaState;
}
    
// *****************************************************************************
/* Function:
    bool DRV_SDHC_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the SDHC.

  Description:
    This function returns the physical attach status of the SDHC. This function 
    returns true if the SD Card is write protected otherwise it returns false.

  Precondition:
    The DRV_SDHC_Initialize() routine must have been called for the specified 
    SDHC driver instance.

    The DRV_SDHC_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns true if the attached SD Card is write protected.
    Returns false if the handle is not valid, or if the SD Card is not write protected.

  Remarks:
    None.
*/

bool DRV_SDHC_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    DRV_SDHC_CLIENT_OBJ *clientObj;
    DRV_SDHC_OBJ *dObj;

    clientObj = _DRV_SDHC_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return false;

    dObj = (DRV_SDHC_OBJ*)clientObj->driverObject;
    if (dObj->mediaState == SYS_FS_MEDIA_DETACHED)
        return false;

    return dObj->isWriteProtected;
}

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Queue handling functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_SDHC_QUEUE_HANDLE _DRV_SDHC_QueueInitialize 
     ( 
         const SYS_MODULE_INDEX drvIndex
     );

  Summary:
    Initialize the queue.

  Description:
    Intializes the queue bufferpool pointer with that of the transfer object address
    and reset the queue indexes.

  Parameters:
    drvIndex - Index of SD card driver opened.

  Returns:
    A handle to the initialized queue.

  Remarks:
    None.
*/
DRV_SDHC_QUEUE_HANDLE _DRV_SDHC_QueueInitialize 
( 
    const SYS_MODULE_INDEX drvIndex
)
{
    gDrvSDHCQueueObj[drvIndex].bufferPool = &gDrvSDHCTransferObj[drvIndex][0];
    gDrvSDHCQueueObj[drvIndex].startIndex = 0;
    gDrvSDHCQueueObj[drvIndex].endIndex = 0;
    gDrvSDHCQueueObj[drvIndex].size = DRV_SDHC_QUEUE_POOL_SIZE;

    return ((DRV_SDHC_QUEUE_HANDLE)&gDrvSDHCQueueObj[drvIndex].startIndex);
}

// *****************************************************************************
/* Function:
    DRV_SDHC_COMMAND_HANDLE _DRV_SDHC_AddToQueue 
    ( 
        DRV_HANDLE handle,
        DRV_SDHC_TRANSFER_TYPE readWrite,
        uint8_t *buffer,
        uint32_t blockStart,
        uint32_t nBlock
    )
  Summary:
    Adds a new xfer object to the queue.

  Description:
    Adds a new xfer object to the queue if the queue is not full and
    if the media is attached.

  Parameters:
    handle      - Driver Handle
    readWrite   - Indicates if operation is read or write
    buffer      - buffer to be used for the operation
    blockStart  - media start address
    nBlock      - number of blocks

  Returns:
    DRV_SDHC_COMMAND_HANDLE_INVALID - On failure to add to the queue. This could be
                                        be due to either the queue being full or the
                                        media being detached.
    VALID Handle                      - Successfully added to the queue.

  Remarks:
    None.
*/
DRV_SDHC_COMMAND_HANDLE _DRV_SDHC_AddToQueue 
( 
    DRV_HANDLE handle,
    DRV_SDHC_TRANSFER_TYPE readWrite,
    uint8_t *buffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SDHC_QUEUE_OBJECT *qObj;
    unsigned char           index;
    DRV_SDHC_CLIENT_OBJ   *clientObj =(DRV_SDHC_CLIENT_OBJ*)_DRV_SDHC_CLIENT_OBJ_GET(handle);
    DRV_SDHC_OBJ          *dObj = (DRV_SDHC_OBJ *)clientObj->driverObject;

    qObj = (DRV_SDHC_QUEUE_OBJECT*)&gDrvSDHCQueueObj[clientObj->drvIndex];
    index = qObj->endIndex;

    if ((qObj->bufferPool[index].inUse == true) || (dObj->mediaState != SYS_FS_MEDIA_ATTACHED))
        return DRV_SDHC_COMMAND_HANDLE_INVALID;

    qObj->bufferPool[index].inUse         = true;

    qObj->bufferPool[index].hClient       = handle;
    qObj->bufferPool[index].commandHandle = _DRV_SDHC_MAKE_HANDLE(gDrvSDHCBufferToken, index);
    qObj->bufferPool[index].buffer        = buffer;
    qObj->bufferPool[index].readWrite     = readWrite;
    qObj->bufferPool[index].sectorAddress = blockStart;
    qObj->bufferPool[index].sectorCount   = nBlock;
    qObj->bufferPool[index].status        = DRV_SDHC_COMMAND_QUEUED;    
    qObj->bufferPool[index].blockIncr = 0;
    
    qObj->endIndex = (index + 1) % qObj->size;

    _DRV_SDHC_UPDATE_BUF_TOKEN(gDrvSDHCBufferToken);

    return qObj->bufferPool[index].commandHandle;
}

// *****************************************************************************
/* Function:
    DRV_SDHC_XFER_OBJECT* _DRV_SDHC_ReadFromQueue
    (
        DRV_SDHC_QUEUE_HANDLE *handle
    );

  Summary:
    Read the next available object from the queue.

  Description:
    Read the next available object from the queue.

  Parameters:
    handle -    A queue handle.

  Returns:
    NULL                   - If the queue is empty.
    DRV_SDHC_XFER_OBJECT - Pointer to the xfer object.

  Remarks:
    None.
*/

DRV_SDHC_XFER_OBJECT* _DRV_SDHC_ReadFromQueue 
(
    DRV_SDHC_QUEUE_HANDLE handle
)
{
    DRV_SDHC_QUEUE_OBJECT *qObj = (DRV_SDHC_QUEUE_OBJECT*)handle;
    DRV_SDHC_XFER_OBJECT  *tObj;

    /* Check for queue empty */
    if (qObj->bufferPool[qObj->startIndex].inUse == false)
        return NULL;

    tObj = &qObj->bufferPool[qObj->startIndex];

    qObj->startIndex = (qObj->startIndex + 1) % qObj->size;

    return tObj;
}

/*******************************************************************************
End of File
*/

