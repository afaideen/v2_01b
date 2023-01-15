/*******************************************************************************
  SRAM Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sram.c

  Summary:
    SRAM Driver Interface Definition

  Description:
    The SRAM Driver provides a interface to access the SRAM on the PIC32
    microcontroller. This file implements the SRAM Driver interface. This file
    should be included in the project if SRAM driver functionality is needed.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

#include "driver/sram/src/drv_sram_local.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_SRAM_OBJECT        gDrvSRAMObj[DRV_SRAM_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_SRAM_CLIENT_OBJECT gDrvSRAMClientObj[DRV_SRAM_CLIENTS_NUMBER];

/*************************************************
 * Driver Buffer Objects. These transport the
 * read, write and erase requests.
 *************************************************/

DRV_SRAM_BUFFER_OBJECT   gDrvSRAMBufferObject[DRV_SRAM_BUFFER_OBJECT_NUMBER];

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvSRAMBufferToken = 0;

/*************************************************
 * OSAL Declarations
 *************************************************/
/* SRAM Client Object Mutex */
OSAL_MUTEX_DECLARE(sramClientObjMutex);

/* SRAM Buffer Object Mutex*/
OSAL_MUTEX_DECLARE(sramBufObjMutex);

const SYS_FS_MEDIA_FUNCTIONS sramMediaFunctions =
{
    .mediaStatusGet     = DRV_SRAM_IsAttached,
    .mediaGeometryGet   = DRV_SRAM_GeometryGet,
    .sectorRead         = DRV_SRAM_Read,
    .sectorWrite        = DRV_SRAM_Write,
    .eventHandlerset    = DRV_SRAM_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_SRAM_CommandStatus,
    .Read               = DRV_SRAM_Read,
    .erase              = NULL,
    .addressGet         = DRV_SRAM_AddressGet,
    .open               = DRV_SRAM_Open,
    .close              = DRV_SRAM_Close,
    .tasks              = DRV_SRAM_Tasks,
};

// *****************************************************************************
/* Function:
    DRV_SRAM_CLIENT * _DRV_SRAM_ClientHandleValidate(DRV_HANDLE handle);

  Summary:
    Validate the driver handle.

  Description:
    This function validates the driver handle and returns the client object pointer
    associated with the driver handle if the handle is valid. If the driver handle
    is not valid or if the driver is in a not ready state then NULL is returned.

  Remarks:
    None
*/

static DRV_SRAM_CLIENT_OBJECT * _DRV_SRAM_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT * clientObj;
    DRV_SRAM_OBJECT * dObj;

    /* Validate the handle */
    _DRV_SRAM_VALIDATE_EXPR((0 == handle), NULL);
    _DRV_SRAM_VALIDATE_EXPR((DRV_HANDLE_INVALID == handle), NULL);

    /* See if the client has been opened */
    clientObj = (DRV_SRAM_CLIENT_OBJECT *)handle;
    _DRV_SRAM_VALIDATE_EXPR((!clientObj->inUse), NULL);

    /* Check if the driver is ready for operation */
    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;
    _DRV_SRAM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), NULL);

    return clientObj;
}

static void _DRV_SRAM_AddToQueue
(
    DRV_SRAM_OBJECT *dObj,
    DRV_SRAM_BUFFER_OBJECT *bufferObject
)
{
    if (dObj->rwQueue == NULL)
    {
        dObj->rwQueue = bufferObject;
    }
    else
    {
        if (dObj->rwQueue->previous != NULL)
        {
            dObj->rwQueue->previous->next = bufferObject;
            bufferObject->previous = dObj->rwQueue->previous;
            dObj->rwQueue->previous = bufferObject;
        }
        else
        {
            dObj->rwQueue->previous = bufferObject;
            dObj->rwQueue->next = bufferObject;
            bufferObject->previous = dObj->rwQueue;
        }
    }
}

static DRV_SRAM_BUFFER_OBJECT * _DRV_SRAM_ReadFromQueue
(
    DRV_SRAM_OBJECT *dObj
)
{
    DRV_SRAM_BUFFER_OBJECT * bufferObject = dObj->rwQueue;

    if (dObj->rwQueue != NULL)
    {
        if (dObj->rwQueue->next != NULL)
        {
            dObj->rwQueue = dObj->rwQueue->next;
            if (dObj->rwQueue == bufferObject->previous)
            {
                dObj->rwQueue->previous = NULL;
            }
            else
            {
                dObj->rwQueue->previous = bufferObject->previous;
            }
        }
        else
        {
            dObj->rwQueue = NULL;
        }
    }

    return bufferObject;
}

/* This function removes the buffer objects that were queued by a client. */
static void _DRV_SRAM_RemoveClientBufferObjects
(
    const DRV_HANDLE handle,
    DRV_SRAM_OBJECT *dObj
)
{
    DRV_SRAM_BUFFER_OBJECT *bufferObject = NULL;
    DRV_SRAM_BUFFER_OBJECT *lastObject = NULL;
    DRV_SRAM_BUFFER_OBJECT *head = NULL;
    DRV_SRAM_BUFFER_OBJECT *temp = NULL;

    bufferObject = dObj->rwQueue;

    if (dObj->rwQueue != NULL)
    {
        dObj->rwQueue->previous = NULL;
    }

    while (bufferObject != NULL)
    {
        temp = bufferObject->next;
        if (bufferObject->hClient == handle)
        {
            bufferObject->inUse = false;

            if(bufferObject->previous != NULL)
            {
                bufferObject->previous->next = bufferObject->next;
            }

            if(bufferObject->next != NULL)
            {
                bufferObject->next->previous = bufferObject->previous;
            }

            bufferObject->previous = NULL;
            bufferObject->next = NULL;
        }
        else
        {
            if (head == NULL)
            {
                head = bufferObject;
            }

            lastObject = bufferObject;
        }

        bufferObject = temp;
    }

    dObj->rwQueue = head;

    if ((head != NULL) && (head != lastObject))
    {
        dObj->rwQueue->previous = lastObject;
    }
}

static DRV_SRAM_BUFFER_OBJECT * _DRV_SRAM_GetFreeBufferObject
(
    uint16_t *position
)
{
    uint16_t iEntry = 0;
    DRV_SRAM_BUFFER_OBJECT *bufferObj = NULL;

    *position = 0;

    for (iEntry = 0; iEntry < DRV_SRAM_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvSRAMBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvSRAMBufferObject[iEntry];
            *position = iEntry;
            break;
        }
    }

    return bufferObj;
}


// *****************************************************************************
// *****************************************************************************
// Section: SRAM Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SRAM_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Initializes the SRAM instance for the specified driver index

  Description:
    This routine initializes the SRAM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_MODULE_OBJ DRV_SRAM_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal;
    DRV_SRAM_OBJECT *dObj = (DRV_SRAM_OBJECT *)NULL;
    DRV_SRAM_INIT *sramInit = NULL;

    /* Validate the driver index */
    _DRV_SRAM_VALIDATE_EXPR((drvIndex > DRV_SRAM_INSTANCES_NUMBER), SYS_MODULE_OBJ_INVALID);

    /* Validate the init parameter */
    _DRV_SRAM_VALIDATE_EXPR((init == NULL), SYS_MODULE_OBJ_INVALID);

    /* Instance has already been initialized */
    _DRV_SRAM_VALIDATE_EXPR((gDrvSRAMObj[drvIndex].inUse), SYS_MODULE_OBJ_INVALID);

    /* Assign to the local pointer the init data passed */
    sramInit = (DRV_SRAM_INIT *)init;

    retVal = OSAL_MUTEX_Create(&sramClientObjMutex);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);

    retVal = OSAL_MUTEX_Create(&sramBufObjMutex);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);

    dObj = &gDrvSRAMObj[drvIndex];

    /* Indicate that this object is in use */
    dObj->inUse = true;

    /* Initialize number of clients */
    dObj->numClients = 0;

    dObj->blockStartAddress = sramInit->mediaStartAddress;
    dObj->sramMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)sramInit->sramMediaGeometry;

    _DRV_SRAM_RegisterWithSysFs (drvIndex, drvIndex, sramMediaFunctions);

    /* Set the current driver state */
    dObj->status = SYS_STATUS_READY;

    /* Return the driver index as the System Module Object */
    return (SYS_MODULE_OBJ)drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_SRAM_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SRAM driver module

  Description:
    Deinitializes the specified instance of the SRAM driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SRAM_OBJECT *dObj = (DRV_SRAM_OBJECT*)NULL;

    /* Validate the object */
    _DRV_SRAM_VALIDATE_EXPR((SYS_MODULE_OBJ_INVALID == object), (void)0);
    _DRV_SRAM_VALIDATE_EXPR((object >= DRV_SRAM_INSTANCES_NUMBER), (void)0);

    dObj = (DRV_SRAM_OBJECT*)&gDrvSRAMObj[object];

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Reset the queue */
    dObj->rwQueue = NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    OSAL_MUTEX_Delete(&sramClientObjMutex);
    OSAL_MUTEX_Delete(&sramBufObjMutex);

    /* Hardware instance object is no more in use */
    dObj->inUse = false;
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_SRAM_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SRAM driver module.

  Description:
    This routine provides the current status of the SRAM driver module.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_STATUS DRV_SRAM_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    _DRV_SRAM_VALIDATE_EXPR((SYS_MODULE_OBJ_INVALID == object), SYS_STATUS_UNINITIALIZED);
    _DRV_SRAM_VALIDATE_EXPR((object >= DRV_SRAM_INSTANCES_NUMBER), SYS_STATUS_UNINITIALIZED);

    /* Return the driver status */
    return (gDrvSRAMObj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: SRAM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SRAM_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )
    
  Summary:
    Opens the specified SRAM driver instance and returns a handle to it
  
  Description:
    This routine opens the specified SRAM driver instance and provides a handle. 
    This handle must be provided to all other client-level operations to identify
    the caller and the instance of the driver.
  
  Remarks:
    Refer to drv_sram.h for usage information.
*/

DRV_HANDLE DRV_SRAM_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = (DRV_SRAM_CLIENT_OBJECT *)gDrvSRAMClientObj;
    DRV_SRAM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    uint8_t iClient = 0;
    DRV_HANDLE drvHandle = DRV_HANDLE_INVALID;

    /* Validate the driver index */
    _DRV_SRAM_VALIDATE_EXPR((drvIndex >= DRV_SRAM_INSTANCES_NUMBER), DRV_HANDLE_INVALID);

    dObj = &gDrvSRAMObj[drvIndex];

    /* Check if the driver is ready to be opened */
    _DRV_SRAM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), DRV_HANDLE_INVALID);

    /* Check if the driver has already been opened in exclusive mode */
    _DRV_SRAM_VALIDATE_EXPR((dObj->isExclusive), DRV_HANDLE_INVALID);

    /* Driver has already been opened and cannot be opened exclusively */
    _DRV_SRAM_VALIDATE_EXPR(((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE)), DRV_HANDLE_INVALID);

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&sramClientObjMutex,OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), DRV_HANDLE_INVALID);

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_SRAM_CLIENTS_NUMBER; iClient++)
    {
        if (!clientObj->inUse)
        {
            /* Found a client object that can be used */
            clientObj->inUse = true;
            clientObj->driverObj = dObj;
            clientObj->intent = ioIntent;
            clientObj->eventHandler = NULL;

            if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
            {
                /* Exclusive mode of access */
                dObj->isExclusive = true;
            }

            dObj->numClients ++;
            drvHandle = (DRV_HANDLE)clientObj;

            break;
        }

        clientObj += 1;
    }

    OSAL_MUTEX_Unlock(&sramClientObjMutex);

    return drvHandle;
}


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SRAM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the SRAM driver

  Description:
    This routine closes an opened-instance of the SRAM driver, invalidating the
    handle.

  Remarks:
    Refer to drv_sram.h for usage infomration.
*/

void DRV_SRAM_Close
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), (void)0);

    dObj = clientObj->driverObj;

    _DRV_SRAM_RemoveClientBufferObjects (handle, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    return;
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_Read
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Reads blocks of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in memory.
    This operation is non blocking and returns with the required data in the
    target buffer. This function should not be used to read areas of memory 
    which are queued to be programmed or erased. If required, the program or
    erase operations should be allowed to complete. The function returns
    DRV_SRAM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Read
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE * commandHandle,
    void    *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SRAM_OBJECT *dObj = NULL;
    DRV_SRAM_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_BUFFER_OBJECT *bufferObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    uint16_t iEntry = 0;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SRAM_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), (void)0);
    _DRV_SRAM_VALIDATE_EXPR(!(clientObj->intent & DRV_IO_INTENT_READ), (void)0);
    _DRV_SRAM_VALIDATE_EXPR((targetBuffer == NULL), (void)0); 
    _DRV_SRAM_VALIDATE_EXPR((nBlock == 0), (void)0);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;

    _DRV_SRAM_VALIDATE_EXPR(((blockStart + nBlock) > dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_READ_ENTRY].numBlocks), (void)0);

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&sramBufObjMutex, OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), (void)0);

    bufferObj = _DRV_SRAM_GetFreeBufferObject(&iEntry);
    if (bufferObj != NULL)
    {
        bufferObj->inUse         = true;
        bufferObj->commandHandle = _DRV_SRAM_MAKE_HANDLE(gDrvSRAMBufferToken, iEntry);
        bufferObj->hClient       = handle;
        bufferObj->buffer        = targetBuffer;
        bufferObj->address       = dObj->blockStartAddress + (blockStart * dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_READ_ENTRY].blockSize);
        bufferObj->nBlocks       = nBlock;
        bufferObj->operation     = DRV_SRAM_BUFFER_OPERATION_READ;
        bufferObj->status        = DRV_SRAM_COMMAND_IN_PROGRESS;

        /* Update the token number. */
        _DRV_SRAM_UPDATE_BUF_TOKEN(gDrvSRAMBufferToken);

        /* Queue the request */
        bufferObj->next = NULL;
        bufferObj->previous = NULL;

        _DRV_SRAM_AddToQueue (dObj, bufferObj);

        *tempHandle1 = (bufferObj->commandHandle);
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&sramBufObjMutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_Write
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Writes blocks of data starting from the specified address in memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into memory. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SRAM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the number
      of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SRAM_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SRAM_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Write
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SRAM_OBJECT *dObj = NULL;
    DRV_SRAM_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_BUFFER_OBJECT *bufferObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    uint16_t iEntry = 0;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SRAM_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), (void)0);
    _DRV_SRAM_VALIDATE_EXPR(!(clientObj->intent & DRV_IO_INTENT_WRITE), (void)0);
    _DRV_SRAM_VALIDATE_EXPR((sourceBuffer == NULL), (void)0); 
    _DRV_SRAM_VALIDATE_EXPR((nBlock == 0), (void)0);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;

    _DRV_SRAM_VALIDATE_EXPR(((blockStart + nBlock) > dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks), (void)0);

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&sramBufObjMutex, OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), (void)0);

    bufferObj = _DRV_SRAM_GetFreeBufferObject (&iEntry);
    if (bufferObj != NULL)
    {
        bufferObj->inUse         = true;
        bufferObj->commandHandle = _DRV_SRAM_MAKE_HANDLE(gDrvSRAMBufferToken, iEntry);
        bufferObj->hClient       = handle;
        bufferObj->buffer        = sourceBuffer;
        bufferObj->address       = dObj->blockStartAddress + (blockStart * dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_WRITE_ENTRY].blockSize);
        bufferObj->nBlocks       = nBlock;
        bufferObj->operation     = DRV_SRAM_BUFFER_OPERATION_WRITE;
        bufferObj->status        = DRV_SRAM_COMMAND_IN_PROGRESS;

        /* Update the token number. */
        _DRV_SRAM_UPDATE_BUF_TOKEN(gDrvSRAMBufferToken);

        /* Queue the request */
        bufferObj->next = NULL;
        bufferObj->previous = NULL;

        _DRV_SRAM_AddToQueue (dObj, bufferObj);

        *tempHandle1 = (bufferObj->commandHandle);
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&sramBufObjMutex);

    return;
}

// ****************************************************************************
/* Function:
    void DRV_SRAM_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.

  Description:
    This routine is used to maintain the driver's internal write and erase state
    machine and implement its ISR for interrupt-driven implementations.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_SRAM_OBJECT *dObj = NULL;
    DRV_SRAM_BUFFER_OBJECT *bufferObj = NULL;
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    uint8_t *source = NULL;
    uint8_t *destination = NULL;

    uint32_t numBytes = 0;
    uint32_t i = 0;
    uint16_t blockSize = 0;
    
    if(SYS_MODULE_OBJ_INVALID == object)
    {
        return;
    }

    dObj = &gDrvSRAMObj[object];
    if(dObj->status != SYS_STATUS_READY)
    {
        return;
    }

    retVal = OSAL_MUTEX_Lock(&sramBufObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return;
    }

    bufferObj = _DRV_SRAM_ReadFromQueue (dObj);
    do
    {
        if (bufferObj == NULL)
        {
            /* Queue is empty. */
            break;
        }

        if (bufferObj->operation == DRV_SRAM_BUFFER_OPERATION_READ)
        {
            blockSize = dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_READ_ENTRY].blockSize;
            source = (uint8_t *)bufferObj->address;
            destination = bufferObj->buffer;
        }
        else
        {
            blockSize = dObj->sramMediaGeometry->geometryTable[GEOMETRY_TABLE_WRITE_ENTRY].blockSize;
            source = bufferObj->buffer;
            destination = (uint8_t *)bufferObj->address;
        }

        numBytes = bufferObj->nBlocks * blockSize;

        for (i = 0; i < numBytes; i++)
        {
            *destination++ = *source++;
        }

        bufferObj->status = DRV_SRAM_COMMAND_COMPLETED;
        clientObj = (DRV_SRAM_CLIENT_OBJECT *)bufferObj->hClient;
        if(clientObj->eventHandler != NULL)
        {
            clientObj->eventHandler(DRV_SRAM_EVENT_COMMAND_COMPLETE, (DRV_SRAM_COMMAND_HANDLE)bufferObj->commandHandle, clientObj->context);
        }

        if (bufferObj->operation == DRV_SRAM_BUFFER_OPERATION_WRITE)
        {
            /* Invalidate the cache for this address range. */
            SYS_DEVCON_DataCacheInvalidate(bufferObj->buffer, numBytes);
        }

        bufferObj->inUse = false;

        bufferObj = _DRV_SRAM_ReadFromQueue (dObj);

    } while (bufferObj != NULL); 

    OSAL_MUTEX_Unlock(&sramBufObjMutex);

    return;
}

// *****************************************************************************
/* Function:
    DRV_SRAM_COMMAND_STATUS DRV_SRAM_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SRAM_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_SRAM_COMMAND_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the buffer status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

DRV_SRAM_COMMAND_STATUS DRV_SRAM_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_SRAM_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* Validate the client handle */
    _DRV_SRAM_VALIDATE_EXPR((NULL == _DRV_SRAM_ClientHandleValidate(handle)), DRV_SRAM_COMMAND_HANDLE_INVALID);

    /* The upper 16 bits of the buffer handle
     * are the token and the lower 16 bits of the
     * are buffer index into the gDrvSRAMBufferObject
     * array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle
     * in the object */
    if (gDrvSRAMBufferObject[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another
         * request. Indicate that the operation is completed.
         * */
        return (DRV_SRAM_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvSRAMBufferObject[iEntry].status);
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_EventHandlerSet
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
    the driver to call back when queued operation has completed. When a client
    calls a write or erase function, it is provided with a handle identifying
    the buffer that was added to the driver's buffer queue. The driver will 
    pass this handle back to the client by calling "eventHandler" function when
    the queued operation has completed.
    
    The event handler should be set before the client performs any write or erase
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SRAM_CLIENT_OBJECT * clientObj;

    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    /* Check if the client handle is valid */
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), (void)0);

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet( const DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SRAM Media:
    - Media Property
    - Number of Read/Write/Erase regions in the SRAM Media
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), NULL);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;
    
    return (dObj->sramMediaGeometry);
}

// *****************************************************************************
/* Function:
    bool DRV_SRAM_isAttached( const DRV_HANDLE handle );

  Summary:
    Returns the physical attach status of the SRAM.

  Description:
    This function returns the physical attach status of the SRAM. This
    function returns false if the driver handle is invalid otherwise returns
    true.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

bool DRV_SRAM_IsAttached
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    _DRV_SRAM_VALIDATE_EXPR((NULL == _DRV_SRAM_ClientHandleValidate(handle)), false);

   return true;
}

// *****************************************************************************
/* Function:
    bool DRV_SRAM_isWriteProtected( const DRV_HANDLE handle );

  Summary:
    Returns the write protect status of SRAM.

  Description:
    This function returns the write protect status of the SRAM. Always returns
    false.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

bool DRV_SRAM_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_SRAM_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the SRAM media start address

  Description:
    This function returns the SRAM Media start address.

  Example:
    <code>

    uintptr_t startAddress;
    startAddress = DRV_SRAM_AddressGet(drvSRAMHandle);

    </code>

  Remarks:
    None.
*/
uintptr_t DRV_SRAM_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = _DRV_SRAM_ClientHandleValidate(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), NULL);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;
    
    return (dObj->blockStartAddress);
}

