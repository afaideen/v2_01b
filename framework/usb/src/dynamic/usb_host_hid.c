/*******************************************************************************
  USB Host HID client driver implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hid.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host HID client driver.

  Description:
    This file contains the USB host HID client driver implementation. This file 
    should be included in the project if USB HID Host functionality is 
    required to support attached HID devices.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host_hid.h"
#include "usb/src/usb_host_hid_local.h"
#include "usb/src/usb_host_local.h"
#include "usb/usb_host_client_driver.h"
#include "system/debug/sys_debug.h"
#include <p32xxxx.h>


void _USB_HOST_HID_Initialize(void * hidInitData);
void _USB_HOST_HID_Deinitialize(void);
void _USB_HOST_HID_Reinitialize(void * hidInitData);
void _USB_HOST_HID_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor
);
void _USB_HOST_HID_InterfaceRelease(USB_HOST_DEVICE_INTERFACE_HANDLE interface);
USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HID_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
);
void _USB_HOST_HID_InterfaceTasks(USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle);


USB_HOST_CLIENT_DRIVER gUSBHostHIDClientDriver = 
{
    .initialize = _USB_HOST_HID_Initialize,
    .deinitialize = _USB_HOST_HID_Deinitialize,
    .reinitialize = _USB_HOST_HID_Reinitialize,
    .interfaceAssign = _USB_HOST_HID_InterfaceAssign,
    .interfaceRelease = _USB_HOST_HID_InterfaceRelease,
    .interfaceEventHandler = _USB_HOST_HID_InterfaceEventHandler,
    .interfaceTasks = _USB_HOST_HID_InterfaceTasks,
    .deviceEventHandler = NULL,
    .deviceAssign = NULL,
    .deviceEventHandler = NULL,
    .deviceRelease = NULL
         
};

/**************************************************
 * Global pointer pointing to Usage Driver initialization
 * table. It contains all the usage drivers registered
 * along with their interface functions.
 **************************************************/
USB_HOST_HID_INIT * gUSBHostHIDInitData = NULL;

/**************************************************
 * Global array of HID Handle Objects. Each for
 * one HID successful object handle assign call.
 **************************************************/
USB_HOST_HID_OBJECT_HANDLE_POOL gUSBHostHIDObjectHandlePool
        [USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER];

/**************************************************
 * Global array of HID Instance Objects. Each for
 * one HID interface attached.
 **************************************************/
USB_HOST_HID_INSTANCE  gUSBHostHIDInstance[USB_HOST_HID_INSTANCES_NUMBER];

// *****************************************************************************
// *****************************************************************************
// USB Host HID Local Functions
// *****************************************************************************
// *****************************************************************************


/*************************************************************************/
/* Function:
    USB_HOST_HID_OBJ_HANDLE  _USB_HOST_HID_ObjectHandleAssign
    (
        USB_HOST_HID_INDEX  instance,
        uint32_t usage
    )

  Summary:
    Function assigns unique handle specific to top level usage that the usage
    driver owns.

  Description:
    Function assigns unique handle for each of the usages present.

  Remarks:
    None.
*/

USB_HOST_HID_OBJ_HANDLE  _USB_HOST_HID_ObjectHandleAssign
(
    USB_HOST_HID_INDEX  instance,
    uint32_t usage,
    uint8_t index
)
{
    /* Start of local variables */
    USB_HOST_HID_INSTANCE  *hidInstanceInfo = (USB_HOST_HID_INSTANCE  *) NULL;
    USB_HOST_HID_OBJ_HANDLE hidHandle = USB_HOST_HID_OBJ_HANDLE_INVALID;
    uint8_t hidHandleCount = 0;
    /* End of local variables */

    if(instance >= USB_HOST_HID_INSTANCES_NUMBER)
    {
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO ,
                "\r\nUSBHID Client Driver: Invalid HID client driver index");
    }
    else
    {   
        /* Get instance driver information */
        hidInstanceInfo = &(gUSBHostHIDInstance[instance]);

        /* Check if instance has been initialized */    
        if(hidInstanceInfo->assigned == true)
        {
            /* Find free index from the handle pool */
            for (hidHandleCount = 0;
                    hidHandleCount < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                    hidHandleCount++)
            {
                if(gUSBHostHIDObjectHandlePool[hidHandleCount].inUse == false)
                {
                    /* Grab index from the open pool */
                    gUSBHostHIDObjectHandlePool[hidHandleCount].inUse = true;
                    gUSBHostHIDObjectHandlePool[hidHandleCount].usage = usage;
                    gUSBHostHIDObjectHandlePool[hidHandleCount].hidInstanceIndex =
                            instance;
                    gUSBHostHIDObjectHandlePool[hidHandleCount].usageInstanceIndex
                            = index;
                    /* Update Valid handle */
                    hidHandle =
                            (USB_HOST_HID_OBJ_HANDLE)
                            (&(gUSBHostHIDObjectHandlePool[hidHandleCount]));
                    break;
                }
            }
        }
        else
        {
            /* Instance not yet initialized */
            SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO,
                    "\r\nUSBHID Client Driver: HID Instance not yet initialized");
        }
    }
    /*
     * On success: Valid USB_HOST_HID_OBJ_HANDLE 
     * On failure: USB_HOST_HID_OBJ_HANDLE_INVALID
     */
    return hidHandle;

}/* End of _USB_HOST_HID_ObjectHandleAssign() */


/*************************************************************************/
/* Function:
    void _USB_HOST_HID_ObjectHandleRelease (USB_HOST_HID_OBJ_HANDLE handle)

  Summary:
    Function releases unique object handle assigned.

  Description:
    Function releases unique object handle assigned.

  Remarks:
    None.
*/

void _USB_HOST_HID_ObjectHandleRelease (USB_HOST_HID_OBJ_HANDLE handle)
{
    /* Start of local variables */
    uint8_t hidHandleCount = 0;
    /* End of local variables */

    if(handle != USB_HOST_HID_OBJ_HANDLE_INVALID)
    {
        for (hidHandleCount = 0;
                hidHandleCount < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                hidHandleCount++)
        {
            /* Release the handle only if handle matches in the pool and
             that pool entry has not yet been released. */
            if((&gUSBHostHIDObjectHandlePool[hidHandleCount] ==
                    (USB_HOST_HID_OBJECT_HANDLE_POOL *) handle) &&
                    (gUSBHostHIDObjectHandlePool[hidHandleCount].inUse == true))
            {
                /* Release it from the unique handle pool */
                gUSBHostHIDObjectHandlePool[hidHandleCount].inUse = false;
                break;
            }
        }
    }
    else
    {
        SYS_DEBUG_MESSAGE ( SYS_ERROR_INFO ,
                "\r\nUSBHID Client Driver: Invalid HID object handle");
    }

} /* End of _USB_HOST_HID_ObjectHandleRelease() */


// *****************************************************************************
/* Function:
   int8_t _USB_HOST_HID_InterfaceHandleToHIDIndex
   ( 
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
   )

  Summary:
    This function will return the HID instance array index that is associated
    with this interface.

  Description:
    This function will return the HID instance array index that is associated
    with this interface.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

int8_t _USB_HOST_HID_InterfaceHandleToHIDIndex
( 
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    /* Start of local variables */
    uint8_t iterator = 0;
    int8_t hidInstanceIndex = (-1);
    /* End of local variables */

    /* Find the HID Instance object that owns this interface */
    for (iterator = 0; iterator < USB_HOST_HID_INSTANCES_NUMBER; iterator ++)
    {
        if(gUSBHostHIDInstance[iterator].assigned)
        {
            if(gUSBHostHIDInstance[iterator].interfaceHandle == interfaceHandle)
            {
                /* Found the appropriate HID instance object. Store the index */
                hidInstanceIndex = (int8_t)iterator;
                break;
            }
        }
    }

    /*
     * On success: Valid hidInstanceIndex
     * On failure: (-1)
     */
    return hidInstanceIndex;
    
} /* End of _USB_HOST_HID_InterfaceHandleToHIDIndex() */


int8_t _USB_HOST_HID_ObjectHandleToHIDIndex(USB_HOST_HID_OBJ_HANDLE handle)
{
    /* Start of local variables */
    uint8_t iterator = 0;
    int8_t hidInstanceIndex = (-1);
    /* End of local variables */
    
    /* Find the HID Instance object that owns this interface */
    for (iterator = 0; iterator < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
            iterator ++)
    {
        if(gUSBHostHIDObjectHandlePool[iterator].inUse && 
                &gUSBHostHIDObjectHandlePool[iterator] ==
                            (USB_HOST_HID_OBJECT_HANDLE_POOL *)handle)
        {
            hidInstanceIndex =
                    gUSBHostHIDObjectHandlePool[iterator].hidInstanceIndex;
            break;
        }
    }

    /*
     * On success: Valid hidInstanceIndex
     * On failure: (-1)
     */
    return hidInstanceIndex;
    
} /* End of _USB_HOST_HID_ObjectHandleToHIDIndex() */


int8_t _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex
(
    USB_HOST_HID_OBJ_HANDLE handle
)
{
    /* Start of local variables */
    uint8_t iterator = 0;
    int8_t usageInstanceIndex = (-1);
    /* End of local variables */
    
    /* Find the HID Pool object that owns this interface */
    for (iterator = 0; iterator < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
            iterator ++)
    {
        if(gUSBHostHIDObjectHandlePool[iterator].inUse && 
                &gUSBHostHIDObjectHandlePool[iterator] ==
                            (USB_HOST_HID_OBJECT_HANDLE_POOL *)handle)
        {
            usageInstanceIndex =
                    gUSBHostHIDObjectHandlePool[iterator].usageInstanceIndex;
            break;
        }
    }

    /*
     * On success: Valid usageInstanceIndex
     * On failure: (-1)
     */
    return usageInstanceIndex;
    
} /* End of _USB_HOST_HID_ObjectHandleToHIDIndex() */

// *****************************************************************************
/* Function:
   USB_HOST_HID_RESULT _USB_HOST_HID_FindTopLevelUsage
   (
       uint8_t hidInstanceIndex
   )

  Summary:
    This function will traverse across the entire Report Descriptor and will
    extract the top level Usages present.

  Description:
    This function will traverse across the entire Report Descriptor and will
    extract the top level Usages present.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

USB_HOST_HID_RESULT _USB_HOST_HID_FindTopLevelUsage
(
    uint8_t hidInstanceIndex
)
{
    /* Start of local variables */
    uint8_t mainItemIndex = 1;
    USB_HOST_HID_RESULT result = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_HID_LOCAL_ITEM localItem;
    USB_HOST_HID_MAIN_ITEM mainItem;
    USB_HOST_HID_GLOBAL_ITEM globalItem;
    /* End of local variables */
    
    
    if(!gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing)
    {
        gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = true;
        /* Flag identifying that Report descriptor parsing is
         * because of extraction of top level Usages and not
         * Usage driver initiated
         */
        gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing = true;    
        
        mainItem.localItem = &localItem;
        mainItem.globalItem =  &globalItem;
        gUSBHostHIDInstance[hidInstanceIndex].mainItemData = &mainItem;
        
        /* Reset global items only once as they are applicable through out */
        memset(&globalItem, 0, (size_t)sizeof(USB_HOST_HID_GLOBAL_ITEM));
        
        while(gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing
                == true)
        {
            /* Reset the field data except global items */
            memset(&localItem, 0, (size_t)sizeof(USB_HOST_HID_LOCAL_ITEM));
            memset(&(mainItem.data), 0,
                    (size_t)sizeof(USB_HID_MAIN_ITEM_OPTIONAL_DATA));
            mainItem.tag = 0;
            
            /* Extract the Main item from the Report Descriptor having index
             * mainItemIndex starting from 1 */ 
            if(USB_HOST_HID_RESULT_SUCCESS != _USB_HOST_HID_ItemGet
                                    (
                                        /* HID Client driver instance Index */
                                        (uint8_t)hidInstanceIndex,
                                        /* Main Item index (e.g. 1,2,3...)*/
                                        mainItemIndex,
                                        /* Not Usage driver initiated call */
                                        false,
                                        /* Not Usage driver initiated so field
                                         * count is 0 */
                                        0,
                                        /* Not Usage driver initiated.
                                         * Internal data structure will be used */
                                        NULL,
                                        /* Not Usage driver initiated so query
                                         * type is not applicable */
                                        0
                                    ))
            {
                /* Scanned all main items available for top level usages.
                 * There is no more Main item available */
                gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing =
                        false;
                gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
                result = USB_HOST_HID_RESULT_SUCCESS;
                break;;
            }
            /* Increment the Main item index */
            mainItemIndex++;
        } /* end of while() */
    }
    else
    {
        /* Return busy */
        result = USB_HOST_HID_RESULT_REQUEST_BUSY;
    }
    
    return result;

} /* End of _USB_HOST_HID_FindTopLevelUsage() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_ControlTransferCallback
    (
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        USB_HOST_REQUEST_HANDLE requestHandle,
        USB_HOST_RESULT result,
        size_t size,
        uintptr_t context
    );

  Summary:
    This function is called when a control transfer completes.

  Description:
    This function is called when a control transfer completes.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_ControlTransferCallback
(
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    USB_HOST_REQUEST_HANDLE requestHandle,
    USB_HOST_RESULT result,
    size_t size,
    uintptr_t context
)
{
    /* Start of local variables */
    USB_HOST_HID_INSTANCE * hidInstanceInfo = NULL;
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY * usageDriverTableEntry = NULL;
    USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA responseData;
    /* End of Local variables */
    
    /* The context will be a pointer to the HID instance */
    hidInstanceInfo = (USB_HOST_HID_INSTANCE *)(context);
    
    /* Update the request object with the result */
    hidInstanceInfo->requestObj.result = result;
    hidInstanceInfo->requestObj.size = size;

    /* Reset the controlRequestDone flag signifying that there is
     * no ongoing CONTROL request */
    hidInstanceInfo->requestObj.controlRequestDone = true;

    switch ( hidInstanceInfo->requestObj.setupPacket.bRequest)
    {
        case USB_HID_REQUESTS_SET_IDLE:
            /*
             * usageDriverRequest = true: SET IDLE CONTROL request was
             * send due to Usage driver initiated requests.
             *
             * usageDriverRequest = false: SET IDLE CONTROL request was
             * send as part of HID specific enumeration flow.
             */
            if(hidInstanceInfo->requestObj.usageDriverRequest == true)
            {
                /* We are here because of usage driver request */
                hidInstanceInfo->requestObj.usageDriverRequest = false;
                usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
                responseData.handle = requestHandle;
                if(result != USB_HOST_RESULT_SUCCESS)
                {
                    /* Failure case */
                    if(result == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        responseData.result = 
                                USB_HOST_HID_RESULT_REQUEST_STALLED;
                    }
                    else
                    {
                        responseData.result = USB_HOST_HID_RESULT_FAILURE;
                    }
                }
                else
                {
                    /* Success case */
                    responseData.result = USB_HOST_HID_RESULT_SUCCESS;
                } 
                if(usageDriverTableEntry != NULL &&
                (usageDriverTableEntry + 
                        hidInstanceInfo->requestObj.usageDriverTableIndex)
                    != NULL)
                {
                    ((usageDriverTableEntry + 
                            hidInstanceInfo->requestObj.usageDriverTableIndex)
                            ->interface)->usageDriverEventHandler
                            (
                                hidInstanceInfo->requestObj.handle,
                                USB_HOST_HID_EVENT_SET_IDLE_TIME_COMPLETE,
                                &responseData
                            );
                }

            }
            else
            {
                /* We are here as part of HID specific enumeration.
                 * SET IDLE has been done. Next step is to send SET PROTOCOL
                 * request. Changing the state accordingly.
                 */
                hidInstanceInfo->state = USB_HOST_HID_STATE_SET_PROTOCOL_SEND;
                if (result == USB_HOST_RESULT_SUCCESS)
                {
                    /* SET IDLE passed */
                }
                else
                {
                    /* SET IDLE failed. As per specification SET IDLE is optional
                     * for both boot and non-boot mouse.For e.g. Microsoft mouse
                     * does NOT support SET IDLE request and STALLs it.
                     * However for keyboard it is mandatory. */

                    /* Check if it is a keyboard */
                    if(1 == (hidInstanceInfo->hidDeviceInfo.isKeyboardDevice))
                    {
                        /* Failure case. There is no point proceeding with
                         * this HID interface. So we detach the interface. */
                        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                                "\r\nUSBHID Client Driver: SET IDLE failed for keyboard type device");
                        hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                    }
                } /* end of(SET IDLE failed) */

            } /* end of (Enumeration flow) */

            break;
        
        case USB_HID_REQUESTS_SET_PROTOCOL :
            /*
             * usageDriverRequest = true: SET PROTOCOL CONTROL request was
             * send due to Usage driver initiated requests.
             *
             * usageDriverRequest = false: SET PROTOCOL CONTROL request was
             * send as part of HID specific enumeration flow.
             */
            if(hidInstanceInfo->requestObj.usageDriverRequest == true)
            {
                /* We are here because of usage driver request */
                hidInstanceInfo->requestObj.usageDriverRequest = false;
                usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
                responseData.handle = requestHandle;
                if(result != USB_HOST_RESULT_SUCCESS)
                {
                    /* Failure case */
                    if(result == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        responseData.result = 
                                USB_HOST_HID_RESULT_REQUEST_STALLED;
                    }
                    else
                    {
                        responseData.result = USB_HOST_HID_RESULT_FAILURE;
                    }
                }
                else
                {
                    /* Success case */
                    responseData.result = USB_HOST_HID_RESULT_SUCCESS;
                } 
                if(usageDriverTableEntry != NULL &&
                (usageDriverTableEntry + 
                        hidInstanceInfo->requestObj.usageDriverTableIndex)
                    != NULL)
                {
                    ((usageDriverTableEntry + 
                            hidInstanceInfo->requestObj.usageDriverTableIndex)
                            ->interface)->usageDriverEventHandler
                            (
                                hidInstanceInfo->requestObj.handle,
                                USB_HOST_HID_EVENT_SET_PROTOCOL_COMPLETE,
                                &responseData
                            );
                }
            }
            else
            {
                /* We are here as part of HID specific enumeration.
                 * SET PROTOCOL has been done. Next step is to send GET REPORT
                 * DESCRIPTOR request. Changing the state accordingly.
                 */
                hidInstanceInfo->state = USB_HOST_HID_STATE_REPORT_DESCRIPTOR_GET;
                if (!(result == USB_HOST_RESULT_SUCCESS))
                {
                    /* SET PROTOCOL request has failed. It can fail for mouse
                     * as it is optional for mouse. Howver for Keyboard it is
                     * mandatory. For NON boot devices it is optional.
                     */
                    if(1 == (hidInstanceInfo->hidDeviceInfo.isKeyboardDevice))
                    {
                        /* Failure case. There is no point proceeding with
                         * this HID interface. So we detach the interface. */
                        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                                "\r\nUSBHID Client Driver: SET PROTOCOL failed for keyboard type device");
                        hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                    }
                } /* end of(SET PROTOCOL failed) */

            } /* end of (Enumeration flow) */

            break;

            case USB_HID_REQUESTS_GET_IDLE :
            if(hidInstanceInfo->requestObj.usageDriverRequest == true)
            {
                /* We are here because of usage driver request */
                hidInstanceInfo->requestObj.usageDriverRequest = false;
                usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
                responseData.handle = requestHandle;
                if(result != USB_HOST_RESULT_SUCCESS)
                {
                    /* Failure case */
                    if(result == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        responseData.result = 
                                USB_HOST_HID_RESULT_REQUEST_STALLED;
                    }
                    else
                    {
                        responseData.result = USB_HOST_HID_RESULT_FAILURE;
                    }
                }
                else
                {
                    /* Success case */
                    responseData.result = USB_HOST_HID_RESULT_SUCCESS;
                } 
                if(usageDriverTableEntry != NULL &&
                (usageDriverTableEntry + 
                        hidInstanceInfo->requestObj.usageDriverTableIndex)
                    != NULL)
                {
                    ((usageDriverTableEntry + 
                            hidInstanceInfo->requestObj.usageDriverTableIndex)
                            ->interface)->usageDriverEventHandler
                            (
                                hidInstanceInfo->requestObj.handle,
                                USB_HOST_HID_EVENT_GET_IDLE_TIME_COMPLETE,
                                &responseData
                            );
                }
            }
            else
            {
                /* The code flow can come here only as part of HID specific
                 * enumeration. But GET IDLE TIME is not used in
                 * enumeration process. Hence under no circumstances
                 * the code should come to the else loop.
                 */
            }
            break;
        case USB_HID_REQUESTS_GET_PROTOCOL :
            if(hidInstanceInfo->requestObj.usageDriverRequest == true)
            {
                /* We are here because of usage driver request */
                hidInstanceInfo->requestObj.usageDriverRequest = false;
                usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
                responseData.handle = requestHandle;
                if(result != USB_HOST_RESULT_SUCCESS)
                {
                    /* Failure case */
                    if(result == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        responseData.result = 
                                USB_HOST_HID_RESULT_REQUEST_STALLED;
                    }
                    else
                    {
                        responseData.result = USB_HOST_HID_RESULT_FAILURE;
                    }
                }
                else
                {
                    /* Success case */
                    responseData.result = USB_HOST_HID_RESULT_SUCCESS;
                } 
                if(usageDriverTableEntry != NULL &&
                (usageDriverTableEntry + 
                        hidInstanceInfo->requestObj.usageDriverTableIndex)
                    != NULL)
                {
                    ((usageDriverTableEntry + 
                            hidInstanceInfo->requestObj.usageDriverTableIndex)
                            ->interface)->usageDriverEventHandler
                            (
                                hidInstanceInfo->requestObj.handle,
                                USB_HOST_HID_EVENT_GET_PROTOCOL_COMPLETE,
                                &responseData
                            );
                }
            }
            else
            {
                /* The code flow can come here only as part of HID specific
                 * enumeration. But GET PROTOCOL is not used in
                 * enumeration process. Hence under no circumstances
                 * the code should come to the else loop.
                 */
            }
            break;
        case USB_REQUEST_GET_DESCRIPTOR:
            /*
             * GET REPORT DESCRIPTOR has been done. Next step is to parse
             * report descriptor. Changing the state accordingly.
             */
            hidInstanceInfo->state = USB_HOST_HID_STATE_REPORT_DESCRIPTOR_PARSE;
            if (!(result == USB_HOST_RESULT_SUCCESS))
            {
                /* This is a mandatory request. If it does not pass,
                 * HID will release this interface.
                 */
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: GET REPORT DESCRIPTOR failed");
                hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
            }
            break;
        case USB_HID_REQUESTS_SET_REPORT:
            if(hidInstanceInfo->requestObj.usageDriverRequest == true)
            {
                /* We are here because of usage driver request */
                hidInstanceInfo->requestObj.usageDriverRequest = false;
                usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
                responseData.handle = requestHandle;
                if(result != USB_HOST_RESULT_SUCCESS)
                {
                    /* Failure case */
                    if(result == USB_HOST_RESULT_REQUEST_STALLED)
                    {
                        responseData.result = 
                                USB_HOST_HID_RESULT_REQUEST_STALLED;
                    }
                    else
                    {
                        responseData.result = USB_HOST_HID_RESULT_FAILURE;
                    }
                }
                else
                {
                    /* Success case */
                    responseData.result = USB_HOST_HID_RESULT_SUCCESS;
                } 
                if(usageDriverTableEntry != NULL &&
                (usageDriverTableEntry + 
                        hidInstanceInfo->requestObj.usageDriverTableIndex)
                    != NULL)
                {
                    ((usageDriverTableEntry + 
                            hidInstanceInfo->requestObj.usageDriverTableIndex)
                            ->interface)->usageDriverEventHandler
                            (
                                hidInstanceInfo->requestObj.handle,
                                USB_HOST_HID_EVENT_REPORT_SENT,
                                &responseData
                            );
                }
            }
            else
            {
                /* The code flow can come here only as part of HID specific
                 * enumeration. But SET REPORT is not used in
                 * enumeration process. Hence under no circumstances
                 * the code should come to the else loop.
                 */
            }
            break;
        case USB_REQUEST_CLEAR_FEATURE:
            if (!(result == USB_HOST_RESULT_SUCCESS))
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: CLEAR ENDPOINT request failed");
                hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
            }
            else
            {
                hidInstanceInfo->state = USB_HOST_HID_STATE_READY;
            }
            break;
        default:
            break;
    } /* end of switch() */

} /* End of _USB_HOST_HID_ControlTransferCallback() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_SetIdlePacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber,
       uint8_t idleTime,
       uint8_t reportID
   )

  Summary:
    This function will create the SET IDLE setup packet

  Description:
    This function will create the SET IDLE setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_SetIdlePacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   uint8_t idleTime,
   uint8_t reportID
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_SET_IDLE ;
        /* Upper byte = IDLE TIME; Lower byte = REPORT ID */
        requestObj->setupPacket.wValue =
        ((((uint16_t)idleTime) << 8) | reportID);
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        requestObj->setupPacket.wLength = 0x00; /* No Data Phase */
    }
} /* End of _USB_HOST_HID_SetIdlePacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_GetIdlePacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber,
       uint8_t reportID
   )

  Summary:
    This function will create the GET IDLE setup packet

  Description:
    This function will create the GET IDLE setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_GetIdlePacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   uint8_t reportID
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_GET_IDLE;
        /* Upper byte = 0; Lower byte = REPORT ID */
        requestObj->setupPacket.wValue = (0x0000 | reportID);
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        requestObj->setupPacket.wLength = 0x01; /* 1 byte CONTROL IN data phase */
    }
} /* End of _USB_HOST_HID_GetIdlePacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_GetProtocolPacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber
   )

  Summary:
    This function will create the GET PROTOCOL setup packet

  Description:
    This function will create the GET PROTOCOL setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_GetProtocolPacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_RECIPIENT_INTERFACE );
        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_GET_PROTOCOL ;
        /* Upper byte = 0; Lower byte = 0 */
        requestObj->setupPacket.wValue = 0;
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        /* 1 byte CONTROL IN data phase */
        requestObj->setupPacket.wLength = 0x01;
    }
} /* End of _USB_HOST_HID_GetProtocolPacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_SetProtocolPacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber,
       USB_HID_PROTOCOL_TYPE protocolType
   )

  Summary:
    This function will create the SET PROTOCOL setup packet

  Description:
    This function will create the SET PROTOCOL setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_SetProtocolPacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   USB_HID_PROTOCOL_TYPE protocolType
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_SET_PROTOCOL;
        /* 1 = Report Protocol, 0 = Boot Protocol */
        requestObj->setupPacket.wValue = (uint16_t)protocolType;
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        requestObj->setupPacket.wLength = 0x00; /* No Data Phase */
    }
} /* End of _USB_HOST_HID_SetProtocolPacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_GetReportDescPacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber,
       uint16_t reportDescLength
   )

  Summary:
    This function will create the GET REPORT DESCRIPTOR setup packet

  Description:
    This function will create the GET REPORT DESCRIPTOR setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_GetReportDescPacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   uint16_t reportDescLength
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_DIRN_DEVICE_TO_HOST |
            USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_REQUEST_GET_DESCRIPTOR;
        /* Descriptor Type = Report Descriptor */
        requestObj->setupPacket.wValue =
                (0x0000 | USB_HID_DESCRIPTOR_TYPES_REPORT) << 8;
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        /* Size of report descriptor */
        requestObj->setupPacket.wLength = reportDescLength;
    }
} /* End of _USB_HOST_HID_GetReportDescPacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_SetReportPacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber
       USB_HID_REPORT_TYPE reportType,
       uint8_t reportID,
       uint16_t reportLength
   )

  Summary:
    This function will create the SET REPORT setup packet

  Description:
    This function will create the SET REPORT setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_SetReportPacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   USB_HID_REPORT_TYPE reportType,
   uint8_t reportID,
   uint16_t reportLength
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_SET_REPORT;
        requestObj->setupPacket.wValue = (((uint16_t)reportType) << 8) | reportID;
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        requestObj->setupPacket.wLength = reportLength;
    }
} /* End of _USB_HOST_HID_SetReportPacketCreate() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_GetReportPacketCreate
   ( 
       USB_HOST_HID_REQUEST * requestObj,
       uint8_t bInterfaceNumber
       USB_HID_REPORT_TYPE reportType,
       uint8_t reportID,
       uint16_t reportLength
   )

  Summary:
    This function will create the SET REPORT setup packet

  Description:
    This function will create the SET REPORT setup packet

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_GetReportPacketCreate
(
   USB_HOST_HID_REQUEST * requestObj,
   uint8_t bInterfaceNumber,
   USB_HID_REPORT_TYPE reportType,
   uint8_t reportID,
   uint16_t reportLength
)
{
    if(requestObj != NULL)
    {
        /* Fill setup packet */
        requestObj->setupPacket.bmRequestType  = ( USB_SETUP_TYPE_CLASS |
            USB_SETUP_DIRN_DEVICE_TO_HOST | USB_SETUP_RECIPIENT_INTERFACE );

        /* Setup the other setup packet values */
        requestObj->setupPacket.bRequest =  USB_HID_REQUESTS_SET_REPORT;
        requestObj->setupPacket.wValue = (((uint16_t)reportType) << 8) | reportID;
        requestObj->setupPacket.wIndex = bInterfaceNumber;
        requestObj->setupPacket.wLength = reportLength;
    }
} /* End of _USB_HOST_HID_GetReportPacketCreate() */


// ******************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeSet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint8_t idleTime,
        uint8_t reportID,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle
    )
 
  Summary:
    SET IDLE request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending SET IDLE request.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeSet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t idleTime,
    uint8_t reportID,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result =  USB_HOST_RESULT_FAILURE;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE * hidInstanceInfo = 
        (USB_HOST_HID_INSTANCE *) USB_HOST_HID_OBJ_HANDLE_INVALID;
    int8_t instanceNumber = (-1);
    /* End of local variable */

    if(requestHandle != NULL)
    {
        /* Initialize requestHandle only if NOT NULL */
        requestHandle =
            (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
    }
    if(handle == USB_HOST_HID_OBJ_HANDLE_INVALID || (requestHandle == NULL))
    {
        /* Invalid Parameter */
        status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Obtain instance number of HID driver instance which owns this
         * handle */
        instanceNumber = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);

        if(instanceNumber > (-1))
        {
            hidInstanceInfo = &(gUSBHostHIDInstance[instanceNumber]);
            
            if(hidInstanceInfo->requestObj.controlRequestDone == true)
            {
                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                hidInstanceInfo->requestObj.controlRequestDone = false;

                /* Create the standard USB packet for SET IDLE request */
                _USB_HOST_HID_SetIdlePacketCreate
                    (
                         /* PLACE HOLDER FOR SET UP PACKET */
                         &hidInstanceInfo->requestObj,
                         /* INTERFACE NUMBER */
                         hidInstanceInfo->bInterfaceNumber,
                         /* IDLE TIME */
                         idleTime,
                         /* REPORT ID */
                         reportID
                    );

                hidInstanceInfo->requestObj.usageDriverRequest = true;
                hidInstanceInfo->requestObj.usageDriverTableIndex = 
                    _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex(handle);

                if(hidInstanceInfo->requestObj.usageDriverTableIndex < 0)
                {
                    /* Reset the flags and status */
                    hidInstanceInfo->requestObj.controlRequestDone = true;
                    hidInstanceInfo->requestObj.usageDriverRequest = false;
                    status = USB_HOST_HID_RESULT_FAILURE;
                }

                else if(hidInstanceInfo->requestObj.controlRequestDone == false)
                {
                    hidInstanceInfo->requestObj.handle =  handle;
                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                            (
                                 /* CONTROL pipe handle */
                                 hidInstanceInfo->controlPipeHandle,
                                 /* Transfer Handle */
                                 &transferHandle,
                                 /* SET IDLE set up packet */
                                 &hidInstanceInfo->requestObj.setupPacket,
                                 /* No data phase */
                                 NULL,
                                 /* CONTROL Callback */
                                 _USB_HOST_HID_ControlTransferCallback,
                                 /* Context */
                                 (uintptr_t)(hidInstanceInfo)
                            );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                        requestHandle =
                            (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                        status = USB_HOST_HID_RESULT_SUCCESS;
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        hidInstanceInfo->requestObj.usageDriverRequest = false;
                        
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* Ongoing control transfer being handled by the host
                               layer */
                            status = USB_HOST_HID_RESULT_REQUEST_BUSY;
                        }
                        else
                        {
                            status = USB_HOST_HID_RESULT_FAILURE;
                        }
                    } /* Request submission failed */
                } /* if(controlRequestDone == false) */
            }/* if(controlRequestDone == true) */
            else
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
        } /* if(instanceNumber > (-1)) */
        else
        {
            /* Invalid instance number */
            status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
    }/* end of else() */

    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of USB_HOST_HID_IdleTimeSet() */


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ProtocolSet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HID_PROTOCOL_TYPE  protocolType,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle
    );
 
  Summary:
    SET PROTOCOL request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending SET PROTOCOL request.

  Remarks:
    This request is supported by devices in the Boot subclass. Non-boot
    devices may STALL this request.
*/

USB_HOST_HID_RESULT USB_HOST_HID_ProtocolSet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HID_PROTOCOL_TYPE  protocolType,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result =  USB_HOST_RESULT_FAILURE;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE *hidInstanceInfo = 
    (USB_HOST_HID_INSTANCE *) USB_HOST_HID_OBJ_HANDLE_INVALID;
    int8_t instanceNumber = (-1);
    /* End of local variable */

    if(requestHandle != NULL)
    {
        /* Initialize requestHandle only if NOT NULL */
        requestHandle =
            (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
    }
    if(handle == USB_HOST_HID_OBJ_HANDLE_INVALID || (requestHandle == NULL))
    {
        /* Invalid Parameter */
        status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Obtain instance number of HID driver instance which owns this
         * handle */
        instanceNumber = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
        
        if(instanceNumber > (-1))
        {
            hidInstanceInfo = &(gUSBHostHIDInstance[instanceNumber]);
            
            if(hidInstanceInfo->requestObj.controlRequestDone == true)
            {
                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                hidInstanceInfo->requestObj.controlRequestDone = false;
                
                /* Create the standard USB packet for SET PROTOCOL request */
                _USB_HOST_HID_SetProtocolPacketCreate
                (
                    /* PLACE HOLDER FOR SET UP PACKET */
                    &hidInstanceInfo->requestObj,
                    /* INTERFACE NUMBER */
                    hidInstanceInfo->bInterfaceNumber,
                    /* PROTOCOL TYPE */
                    protocolType
                );

                hidInstanceInfo->requestObj.usageDriverRequest = true;
                hidInstanceInfo->requestObj.usageDriverTableIndex = 
                          _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex(handle);

                if(hidInstanceInfo->requestObj.usageDriverTableIndex < 0)
                {
                    /* Reset the flags and status */
                    hidInstanceInfo->requestObj.controlRequestDone = true;
                    hidInstanceInfo->requestObj.usageDriverRequest = false;
                    status = USB_HOST_HID_RESULT_FAILURE;
                }

                else if(hidInstanceInfo->requestObj.controlRequestDone == false)
                {
                    hidInstanceInfo->requestObj.handle =  handle;
                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                            (
                                /* CONTROL pipe handle */
                                hidInstanceInfo->controlPipeHandle,
                                /* Transfer Handle */
                                &transferHandle,
                                /* SET PROTOCOL set up packet */
                                &hidInstanceInfo->requestObj.setupPacket,
                                /* No data phase */
                                NULL,
                                /* CONTROL Callback */
                                _USB_HOST_HID_ControlTransferCallback,
                                /* Context */
                                (uintptr_t)(hidInstanceInfo)
                            );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                        requestHandle =
                                (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                        status = USB_HOST_HID_RESULT_SUCCESS;
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        hidInstanceInfo->requestObj.usageDriverRequest = false;
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* Ongoing control transfer being handled by the host
                             layer */
                            status = USB_HOST_HID_RESULT_REQUEST_BUSY;
                        }
                        else
                        {
                            status = USB_HOST_HID_RESULT_FAILURE;
                        }
                    } /* Request submission failed */
                } /* if(controlRequestDone == false) */
            }/* if(controlRequestDone == true) */
            else
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
        } /* if(instanceNumber > (-1)) */
        else
        {
            /* Invalid instance number */
            status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
    }
    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of USB_HOST_HID_ProtocolSet() */


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ProtocolGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        USB_HID_PROTOCOL_TYPE *protocol
    )
 
  Summary:
    GET PROTOCOL request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending GET PROTOCOL request.

  Remarks:
    This request is supported by devices in the Boot subclass. Non-boot
    devices may STALL this request.
*/
USB_HOST_HID_RESULT USB_HOST_HID_ProtocolGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    USB_HID_PROTOCOL_TYPE *protocol
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result =  USB_HOST_RESULT_FAILURE;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE *hidInstanceInfo = 
    (USB_HOST_HID_INSTANCE *) USB_HOST_HID_OBJ_HANDLE_INVALID;
    int8_t instanceNumber = (-1);
    /* End of local variable */

    /* Checking invalid parameters */
    if(requestHandle == NULL || protocol == NULL || 
            handle == USB_HOST_HID_OBJ_HANDLE_INVALID)
    {
        /* Update the requestHandle only if not NULL */
        if(requestHandle != NULL)
        {
            requestHandle = (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
        }
        status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Initialize requestHandle only if Parameters are valid */
        requestHandle = (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;

        /* Obtain the instance number of the HID driver instance which owns this handle */
        instanceNumber = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
        
        if(instanceNumber > (-1))
        {
            hidInstanceInfo = &(gUSBHostHIDInstance[instanceNumber]);
            
            if(hidInstanceInfo->requestObj.controlRequestDone == true)
            {
                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                hidInstanceInfo->requestObj.controlRequestDone = false;
                
                /* Create the standard USB packet for GET PROTOCOL request */
                _USB_HOST_HID_GetProtocolPacketCreate
                (
                    /* PLACE HOLDER FOR SET UP PACKET */
                    &hidInstanceInfo->requestObj,
                    /* INTERFACE NUMBER */
                    hidInstanceInfo->bInterfaceNumber
                );

                hidInstanceInfo->requestObj.usageDriverRequest = true;
                hidInstanceInfo->requestObj.usageDriverTableIndex = 
                          _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex(handle);

                if(hidInstanceInfo->requestObj.usageDriverTableIndex < 0)
                {
                    /* Reset the flags and status */
                    hidInstanceInfo->requestObj.controlRequestDone = true;
                    hidInstanceInfo->requestObj.usageDriverRequest = false;
                    status = USB_HOST_HID_RESULT_FAILURE;
                }

                else if(hidInstanceInfo->requestObj.controlRequestDone == false)
                {
                    hidInstanceInfo->requestObj.handle =  handle;
                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                            (
                                /* CONTROL pipe handle */
                                hidInstanceInfo->controlPipeHandle,
                                /* Transfer Handle */
                                &transferHandle,
                                /* GET PROTOCOL set up packet */
                                &hidInstanceInfo->requestObj.setupPacket,
                                /* IN Data buffer */
                                protocol,
                                /* CONTROL Callback */
                                _USB_HOST_HID_ControlTransferCallback,
                                /* Context */
                                (uintptr_t)(hidInstanceInfo)
                            );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                        requestHandle =
                                (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                        status = USB_HOST_HID_RESULT_SUCCESS;
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        hidInstanceInfo->requestObj.usageDriverRequest = false;
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* Ongoing control transfer being handled by the host
                             layer */
                            status = USB_HOST_HID_RESULT_REQUEST_BUSY;
                        }
                        else
                        {
                            status = USB_HOST_HID_RESULT_FAILURE;
                        }
                    } /* Request submission failed */
                } /* if(controlRequestDone == false) */
            }/* if(controlRequestDone == true) */
            else
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
        } /* if(instanceNumber > (-1)) */
        else
        {
            /* Invalid instance number */
            status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
    }
    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of USB_HOST_HID_ProtocolGet() */


// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeGet
    (
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t reportID,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    uint8_t *idleTime
    )
 
  Summary:
    GET IDLE TIME request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending GET IDLE TIME request.

  Remarks:
    Output parameter idleTime is mapped with a 4 millisecond resolution.
    idleTime = 0 implies infinite inhibition of report generation,
    unless there is change in report data.

*/
USB_HOST_HID_RESULT USB_HOST_HID_IdleTimeGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t reportID,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    uint8_t *idleTime
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result =  USB_HOST_RESULT_FAILURE;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE *hidInstanceInfo = 
    (USB_HOST_HID_INSTANCE *) USB_HOST_HID_OBJ_HANDLE_INVALID;
    int8_t instanceNumber = (-1);
    /* End of local variable */

    /* Checking invalid parameters */
    if((requestHandle == NULL) || (idleTime == NULL) ||
        (handle == USB_HOST_HID_OBJ_HANDLE_INVALID))
    {
        /* Update requestHandle only if not NULL */
        if(requestHandle != NULL)
        {
            requestHandle = (USB_HOST_HID_REQUEST_HANDLE *)
                    USB_HOST_HID_REQUEST_HANDLE_INVALID;
        }
        status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Initialize requestHandle only if Parameters are valid */
        requestHandle =
                (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
        
        /* Obtain instance number of HID driver instance which owns this handle */
        instanceNumber = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
        
        if(instanceNumber > (-1))
        {
            hidInstanceInfo = &(gUSBHostHIDInstance[instanceNumber]);
            
            if(hidInstanceInfo->requestObj.controlRequestDone == true)
            {
                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                hidInstanceInfo->requestObj.controlRequestDone = false;
                
                /* Create the standard USB packet for GET IDLE TIME request */
                _USB_HOST_HID_GetIdlePacketCreate
                (
                    /* PLACE HOLDER FOR SET UP PACKET */
                    &hidInstanceInfo->requestObj,
                    /* INTERFACE NUMBER */
                    hidInstanceInfo->bInterfaceNumber,
                    /* REPORT ID */
                    reportID
                );

                hidInstanceInfo->requestObj.usageDriverRequest = true;
                hidInstanceInfo->requestObj.usageDriverTableIndex = 
                          _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex(handle);

                if(hidInstanceInfo->requestObj.usageDriverTableIndex < 0)
                {
                    /* Reset the flags and status */
                    hidInstanceInfo->requestObj.controlRequestDone = true;
                    hidInstanceInfo->requestObj.usageDriverRequest = false;
                    status = USB_HOST_HID_RESULT_FAILURE;
                }

                else if(hidInstanceInfo->requestObj.controlRequestDone == false)
                {
                    hidInstanceInfo->requestObj.handle =  handle;
                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                            (
                                /* CONTROL pipe handle */
                                hidInstanceInfo->controlPipeHandle,
                                /* Transfer Handle */
                                &transferHandle,
                                /* GET IDLE TIME set up packet */
                                &hidInstanceInfo->requestObj.setupPacket,
                                /* IN Data buffer */
                                idleTime,
                                /* CONTROL Callback */
                                _USB_HOST_HID_ControlTransferCallback,
                                /* Context */
                                (uintptr_t)(hidInstanceInfo)
                            );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                        requestHandle =
                                (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                        status = USB_HOST_HID_RESULT_SUCCESS;
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        hidInstanceInfo->requestObj.usageDriverRequest = false;
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* Ongoing control transfer being handled by the host
                             layer */
                            status = USB_HOST_HID_RESULT_REQUEST_BUSY;
                        }
                        else
                        {
                            status = USB_HOST_HID_RESULT_FAILURE;
                        }
                    } /* Request submission failed */
                } /* if(controlRequestDone == false) */
            }/* if(controlRequestDone == true) */
            else
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
        } /* if(instanceNumber > (-1)) */
        else
        {
            /* Invalid instance number */
            status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
    }
    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of USB_HOST_HID_IdleTimeGet() */

// *****************************************************************************
/* Function:
    USB_HOST_HID_RESULT _USB_HOST_HID_reportdescriptorGet
    (
        uint8_t  instanceNumber
    )
 
  Summary:
    GET REPORT DESCRIPTOR request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending GET REPORT DESCRIPTOR request.

  Remarks:
    This is local function and should not be called directly by the application.
*/
USB_HOST_HID_RESULT _USB_HOST_HID_reportdescriptorGet
(
    uint8_t  instanceNumber
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;    
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE *hidInstanceInfo =
            &(gUSBHostHIDInstance[instanceNumber]);
    /* End of local variable */
    
    if(hidInstanceInfo->requestObj.controlRequestDone == true)
    {
        /* Set the flag indicating we are waiting for the control
        * request to complete */
        hidInstanceInfo->requestObj.controlRequestDone = false;
        
        if(hidInstanceInfo->reportDescBuffer != NULL)
        {
            USB_HOST_FREE(hidInstanceInfo->reportDescBuffer);
            hidInstanceInfo->reportDescBuffer = NULL;
        }

        /* Allocate the memory for Report Descriptor Buffer */
        hidInstanceInfo->reportDescBuffer = 
                USB_HOST_MALLOC((size_t)(hidInstanceInfo->reportDescLength));
        if(hidInstanceInfo->reportDescBuffer == NULL)
        {
            /* Memory Allocation failed */
            status = USB_HOST_HID_RESULT_FAILURE;
            hidInstanceInfo->requestObj.controlRequestDone = true;
            return status;
        }
        /* Create the standard USB packet for GET REPORT DESCRIPTOR request */
        _USB_HOST_HID_GetReportDescPacketCreate
        (
           &hidInstanceInfo->requestObj,
           hidInstanceInfo->bInterfaceNumber,
           hidInstanceInfo->reportDescLength
        );

        /* Launch the request */
        result = USB_HOST_DeviceControlTransfer
        (
            /* CONTROL pipe handle */
            hidInstanceInfo->controlPipeHandle,
            /* Transfer Handle */
            &transferHandle,
            /* GET REPORT DESCRIPTOR set up packet */
            &hidInstanceInfo->requestObj.setupPacket,
            /* Place holder for Report Descriptor */
            hidInstanceInfo->reportDescBuffer,
            /* CONTROL Callback */
            _USB_HOST_HID_ControlTransferCallback,
            /* Context */
            (uintptr_t)(hidInstanceInfo)
         );
        if(USB_HOST_RESULT_SUCCESS == result)
        {
            status = USB_HOST_HID_RESULT_SUCCESS;
        }
        else
        {
            /* Request submission failed */
            hidInstanceInfo->requestObj.controlRequestDone = true;
            if(USB_HOST_RESULT_REQUEST_BUSY == result)
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
            else
            {
                status = USB_HOST_HID_RESULT_FAILURE;
            }
        }
    }
    else
    {
        /* Last CONTROL request not yet completed */
        status = USB_HOST_HID_RESULT_REQUEST_BUSY;
    }
    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of _USB_HOST_HID_reportdescriptorGet () */

// ******************************************************************************
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_ReportSend
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HID_REPORT_TYPE reportType,
        uint8_t reportID,
        uint16_t reportLength,
        USB_HOST_HID_REQUEST_HANDLE *requestHandle,
        const void *report
    )
 
  Summary:
    SET REPORT request is send to the HID device.
  
  Description:
    Function facilitates in forming of set up packet and submission of request 
    for sending SET REPORT request.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_ReportSend
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HID_REPORT_TYPE reportType,
    uint8_t reportID,
    uint16_t reportLength,
    USB_HOST_HID_REQUEST_HANDLE *requestHandle,
    const void *report
)
{
    /* Start of local variable */
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result =  USB_HOST_RESULT_FAILURE;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_INSTANCE *hidInstanceInfo = 
    (USB_HOST_HID_INSTANCE *) USB_HOST_HID_OBJ_HANDLE_INVALID;
    int8_t instanceNumber = (-1);
    /* End of local variable */

    /* Checking invalid arguments */
    if((requestHandle == NULL) || (report == NULL) || 
            (handle == USB_HOST_HID_OBJ_HANDLE_INVALID) ||
            (reportType != USB_HID_REPORT_TYPE_OUTPUT))
    {
        /* Update requestHandle only if not NULL */
        if(requestHandle != NULL)
        {
            requestHandle = (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
        }
        status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
        /* Initialize requestHandle only if Parameters are valid */
        requestHandle =
                (USB_HOST_HID_REQUEST_HANDLE *)USB_HOST_HID_REQUEST_HANDLE_INVALID;
        
        /* Obtain instance number of HID driver instance which owns this handle */
        instanceNumber = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
        
        if(instanceNumber > (-1))
        {
            hidInstanceInfo = &(gUSBHostHIDInstance[instanceNumber]);
            
            /*
             * If Interrupt OUT endpoint is present OUTPUT Report will be sent
             * through Interrupt OUT endpoint only, not CONTROL endpoint.
             */
            if(hidInstanceInfo->interruptOutPipeHandle !=
                    USB_HOST_PIPE_HANDLE_INVALID)
            {
                hidInstanceInfo->state = USB_HOST_HID_STATE_WAITING_SET_REPORT;
                /* Create the standard USB packet for SET REPORT request */
                _USB_HOST_HID_SetReportPacketCreate
                (
                    (USB_HOST_HID_REQUEST *)&(gUSBHostHIDWriteBuffer[instanceNumber][0]),
                    hidInstanceInfo->bInterfaceNumber,
                    reportType,
                    reportID,
                    reportLength
                );
                if(USB_HOST_RESULT_SUCCESS == USB_HOST_DeviceTransfer
                        (
                            hidInstanceInfo->interruptOutPipeHandle,
                            &transferHandle, 
                            (void *)&(gUSBHostHIDWriteBuffer[instanceNumber][0]),
                            reportLength,
                            (uintptr_t)(hidInstanceInfo)
                        ))
                {
                    /* Request submission was successful */
                    requestHandle = (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                    status = USB_HOST_HID_RESULT_SUCCESS;
                }
                else
                {
                    status = USB_HOST_HID_RESULT_FAILURE;
                    hidInstanceInfo->state = USB_HOST_HID_STATE_READY;
                }
            }
            
            else if(hidInstanceInfo->requestObj.controlRequestDone == true)
            {
                /* Set the flag indicating we are waiting for the control
                 * request to complete */
                hidInstanceInfo->requestObj.controlRequestDone = false;
                
                /* Create the standard USB packet for SET REPORT request */
                _USB_HOST_HID_SetReportPacketCreate
                (
                    &hidInstanceInfo->requestObj,
                    hidInstanceInfo->bInterfaceNumber,
                    reportType,
                    reportID,
                    reportLength
                );

                hidInstanceInfo->requestObj.usageDriverRequest = true;
                hidInstanceInfo->requestObj.usageDriverTableIndex = 
                          _USB_HOST_HID_ObjectHandleToUsageDriverTableIndex(handle);

                if(hidInstanceInfo->requestObj.usageDriverTableIndex < 0)
                {
                    /* Reset the flags and status */
                    hidInstanceInfo->requestObj.controlRequestDone = true;
                    hidInstanceInfo->requestObj.usageDriverRequest = false;
                    status = USB_HOST_HID_RESULT_FAILURE;
                }

                else if(hidInstanceInfo->requestObj.controlRequestDone == false)
                {
                    hidInstanceInfo->requestObj.handle =  handle;
                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                            (
                                /* CONTROL pipe handle */
                                hidInstanceInfo->controlPipeHandle,
                                /* Transfer Handle */
                                &transferHandle,
                                /* SET REPORT set up packet */
                                &hidInstanceInfo->requestObj.setupPacket,
                                /* Report Data */
                                (void *)report,
                                /* CONTROL Callback */
                                _USB_HOST_HID_ControlTransferCallback,
                                /* Context */
                                (uintptr_t)(hidInstanceInfo)
                            );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                        requestHandle =
                                (USB_HOST_HID_REQUEST_HANDLE *)transferHandle;
                        status = USB_HOST_HID_RESULT_SUCCESS;
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        hidInstanceInfo->requestObj.usageDriverRequest = false;
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* Ongoing control transfer being handled by the host
                             layer */
                            status = USB_HOST_HID_RESULT_REQUEST_BUSY;
                        }
                        else
                        {
                            status = USB_HOST_HID_RESULT_FAILURE;
                        }
                    } /* Request submission failed */
                } /* if(controlRequestDone == false) */
            }/* else if(controlRequestDone == true) */
            else
            {
                status = USB_HOST_HID_RESULT_REQUEST_BUSY;
            }
        } /* if(instanceNumber > (-1)) */
        else
        {
            /* Invalid instance number */
            status = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
    } /* end of else () */
    /*
     * status = USB_HOST_HID_RESULT_SUCCESS: On success
     *        = USB_HOST_HID_RESULT_REQUEST_BUSY: Pending CONTROL request for
     *          this hid instance
     *        = USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     *        = USB_HOST_HID_RESULT_FAILURE: On other failures
     */
    return status;
}/* end of USB_HOST_HID_ReportSend() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_Initialize(void * hidInitData)

  Summary:
    This function will initialize HID Host client driver.

  Description:
    This function will initialize HID client driver. Functionalities include:
    1. Invalidating all pipe handles
    2. Storing the Usage Driver Registration Table
    3. Usage driver initialization

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_Initialize(void * hidInitData)
{   
    /* Start of local variable */ 
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY * usageDriverTableEntry = NULL;
    uint8_t iterator = 0;
    uint8_t pipeHandleCount = 0;
    /* End of local variable */
    
    /* Reset the HID instance data */
    for (iterator = 0; iterator < USB_HOST_HID_INSTANCES_NUMBER; iterator ++)
    {
        gUSBHostHIDInstance[iterator].assigned = false;
        gUSBHostHIDInstance[iterator].isFieldProcessing = false;
        gUSBHostHIDInstance[iterator].isHIDDriverAttached = false;
        gUSBHostHIDInstance[iterator].mainItemData = NULL;
        gUSBHostHIDInstance[iterator].usageTagsCount = 0;
        gUSBHostHIDInstance[iterator].globalStackIndex = 0;
        
        memset(gUSBHostHIDInstance[iterator].globalStack, 0,
                (size_t)(sizeof(USB_HOST_HID_GLOBAL_ITEM) * 
                USB_HID_GLOBAL_PUSH_POP_STACK_SIZE));
        
        /* Make sure all the pipe handles are invalid */
        gUSBHostHIDInstance[iterator].controlPipeHandle =
                USB_HOST_CONTROL_PIPE_HANDLE_INVALID;
        /* Initialize requestObject */
        gUSBHostHIDInstance[iterator].requestObj.controlRequestDone = true;
        gUSBHostHIDInstance[iterator].requestObj.usageDriverRequest = false;
        gUSBHostHIDInstance[iterator].requestObj.usageDriverTableIndex = 0;
        gUSBHostHIDInstance[iterator].requestObj.callback = NULL;
        gUSBHostHIDInstance[iterator].requestObj.size = 0;
        
        /* Initialize the pipe handles to invalid */
        for (pipeHandleCount = 0; pipeHandleCount <
                USB_HOST_HID_INTERRUPT_IN_ENDPOINTS_NUMBER; pipeHandleCount ++)
        {
            /* Make sure all the pipe handles are invalid */
            gUSBHostHIDInstance[iterator].interruptInPipeHandle[pipeHandleCount]
                    = USB_HOST_PIPE_HANDLE_INVALID;
        }
        gUSBHostHIDInstance[iterator].interruptOutPipeHandle =
                USB_HOST_PIPE_HANDLE_INVALID;
        gUSBHostHIDInstance[iterator].state = USB_HOST_HID_STATE_WAIT;
    }
    
    if(hidInitData != NULL)
    {
        gUSBHostHIDInitData = (USB_HOST_HID_INIT *)hidInitData;
        usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;

        if((gUSBHostHIDInitData->nUsageDriver > 0) &&
                (usageDriverTableEntry != NULL))
        {
            iterator = 0;
            while(iterator != (uint8_t) gUSBHostHIDInitData->nUsageDriver)
            {
                iterator++;
                /* Check if interface functions are registered for this
                 * Usage driver registration instance.
                 */
                if(usageDriverTableEntry->interface != NULL)
                {
                    /* Check if initialize function is registered for this usage
                     * driver registration instance.
                     */
                    if((usageDriverTableEntry->interface)->initialize != NULL)
                    {
                        /* Call the usage driver initialize. We will call
                         * initialize even if initialize data pointer
                         * is NULL. This is because initialize can be usage
                         * driver implementation specific.
                         * 
                         * Note: Usage driver initialization will be done
                         * irrespective of the usage presence or absence in the 
                         * device report descriptor.
                         */
                        (usageDriverTableEntry->interface)->initialize(
                            usageDriverTableEntry->initializeData);
                    }

                } /* end of if(Interface functions registered) */
                usageDriverTableEntry++;
            } /* end of while() loop */
        } /* end of if((gUSBHostHIDTableEntry->nUsageDriver > 0) && (usageDriverTableEntry != NULL)) */
    } /* end of if(hidInitData != NULL) */

} /* End of _USB_HOST_HID_Initialize() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_Deinitialize(void)

  Summary:
    This function will de initialize HID Host client driver.

  Description:
    This function will de initialize HID client driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/
void _USB_HOST_HID_Deinitialize(void)
{
    /* Start of local variables */
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY *usageDriverTableEntry = NULL;
    size_t nUsageDriver = 0;
    /* End of local variables */

    /* Check if gUSBHostHIDInitData is NULL */
    if(gUSBHostHIDInitData != NULL)
    {
        usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
        nUsageDriver = gUSBHostHIDInitData->nUsageDriver;

        while((usageDriverTableEntry != NULL) && (nUsageDriver != 0)) 
        {
            if(usageDriverTableEntry->interface != NULL && 
                (usageDriverTableEntry->interface)->deinitialize != NULL)
            {
                /* Call the usage driver de-initialize */
                (usageDriverTableEntry->interface)->deinitialize();
            }
            usageDriverTableEntry++;
            nUsageDriver--;
        }
        /* Invalidate the Usage driver registration information */
        gUSBHostHIDInitData = NULL;
    }

} /* End of _USB_HOST_HID_Deinitialize() */

// *****************************************************************************
/* Function:
   void _USB_HOST_HID_Reinitialize(void * hidInitData)

  Summary:
    This function will reinitialize HID Host client driver.

  Description:
    This function will reinitialize HID client driver.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/
void _USB_HOST_HID_Reinitialize(void * hidInitData)
{
    
} /* End of _USB_HOST_HID_Reinitialize () */


// *****************************************************************************
/* Function:
    void _USB_HOST_HID_InterfaceAssign 
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
        USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
        size_t nInterfaces,
        uint8_t * descriptor
    )

  Summary:
    This function will instantiate the HID interface associated with this
    function call.

  Description:
    This function will instantiate the HID interface associated with this
    function call. This function will be called from USB HOST layer as
    callback when matching HID interface has been found in TPL table.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_InterfaceAssign 
(
    USB_HOST_DEVICE_INTERFACE_HANDLE * interfaces,
    USB_HOST_DEVICE_OBJ_HANDLE deviceObjHandle,
    size_t nInterfaces,
    uint8_t * descriptor
)
{
    /* Start of local variable */
    USB_HOST_HID_INSTANCE * hidInstanceInfo = NULL;
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle;
    USB_HOST_ENDPOINT_DESCRIPTOR_QUERY endpointQuery;
    USB_ENDPOINT_DESCRIPTOR * endpointDescriptor = NULL;
    USB_INTERFACE_DESCRIPTOR * interfaceDescriptor = NULL;
    USB_HOST_HID_DEVICE_INFO * hidDeviceInfo = NULL;
    USB_HID_DESCRIPTOR * hidClassDesc;
    bool result = false;
    uint8_t iterator = 0;
    /* End of local variable */

    /* This client driver will support only one interface which is the first
     * interface in the interfaces table */
    interfaceHandle = interfaces[0];

    /* This function is being called because an HID device was attached and the 
     * driver has matched. Check if there is a free HID object. Searching for
     * free HID object need not be mutex protected because this function will
     * always be called from one thread. */

    for (iterator = 0 ; iterator < USB_HOST_HID_INSTANCES_NUMBER ; iterator++)
    {
        if (!gUSBHostHIDInstance[iterator].assigned)
        {
            /* Found a free instance */
            gUSBHostHIDInstance[iterator].assigned = true;
            hidInstanceInfo = &gUSBHostHIDInstance[iterator];
            break;
        }
    } /* End of for(iterator < USB_HOST_HID_INSTANCES_NUMBER) */

    if (hidInstanceInfo != NULL)
    {
        /* Save these handles */
        hidInstanceInfo->deviceObjHandle = deviceObjHandle;
        hidInstanceInfo->interfaceHandle = interfaceHandle;

        /* Get the interface descriptor pointer */
        interfaceDescriptor = (USB_INTERFACE_DESCRIPTOR *)(descriptor);
        hidInstanceInfo->bInterfaceNumber =
            interfaceDescriptor->bInterfaceNumber;


        /* The below code performs error checking for HID descriptor
         * and on success extracts HID descriptor information */
        if (((uint8_t *)interfaceDescriptor) + 
                sizeof(USB_INTERFACE_DESCRIPTOR) != NULL)
        {
            hidClassDesc = (USB_HID_DESCRIPTOR *)
                (((uint8_t *)interfaceDescriptor) +
                 sizeof(USB_INTERFACE_DESCRIPTOR));
            /*
             * Check if the descriptor after interface descriptor is HID Class
             * descriptor.
             */
            if(hidClassDesc->bDescriptorType != 
                    (uint8_t) USB_HID_DESCRIPTOR_TYPES_HID)
            {
                /* No HID Class descriptor present */
                hidInstanceInfo->assigned = false;
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: HID class descriptor not found");
                return;
            }

            if((hidClassDesc->bReportDescriptorType != USB_HID_DESCRIPTOR_TYPES_REPORT) || 
                    (hidClassDesc->bNumDescriptors < 1))
            {
                /* No HID Report descriptor */
                hidInstanceInfo->assigned = false;
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: HID report descriptor not found");
                return;
            }
            /* Update the size of the Report Descriptor */
            hidInstanceInfo->reportDescLength = hidClassDesc->wItemLength;

            /* Extract device information */
            hidDeviceInfo = &(hidInstanceInfo->hidDeviceInfo);
            hidDeviceInfo->countryCode = hidClassDesc->bcdHID;
            hidDeviceInfo->isBootInterfaceClass =
                interfaceDescriptor->bInterfaceSubClass;
            hidDeviceInfo->isKeyboardDevice = false;
            if(hidDeviceInfo->isBootInterfaceClass)
            {
                /*
                 * Now that we know it is a BOOT device, we need to understand
                 * if the device is Keyboard or Mouse. This is because as per
                 * that we decide the SET IDLE rate value.
                 */
                if(USB_HID_PROTOCOL_CODE_KEYBOARD ==
                        interfaceDescriptor->bInterfaceProtocol)
                {
                    /* Keyboard */
                    hidDeviceInfo->isKeyboardDevice = true;
                }
                else
                {
                    /* Mouse */
                }
            }
        } /* End of HID Descriptor error checking */

        /* If the code flow comes here it means that we have found an HID
         * instance object and this device can be processed.
         *
         * Open a control pipe to the device.
         */

        hidInstanceInfo->controlPipeHandle =
            USB_HOST_DeviceControlPipeOpen(deviceObjHandle);

        if(hidInstanceInfo->controlPipeHandle != 
                USB_HOST_CONTROL_PIPE_HANDLE_INVALID)
        {
            /* CONTROL pipe has been opened successfully.
             * Now find the INTERRUPT IN and INTERRUPT OUT endpoint.
             * To do this, first setup the endpoint descriptor query
             */

            /* INTERRUPT IN endpoint query */
            USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);

            endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_DIRECTION |
                USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE;
            endpointQuery.direction  = USB_DATA_DIRECTION_DEVICE_TO_HOST;
            endpointQuery.transferType = USB_TRANSFER_TYPE_INTERRUPT;

            iterator = 0;
            do
            {
                /* Now find the endpoint */
                endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery
                    (interfaceDescriptor, &endpointQuery);
                if(endpointDescriptor != NULL)
                {
                    /* We have found INTERRUPT IN endpoint.
                     * Set flag to catch this later.
                     */
                    result = true;
                    /* Save the interrupt IN endpoint size */
                    hidInstanceInfo->interruptInEndpointSize[iterator] =
                        (uint8_t)(endpointDescriptor->wMaxPacketSize);
                    /* Try opening a pipe on this endpoint */
                    hidInstanceInfo->interruptInPipeHandle[iterator] = 
                        USB_HOST_DevicePipeOpen(interfaceHandle,
                                endpointDescriptor->bEndpointAddress);
                }
                else
                {
                    /* HID interface can have only INTERRUPT endpoints.
                     * Hence if we get endpointDescriptor == NULL while
                     * enquiry for endpointDescriptor, there are only 2
                     * possibilities:
                     * 1. There is no INTERRUPT IN endpoint present
                     * 2. There is no MORE INTERRUPT IN endpoint present
                     *
                     * The code can come here in either of the scenarios.
                     */
                    if(result == true)
                    {
                        /* At least 1 INTERRUPT IN endpoint has been found 
                         * as result == true. result is set to true if we
                         * get a valid endpointDescriptor for INTERRUPT IN
                         * endpoint query.
                         */ 
                    }
                    else
                    {
                        /* Not a single INTERRUPT IN endpoint has been found */
                        result = false;
                    }
                    break;
                }
                iterator++;
            } while(endpointDescriptor != NULL &&
                    iterator != USB_HOST_HID_INTERRUPT_IN_ENDPOINTS_NUMBER);

            if(result == false)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: No INTERRUPT IN endpoint found on HID device");

                /* Give up the claim on this HID interface */
                /* Let the host know that this interface cannot be processed */

                _USB_HOST_HID_InterfaceRelease(interfaceHandle);

                return;
            }

            /* 
             * INTERRUPT OUT endpoint query. interruptOutPipeHandle is not an
             * array as there can only be 1 optional interrupt out EP for HID
             */
            USB_HOST_DeviceEndpointQueryContextClear(&endpointQuery);

            endpointQuery.flags = USB_HOST_ENDPOINT_QUERY_BY_DIRECTION |
                USB_HOST_ENDPOINT_QUERY_BY_TRANSFER_TYPE;
            endpointQuery.direction  = USB_DATA_DIRECTION_HOST_TO_DEVICE;
            endpointQuery.transferType = USB_TRANSFER_TYPE_INTERRUPT;

            result = false;

            /* Now find the endpoint */
            endpointDescriptor = USB_HOST_DeviceEndpointDescriptorQuery
                (interfaceDescriptor, &endpointQuery);
            if(endpointDescriptor != NULL)
            {
                /* We have found the INTERRUPT OUT endpoint. Try opening a pipe
                 * on this endpoint */
                result = true;
                /* Save the interrupt OUT endpoint size */
                hidInstanceInfo->interruptOutEndpointSize = 
                    (uint8_t)(endpointDescriptor->wMaxPacketSize);
                hidInstanceInfo->interruptOutPipeHandle =
                    USB_HOST_DevicePipeOpen(interfaceHandle, 
                            endpointDescriptor->bEndpointAddress);
            }
            else
            {
                /* We could not find any INTERRUPT OUT endpoint.
                   INTERRUPT OUT endpoint is optional for HID devices.
                   So it is a valid case. */
                result = true;
            }

        } /* End of if(CONTROL pipe open) */
        else
        {
            /* Control pipe could not be opened */
            result = false;
        }
    } /* End of hidInstanceInfo != NULL */

    if (result == false)
    {
        /* Let the host know that this interface cannot be processed */
        _USB_HOST_HID_InterfaceRelease(interfaceHandle);
        /* The debug message for other error scenarios has already been taken
         * care before except no free HID object scenario */
        if(hidInstanceInfo != NULL)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                    "\r\nUSBHID Client Driver: Could not find HID object");
        }

    }
    else
    {
        /* HID interface is valid. We should launch the first control request */
        hidInstanceInfo->state = USB_HOST_HID_STATE_SET_IDLE_SEND;
    }

    return;
} /* End of _USB_HOST_HID_InterfaceAssign() */


// *****************************************************************************
/* Function:
   void _USB_HOST_HID_InterfaceRelease
   (
       USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
   );

  Summary:
    This function will release the HID interface. It will close any open
    pipes.

  Description:
    This function will release the HID interface. It will close any open
    pipes and will let the host know that this interface has been released.
    Function will be called by USB Host layer as callback when HID device
    has been detached.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _USB_HOST_HID_InterfaceRelease
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
)
{
    /* Start of local variable */
    int8_t hidInstanceIndex = 0;
    uint8_t iterator = 0;
    uint8_t index = 0;
    uint8_t loop = 0;
    USB_HOST_HID_INSTANCE * hidInstanceInfo = NULL;
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY * usageDriverTableEntry = NULL;
    /* End of local variable */

    /* Find the HID instance for this interface */
    hidInstanceIndex = _USB_HOST_HID_InterfaceHandleToHIDIndex(interfaceHandle);

    if(hidInstanceIndex >= 0)
    {
        /* Get the object pointer */
        hidInstanceInfo = &(gUSBHostHIDInstance[hidInstanceIndex]);
        /* Let the host know that this interface cannot be processed */
        USB_HOST_DeviceInterfaceRelease(interfaceHandle);

        for (iterator = 0;
                iterator < USB_HOST_HID_INTERRUPT_IN_ENDPOINTS_NUMBER;
                iterator++)
        {
            /* Close any open pipes */
            if(hidInstanceInfo->interruptInPipeHandle[iterator] != 
                    USB_HOST_PIPE_HANDLE_INVALID)
            {
                /* Close the INTERRUPT IN pipe */
                USB_HOST_DevicePipeClose
                    (hidInstanceInfo->interruptInPipeHandle[iterator]);
                hidInstanceInfo->interruptInPipeHandle[iterator] =
                    USB_HOST_PIPE_HANDLE_INVALID;
            }
            else
            {
                /* No need of further looping. After this there cannot
                 * be any more INTERRUPT IN endpoint handle
                 */
                break;
            }
            iterator++;
        } /* end of for(all INTERRUPT IN endpoint) */

        if(hidInstanceInfo->interruptOutPipeHandle !=
                USB_HOST_PIPE_HANDLE_INVALID)
        {
            /* Close the INTERRUPT OUT pipe */
            USB_HOST_DevicePipeClose(hidInstanceInfo->interruptOutPipeHandle);
            hidInstanceInfo->interruptOutPipeHandle =
                USB_HOST_PIPE_HANDLE_INVALID;
        }

        hidInstanceInfo->assigned = false;
        hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
        hidInstanceInfo->isFieldProcessing = false;

        hidInstanceInfo->collectionNestingLevel = 0;
        gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing = false;

        hidInstanceInfo->usageTagsCount = 0;
        hidInstanceInfo->globalStackIndex = 0;

        memset(hidInstanceInfo->globalStack, 0,
                (size_t)(sizeof(USB_HOST_HID_GLOBAL_ITEM) * 
                    USB_HID_GLOBAL_PUSH_POP_STACK_SIZE));
        
        /* Reset request Object for this HID instance */
        hidInstanceInfo->requestObj.controlRequestDone = true;
        hidInstanceInfo->requestObj.usageDriverRequest = false;
        hidInstanceInfo->requestObj.usageDriverTableIndex = 0;
        hidInstanceInfo->requestObj.callback = NULL;
        hidInstanceInfo->requestObj.size = 0;

        /* isHIDDriverAttached is true only if for this HID instance at least
         * 1 usage driver ATTACH notification has happened. If not there is
         * no question of DETACH notification. There will NOT be any slot
         * allocated in gUSBHostHIDObjectHandlePool data structure for this HID
         * instance.
         */
        if(hidInstanceInfo->isHIDDriverAttached)
        {
            usageDriverTableEntry =
                gUSBHostHIDInitData->usageDriverTable;
            
            for(index=0; index < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                    index ++)
            {   
                if(gUSBHostHIDObjectHandlePool[index].inUse &&
                        (gUSBHostHIDObjectHandlePool[index].hidInstanceIndex == 
                         hidInstanceIndex))
                {
                    /* Release the handle */
                    _USB_HOST_HID_ObjectHandleRelease((USB_HOST_HID_OBJ_HANDLE)
                            (&gUSBHostHIDObjectHandlePool[index]));
                    /*
                     * Now we need to find out this usage specific to the object
                     * handle is owned by which entry in the usage driver
                     * registration table. This is because DETACH notification
                     * needs to be send to that usage driver.
                     */
                    for(loop = 0; loop < gUSBHostHIDInitData->nUsageDriver;
                            loop++)
                    {
                        if((&usageDriverTableEntry[loop])->usage ==
                                gUSBHostHIDObjectHandlePool[index].usage)
                        {
                            /* Usage driver callback with DETACH event */
                            ((&usageDriverTableEntry[loop])->interface)->
                                usageDriverEventHandler
                                (
                                 (USB_HOST_HID_OBJ_HANDLE)
                                 &gUSBHostHIDObjectHandlePool[index],
                                 USB_HOST_HID_EVENT_DETACH,
                                 NULL
                                );
                            break;
                        }
                    }
                }
            }
        }
        hidInstanceInfo->isHIDDriverAttached = false;
    }/* end of if(hidInstanceIndex >= 0) */

    return;

} /* End of _USB_HOST_HID_InterfaceRelease() */


// *****************************************************************************
/* Function:
    USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HID_InterfaceEventHandler
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
        USB_HOST_DEVICE_INTERFACE_EVENT event,
        void * eventData,
        uintptr_t context
    )
 
  Summary:
  
  Description:

  Remarks:
    This is local function and should not be called directly by the application.
*/
USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE _USB_HOST_HID_InterfaceEventHandler
(
    USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle,
    USB_HOST_DEVICE_INTERFACE_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    /* Start of local variables */
    USB_HOST_HID_INSTANCE * hidInstanceInfo = NULL;
    USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA * 
            transferCompleteEventData =  NULL;
    USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE_DATA *
            haltClearCompleteEventData = NULL;
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY * usageDriverTableEntry = NULL;
    USB_HOST_HID_USAGE_DRIVER_REQUEST_RESPONSE_DATA responseData;
    int8_t hidInstanceIndex = (-1);
    uint8_t index = 0;
    uint8_t loop = 0;
    /* End of local variables */
    
    /* The context will be a pointer to the HID instance */
    hidInstanceInfo = (USB_HOST_HID_INSTANCE *)(context);

    /* Find the HID instance for this interface */
    hidInstanceIndex = _USB_HOST_HID_InterfaceHandleToHIDIndex(interfaceHandle);
    
    if(hidInstanceIndex < 0)
    {
        return USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE;
    }

    if(hidInstanceInfo->state == USB_HOST_HID_STATE_WAIT)
    {
        /* Change the state to make sure that the next INTERRUPT IN IRP
         submission happens. */
        hidInstanceInfo->state = USB_HOST_HID_STATE_READY;
    }
    switch(event)
    {
        case USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE:
            /* This means a transfer completed. Update the transfer state
             * machine. */
            transferCompleteEventData =
                    (USB_HOST_DEVICE_INTERFACE_EVENT_TRANSFER_COMPLETE_DATA *)
                        (eventData);
            if(USB_HOST_RESULT_SUCCESS == transferCompleteEventData->result)
            {
                /* isHIDDriverAttached is true only if for this HID instance at
                 * least 1 usage driver ATTACH notification has happened. If not
                 * there is no question of REPORT notification. 
                 * In that case there will NOT be any slot allocated in 
                 * gUSBHostHIDObjectHandlePool data structure for this HID
                 * instance.
                 */
                if(hidInstanceInfo->isHIDDriverAttached)
                {
                    usageDriverTableEntry =
                            gUSBHostHIDInitData->usageDriverTable;
                    for(index=0;
                            index < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                            index ++)
                    {
                        if(gUSBHostHIDObjectHandlePool[index].inUse &&
                            (gUSBHostHIDObjectHandlePool[index].hidInstanceIndex
                            == hidInstanceIndex))
                        {
                            if(hidInstanceInfo->state == 
                                            USB_HOST_HID_STATE_WAITING_SET_REPORT)
                            {
                                hidInstanceInfo->state =
                                        USB_HOST_HID_STATE_READY;
                                responseData.result = 
                                        USB_HOST_HID_RESULT_SUCCESS;
                                responseData.handle = 
                                        transferCompleteEventData->transferHandle;
                                ((&usageDriverTableEntry
                                    [gUSBHostHIDObjectHandlePool[index].usageInstanceIndex])
                                        ->interface)->usageDriverEventHandler
                                    (
                                        (USB_HOST_HID_OBJ_HANDLE)
                                        &gUSBHostHIDObjectHandlePool[index],
                                        USB_HOST_HID_EVENT_REPORT_SENT, 
                                        &responseData
                                    );
                            }
                            else
                            {
                                ((&usageDriverTableEntry
                                        [gUSBHostHIDObjectHandlePool[index].usageInstanceIndex]
                                        )->interface)->usageDriverEventHandler
                                    (
                                        (USB_HOST_HID_OBJ_HANDLE)
                                        &gUSBHostHIDObjectHandlePool[index],
                                        USB_HOST_HID_EVENT_REPORT_RECEIVED, 
                                        hidInstanceInfo->readBuffer
                                    );
                            }
                        }
                    }
                }
            }
            else if(USB_HOST_RESULT_REQUEST_STALLED == 
                    transferCompleteEventData->result)
            {
                /* INTERRUPT transfer stalled. Clear the endpoint.
                 * Assume now it is INTERRUPT IN pipe that has been STALLed*/
                hidInstanceInfo->state = 
                        USB_HOST_HID_STATE_INTERRUPT_IN_ENDPOINT_CLEAR;
                
                if((hidInstanceInfo->isHIDDriverAttached) &&
                        (hidInstanceInfo->state == 
                                            USB_HOST_HID_STATE_WAITING_SET_REPORT))
                {
                    /*
                     Only if interrupt OUT is used to transfer SET REPORT state
                     is changed to USB_HOST_HID_STATE_WAITING_SET_REPORT.
                     Otherwise state will be ready or in wait state based on
                     INTERRUPT IN token condition.
                     Thus state = USB_HOST_HID_STATE_WAITING_SET_REPORT gives us
                     an indication that the code is here due to INTERRUPT OUT
                     pipe STALL*/
                    /* Change the state to INETRRUPT OUT endpoint clear as we
                     see now that the STALL ed pipe is INTERRUPT OUT pipe.*/
                    hidInstanceInfo->state = 
                        USB_HOST_HID_STATE_INTERRUPT_OUT_ENDPOINT_CLEAR;
                    
                    usageDriverTableEntry =
                            gUSBHostHIDInitData->usageDriverTable;
                    for(index=0;
                            index < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                            index ++)
                    {
                        if(gUSBHostHIDObjectHandlePool[index].inUse &&
                            (gUSBHostHIDObjectHandlePool[index].hidInstanceIndex
                            == hidInstanceIndex))
                        {
                            /*
                             * Now we need to find out this usage is owned by which
                             * entry in the usage driver registration table
                             */
                            for(loop = 0;
                                    loop < gUSBHostHIDInitData->nUsageDriver;
                                    loop++)
                            {
                                if((&usageDriverTableEntry[loop])->usage ==
                                    gUSBHostHIDObjectHandlePool[index].usage)
                                {
                                    responseData.result = 
                                            USB_HOST_RESULT_REQUEST_STALLED;
                                    responseData.handle = 
                                            transferCompleteEventData->transferHandle;
                                    
                                    ((&usageDriverTableEntry[loop])->interface)->
                                            usageDriverEventHandler
                                            (
                                                (USB_HOST_HID_OBJ_HANDLE)
                                                &gUSBHostHIDObjectHandlePool[index],
                                                USB_HOST_HID_EVENT_REPORT_SENT, 
                                                &responseData
                                            );
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            
            else
            {
                /* Other errors. No handling is effective */
            }
            break;
        case USB_HOST_DEVICE_INTERFACE_EVENT_SET_INTERFACE_COMPLETE:
            break;
        case USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE:
            haltClearCompleteEventData =
                (USB_HOST_DEVICE_INTERFACE_EVENT_PIPE_HALT_CLEAR_COMPLETE_DATA *)
                    (eventData);
            hidInstanceInfo->requestObj.controlRequestDone = true;
            if(USB_HOST_RESULT_SUCCESS == haltClearCompleteEventData->result)
            {
                hidInstanceInfo->state = USB_HOST_HID_STATE_READY;
            }
            else
            {
                hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
            }
            break;
        default:
            break;
    }
    
    return(USB_HOST_DEVICE_INTERFACE_EVENT_RESPONSE_NONE);

} /* End of _USB_HOST_HID_InterfaceEventHandler() */


/*************************************************************************/
/* Function:
    int32_t _USB_HOST_HID_SignedDataGet(USB_HOST_HID_ITEM * itemData)

  Summary:
    Function extracts signed item data as per the item size.

  Description:
    Function extracts signed item data as per the item size.

  Remarks:
    None.
*/

int32_t _USB_HOST_HID_SignedDataGet(USB_HOST_HID_ITEM * itemData)
{
    /* Read data value from item */
    switch (itemData->size)
    {
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_1_BYTE:
            return itemData->optionalItemData.signedData8;
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_2_BYTE:
            return itemData->optionalItemData.signedData16;
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_4_BYTE: 
            return itemData->optionalItemData.signedData32;
    }
    return 0;

}/* end of _USB_HOST_HID_SignedDataGet () */


/*************************************************************************/
/* Function:
    uint32_t _USB_HOST_HID_UnsignedDataGet(USB_HOST_HID_ITEM * itemData)

  Summary:
    Function extracts unsigned item data as per the item size.

  Description:
    Function extracts unsigned item data as per the item size.

  Remarks:
    None.
*/

uint32_t _USB_HOST_HID_UnsignedDataGet(USB_HOST_HID_ITEM * itemData)
{
    /* Read data value from item */
    switch (itemData->size)
    {
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_1_BYTE:
            return itemData->optionalItemData.unsignedData8;
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_2_BYTE:
            return itemData->optionalItemData.unsignedData16;
        case USB_HOST_HID_SHORT_ITEM_DATA_SIZE_4_BYTE: 
            return itemData->optionalItemData.unsignedData32;
    }
    return 0;

}/* end of _USB_HOST_HID_UnsignedDataGet () */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT _USB_HOST_HID_MainItemParse
    (
        uint8_t hidInstanceIndex,
        USB_HOST_HID_ITEM *itemData
    )

  Summary:
    Function parses and extracts MAIN item data.

  Description:
    Function parses and extracts MAIN item data.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT _USB_HOST_HID_MainItemParse
(
    uint8_t hidInstanceIndex,
    USB_HOST_HID_ITEM *itemData
)
{
    /* Start of local variables */
    uint32_t data;
    /* End of local variables */

    /* Extract the data from HID item structure */
    data = _USB_HOST_HID_UnsignedDataGet(itemData);

    /* Process according to item tag */
    switch(itemData->tag) 
    {
        case USB_HID_MAIN_ITEM_TAG_BEGIN_COLLECTION:
            gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex = 0;
            memset(gUSBHostHIDInstance[hidInstanceIndex].globalStack, 0,
                (size_t)(sizeof(USB_HOST_HID_GLOBAL_ITEM) * 
                USB_HID_GLOBAL_PUSH_POP_STACK_SIZE));
            
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->data.data4Bytes
                        = data;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->tag
                        = USB_HID_MAIN_ITEM_TAG_BEGIN_COLLECTION;
            gUSBHostHIDInstance[hidInstanceIndex].collectionNestingLevel++;
            return USB_HOST_HID_RESULT_SUCCESS;
            break;
            
        case USB_HID_MAIN_ITEM_TAG_END_COLLECTION:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->data.data4Bytes
                        = data;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->tag
                        = USB_HID_MAIN_ITEM_TAG_END_COLLECTION;
            gUSBHostHIDInstance[hidInstanceIndex].collectionNestingLevel--;
            return USB_HOST_HID_RESULT_SUCCESS;
            break;
            
        case USB_HID_MAIN_ITEM_TAG_INPUT:
            if((gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMinimum <=
                       gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMaximum)
                    && (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMinimum <=
                    gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMaximum))
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->data.data4Bytes
                        = data;
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->tag
                        = USB_HID_MAIN_ITEM_TAG_INPUT;
                
                return USB_HOST_HID_RESULT_SUCCESS;
            }
            else
            {
                return USB_HOST_HID_RESULT_FAILURE;
            }
            
        case USB_HID_MAIN_ITEM_TAG_OUTPUT:
            if((gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMinimum <=
                       gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMaximum)
                    && (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMinimum <=
                    gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMaximum))
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->data.data4Bytes
                        = data;
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->tag
                        = USB_HID_MAIN_ITEM_TAG_OUTPUT;
                
                return USB_HOST_HID_RESULT_SUCCESS;
            }
            else
            {
                return USB_HOST_HID_RESULT_FAILURE;
            }

        case USB_HID_MAIN_ITEM_TAG_FEATURE:
            if((gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMinimum <=
                       gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->logicalMaximum)
                    && (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMinimum <=
                    gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->physicalMaximum))
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->data.data4Bytes
                        = data;
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->tag
                        = USB_HID_MAIN_ITEM_TAG_FEATURE;
                
                return USB_HOST_HID_RESULT_SUCCESS;
            }
            else
            {
                return USB_HOST_HID_RESULT_FAILURE;
            }
            
        default:
            SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                    "\r\nUSBHID Client Driver: Unknown Main item tag");
            return USB_HOST_HID_RESULT_FAILURE;
    }
} /* End of _USB_HOST_HID_MainItemParse() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT _USB_HOST_HID_LocalItemParse
    (
        uint8_t hidInstanceIndex,
        USB_HOST_HID_ITEM *itemData,
        bool flag,
        uint32_t fieldIndex,
        uint32_t * buffer,
        USB_HOST_HID_QUERY_TYPE query
    )

  Summary:
    Function parses and extracts LOCAL item data.

  Description:
    Function parses and extracts LOCAL item data.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT _USB_HOST_HID_LocalItemParse
(
    uint8_t hidInstanceIndex,
    USB_HOST_HID_ITEM *itemData,
    bool flag,
    uint32_t fieldIndex,
    uint32_t * buffer,
    USB_HOST_HID_QUERY_TYPE query
)
{
    /* Start of local variables */
    uint32_t data = 0;
    /* End of local variables */
    if(0 == itemData->size)
    {
        SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                "\r\nUSBHID Client Driver: Invalid item data");
        return USB_HOST_HID_RESULT_FAILURE;
    }
    data = _USB_HOST_HID_UnsignedDataGet(itemData);
    
    switch(itemData->tag)
    {
        case USB_HID_LOCAL_ITEM_TAG_DELIMITER:
            if(1 == data)
            {
                if(!(0 == gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        localItem->delimiterCounter))
                {
                    SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                            "\r\nUSBHID Client Driver: Nested delimiter");
                    return USB_HOST_HID_RESULT_FAILURE;
                }
                ++gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        localItem->delimiterCounter;
                ++gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        localItem->delimiterBranch;
            }
            else
            {
                if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        localItem->delimiterCounter < 1)
                {
                    SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                            "\r\nUSBHID Client Driver: Invalid close delimiter");
                    return USB_HOST_HID_RESULT_FAILURE;
                }
                --gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        localItem->delimiterCounter;
            }
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_LOCAL_ITEM_TAG_USAGE:
            if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    delimiterBranch > 1) 
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Alternative usage ignored");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            
            if(gUSBHostHIDInstance[hidInstanceIndex].collectionNestingLevel == 0
                    && gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing)
            {
                if(gUSBHostHIDInstance[hidInstanceIndex].nTopLevelUsages <
                        USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER)
                {
                    if(itemData->size <= 2)
                    {
                        data = (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                                globalItem->usagePage << USB_HOST_HID_USAGE_PAGE_SHIFT)
                                + data;
                    }
                    else
                    {
                        SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                                "\r\nUSBHID Client Driver: Usage not qualified for top level usage");
                        return USB_HOST_HID_RESULT_FAILURE;
                    }
                    gUSBHostHIDInstance[hidInstanceIndex].topLevelUsages
                            [gUSBHostHIDInstance[hidInstanceIndex].nTopLevelUsages] 
                        = data;
                    gUSBHostHIDInstance[hidInstanceIndex].nTopLevelUsages++;
                }
            }
            
            /* This part of the code is useful only for handling USAGE TAGS.
             * We indicate the presence of usage tags during Main item query
             * and on subsequent query by the usage driver, the usage tags
             * count value is passed as an output parameter.
             */
            if((flag == true) && (query == USB_HOST_HID_QUERY_USAGE))
            {
                gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount++;
                if((gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount == 
                        fieldIndex))
                {
                    /* Usage is 16 bits or less it is interpreted as Usage Id and
                     * concatenated with Usage Page to form 32-bit Usage */
                    if(itemData->size <= 2)
                    {
                        data = (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                                globalItem->usagePage << USB_HOST_HID_USAGE_PAGE_SHIFT)
                                + data;
                    }
                    *buffer = data;
                }
            }

            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    usageMinMax.valid = false;
            
            return USB_HOST_HID_RESULT_SUCCESS;
        
        case USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_INDEX:
            /* This part of the code is useful only for handling Designator index.
             * We indicate the presence of designator Index during Main item query
             * and on subsequent query by the usage driver, the designator Index
             * count value is passed as an output parameter.
             */
            if((flag == true) && (query == USB_HOST_HID_QUERY_DESIGNATOR))
            {
                gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount++;
                if((gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount == 
                        fieldIndex))
                {
                    *buffer = data;
                }
            }
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    designatorMinMax.valid = false;

            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_LOCAL_ITEM_TAG_STRING_INDEX:
            /* This part of the code is useful only for handling string Descriptor index.
             * We indicate the presence of designator Index during Main item query
             * and on subsequent query by the usage driver, the string Descriptor
             * index count value is passed as an output parameter.
             */
            if((flag == true) && (query == USB_HOST_HID_QUERY_STRING))
            {
                gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount++;
                if((gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount == 
                        fieldIndex))
                {
                    *buffer = data;
                }
            }
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    stringMinMax.valid = false;

            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_LOCAL_ITEM_TAG_USAGE_MINIMUM:
            if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    delimiterBranch > 1)
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Alternative usage ignored");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            if(itemData->size <= 2)
            {
                data = (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        globalItem->usagePage << USB_HOST_HID_USAGE_PAGE_SHIFT)
                        + data;
            }
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                usageMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    usageMinMax.min = data;
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM:
            if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    delimiterBranch > 1)
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Alternative usage ignored");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            if(itemData->size <= 2)
            {   
                data = (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                        globalItem->usagePage << USB_HOST_HID_USAGE_PAGE_SHIFT)
                        + data;
            }
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    usageMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    usageMinMax.max = data;
                    
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_MINIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                designatorMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    designatorMinMax.min = data;
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_MAXIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                designatorMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    designatorMinMax.max = data;
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_LOCAL_ITEM_TAG_STRING_MINIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                stringMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    stringMinMax.min = data;
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_LOCAL_ITEM_TAG_STRING_MAXIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                stringMinMax.valid = true;
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->localItem->
                    stringMinMax.max = data;
            return USB_HOST_HID_RESULT_SUCCESS;
            
        default:
            SYS_DEBUG_MESSAGE (SYS_ERROR_INFO, "\r\nUSBHID Client Driver: Unknown local item tag");
            return USB_HOST_HID_RESULT_FAILURE;
    }
} /* End of _USB_HOST_HID_LocalItemParse() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT _USB_HOST_HID_GlobalItemParse
    (
        uint8_t hidInstanceIndex,
        USB_HOST_HID_ITEM *itemData
    )

  Summary:
    Function parses and extracts GLOBAL item data.

  Description:
    Function parses and extracts GLOBAL item data.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT _USB_HOST_HID_GlobalItemParse
(
    uint8_t hidInstanceIndex,
    USB_HOST_HID_ITEM *itemData
)
{
    switch(itemData->tag)
    {
        case USB_HID_GLOBAL_ITEM_TAG_PUSH:
            if(USB_HID_GLOBAL_PUSH_POP_STACK_SIZE == 
                    gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex)
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Global stack overflow");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            if(!(0 == itemData->size))
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Parser error in Push Item Tag");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            /* Copy all global item values on the stack */
            memcpy(gUSBHostHIDInstance[hidInstanceIndex].globalStack +
                    gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex++,
                    gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem,
                    sizeof(USB_HOST_HID_GLOBAL_ITEM));
            
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_POP:
            if(0 == gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex)
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Global item stack underflow");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            if(!(0 == itemData->size))
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Parser error in Pop Item Tag");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex--;
            /* Copy all the global item values from stack */
            memcpy(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem,
                    gUSBHostHIDInstance[hidInstanceIndex].globalStack +
                    gUSBHostHIDInstance[hidInstanceIndex].globalStackIndex,
                   sizeof(USB_HOST_HID_GLOBAL_ITEM));
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_USAGE_PAGE:
            (gUSBHostHIDInstance[hidInstanceIndex].mainItemData)->globalItem->
                    usagePage = _USB_HOST_HID_UnsignedDataGet(itemData);
            if(0 == gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->usagePage)
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO,
                        "\r\nUSBHID Client Driver: Parser error in Usage PageItem Tag");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    logicalMinimum = 
                    _USB_HOST_HID_SignedDataGet(itemData);
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM:
            if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    logicalMinimum < 0)
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                        logicalMaximum =
                    _USB_HOST_HID_SignedDataGet(itemData);
            }
            else
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                        logicalMaximum =
                    _USB_HOST_HID_UnsignedDataGet(itemData);
            }
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_PHY_MINIMUM:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    physicalMinimum =
                _USB_HOST_HID_SignedDataGet(itemData);
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_PHY_MAXIMUM:
            if(gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    physicalMinimum < 0)
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                        physicalMaximum =
                    _USB_HOST_HID_SignedDataGet(itemData);
            }
            else
            {
                gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                        physicalMaximum =
                    _USB_HOST_HID_UnsignedDataGet(itemData);
            }
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    unitExponent =
                _USB_HOST_HID_UnsignedDataGet(itemData);
            return USB_HOST_HID_RESULT_SUCCESS;
            
        case USB_HID_GLOBAL_ITEM_TAG_UNIT:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->unit
                    = _USB_HOST_HID_UnsignedDataGet(itemData);
            return USB_HOST_HID_RESULT_SUCCESS;
        
        case USB_HID_GLOBAL_ITEM_TAG_REPORT_SIZE:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    reportSize
                    = _USB_HOST_HID_UnsignedDataGet(itemData);
            return USB_HOST_HID_RESULT_SUCCESS;
        
        case USB_HID_GLOBAL_ITEM_TAG_REPORT_COUNT:
            gUSBHostHIDInstance[hidInstanceIndex].mainItemData->globalItem->
                    reportCount =
                       _USB_HOST_HID_UnsignedDataGet(itemData);
            /*
             Ideally Report Count should not be 0 as it that case there will not
             be any field applicable. But there are some devices e.g. Dell mouse
             which reports field with report count = 0. So this will be a valid
             case of considering a field, with no field created or considered in
             the usage driver.
             */
            return USB_HOST_HID_RESULT_SUCCESS;

        case USB_HID_GLOBAL_ITEM_TAG_REPORT_ID:
            if(0 == (gUSBHostHIDInstance[hidInstanceIndex].mainItemData->
                    globalItem->reportID =
                        _USB_HOST_HID_UnsignedDataGet(itemData)))
            {
                SYS_DEBUG_MESSAGE (SYS_ERROR_INFO ,
                        "\r\nUSBHID Client Driver: ReportID invalid");
                return USB_HOST_HID_RESULT_FAILURE;
            }
            return USB_HOST_HID_RESULT_SUCCESS;

        default:
            SYS_DEBUG_MESSAGE (SYS_ERROR_INFO ,
                    "\r\nUSBHID Client Driver: Unknown global tag");
            return USB_HOST_HID_RESULT_FAILURE;
    }
} /* End of _USB_HOST_HID_GlobalItemParse() */


/*************************************************************************/
/* Function:
    uint8_t * _USB_HOST_HID_ItemFetch
    (
        uint8_t *startAddress,
        uint8_t *endAddress,
        USB_HOST_HID_ITEM *itemData
    )

  Summary:
    Function extracts item data as per item size and returns the start
    address of the next item.

  Description:
    Function extracts item data as per item size and returns the start
    address of the next item.

  Remarks:
    None.
*/

uint8_t * _USB_HOST_HID_ItemFetch
(
    uint8_t *startAddress,
    uint8_t *endAddress,
    USB_HOST_HID_ITEM *itemData
)
{
    uint8_t temp = 0;
    /* Extract the 1-byte prefix to item */
    temp = *startAddress++;
    /* Extract the item type and tag */
    itemData->type = (temp >> USB_HOST_HID_ITEM_TYPE_SHIFT) &
       USB_HOST_HID_ITEM_TYPE_MASK;
    itemData->tag = (temp >> USB_HOST_HID_ITEM_TAG_SHIFT) &
        USB_HOST_HID_ITEM_TAG_MASK;

    if(USB_HOST_HID_ITEM_TAG_LONG == itemData->tag)
    {
        /* LONG ITEM obtained */
        return NULL;
    }
    /* Obtaining the Item Size */
    itemData->size = temp & USB_HOST_HID_ITEM_SIZE_MASK;

    /* Obtain each Items according to Item size */
    switch(itemData->size)
    {
        case 0:
            return startAddress;
        case 1:
            if((endAddress - startAddress) < 1)
            {
                return NULL;
            }
            itemData->optionalItemData.unsignedData8 = *startAddress++;
            return startAddress;
        case 2:
            if((endAddress - startAddress) < 2)
            {
                return NULL;
            }
            itemData->optionalItemData.unsignedData16 = 
                    USB_HOST_HID_GET_UNALIGNED((uint16_t *)startAddress);
            startAddress = (uint8_t *)((uint16_t *) startAddress + 1);
            return startAddress;
        case 3:
            itemData->size++;
            if((endAddress - startAddress) < 4)
            {
                return NULL;
            }
            itemData->optionalItemData.unsignedData32 = 
                    USB_HOST_HID_GET_UNALIGNED((uint32_t *)startAddress);
            startAddress = (uint8_t *)((uint32_t *)startAddress + 1);
            return startAddress;
    }
    return NULL;
    
} /* End of _USB_HOST_HID_ItemFetch() */


USB_HOST_HID_RESULT _USB_HOST_HID_ItemGet
(
    uint8_t hidInstanceIndex,
    uint8_t index,
    bool flag,
    uint32_t fieldIndex,
    uint32_t * buffer,
    USB_HOST_HID_QUERY_TYPE query
)
{
    /* Start of local variables */
    uint16_t reportDescLength =
            gUSBHostHIDInstance[hidInstanceIndex].reportDescLength;
    uint8_t *startAddress =
            gUSBHostHIDInstance[hidInstanceIndex].reportDescBuffer;
    uint8_t *endAddress = startAddress + reportDescLength;
    uint8_t fieldCount = 0;
    USB_HOST_HID_ITEM itemData;
    /* End of local variables */
    
    if(index == 0)
    {
        return USB_HOST_HID_RESULT_FAILURE;
    }
    if(flag == true)
    {
        /* We are here due to usage driver query */
        if(fieldIndex == 0)
        {
            return USB_HOST_HID_RESULT_FAILURE;
        }
    }
    do
    {
        startAddress = _USB_HOST_HID_ItemFetch(startAddress, endAddress,
                &itemData);
        if(itemData.type == USB_HID_REPORT_ITEM_HEADER_BTYPE_MAIN)
        {
            /* MAIN ITEM TAG */
            if(gUSBHostHIDInstance[hidInstanceIndex].topLevelUsageProcessing)
            {
                fieldCount++;
                if(fieldCount == index)
                {
                    if(USB_HOST_HID_RESULT_SUCCESS == 
                        _USB_HOST_HID_MainItemParse(hidInstanceIndex, &itemData))
                    {
                        /* Got a valid field */
                    }
                    else
                    {
                        return USB_HOST_HID_RESULT_FAILURE;
                    }  
                }
            }
            else if(USB_HOST_HID_RESULT_SUCCESS == 
                        _USB_HOST_HID_MainItemParse(hidInstanceIndex, &itemData))
            {
                /* Got a valid field */
                fieldCount++;
            }
            else
            {
                return USB_HOST_HID_RESULT_FAILURE;
            }
            if(fieldCount == index)
            {
                /* Got all field data for the requested field index */
                if(flag == false)
                {
                    return USB_HOST_HID_RESULT_SUCCESS;
                }
                else
                {
                    if(query == USB_HOST_HID_QUERY_USAGE)
                    {
                        if(fieldIndex <=
                                gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount)
                        {
                            /* Obtained the individual tags as queried by the
                             usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount = 0;
                            return USB_HOST_HID_RESULT_SUCCESS;
                        }
                        else
                        {
                            /* Did not obtain all the individual tags as queried
                               by the usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount = 0;
                            return USB_HOST_HID_RESULT_FAILURE;
                        }
                    }
                    if(query == USB_HOST_HID_QUERY_STRING)
                    {
                        if(fieldIndex <=
                                gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount)
                        {
                            /* Obtained the individual tags as queried by the
                             usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount = 0;
                            return USB_HOST_HID_RESULT_SUCCESS;
                        }
                        else
                        {
                            /* Did not obtain all the individual tags as queried
                               by the usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount = 0;
                            return USB_HOST_HID_RESULT_FAILURE;
                        }
                    }
                    if(query == USB_HOST_HID_QUERY_DESIGNATOR)
                    {
                        if(fieldIndex <=
                                gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount)
                        {
                            /* Obtained the individual tags as queried by the
                             usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount = 0;
                            return USB_HOST_HID_RESULT_SUCCESS;
                        }
                        else
                        {
                            /* Did not obtain all the individual tags as queried
                               by the usage driver */
                            gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount = 0;
                            return USB_HOST_HID_RESULT_FAILURE;
                        }
                    }
                    
                }
            }
        }
        else if(itemData.type == USB_HID_REPORT_ITEM_HEADER_BTYPE_LOCAL)
        {
            /* LOCAL ITEM TAG */
            /* Update local item for the required field only */
            if(fieldCount == (index - 1))
            {
                if(USB_HOST_HID_RESULT_SUCCESS != 
                        _USB_HOST_HID_LocalItemParse(hidInstanceIndex,
                        &itemData, flag, fieldIndex, buffer, query))
                {
                    return USB_HOST_HID_RESULT_FAILURE;
                }
            }
        }
        else if(itemData.type == USB_HID_REPORT_ITEM_HEADER_BTYPE_GLOBAL)
        {
            /* GLOBAL ITEM TAG */
            /* There is no point in parsing global items while scanning
             for top level usages. Only local and main item are good enough.
             So we do not need to obtain global items from report descriptor
             */
            if(USB_HOST_HID_RESULT_SUCCESS != 
                    _USB_HOST_HID_GlobalItemParse(hidInstanceIndex, &itemData))
            {
                return USB_HOST_HID_RESULT_FAILURE;
            }
        }
        else if(itemData.type == USB_HID_REPORT_ITEM_HEADER_BTYPE_RESERVED)
        {
            /* Do Nothing */
        }
    } while((startAddress != endAddress) && (startAddress != NULL));
    
    return USB_HOST_HID_RESULT_FAILURE;

} /* End of _USB_HOST_HID_ItemGet() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_MainItemGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint8_t mainItemIndex,
        USB_HOST_HID_MAIN_ITEM *pMainItemData
    )

  Summary:
    Function obtains the field information for the appropriate mainItemIndex.

  Description:
    Function obtains the field information for the appropriate mainItemIndex.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_MainItemGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint8_t mainItemIndex,
    USB_HOST_HID_MAIN_ITEM *pMainItemData
)
{
    /* Start of local variables */
    int8_t hidInstanceIndex = (-1);
    /* End of local variables */
    
    if(handle == USB_HOST_HID_OBJ_HANDLE_INVALID || pMainItemData == NULL)
    {
        return USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    /* Obtain instance number of HID driver instance which owns this handle */
    hidInstanceIndex = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
    if(hidInstanceIndex < 0)
    {
        /* HID index not obtained */
        return USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    if(!gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing)
    {
        gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = true;
        gUSBHostHIDInstance[hidInstanceIndex].mainItemData = pMainItemData;
    }
    else
    {
        /* Return busy */
        return USB_HOST_HID_RESULT_REQUEST_BUSY;
    }

    /* Note the usage of the last 3 parameters in _USB_HOST_HID_ItemGet()
     * function call. If flag = false, that means we are calling for
     * Main item query due to usage driver request.
     * If flag = true, that means this function is called for usage/string/
     * designator value query. The other 2 parameters become useful in this
     * case only.
     */
    if(USB_HOST_HID_RESULT_SUCCESS !=
            _USB_HOST_HID_ItemGet((uint8_t)hidInstanceIndex, mainItemIndex,
                                   false, 0, NULL, 0))
    {
        gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
        return USB_HOST_HID_RESULT_FAILURE;
    }
    gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
    return USB_HOST_HID_RESULT_SUCCESS;
    
}/* End of USB_HOST_HID_MainItemGet() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_UsageGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *usage
    )

  Summary:
    Function obtains the Usage ID specific to the main item and field index.

  Description:
    Function obtains the Usage ID specific to the main item and field index.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_UsageGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *usage
)
{
    /* Start of local variables */
    int8_t hidInstanceIndex = (-1);
    USB_HOST_HID_RESULT result = USB_HOST_HID_RESULT_FAILURE;
    /* End of local variables */

    /* Check if parameters are invalid */
    if((handle == USB_HOST_HID_OBJ_HANDLE_INVALID) || (usage == NULL))
    {
        result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {

        /* Obtain the instance number of the HID driver instance which
         * owns this handle */
        hidInstanceIndex = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);

        if(hidInstanceIndex < 0)
        {
            /* HID index not obtained */
            result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
        /* Obtain the locking on Field Processing. Ideally there should NOT
         * be any Race condition as all Usage driver task routines run
         * in HID client driver task routine context. So what that means
         * is Usage driver task routines cannot preempt each other even
         * in RTOS environment.
         *
         * The recommended model for Usage drivers is to perform
         * Field related operations in task context. But for very specific
         * use cases it might be done from ISR context after getting the IN
         * report event.
         *
         * We apply test and set design ideology through the use of
         * isFieldProcessing. Now Report Descriptor Field related operation
         * has to be atomic.
         * */
        else if(!gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing)
        {
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = true;
            gUSBHostHIDInstance[hidInstanceIndex].usageTagsCount = 0;

            if(USB_HOST_HID_RESULT_SUCCESS != _USB_HOST_HID_ItemGet((uint8_t)hidInstanceIndex,
                        mainItemIndex, true, fieldIndex, usage, USB_HOST_HID_QUERY_USAGE))
            {
                result = USB_HOST_HID_RESULT_FAILURE;
            }
            else
            {
                result = USB_HOST_HID_RESULT_SUCCESS;
            }
            /* Release the locking on Field Processing */
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
        }
        else
        {
            /* HID client driver busy to accpet this request.
             * Usage driver can retry later */
            result = USB_HOST_HID_RESULT_REQUEST_BUSY;
        }
    }

    /*
     * USB_HOST_HID_RESULT_SUCCESS: On success
     * USB_HOST_HID_RESULT_FAILURE: Corresponding item not found
     * USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     * USB_HOST_HID_RESULT_REQUEST_BUSY: HID client driver busy
     */
    return result;

}/* End of USB_HOST_HID_UsageGet() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_StringIndexGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *stringDescriptorIndex
    )

  Summary:
    Function obtains the String ID specific to the main item and field index.

  Description:
    Function obtains the String ID specific to the main item and field index.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_StringIndexGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *stringDescriptorIndex
)
{
    /* Start of local variables */
    int8_t hidInstanceIndex = (-1);
    USB_HOST_HID_RESULT result = USB_HOST_HID_RESULT_FAILURE;
    /* End of local variables */

    /* Check if parameters are invalid */
    if((handle == USB_HOST_HID_OBJ_HANDLE_INVALID) || (stringDescriptorIndex == NULL))
    {
        result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {
    
        /* Obtain the instance number of the HID driver instance which
         * owns this handle */
        hidInstanceIndex = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);
        
        if(hidInstanceIndex < 0)
        {
            /* HID index not obtained */
            result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
        /* Obtain the locking on Field Processing. Ideally there should NOT
         * be any Race condition as all Usage driver task routines run
         * in HID client driver task routine context. So what that means
         * is Usage driver task routines cannot preempt each other even
         * in RTOS environment.
         *
         * The recommended model for Usage drivers is to perform
         * Field related operations in task context. But for very specific
         * usecases it might be done from ISR context after getting the IN
         * report event.
         *
         * We apply test and set design ideology through the use of
         * isFieldProcessing. Now Report Descriptor Field related operation
         * has to be atomic.
         * */
        else if(!gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing)
        {
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = true;
            gUSBHostHIDInstance[hidInstanceIndex].stringDescriptorIndexCount = 0;
        
            if(USB_HOST_HID_RESULT_SUCCESS != _USB_HOST_HID_ItemGet((uint8_t)hidInstanceIndex,
                    mainItemIndex, true, fieldIndex, stringDescriptorIndex, USB_HOST_HID_QUERY_STRING))
            {
                result = USB_HOST_HID_RESULT_FAILURE;
            }
            else
            {
                result = USB_HOST_HID_RESULT_SUCCESS;
            }
            /* Release the locking on Field Processing */
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
        }
        else
        {
            /* HID client driver busy to accpet this request.
             * Usage driver can retry later */
            result = USB_HOST_HID_RESULT_REQUEST_BUSY;
        }
    }

    /*
     * USB_HOST_HID_RESULT_SUCCESS: On success
     * USB_HOST_HID_RESULT_FAILURE: Corresponding item not found
     * USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     * USB_HOST_HID_RESULT_REQUEST_BUSY: HID client driver busy
     */
    return result;

}/* End of USB_HOST_HID_StringIndexGet() */


/*************************************************************************/
/* Function:
    USB_HOST_HID_RESULT USB_HOST_HID_DesignatorIndexGet
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        uint32_t mainItemIndex,
        uint32_t fieldIndex,
        uint32_t *physicalDescriptorDesignatorIndex
    )

  Summary:
    Function obtains the Designator specific to the main item and field index.

  Description:
    Function obtains the Designator specific to the main item and field index.

  Remarks:
    None.
*/

USB_HOST_HID_RESULT USB_HOST_HID_DesignatorIndexGet
(
    USB_HOST_HID_OBJ_HANDLE handle,
    uint32_t mainItemIndex,
    uint32_t fieldIndex,
    uint32_t *physicalDescriptorDesignatorIndex
)
{
    /* Start of local variables */
    int8_t hidInstanceIndex = (-1);
    USB_HOST_HID_RESULT result = USB_HOST_HID_RESULT_FAILURE;
    /* End of local variables */

    /* Check if parameters are invalid */
    if((handle == USB_HOST_HID_OBJ_HANDLE_INVALID) || (physicalDescriptorDesignatorIndex == NULL))
    {
        result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
    }
    else
    {

        /* Obtain the instance number of the HID driver instance which
         * owns this handle */
        hidInstanceIndex = _USB_HOST_HID_ObjectHandleToHIDIndex(handle);

        if(hidInstanceIndex < 0)
        {
            /* HID index not obtained */
            result = USB_HOST_HID_RESULT_PARAMETER_INVALID;
        }
        /* Obtain the locking on Field Processing. Ideally there should NOT
         * be any Race condition as all Usage driver task routines run
         * in HID client driver task routine context. So what that means
         * is Usage driver task routines cannot preempt each other even
         * in RTOS environment.
         *
         * The recommended model for Usage drivers is to perform
         * Field related operations in task context. But for very specific
         * usecases it might be done from ISR context after getting the IN
         * report event.
         *
         * We apply test and set design ideology through the use of
         * isFieldProcessing. Now Report Descriptor Field related operation
         * has to be atomic.
         * */
        else if(!gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing)
        {
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = true;
            gUSBHostHIDInstance[hidInstanceIndex].designatorIndexCount = 0;

            if(USB_HOST_HID_RESULT_SUCCESS != _USB_HOST_HID_ItemGet((uint8_t)hidInstanceIndex,
                        mainItemIndex, true, fieldIndex, physicalDescriptorDesignatorIndex, USB_HOST_HID_QUERY_DESIGNATOR))
            {
                result = USB_HOST_HID_RESULT_FAILURE;
            }
            else
            {
                result = USB_HOST_HID_RESULT_SUCCESS;
            }
            /* Release the locking on Field Processing */
            gUSBHostHIDInstance[hidInstanceIndex].isFieldProcessing = false;
        }
        else
        {
            /* HID client driver busy to accpet this request.
             * Usage driver can retry later */
            result = USB_HOST_HID_RESULT_REQUEST_BUSY;
        }
    }

    /*
     * USB_HOST_HID_RESULT_SUCCESS: On success
     * USB_HOST_HID_RESULT_FAILURE: Corresponding item not found
     * USB_HOST_HID_RESULT_PARAMETER_INVALID: Invalid Parameter
     * USB_HOST_HID_RESULT_REQUEST_BUSY: HID client driver busy
     */
    return result;

}/* End of USB_HOST_HID_DesignatorIndexGet() */


// *****************************************************************************
/* Function:
    void _USB_HOST_HID_InterfaceTasks
    (
        USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle
    )
 
  Summary:
    Task routine for HID client driver.
  
  Description:
    Task routine for HID client driver.

  Remarks:
    This is local function and should not be called directly by the application.
*/

void _USB_HOST_HID_InterfaceTasks(USB_HOST_DEVICE_INTERFACE_HANDLE interfaceHandle)
{
    /* Start of local variables */
    USB_HOST_HID_INSTANCE * hidInstanceInfo = NULL;
    USB_HOST_TRANSFER_HANDLE  transferHandle;
    USB_HOST_HID_OBJ_HANDLE handle = USB_HOST_HID_OBJ_HANDLE_INVALID;
    USB_HOST_HID_RESULT status = USB_HOST_HID_RESULT_FAILURE;
    USB_HOST_RESULT result = USB_HOST_RESULT_FAILURE;
    uint32_t usageToCheck = 0;
    uint16_t idleTime = 0;
    uint8_t loop = 0;
    int8_t hidInstanceIndex = 0;
    uint8_t index = 0;
    USB_HOST_HID_USAGE_DRIVER_TABLE_ENTRY * usageDriverTableEntry = NULL;
    /* End of local variables */

    /* Get the HID instance for this interface */
    hidInstanceIndex = _USB_HOST_HID_InterfaceHandleToHIDIndex(interfaceHandle);

    if(hidInstanceIndex >= 0)
    {
        hidInstanceInfo = &gUSBHostHIDInstance[hidInstanceIndex];

        switch(hidInstanceInfo->state)
        {
            case USB_HOST_HID_STATE_SET_IDLE_SEND:
                hidInstanceInfo->state = USB_HOST_HID_STATE_WAITING_SET_IDLE;
                /*
                 * The recommended idle rate is infinite(value 0) for mouse and
                 * 500 ms (value 125) for keyboard. We will start with the HID 
                 * specification recommended values. These values are only
                 * applicable for boot interface devices.
                 *
                 * For Non Boot interfaces the idle rate will be infinite.
                 */
                if(hidInstanceInfo->hidDeviceInfo.isBootInterfaceClass)
                {
                    /* Obtain the IDLE time as per the connected device */
                    if((hidInstanceInfo->hidDeviceInfo.isKeyboardDevice))
                    {
                        /* Keyboard device */
                        idleTime = 500;
                    }
                    else
                    {
                        /* Mouse device */
                    }

                    /* As per the HID specification the IDLE rate resolution
                     * is 4ms.
                     */
                    idleTime = idleTime / 4;
                }/* end of if(Boot Interface) */

                /*
                 * controlRequestDone = true: Last Control request submitted
                 *                            by this HID client driver instance
                 *                            has not yet completed
                 *
                 * controlRequestDone = false: No pending Control request
                 */

                if(hidInstanceInfo->requestObj.controlRequestDone == true)
                {
                    /* Set the flag indicating we are waiting for the control
                     * request to complete */
                    hidInstanceInfo->requestObj.controlRequestDone = false;

                    /* Create the standard USB packet for SET IDLE request */
                    _USB_HOST_HID_SetIdlePacketCreate
                        (
                         /* PLACE HOLDER FOR SET UP PACKET */
                         &hidInstanceInfo->requestObj,
                         /* INTERFACE NUMBER */
                         hidInstanceInfo->bInterfaceNumber,
                         /* IDLE TIME */
                         (uint8_t)idleTime,
                         /* REPORT ID */
                         0
                        );

                    /* Launch the request */
                    result = USB_HOST_DeviceControlTransfer
                        (
                         /* CONTROL pipe handle */
                         hidInstanceInfo->controlPipeHandle,
                         /* Transfer Handle */
                         &transferHandle,
                         /* SET IDLE set up packet */
                         &hidInstanceInfo->requestObj.setupPacket,
                         /* No data phase */
                         NULL,
                         /* CONTROL Callback */
                         _USB_HOST_HID_ControlTransferCallback,
                         /* Context */
                         (uintptr_t)(hidInstanceInfo)
                        );
                    if(USB_HOST_RESULT_SUCCESS == result)
                    {
                        /* Request submission was successful */
                    }
                    else
                    {
                        /* Request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                        if(USB_HOST_RESULT_REQUEST_BUSY == result)
                        {
                            /* If the Host was Busy to accept this request
                             * retry it in the next task routine call.
                             */
                            hidInstanceInfo->state = USB_HOST_HID_STATE_SET_IDLE_SEND;
                        }
                        else
                        {
                            hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                                    "\r\nUSBHID Client Driver: SET IDLE submit failed");
                        }
                    } /* end of(Request submission failed) */

                } /* end of(No ongoing Control request) */
                else
                {
                    /* CONTROL request ongoing. Retry in next task routine call */
                    hidInstanceInfo->state = USB_HOST_HID_STATE_SET_IDLE_SEND;
                }

                break;

            case USB_HOST_HID_STATE_WAITING_SET_IDLE:
                /* Do nothing here.
                 * Task state will be moved from from CONTROL transfer complete
                 * handler*/
                
                break;

            case USB_HOST_HID_STATE_SET_PROTOCOL_SEND:
                /* Checking if Boot Interface Subclass is supported */
                if(hidInstanceInfo->hidDeviceInfo.isBootInterfaceClass)
                {
                    hidInstanceInfo->state =
                        USB_HOST_HID_STATE_WAITING_SET_PROTOCOL;
                    
                    if(hidInstanceInfo->requestObj.controlRequestDone == true)
                    {
                        /* Set the flag indicating we are waiting for the
                         * control request to complete */
                        hidInstanceInfo->requestObj.controlRequestDone = false;

                        /* Create the standard USB packet for SET PROTOCOL
                         * request */
                        _USB_HOST_HID_SetProtocolPacketCreate
                        (
                             /* PLACE HOLDER FOR SET UP PACKET */
                             &hidInstanceInfo->requestObj,
                             /* INTERFACE NUMBER */
                             hidInstanceInfo->bInterfaceNumber,
                             /* REPORT PROTOCOL TYPE */
                             1
                        );

                        /* Launch the request */
                        result = USB_HOST_DeviceControlTransfer
                                (
                                     /* CONTROL pipe handle */
                                     hidInstanceInfo->controlPipeHandle,
                                     /* Transfer Handle */
                                     &transferHandle,
                                     /* SET PROTOCOL set up packet */
                                     &hidInstanceInfo->requestObj.setupPacket,
                                     /* No data phase */
                                     NULL,
                                     /* CONTROL Callback */
                                     _USB_HOST_HID_ControlTransferCallback,
                                     /* Context */
                                     (uintptr_t)(hidInstanceInfo)
                                );
                        if(USB_HOST_RESULT_SUCCESS == result)
                        {
                            /* Request submission was successful */
                        }
                        else
                        {
                            /* Request submission failed */
                            hidInstanceInfo->requestObj.controlRequestDone = true;
                            if(USB_HOST_RESULT_REQUEST_BUSY == result)
                            {
                                /* If the Host was Busy to accept this request
                                 * retry it in the next task routine call.
                                 */
                                hidInstanceInfo->state =
                                    USB_HOST_HID_STATE_SET_PROTOCOL_SEND;
                            }
                            else
                            {
                                hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                                    "\r\nUSBHID Client Driver: SET PROTOCOL submit failed");
                            }
                        }/* end of(Request submission failed) */
                    }
                    else
                    {
                        /* CONTROL request ongoing. Retry in the next task
                         * routine call */
                        hidInstanceInfo->state = USB_HOST_HID_STATE_SET_PROTOCOL_SEND;
                    }
                }/* end of if (Device supports Boot Interface) */
                else
                {
                    /* For NON Boot interface no need to send SET PROTOCOL.
                     * It needs to be in REPORT protocol mode as there is
                     * no more protocol option for NON Boot interface */
                    hidInstanceInfo->state = USB_HOST_HID_STATE_REPORT_DESCRIPTOR_GET;
                }

                break;

            case USB_HOST_HID_STATE_WAITING_SET_PROTOCOL:
                /* Do nothing. The state will be moved from from CONTROL
                 * transfer complete handler*/
                
                break;
            
            case USB_HOST_HID_STATE_REPORT_DESCRIPTOR_GET:
                hidInstanceInfo->state = USB_HOST_HID_STATE_WAITING_REPORT_DESCRIPTOR_GET;

                status = _USB_HOST_HID_reportdescriptorGet((uint8_t)hidInstanceIndex);
                if(status == USB_HOST_HID_RESULT_FAILURE)
                {
                    hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,
                            "\r\nUSBHID Client Driver: GET REPORT descriptor submit failed");
                }
                else if(status == USB_HOST_HID_RESULT_REQUEST_BUSY)
                {
                    /* CONTROL request ongoing. Retry in the next task routine call */
                    hidInstanceInfo->state = USB_HOST_HID_STATE_REPORT_DESCRIPTOR_GET;
                }
                else
                {
                    /* Success case */
                }

                break;

            case USB_HOST_HID_STATE_WAITING_REPORT_DESCRIPTOR_GET:
                /* Do nothing. The state will be moved from from CONTROL
                 * transfer complete handler*/
                
                break;
            
            case USB_HOST_HID_STATE_REPORT_DESCRIPTOR_PARSE:
                /* Reset the global parameters */
                hidInstanceInfo->nTopLevelUsages = 0;
                hidInstanceInfo->collectionNestingLevel = 0;

                /* The below function call will traverse across
                 * the entire Report Descritor and will extract all
                 * the top level Usages present. */
                _USB_HOST_HID_FindTopLevelUsage((uint8_t)hidInstanceIndex);
                
                if(hidInstanceInfo->nTopLevelUsages == 0)
                {
                    /* No top level usage found. We cannot proceed with this
                     * device. */
                    hidInstanceInfo->state = USB_HOST_HID_STATE_DETACHED;
                }
                else
                {
                    hidInstanceInfo->state = USB_HOST_HID_STATE_ATTACHED;
                }

                break;

            case USB_HOST_HID_STATE_ATTACHED:
                /* The code flow comes here when HID specific enumeration
                 * has completed and we know the list of top level
                 * usages present.
                 */
                if(gUSBHostHIDInitData->usageDriverTable != NULL)
                {
                    usageDriverTableEntry =
                        gUSBHostHIDInitData->usageDriverTable;

                    if(gUSBHostHIDInitData->nUsageDriver < 
                            hidInstanceInfo->nTopLevelUsages)
                    {
                        /* We will NOT support actual usages present in the
                         * device if it exceeds the number of usages registered.
                         */
                        hidInstanceInfo->nTopLevelUsages = 
                            gUSBHostHIDInitData->nUsageDriver;
                    }

                    for(loop = 0; loop < hidInstanceInfo->nTopLevelUsages;
                            loop ++)
                    {
                        for(index = 0; index < gUSBHostHIDInitData->nUsageDriver;
                                index ++)
                        {
                            /*
                               The idea here is to check either the usage ID
                               registered in the HID table or the extended usage
                               registered in the HID table. HID client driver
                               does not assume statically whether usage ID or
                               extended Usage has been registered. It is
                               detected on the fly.

                               This is to keep compatability with older
                               applications that use the latest HID client driver.

                               It is recommended to register both usage ID and
                               usage page as an extended usage in initialization
                               table.
                               */
                            usageToCheck = hidInstanceInfo->topLevelUsages[loop];
                            if(((&usageDriverTableEntry[index])->usage) >>USB_HOST_HID_USAGE_PAGE_SHIFT == 0)
                            {
                                /* Not extended usage - Usage Page not registered */
                                usageToCheck = 0x00FF & usageToCheck;
                            }
                            if(usageToCheck ==
                                    (&usageDriverTableEntry[index])->usage)
                            {
                                /* Found matching registered top level usage */
                                handle = _USB_HOST_HID_ObjectHandleAssign
                                    (
                                     hidInstanceIndex,
                                     (&usageDriverTableEntry[index])->usage,
                                     index
                                    );
                                if(USB_HOST_HID_OBJ_HANDLE_INVALID != handle)
                                {
                                    ((&usageDriverTableEntry[index])->interface)->
                                        usageDriverEventHandler
                                        (
                                         handle,
                                         USB_HOST_HID_EVENT_ATTACH,
                                         (void *)NULL
                                        );
                                    hidInstanceInfo->isHIDDriverAttached = true;
                                }
                            }
                        }
                    }
                }
                /*
                 * hidInstanceInfo->isHIDDriverAttached is true if at least
                 * 1 matching usage driver has been found.
                 */
                if (hidInstanceInfo->isHIDDriverAttached == true)
                {
                    hidInstanceInfo->state = USB_HOST_HID_STATE_READY;
                }
                else
                {
                    /*
                     * No matching usage driver found or no usage registration
                     * table done
                     */
                    hidInstanceInfo->state = USB_HOST_HID_STATE_WAIT;
                }
                break;
            case USB_HOST_HID_STATE_READY:
                /* Start submitting INTERRUPT IN requests */
                for(loop = 0; loop < USB_HOST_HID_INTERRUPT_IN_ENDPOINTS_NUMBER;
                        loop++)
                {
                    if(hidInstanceInfo->interruptInPipeHandle[loop] == 
                            USB_HOST_PIPE_HANDLE_INVALID)
                    {
                        break;
                    }
                    hidInstanceInfo->readBuffer = 
                        (void *)&(gUSBHostHIDReadBuffer[hidInstanceIndex][0]);
                    if(USB_HOST_RESULT_SUCCESS == USB_HOST_DeviceTransfer
                            (
                             hidInstanceInfo->interruptInPipeHandle[loop],
                             &transferHandle, 
                             hidInstanceInfo->readBuffer,
                             hidInstanceInfo->interruptInEndpointSize[loop],
                             (uintptr_t)(hidInstanceInfo)
                            ))
                    {
                        hidInstanceInfo->state = USB_HOST_HID_STATE_WAIT;
                    }
                }
                break;
            case USB_HOST_HID_STATE_WAIT:
                break;

            case USB_HOST_HID_STATE_DETACHED:
                /* The question of detach callback comes only if there has been
                 * ATTACH callback. isHIDDriverAttached is used to detect that
                 * in the code.
                 */
                if(hidInstanceInfo->isHIDDriverAttached)
                {
                    usageDriverTableEntry =
                        gUSBHostHIDInitData->usageDriverTable;
                    for(index=0;
                            index < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                            index ++)
                    {
                        if(gUSBHostHIDObjectHandlePool[index].inUse &&
                                (gUSBHostHIDObjectHandlePool[index].hidInstanceIndex == 
                                 hidInstanceIndex))
                        {
                            /* Release the handle */
                            _USB_HOST_HID_ObjectHandleRelease
                                ((USB_HOST_HID_OBJ_HANDLE)
                                 (&gUSBHostHIDObjectHandlePool[index]));
                            /* Usage driver callback with DETACH event */

                            /*
                             * Now we need to find out this usage is owned by
                             * which entry in usage driver registration table
                             */

                            ((&usageDriverTableEntry
                              [gUSBHostHIDObjectHandlePool[index].usageInstanceIndex])
                             ->interface)->usageDriverEventHandler
                                (
                                 (USB_HOST_HID_OBJ_HANDLE)
                                 &gUSBHostHIDObjectHandlePool[index],
                                 USB_HOST_HID_EVENT_DETACH,
                                 NULL
                                );
                        }
                    }
                }
                hidInstanceInfo->isHIDDriverAttached = false;
                break;
            case USB_HOST_HID_STATE_INTERRUPT_IN_ENDPOINT_CLEAR:
                for(loop = 0; loop < USB_HOST_HID_INTERRUPT_IN_ENDPOINTS_NUMBER;
                        loop++)
                {
                    if(hidInstanceInfo->interruptInPipeHandle[loop] == 
                            USB_HOST_PIPE_HANDLE_INVALID)
                    {
                        hidInstanceInfo->state = USB_HOST_HID_STATE_WAIT;
                        break;
                    }
                    if(hidInstanceInfo->requestObj.controlRequestDone == true)
                    {
                        hidInstanceInfo->requestObj.controlRequestDone = false;
                        if(USB_HOST_RESULT_SUCCESS == USB_HOST_DevicePipeHaltClear
                                (
                                 hidInstanceInfo->interruptInPipeHandle[loop],
                                 &transferHandle,
                                 (uintptr_t)(hidInstanceInfo)
                                ))
                        {
                            /* The request was accepted. Wait for completion */
                            hidInstanceInfo->state = USB_HOST_HID_STATE_WAIT;
                        }
                        else
                        {
                            /* Clear endpoint request submission failed */
                            hidInstanceInfo->requestObj.controlRequestDone = true;
                        }
                    }
                }
                break;
            case USB_HOST_HID_STATE_INTERRUPT_OUT_ENDPOINT_CLEAR:
                if(hidInstanceInfo->requestObj.controlRequestDone == true)
                {
                    hidInstanceInfo->requestObj.controlRequestDone = false;
                    if(USB_HOST_RESULT_SUCCESS == USB_HOST_DevicePipeHaltClear
                            (
                             hidInstanceInfo->interruptOutPipeHandle,
                             &transferHandle,
                             (uintptr_t)(hidInstanceInfo)
                            ))

                    {
                        /* The request was accepted. Wait for completion */
                        hidInstanceInfo->state = USB_HOST_HID_STATE_WAIT;
                    }

                    else
                    {
                        /* Clear endpoint request submission failed */
                        hidInstanceInfo->requestObj.controlRequestDone = true;
                    }
                }
                break;
            default:
                break;
        }

        usageDriverTableEntry = gUSBHostHIDInitData->usageDriverTable;
        for(index = 0; index < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                index ++)
        {
            if(gUSBHostHIDObjectHandlePool[index].inUse)
            {
                ((&usageDriverTableEntry[gUSBHostHIDObjectHandlePool[index].usageInstanceIndex])
                 ->interface)->usageDriverTask(
                     (USB_HOST_HID_OBJ_HANDLE)&gUSBHostHIDObjectHandlePool[index]);
            }
        }
    }
} /* End of _USB_HOST_HID_InterfaceTasks() */
