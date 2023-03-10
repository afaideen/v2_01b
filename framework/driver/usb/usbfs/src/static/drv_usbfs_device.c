/*******************************************************************************
  USB Device Driver Implementation of device mode operation routines

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usbfs_device.c

  Summary:
    USB Device Driver static Implementation of device mode operation routines

  Description:
    The USB device driver provides a simple interface to manage the USB modules
    on Microchip microcontrollers.  This file implements the interface routines
    for the USB driver when operating in device mode.

    While building the driver from source, ALWAYS use this file in the build if
    device mode operation is required.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip  Technology  Inc.   All  rights  reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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

#include "system_config.h"
#include "driver/usb/drv_usb.h"
#include "driver/usb/usbfs/src/drv_usbfs_local.h"

/***********************************************************
 * These wrapper functions allow the static USB driver APIs
 * to be mapped to the DRV_USB_DEVICE_INTERFACE structure
 * which is intended for dynamic API.
 ************************************************************/

void        _DRV_USBFS_STATIC_Close(DRV_HANDLE handle);
void        _DRV_USBFS_STATIC_ClientEventCallbackSet(DRV_HANDLE handle, uintptr_t hReferenceData, DRV_USB_EVENT_CALLBACK eventCallback);
void        _DRV_USBFS_STATIC_DEVICE_Attach(DRV_HANDLE handle);
void        _DRV_USBFS_STATIC_DEVICE_Detach(DRV_HANDLE handle);
void        _DRV_USBFS_STATIC_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address);
void        _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStop(DRV_HANDLE handle);
void        _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStart(DRV_HANDLE handle);
bool        _DRV_USBFS_STATIC_DEVICE_EndpointIsStalled(DRV_HANDLE handle, USB_ENDPOINT endpoint);
bool        _DRV_USBFS_STATIC_DEVICE_EndpointIsEnabled(DRV_HANDLE handle, USB_ENDPOINT endpoint);
uint16_t    _DRV_USBFS_STATIC_DEVICE_SOFNumberGet(DRV_HANDLE handle);
USB_SPEED   _DRV_USBFS_STATIC_DEVICE_CurrentSpeedGet(DRV_HANDLE handle);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_EndpointEnable(DRV_HANDLE handle, USB_ENDPOINT endpoint, USB_TRANSFER_TYPE transferType, uint16_t endpointSize);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_EndpointDisable(DRV_HANDLE handle, USB_ENDPOINT endpoint);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_EndpointStall(DRV_HANDLE handle, USB_ENDPOINT endpoint);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_EndpointStallClear(DRV_HANDLE handle, USB_ENDPOINT endpoint);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_IRPSubmit(DRV_HANDLE handle, USB_ENDPOINT endpoint,  USB_DEVICE_IRP * irp);
USB_ERROR   _DRV_USBFS_STATIC_DEVICE_IRPCancelAll(DRV_HANDLE handle, USB_ENDPOINT endpoint);
DRV_HANDLE  _DRV_USBFS_STATIC_Open(SYS_MODULE_INDEX index, DRV_IO_INTENT ioIntent);

/*****************************************************
 * This structure is a pointer to a set of USB Driver
 * Device mode functions. This set is exported to the
 * device layer when the device layer must use the
 * PIC32MX USB Controller. See how the above wrapper 
 * API is connected to this strucutre.
 ******************************************************/

DRV_USB_DEVICE_INTERFACE gDrvUSBFSDeviceInterface =
{
    .open = _DRV_USBFS_STATIC_Open,
    .close = _DRV_USBFS_STATIC_Close,
    .eventHandlerSet = _DRV_USBFS_STATIC_ClientEventCallbackSet,
    .deviceAddressSet = _DRV_USBFS_STATIC_DEVICE_AddressSet,
    .deviceCurrentSpeedGet = _DRV_USBFS_STATIC_DEVICE_CurrentSpeedGet,
    .deviceSOFNumberGet = _DRV_USBFS_STATIC_DEVICE_SOFNumberGet,
    .deviceAttach = _DRV_USBFS_STATIC_DEVICE_Attach,
    .deviceDetach = _DRV_USBFS_STATIC_DEVICE_Detach,
    .deviceEndpointEnable = _DRV_USBFS_STATIC_DEVICE_EndpointEnable,
    .deviceEndpointDisable = _DRV_USBFS_STATIC_DEVICE_EndpointDisable,
    .deviceEndpointStall = _DRV_USBFS_STATIC_DEVICE_EndpointStall,
    .deviceEndpointStallClear = _DRV_USBFS_STATIC_DEVICE_EndpointStallClear,
    .deviceEndpointIsEnabled = _DRV_USBFS_STATIC_DEVICE_EndpointIsEnabled,
    .deviceEndpointIsStalled = _DRV_USBFS_STATIC_DEVICE_EndpointIsStalled,
    .deviceIRPSubmit = _DRV_USBFS_STATIC_DEVICE_IRPSubmit,
    .deviceIRPCancelAll = _DRV_USBFS_STATIC_DEVICE_IRPCancelAll,
    .deviceRemoteWakeupStop = _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStop,
    .deviceRemoteWakeupStart = _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStart,
    .deviceTestModeEnter = NULL
};

extern DRV_USBFS_GROUP gDrvUSBGroup;

DRV_HANDLE _DRV_USBFS_STATIC_Open(SYS_MODULE_INDEX index, DRV_IO_INTENT ioIntent)
{
    return(_DRV_USBFS_MAKE_NAME(Open)(ioIntent));
}

void _DRV_USBFS_STATIC_Close(DRV_HANDLE ioIntent)
{
    _DRV_USBFS_MAKE_NAME(Close)();
}

void _DRV_USBFS_STATIC_ClientEventCallbackSet
(
    DRV_HANDLE  handle,
    uintptr_t    hReferenceData,
    DRV_USB_EVENT_CALLBACK eventCallBack
)
{
    _DRV_USBFS_MAKE_NAME(ClientEventCallBackSet)(hReferenceData, eventCallBack);
}

void _DRV_USBFS_STATIC_DEVICE_Attach(DRV_HANDLE handle)
{
    _DRV_USBFS_MAKE_NAME(DEVICE_Attach)();
}

void _DRV_USBFS_STATIC_DEVICE_Detach(DRV_HANDLE handle)
{
    _DRV_USBFS_MAKE_NAME(DEVICE_Detach)();
}

void _DRV_USBFS_STATIC_DEVICE_AddressSet(DRV_HANDLE handle, uint8_t address)
{
    _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)(address);
}

void _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStop(DRV_HANDLE handle)
{
    _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop)();
}

void _DRV_USBFS_STATIC_DEVICE_RemoteWakeupStart(DRV_HANDLE handle)
{
    _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)();
}

bool _DRV_USBFS_STATIC_DEVICE_EndpointIsStalled
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)(endpoint));
}

bool _DRV_USBFS_STATIC_DEVICE_EndpointIsEnabled
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)(endpoint));
}

uint16_t _DRV_USBFS_STATIC_DEVICE_SOFNumberGet(DRV_HANDLE handle)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet)());
}

USB_SPEED _DRV_USBFS_STATIC_DEVICE_CurrentSpeedGet(DRV_HANDLE handle)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet)());
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_EndpointEnable
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint, 
    USB_TRANSFER_TYPE transferType, 
    uint16_t endpointSize
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable)(endpoint, transferType, endpointSize));
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_EndpointDisable
(DRV_HANDLE handle, USB_ENDPOINT endpoint)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)(endpoint));
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_EndpointStall
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall)(endpoint));
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_EndpointStallClear
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)(endpoint));
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_IRPSubmit
(
    DRV_HANDLE handle,
    USB_ENDPOINT endpoint,
    USB_DEVICE_IRP * irp
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit)(endpoint, irp));
}

USB_ERROR _DRV_USBFS_STATIC_DEVICE_IRPCancelAll
(
    DRV_HANDLE handle, 
    USB_ENDPOINT endpoint
)
{
    return(_DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll)(endpoint));
}

// *****************************************************************************
/* Function:
      _DRV_USBFS_DEVICE_Initialize(DRV_USBFS_OBJ * drvObj, SYS_MODULE_INDEX index)

  Summary:
    static implementation of _DRV_USBFS_DEVICE_Initialize system interface
    function.

  Description:
    This is the static implementation of _DRV_USBFS_DEVICE_Initialize system
    interface initialization function for USB device.  Function enables
    USB_OTG_INT_SESSION_VALID interrupt to detect VBUS session valid\invalid
    scenario. All the other interrupts will be enabled after the device has been
    attached (DRV_USBFS_DEVICE_Attach() function will be called)

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_DEVICE_Initialize(DRV_USBFS_OBJ * drvObj, SYS_MODULE_INDEX index)
{
    /* Initialize device specific flags */
    drvObj->vbusIsValid = false;
    drvObj->isAttached = false;
    drvObj->isSuspended = false;

    /* Disable all interrupts */
    PLIB_USB_AllInterruptEnable(DRV_USBFS_PERIPHERAL_ID, ~USB_INT_ALL,
            ~USB_ERR_INT_ALL, ~USB_OTG_INT_ALL);

}/* end of _DRV_USBFS_DEVICE_Initialize() */

// *****************************************************************************
/* Function:
      void _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)(uint8_t address)

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet) client
    interface function.

  Description:
    This is the static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)
    client interface initiaization function for USB device.  Function updates
    the USB Address Register U1ADDR<6:0> with the address value.

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)(uint8_t address)
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return;
    }
    /* Set the address */
    PLIB_USB_DeviceAddressSet( DRV_USBFS_PERIPHERAL_ID, address );

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_AddressSet)() */

// *****************************************************************************
/* Function:
      USB_SPEED _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet) client interface initiaization
    function for USB device.  Function checks if driver has been opened and on
    success returns value to indicate FULL speed operation. The targetted PIC32
    USB OTG module only supports FULL speed (NO low speed support) as USB
    device.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_SPEED _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet)()
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_SPEED_ERROR;
    }
    /* Return the speed */
    return USB_SPEED_FULL;

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_CurrentSpeedGet)() */

// *****************************************************************************
/* Function:
      _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)
    client interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart) client interface function for
    USB device.  Function checks if driver has been opened and on success
    enables the USB device to drive resume signalling.  RESUME is enabled by
    programming USB Control register (U1CON<2>).  This function should only be
    called if SUSPEND state has persisted for atleast 5 ms. The client should
    drive RESUME for 10ms and then _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop)()
    function should be called to clear RESUME.

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)()
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return;
    }
    /* Enable Resume signalling */
    PLIB_USB_ResumeSignalingEnable(DRV_USBFS_PERIPHERAL_ID);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStart)() */

// *****************************************************************************
/* Function:
    void _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop) client interface function for
    USB device.  Function checks if driver has been opened and on success
    disables the USB device to drive resume signalling.  RESUME is disabled by
    programming USB Control register (U1CON<2>).  This function should only be
    called after 10ms of driving RESUME signal by USB device.

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop)()
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return;
    }
    /* Disable Resume signalling */
    PLIB_USB_ResumeSignalingDisable(DRV_USBFS_PERIPHERAL_ID);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_RemoteWakeupStop) */

// *****************************************************************************
/* Function:
      void _DRV_USBFS_MAKE_NAME(DEVICE_Attach)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_Attach) client interface
    function.

  Description:
    This is the static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_Attach)
    client interface function for USB device.  Function checks if USB driver has
    been opened and on success writes to USB CONTROL register <U1CON> to enable
    USB device mode of operation.  It also enbales the necessaryy USB intrerupts
    except RESUMEIF for USB device operation.  After this function gets called
    the D+/D- pull up resistors are enabled and then only device enumeration can
    start.

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_MAKE_NAME(DEVICE_Attach)()
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return;
    }
    /* Update the driver flag indicating attach */
    gDrvUSBGroup.gDrvUSBObj.isAttached = true;

    /* Configure the peripheral for device mode operation. This function
     * also enables the D+ pull up resistor.  */
    PLIB_USB_OperatingModeSelect(DRV_USBFS_PERIPHERAL_ID, USB_OPMODE_DEVICE);

    /* Enables all interrupts except RESUME. RESUMEIF will be enabled only on
     * getting SUSPEND */
    PLIB_USB_AllInterruptEnable(DRV_USBFS_PERIPHERAL_ID,
            (USB_INT_ALL & ~USB_INT_RESUME), USB_ERR_INT_ALL,
            ((~USB_OTG_INT_ALL) | USB_OTG_INT_SESSION_VALID |
            USB_OTG_INT_ACTIVITY_DETECT));

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_Attach)()*/

// *****************************************************************************
/* Function:
      void _DRV_USBFS_MAKE_NAME(DEVICE_Detach)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_Detach) client interface
    function.

  Description:
    This is the static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_Detach)
    client interface function for USB device.  Function checks if USB driver has
    been opened and on success clears all the interrupts and selects no mode of
    operation. This will disable D+/D- pull-up registers and even if the USB
    device is connected to USB physical bus, the device will not be enumerated
    after the USB device detach is done.

  Remarks:
    See drv_usb.h for usage information.
*/

void _DRV_USBFS_MAKE_NAME(DEVICE_Detach)()
{
    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return;
    }
    /* Update the driver flag indicating detach */
    gDrvUSBGroup.gDrvUSBObj.isAttached = false;

    /* Clear all the interrupts */
    PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_ALL);

    /* Reset the operating mode */
    PLIB_USB_OperatingModeSelect(DRV_USBFS_PERIPHERAL_ID, USB_OPMODE_NONE);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_Detach)() */

// *****************************************************************************
/* Function:
    void _DRV_USBFS_DEVICE_EndpointObjectEnable
    (
        DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject,
        uint16_t endpointSize,
        USB_TRANSFER_TYPE endpointType,
        USB_BUFFER_DATA01 dataToggle
     )

  Summary:
    static implementation of _DRV_USBFS_DEVICE_EndpointObjectEnable function.

  Description:
    This is the static implementation of _DRV_USBFS_DEVICE_EndpointObjectEnable
    function for USB device.  Function populates the endpoint object data
    structure and sets it to enabled state.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USBFS_DEVICE_EndpointObjectEnable
(
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject,
    uint16_t endpointSize,
    USB_TRANSFER_TYPE endpointType,
    USB_BUFFER_DATA01 dataToggle
)
{
    /* This is a helper function */
    endpointObject->nextDataToggle  = USB_BUFFER_DATA0;
    endpointObject->irpQueue        = NULL;
    endpointObject->maxPacketSize   = endpointSize;
    endpointObject->nextPingPong    = USB_BUFFER_EVEN;
    endpointObject->endpointType    = endpointType;
    endpointObject->endpointState  |= DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
    
}/* end of _DRV_USBFS_DEVICE_EndpointObjectEnable() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable)
    (
        USB_ENDPOINT endpointAndDirection,
        USB_TRANSFER_TYPE endpointType,
        uint16_t endpointSize
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable) client interface function for USB
    device.  Function enables the specified endpoint.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable)
(
    USB_ENDPOINT endpointAndDirection, 
    USB_TRANSFER_TYPE endpointType,
    uint16_t endpointSize
)
{
    /* This function can be called from
     * from the USB ISR. Because an endpoint
     * can be owned by one client only, we
     * dont need mutex protection in this 
     * function */

    /* Start of local variables */
    int                           direction;
    int                           iEntry;
    bool                          handshake;
    uint8_t                       endpoint;
    DRV_USBFS_OBJ                 * hDriver;
    DRV_USBFS_BDT_ENTRY           * pBDT;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variables */


    /* Enable the endpoint */

    endpoint     = endpointAndDirection & 0xF;
    direction    = ((endpointAndDirection & 0x80) != 0);
    
    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }

    hDriver      = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);

    /* The BDT table has four entries per endpoint
     * The following statement points pBDT to the
     * first endpoint specific entry */

    pBDT         = hDriver->pBDT + (endpoint * 4);
    
    /* Get the pointer to the endpoint object */
    
    endpointObject = (hDriver->endpointTable + (2 * endpoint) + 0);

    if(endpointType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* For a control endpoint enable both
         * directions. Clear up the BDT entries. */

        uint32_t * pBDT32bit = (uint32_t *)pBDT;

        for(iEntry = 0; iEntry < 7; iEntry ++)
        {
            /* A full duplex endpoint has 4
             * entries, 2 for each direction */

            *(pBDT32bit + iEntry) = 0;
        }
        
        /* The following function enables both directions */
           
        PLIB_USB_EPnAttributesSet(DRV_USBFS_PERIPHERAL_ID, endpoint, 0, true, 0 );
             
        /* Update the endpoint database for both directions */
        
        _DRV_USBFS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType, USB_BUFFER_DATA0);

        /* This is when data moves from device to host. For
         * control end points the Data toggle always starts
         * with DATA1.
         */
   
        endpointObject ++;
        _DRV_USBFS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType, USB_BUFFER_DATA1);

    }
    else
    {
        /* Clear up the even odd entries for this
         * endpoint direction in the BDT. Each entry
         * has 2 32 bit entries */

        pBDT += (2 * direction);

        /* Clear up the even entry */
        pBDT->word[0] = 0;
        pBDT->word[1] = 0;
        pBDT ++;

        /* Clear up the odd entry */
        pBDT->word[0] = 0;
        pBDT->word[1] = 0;
        
        handshake = (endpointType == USB_TRANSFER_TYPE_ISOCHRONOUS) ? false : true;
                        
        PLIB_USB_EPnAttributesSet(DRV_USBFS_PERIPHERAL_ID, endpoint, direction, false, handshake);

        /* Update the endpoint database */

        endpointObject += direction;
        _DRV_USBFS_DEVICE_EndpointObjectEnable(endpointObject, endpointSize, endpointType, USB_BUFFER_DATA0);
    }
    
    return(USB_ERROR_NONE);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointEnable)() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable) client interface function for USB
    device.  Function disables the specified endpoint.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* This routine disables the specified endpoint.
     * It does not check if there is any ongoing 
     * communication on the bus through the endpoint
     */

    /* Start of local variable */
    uint8_t endpoint;
    int     direction, iEntry;
    DRV_USBFS_OBJ * hDriver;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variable */

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }

    /* Get the pointer to associated endpoint object table */

    hDriver     = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);
    endpointObject = hDriver->endpointTable;

    /* If the endpointAndDirection is _DRV_USBFS_DEVICE_ENDPOINT_ALL
     * then this means that the DRV_USBFS_DEVICE_EndpointDisableAll()
     * function was called */

    if(endpointAndDirection == _DRV_USBFS_DEVICE_ENDPOINT_ALL)
    {
        
        for(iEntry = 0; iEntry < DRV_USBFS_ENDPOINTS_NUMBER; iEntry ++)
        {
            PLIB_USB_EPnAttributesClear(DRV_USBFS_PERIPHERAL_ID, iEntry);

            /* Update the endpoint database */

            endpointObject->endpointState  &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;
            endpointObject->endpointState  &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
            endpointObject ++;

        }
        return (USB_ERROR_NONE);
    }

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    /* Setup the endpointObj to point to the correct
     * endpoint object */

    endpointObject += ((2 * endpoint) + direction);

    if(endpointObject->endpointType == USB_TRANSFER_TYPE_CONTROL)
    {
        /* Disable a control endpoint and update the
         * endpoint database. */
        PLIB_USB_EPnAttributesClear(DRV_USBFS_PERIPHERAL_ID, endpoint);
        endpointObject->endpointState  &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
        endpointObject += 1;
        endpointObject->endpointState  &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
    }
    else
    {
        /* Disable a specific endpoint direction for non
         * control endpoints */
        PLIB_USB_EPnDirectionDisable(DRV_USBFS_PERIPHERAL_ID, endpoint, direction);
        endpoint += direction;
        endpointObject->endpointState  &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED;
    }
    return(USB_ERROR_NONE);
    
}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointDisable)() */

// *****************************************************************************
/* Function:
    bool _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)
    client interface function.

  Description:
    This is the static implementation of DRV_USBFS_DEVICE_EndpointIsEnabled
    client interface function for USB device.  Function returns the state of
    specified endpoint(true\false) signifying whether the endpoint is enabled or
    not.

  Remarks:
    See drv_usb.h for usage information.
*/

bool _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variables */
    DRV_USBFS_OBJ * hDriver;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObj;
    /* end of local variables */

    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return false;
    }

    hDriver = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);
    endpointObj = hDriver->endpointTable + (2 * endpoint) + direction;
 
    if((endpointObj->endpointState & DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsEnabled)() */

// *****************************************************************************
/* Function:
    bool _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)
    client interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled) client interface function for
    USB device.  Function returns the state of specified endpoint(true\false)
    signifying whether the endpoint is STALLed or not.

  Remarks:
    See drv_usb.h for usage information.
*/

bool _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variables */
    DRV_USBFS_OBJ * hDriver;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObj;
    uint8_t endpoint = endpointAndDirection & 0xF;
    int direction = ((endpointAndDirection & 0x80) != 0);
    /* end of local variable */

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return true;
    }

    hDriver = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);
    endpointObj = hDriver->endpointTable + (2 * endpoint) + direction;
    
    if((endpointObj->endpointState & DRV_USBFS_DEVICE_ENDPOINT_STATE_STALLED) != 0)
    {
        return true;
    }
    else
    {
        return false;
    }

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointIsStalled)() */

// *****************************************************************************
/* Function:
    void _DRV_USBFS_DEVICE_EndpointBDTEntryArm
    (
        DRV_USBFS_BDT_ENTRY * pBDT,
        DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObj,
        USB_DEVICE_IRP_LOCAL * irp,
        int direction
    )

  Summary:
    static implementation of _DRV_USBFS_DEVICE_EndpointBDTEntryArm client
    interface function.

  Description:
    This is the static implementation of _DRV_USBFS_DEVICE_EndpointBDTEntryArm
    client interface function for USB device.  Function arms a BDT entry slot
    for the endpoint object

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USBFS_DEVICE_EndpointBDTEntryArm
(
    DRV_USBFS_BDT_ENTRY * pBDT, 
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObj, 
    USB_DEVICE_IRP_LOCAL * irp,
    int direction
)
{
    /* pBDT is the pointer to the ping pong BDT entries
     * for the endpoint and direction  In this driver we
     * dont check for the data toggle while receiving data from
     * the host. The assumption here is that the host is correct */

    /* If the endpoint is stalled, the stall will be cleared */

    /* Start of local variable */
    uint16_t size;
    DRV_USBFS_BDT_ENTRY * currentBDTEntry;
    /* end of local variable */

    currentBDTEntry = pBDT + endpointObj->nextPingPong;

    /* Calculate the size of the transaction */
    if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
    {
        /* If data is moving from device to host
         * then enable data toggle syncronization */

        currentBDTEntry->byte[0] = 0x08;

        /* Adjust buffer address for the number of
         * bytes sent */
        currentBDTEntry->word[1] = (uint32_t) (KVA_TO_PA ((uint8_t *)irp->data + irp->size - irp->nPendingBytes));

        if(irp->nPendingBytes == 0)  
        {
            /* This applies when we need to send a ZLP */
            size = 0;
        }
        else
        {
            size = (irp->nPendingBytes > endpointObj->maxPacketSize)
                ? endpointObj->maxPacketSize: irp->nPendingBytes;
        }

        /* Update the pending bytes only if the 
         * data direction is from device to host. The
         * pending bytes for the other direction is 
         * updated in the ISR */

        irp->nPendingBytes -= size;
    }
    else
    {
        /* Data is moving from host to device */
        currentBDTEntry->byte[0] = 0x0;

        /* Adjust the buffer address for the number of bytes
         * received so far */

        currentBDTEntry->word[1] = (uint32_t) (KVA_TO_PA ((uint8_t *)irp->data + irp->nPendingBytes));
        
        size = (irp->size - irp->nPendingBytes > 
                    endpointObj->maxPacketSize) ? endpointObj->maxPacketSize :
                    irp->size - irp->nPendingBytes;

    }

    /* We set up the data toggle. This will be active
     * only if DTS is active. Clear the DATA0/1 and 
     * then set it according to the next data toggle
     * to be used.*/

    currentBDTEntry->byte[0] &= 0xBF;
    currentBDTEntry->byte[0] |= (endpointObj->nextDataToggle << 6);
    
    /* Set the size */
    currentBDTEntry->shortWord[1] = size;
    
    /* Set the UOWN bit */

    currentBDTEntry->byte[0] |= 0x80;

    endpointObj->nextPingPong ^= 0x1;
    endpointObj->nextDataToggle ^= 0x1;

}/* end of _DRV_USBFS_DEVICE_EndpointBDTEntryArm() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit)
    (
        USB_ENDPOINT endpointAndDirection,
        USB_DEVICE_IRP * inputIRP
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit) client
    interface function.

  Description:
    This is the static implementation of DRV_USBFS_DEVICE_IRPSubmit client
    interface function for USB device.  Function checks the validity of the
    input arguments and on success adds the IRP to endpoint object queue linked
    list.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit)
(
    USB_ENDPOINT endpointAndDirection, 
    USB_DEVICE_IRP * inputIRP
)
{
    /* Start of local variables */
    uint8_t                       endpoint;
    bool                          interruptWasEnabled = false;
    int                           direction;
    int                           remainder;
    DRV_USBFS_OBJ                 * hDriver;
    USB_DEVICE_IRP_LOCAL        * irp = (USB_DEVICE_IRP_LOCAL *)inputIRP;
    DRV_USBFS_BDT_ENTRY           * pBDT;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObj;
    /* end of local variables */

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }

    if(irp->status > USB_DEVICE_IRP_STATUS_SETUP)
    {
        /* This means that the IRP is in use */
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Device IRP is already in use");
        return(USB_ERROR_DEVICE_IRP_IN_USE);
    }
   
    /* Check for a valid endpoint */
    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Endpoint is not provisioned for");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    /* Get the driver object, the module ID and the endpoint and direction
     * specific BDT entry and the endpoint object. */

    hDriver     = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);
    pBDT        = hDriver->pBDT + (endpoint * 4) + (2 * direction);
    endpointObj = hDriver->endpointTable + (2 * endpoint) + direction;

    if((endpointObj->endpointState & DRV_USBFS_DEVICE_ENDPOINT_STATE_ENABLED) == 0)
    {
        /* This means the endpoint is disabled */        
        return(USB_ERROR_ENDPOINT_NOT_CONFIGURED);        
    }

    /* Check the size of the IRP. If the endpoint receives data from the host,
     * then IRP size must be multiple of maxPacketSize. If the send ZLP flag is
     * set, then size must be multiple of endpoint size. */

    remainder = irp->size % endpointObj->maxPacketSize;

    if(remainder == 0)
    {
        /* The IRP size is either 0 or a exact multiple of maxPacketSize */

        if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
        {
            if(((irp->flags & USB_DEVICE_IRP_FLAG_DATA_COMPLETE) 
                    == USB_DEVICE_IRP_FLAG_DATA_COMPLETE) &&
                (irp->size != 0))
            {
                /* This means a ZLP should be sent after the data is sent. We
                 * will OR this flag as this flag is temporary and must co-exist
                 * by the driver client defined flag. */

                irp->flags |= USB_DEVICE_IRP_FLAG_SEND_ZLP;
            }
        }
    }

    /* Now we check if the interrupt context is active. If so the we dont need
     * to get a mutex or disable interrupts.  If this were being done in non
     * interrupt context, we, then we would disable the interrupt. In which case
     * we would get the mutex and then disable the interrupt */

    if(!(hDriver->inInterruptContext))
    {
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }

        interruptWasEnabled =
                _DRV_USBFS_InterruptSourceDisable(DRV_USBFS_INTERRUPT_SOURCE);
    }

    irp->next = NULL;

    /* If the data is moving from device to host then pending bytes is data
     * remaining to be sent to the host. If the data is moving from host to
     * device, nPendingBytes tracks the amount of data received so far */

    if(USB_DATA_DIRECTION_DEVICE_TO_HOST == direction)
    {
        irp->nPendingBytes = irp->size;
    }
    else
    {
        irp->nPendingBytes = 0;
    }

    /* Mark the IRP status as pending */
    irp->status = USB_DEVICE_IRP_STATUS_PENDING;

    /* Get the last object in the endpoint object IRP Queue */
    if(endpointObj->irpQueue == NULL)
    {
        /* Queue is empty */
        endpointObj->irpQueue = irp;
        irp->previous = NULL;

        /* Because this is the first IRP in the queue then we we must arm the
         * endpoint entry in the BDT. */

        irp->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
        _DRV_USBFS_DEVICE_EndpointBDTEntryArm(pBDT,endpointObj, irp, direction);
    }
    else
    {
        /* This means we should surf the linked list to get to the last entry .
         * */
        USB_DEVICE_IRP_LOCAL * iterator;
        iterator = endpointObj->irpQueue;
        while(iterator->next != NULL)
        {
            iterator = iterator->next;
        }
        iterator->next = irp;
        irp->previous = iterator;
        irp->status = USB_DEVICE_IRP_STATUS_PENDING;
    }

    if(!(hDriver->inInterruptContext))
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt only if it was enabled */
            _DRV_USBFS_InterruptSourceEnable(DRV_USBFS_INTERRUPT_SOURCE);
        }
        /* Unlock the mutex */
        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
	    SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
    return(USB_ERROR_NONE);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_IRPSubmit)() */

// *****************************************************************************
/* Function:
    void _DRV_USBFS_DEVICE_IRPQueueFlush
    (
        DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject
    )

  Summary:
    static implementation of _DRV_USBFS_DEVICE_IRPQueueFlush function.

  Description:
    This is the static implementation of _DRV_USBFS_DEVICE_IRPQueueFlush function
    for USB device.  Function scans for all the IRPs on the endpoint queue and
    cancels them all.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_USBFS_DEVICE_IRPQueueFlush(DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject)
{
    /* Start of local variable */
    USB_DEVICE_IRP_LOCAL * iterator = NULL;
    /* End of local variable */
    
    if(endpointObject != NULL)
    {
        /* Check if any IRPs are assigned on this endpoint and
         * abort them */
        if(endpointObject->irpQueue != NULL)
        {
            /* Scan for all the IRPs on this endpoint
             * Cancel the IRP and deallocate driver IRP
             * objects */
            iterator = endpointObject->irpQueue;
            while(iterator != NULL)
            {
                iterator->status = USB_DEVICE_IRP_STATUS_ABORTED;
                if(iterator->callback != NULL)
                {
                    iterator->callback((USB_DEVICE_IRP *)iterator);
                }
                iterator = iterator->next;
            }/* end of while(IRP != NULL) */
        }
        /* Set the head pointer to NULL */
        endpointObject->irpQueue = NULL;
    }
}/* end of _DRV_USBFS_DEVICE_IRPQueueFlush() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll) client interface function for USB
    device.  Function checks if USB driver has been opened and on success
    cancels all the IRPs on the specific endpoint
    object queue.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variable */
    uint8_t                       endpoint;
    bool                          interruptWasEnabled = false;
    int                           direction;
    DRV_USBFS_OBJ                 * hDriver;
    DRV_USBFS_BDT_ENTRY           * pBDT;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variable */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    /* Check if the endpoint number is within limit */
    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }

    hDriver      = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint) + direction;

    /* Get the BDT entry for this endpoint */
    pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction);

    if(!(hDriver->inInterruptContext))
    {
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }

        /* Disable the interrupt */
         interruptWasEnabled = _DRV_USBFS_InterruptSourceDisable(DRV_USBFS_INTERRUPT_SOURCE);
    }

    /* Get the odd and even endpoint BDT back */
    pBDT->byte[0] = 0x0;
    (pBDT + 1)->byte[0] = 0x0;

    /* Flush the endpoint */
    _DRV_USBFS_DEVICE_IRPQueueFlush(endpointObject);

    if(!(hDriver->inInterruptContext))
    {
        if(interruptWasEnabled)
        {
            _DRV_USBFS_InterruptSourceEnable(DRV_USBFS_INTERRUPT_SOURCE);
        }

        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }

    return(USB_ERROR_NONE);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_IRPCancelAll)() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall) client interface function for USB
    device.  Function sets the STALL state of the specified endpoint.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* start of local variables */
    int     direction;
    bool    interruptWasEnabled = false;
    uint8_t endpoint;
    DRV_USBFS_OBJ     * hDriver;
    DRV_USBFS_BDT_ENTRY * pBDT;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* end of local variables */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }

    hDriver      = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);

    if(!(hDriver->inInterruptContext))
    {
        //OSAL: Get Mutex
        if(OSAL_MUTEX_Lock(&hDriver->mutexID, OSAL_WAIT_FOREVER) !=
                OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "mutex lock failed");
            return USB_ERROR_OSAL_FUNCTION;
        }

        /* Disable the interrupt */
        interruptWasEnabled =
                _DRV_USBFS_InterruptSourceDisable(DRV_USBFS_INTERRUPT_SOURCE);
    }
   
    if(endpoint == 0)
    {
        /* For zero endpoint we stall both directions */

        endpointObject = hDriver->endpointTable;
        pBDT = hDriver->pBDT + (endpointObject->nextPingPong);
        
        /* This is the RX direction for EP0. Get the
         * BDT back, stall it, flush all IRPs and then
         * set the endpoint state */
        
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USBFS_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USBFS_DEVICE_ENDPOINT_STATE_STALLED;

        /* Now do the same for the TX direction */

        endpointObject = hDriver->endpointTable + 1;
        pBDT = hDriver->pBDT + 2 + (endpointObject->nextPingPong);
        
        /* This is the TX direction for EP0. Get the
         * BDT back, stall it, flush all IRPs and then
         * set the endpoint state */
        
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USBFS_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USBFS_DEVICE_ENDPOINT_STATE_STALLED;

    } 
    else
    {
        /* For non zero endpoints we stall the specified direction */

        /* Get the endpoint object */
        endpointObject = hDriver->endpointTable + (2 * endpoint)
            + direction;

        /* Get the BDT entry for this endpoint */
        pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction)
            + (endpointObject->nextPingPong);
    
        /* Get the endpoint BDT back. Stall the entry.
         * Flush the endpoint and set the object state. */
        pBDT->byte[0] = 0x0;
        pBDT->byte[0] |= 0x84;
        _DRV_USBFS_DEVICE_IRPQueueFlush(endpointObject);
        endpointObject->endpointState |= DRV_USBFS_DEVICE_ENDPOINT_STATE_STALLED;

    }
    
    if(!(hDriver->inInterruptContext))
    {
        if(interruptWasEnabled)
        {
            /* Enable the interrupt */
            _DRV_USBFS_InterruptSourceEnable(DRV_USBFS_INTERRUPT_SOURCE);
        }

        if(OSAL_MUTEX_Unlock(&hDriver->mutexID) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Mutex unlock failed");
        }
    }
    
    return(USB_ERROR_NONE);

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStall)() */

// *****************************************************************************
/* Function:
    USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)
    (
        USB_ENDPOINT endpointAndDirection
    )

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)
    client interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear) client interface function for
    USB device.  Function clears the STALL state of the specified endpoint and
    resets the data toggle value.

  Remarks:
    See drv_usb.h for usage information.
*/

USB_ERROR _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)
(
    USB_ENDPOINT endpointAndDirection
)
{
    /* Start of local variables */
    int                           direction;
    uint8_t                       endpoint;
    DRV_USBFS_OBJ                 * hDriver;
    DRV_USBFS_BDT_ENTRY           * pBDT;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * endpointObject;
    /* End of local variables */

    endpoint = endpointAndDirection & 0xF;
    direction = ((endpointAndDirection & 0x80) != 0);

    if(endpoint >= DRV_USBFS_ENDPOINTS_NUMBER)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"Unsupported endpoint");
        return USB_ERROR_DEVICE_ENDPOINT_INVALID;
    }

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
        return USB_ERROR_CLIENT_NOT_READY;
    }
	
    hDriver      = (DRV_USBFS_OBJ *)&(gDrvUSBGroup.gDrvUSBObj);

    /* Get the endpoint object */
    endpointObject = hDriver->endpointTable + (2 * endpoint) + direction;
    
    /* Get the BDT entry for this endpoint */
    pBDT = hDriver->pBDT + (4 * endpoint) + (2 * direction) + (endpointObject->nextPingPong);

    /* Clear the stall and data toggle on the 
     * endpoint */
    pBDT->byte[0] = 0x0;
    if((endpoint == 0) && (direction == USB_DATA_DIRECTION_DEVICE_TO_HOST))
    {
        /* All endpoint 0 transmit must start with data toggle DATA1 */
        endpointObject->nextDataToggle = USB_BUFFER_DATA1;
    }
    else
    {
       endpointObject->nextDataToggle = USB_BUFFER_DATA0;
    }

    endpointObject->endpointState &= ~DRV_USBFS_DEVICE_ENDPOINT_STATE_STALLED;

    return(USB_ERROR_NONE);
    
}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_EndpointStallClear)() */

// *****************************************************************************
/* Function:
      uint16_t _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet)()

  Summary:
    static implementation of _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet) client
    interface function.

  Description:
    This is the static implementation of
    _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet) client interface function for USB
    device.  Function checks the validity of the input arguments and on success
    returns the Frame count value. Frame count value is obtained from USB Frame
    registers (High and Low).

  Remarks:
    See drv_usb.h for usage information.
*/

uint16_t _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet)()
{
    /* Start of local variable */
    uint16_t sofNumber = 0;
    /* end of local variable */

    if(!(gDrvUSBGroup.gDrvUSBObj.isOpened))
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Driver has not been opened");
    }
    else
    {
        /* Get the Frame count */
        sofNumber = PLIB_USB_FrameNumberGet(DRV_USBFS_PERIPHERAL_ID);
    }

    return sofNumber;

}/* end of _DRV_USBFS_MAKE_NAME(DEVICE_SOFNumberGet)() */

// *****************************************************************************
/* Function:
      void _DRV_USBFS_DEVICE_Tasks_ISR(DRV_USBFS_OBJ * hDriver)

  Summary:
    static implementation of _DRV_USBFS_DEVICE_Tasks_ISR ISR handler function.

  Description:
    This is the static implementation of _DRV_USBFS_DEVICE_Tasks_ISR ISR handler
    function for USB device.  Function will get called automatically due to USB
    interrupts in interrupt mode.  In polling mode this function will be
    routinely called from USB driver DRV_USBFS_Tasks() function.  This function
    performs necessary action based on the interrupt and clears the interrupt
    after that. The USB device layer callback is called with the interrupt event
    details, if callback function is registered.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/


void _DRV_USBFS_DEVICE_Tasks_ISR(DRV_USBFS_OBJ * hDriver)
{
    /* Start of local variables */

    bool                          queueWasEmpty = false;
    bool                          processNextIRP;
    uint8_t                       lastEndpoint = 0;
    int                           iEntry;
    unsigned int                  errorType;
    DRV_USBFS_EVENT                 eventType = 0;
    USB_DEVICE_IRP_LOCAL        * irp;
    DRV_USBFS_BDT_ENTRY           * currentBDTEntry;
    DRV_USBFS_BDT_ENTRY           * lastBDTEntry;
    DRV_USBFS_BDT_ENTRY           * ep0TransmitBDTEntry;
    USB_PING_PONG_STATE           lastPingPong = 0;
    USB_BUFFER_DIRECTION          lastDirection = 0;
    DRV_USBFS_DEVICE_ENDPOINT_OBJ * lastEndpointObj;
    /* end of local variables */

    /* Check is there was a change in VBUS voltage level */
    if(PLIB_USB_OTG_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_SESSION_VALID)
            && PLIB_USB_OTG_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_SESSION_VALID))
    {

        /* This means there was a change in the VBUS voltage detected level. We
         * can find out if the VBUS is valid */

        if(PLIB_USB_OTG_SessionValid(DRV_USBFS_PERIPHERAL_ID))
        {
            /* This means we detected a valid VBUS voltage. */
            hDriver->vbusIsValid = true;
            eventType = DRV_USBFS_EVENT_DEVICE_SESSION_VALID; 
        }
        else
        {
            /* This means the VBUS is not valid anymore */
            hDriver->vbusIsValid = false;
            eventType = DRV_USBFS_EVENT_DEVICE_SESSION_INVALID;
        }

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, eventType,  (void *)NULL);
        }

        /* Clear the interrupt */
        PLIB_USB_OTG_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_SESSION_VALID);
    }

    /* If there is no VBUS or if the device pull up is not enabled yet, then
     * clear all the interrupt flags except VBUS session interrupt. Clear all
     * error, OTG and general interrupts. We cannot clear the session valid
     * interrupt because this may have change already by the time we have
     * reached here. After clearing the interrup we exit as we have nothing more
     * to be done in the ISR. */

    if((!(hDriver->vbusIsValid)) || (!(hDriver->isAttached)))
    {
        PLIB_USB_ErrorInterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_ERR_INT_ALL);
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_ALL);
        PLIB_USB_OTG_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, (USB_OTG_INT_ALL & (~ USB_OTG_INT_SESSION_VALID))); 
        return;
    }

    /* Check if there was activity on the bus. This interrupt is enabled before
     * entering suspend mode. This interrupt must be enabled to wake up the 
     * microcontroller from sleep due to USB activity. The driver wraps this
     * interrupt as resume detection. */

    if(PLIB_USB_OTG_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_ACTIVITY_DETECT)
            && PLIB_USB_OTG_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_ACTIVITY_DETECT ))
    {
        /* Clear the interrupt */
        PLIB_USB_OTG_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_ACTIVITY_DETECT);

        /* Disable the interrupt */
        PLIB_USB_OTG_InterruptDisable(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_ACTIVITY_DETECT);

        /* Device is not suspended any more */
        hDriver->isSuspended = false;

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_RESUME_DETECT,  (void *)NULL);
        }

    }

    /* The RESUME event can only occur after the device has been suspended. */
    if (PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME)
            && PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME))
    {
        /* Device is not suspended any more */
        hDriver->isSuspended = false;
        
        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_RESUME_DETECT,  (void *)NULL);
        }

        /* Clear the interrupt flag and disable it. This event will be enabled
         * the next time we enter suspend mode. */
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME);
        PLIB_USB_InterruptDisable(DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME);
    }

    /* Check if RESET signalling was received */
    if ( PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_DEVICE_RESET)
            && PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_DEVICE_RESET))
    {
        /* Device is not suspended any more */
        hDriver->isSuspended = false;

        /* The host may have suspended the device before reset.
         * Clear the suspend flag and the resume flag */
        PLIB_USB_InterruptFlagClear( DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME );
        PLIB_USB_InterruptFlagClear( DRV_USBFS_PERIPHERAL_ID, USB_INT_IDLE_DETECT );

        /* Make sure that all BDs are returned
         * back to the application */

        for(iEntry = 0; iEntry < DRV_USBFS_ENDPOINTS_NUMBER; iEntry++)
        {
            currentBDTEntry = hDriver->pBDT + (4 * iEntry);
            (currentBDTEntry + 0)->word[0] = 0x0;
            (currentBDTEntry + 1)->word[0] = 0x0;
            (currentBDTEntry + 2)->word[0] = 0x0;
            (currentBDTEntry + 3)->word[0] = 0x0;
        }

        /* Reset all ping pong buffers to even */
        PLIB_USB_PingPongReset(DRV_USBFS_PERIPHERAL_ID);

        /* Reset address to default value (0) */
        PLIB_USB_DeviceAddressSet( DRV_USBFS_PERIPHERAL_ID, 0 );

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_RESET_DETECT,  (void *)NULL);
        }

        /* Clear the interrupt flag */
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_DEVICE_RESET);

    }

    /* Check if the host has suspended the bus. */
    if (PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_IDLE_DETECT)
            && PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_IDLE_DETECT))
    {
        /* The bus is IDLE and is suspended. Send the event to the client. */
        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_IDLE_DETECT,  (void *)NULL);
        }

        /* Clear the interrupt flag and disable it. This event will be enabled
         * the next time we enter suspend mode. */
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_IDLE_DETECT);

        /* Enable the actvity interrupt */
        PLIB_USB_InterruptEnable(DRV_USBFS_PERIPHERAL_ID, USB_OTG_INT_ACTIVITY_DETECT);

        /* Enable the resume interrupt */
        PLIB_USB_InterruptEnable(DRV_USBFS_PERIPHERAL_ID, USB_INT_RESUME);

        /* Set the suspended flag */
        hDriver->isSuspended = true;
    }

    /* Check if an SOF was received */

    if (PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID,USB_INT_SOF)
            && PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_SOF))
    {
        /* SOF received by Device or SOF threshold reached by Host
         * no event data to send. */

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_SOF_DETECT,  (void *)NULL);
        }

        /* Clear the interrupt flag */
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_SOF);
    }

    /* The following event occurs when and endpoint has sent stall to the host.
     * If the stall was sent from endpoint 0, the device layer would know about
     * it. If it was sent from an non-zero endpoint, the host would send a clear
     * feature control request. This event therefore does not send any event
     * data to the client. */
    if (PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_STALL)
            && PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_STALL))
    {
        unsigned int iEndpoint;
        for ( iEndpoint = 0; iEndpoint < DRV_USBFS_ENDPOINTS_NUMBER; iEndpoint++ )
        {
            if ( PLIB_USB_EPnIsStalled(DRV_USBFS_PERIPHERAL_ID,iEndpoint) )
            {
                PLIB_USB_EPnStallClear(DRV_USBFS_PERIPHERAL_ID,iEndpoint);
            }
        }

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_STALL,  (void *)NULL);
        }

        /* Clear the interrupt flag */
        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID, USB_INT_STALL);

    }

    /* Check if an error has occurred */
    if ( PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_ERROR ) &&
            PLIB_USB_InterruptIsEnabled(DRV_USBFS_PERIPHERAL_ID, USB_INT_ERROR))
    {

        /* Get the error type to send to the client */
        errorType = PLIB_USB_ErrorInterruptFlagAllGet(DRV_USBFS_PERIPHERAL_ID);

        /* Clear the base error flags and the interrupt flag */
        PLIB_USB_ErrorInterruptFlagClear( DRV_USBFS_PERIPHERAL_ID, errorType );
        PLIB_USB_InterruptFlagClear( DRV_USBFS_PERIPHERAL_ID, USB_INT_ERROR );

        if(hDriver->pEventCallBack != NULL)
        {
            /* Send this event to the client */
            hDriver->pEventCallBack(hDriver->hClientArg, DRV_USBFS_EVENT_ERROR,  (void *)&errorType);
        }
    }

    /* This while loop will empty the token received FIFO */
    while(PLIB_USB_InterruptFlagGet(DRV_USBFS_PERIPHERAL_ID, USB_INT_TOKEN_DONE))
    {
        /* Get the details of the last transaction */

        PLIB_USB_LastTransactionDetailsGet(DRV_USBFS_PERIPHERAL_ID, &lastDirection, 
                &lastPingPong, &lastEndpoint); 

        /* Get the associated endpoint object */
        lastEndpointObj = hDriver->endpointTable + (lastEndpoint * 2) 
            + lastDirection;

        /* Get the first IRP in the queue */
        irp = lastEndpointObj->irpQueue; 

        /* Get the BDT entry for this direction. currentBDTEntry points to the
         * ping pong set. lastBDTEntry points to the specific ping or pong
         * entry. */

        currentBDTEntry = hDriver->pBDT + (4 * lastEndpoint) + 
            (2 * lastDirection); 

        lastBDTEntry = currentBDTEntry + lastPingPong;

        /* This flag lets us know if the current IRP is done and that the next
         * IRP should be processed */

        processNextIRP = false;
        switch(lastBDTEntry->byte[0] & 0x3C)
        {
            case 0x34 :

                /* This means a setup packet has been received */

                irp->status = USB_DEVICE_IRP_STATUS_SETUP;
                irp->size   = lastBDTEntry->word[1];

                /* currentBDTEntry at this point will point to enpoint 0 BDT
                 * entry. We should get the transmit BDT entries and clear the
                 * stall conditions */

                ep0TransmitBDTEntry = currentBDTEntry + 2;

                /* ep0TranmitBDTEntry at this point should point to the EP0
                 * transmit even BDT entry. Clearing byte 0 will clear clear the
                 * stall */
                ep0TransmitBDTEntry->byte[0] = 0;

                /* Now get the transmit BDT odd entry and do the same */

                ep0TransmitBDTEntry ++;
                ep0TransmitBDTEntry->byte[0] = 0;

                /* Reset the data toggle on the TX endpoint to DATA1 because we
                 * received a setup packet. Any packet that the device transmit
                 * on this endpoint must start with DATA1 toggle. */

                (lastEndpointObj + USB_DATA_DIRECTION_DEVICE_TO_HOST)->nextDataToggle 
                    = USB_BUFFER_DATA1;

                PLIB_USB_PacketTransferEnable(DRV_USBFS_PERIPHERAL_ID);


                /* We should get the next IRP in the queue . */

                processNextIRP = true;
                break;

            case 0x4:
                /* We received an OUT token. Check if the size is less than
                 * maxPacketSize. This means the end of the transfer. If the
                 * pending size is 0 then again we end the transfer */


                irp->nPendingBytes 
                    += lastBDTEntry->shortWord[1];

                if((lastBDTEntry->shortWord[1] < lastEndpointObj->maxPacketSize)
                        || (irp->nPendingBytes >= irp->size))
                {
                    /* We end the transfer because we either got the amount of
                     * data that we were expecting or we got the a short
                     * packet*/

                    /* If we got less data than we were expecting, then set the
                     * IRP status to short else say it is completed */
                    if(irp->nPendingBytes >= irp->size)
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                    }
                    else
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED_SHORT;
                    }

                    /* Update the irp size with received data */
                    irp->size = irp->nPendingBytes;

                    processNextIRP = true;

                }
                else
                {
                    /* We must continue this transfer */
                    _DRV_USBFS_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                            lastEndpointObj, irp, lastDirection);
                }
                break;

            case 0x24:
                /* This means that a IN token was received from
                 * the host */

                if(irp->nPendingBytes == 0)
                {
                    if((irp->flags & USB_DEVICE_IRP_FLAG_SEND_ZLP) != 0)
                    {
                        /* This means a ZLP must be sent */
                        irp->flags &= ~USB_DEVICE_IRP_FLAG_SEND_ZLP;
                        _DRV_USBFS_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                                lastEndpointObj, irp, lastDirection);
                    }
                    else
                    {
                        irp->status = USB_DEVICE_IRP_STATUS_COMPLETED;
                        processNextIRP = true;
                    }
                }
                else
                {
                    /* We must continue this transfer */
                    _DRV_USBFS_DEVICE_EndpointBDTEntryArm(currentBDTEntry, 
                            lastEndpointObj, irp, lastDirection);
                }

                break;
            default:
                PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID,USB_INT_TOKEN_DONE);
                continue;
                SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Unknown TOKEN received from host");
                break;
        }

        /* Reset the BDT status */
        lastBDTEntry->byte[0] = 0;

        if(processNextIRP)
        {
            /* Check the queue and get the next IRP */

            lastEndpointObj->irpQueue = irp->next;

            /* Check if the queue is empty. This will then allow us to track if
             * a IRP was submitted after the  IRP callback. If so, then we
             * should not call the _DRV_USBFS_DEVICE_EndpointBDTEntryArm() */

            if(lastEndpointObj->irpQueue == NULL)
            {
                /* Queue was empty before the call back */
                queueWasEmpty = true;
            }

            /* Now do the IRP callback*/

            if(irp->callback != NULL)
            {
                /* Invoke the callback */
                irp->callback((USB_DEVICE_IRP *)irp);
            }

            if((lastEndpointObj->irpQueue != NULL) &&
                    (!(queueWasEmpty)))
            {
                /* This means we have something in the queue and this was not
                 * added in the IRP callback. We can arm the endpoint. */
                lastEndpointObj->irpQueue->status = USB_DEVICE_IRP_STATUS_IN_PROGRESS;
                _DRV_USBFS_DEVICE_EndpointBDTEntryArm( currentBDTEntry, lastEndpointObj,
                        lastEndpointObj->irpQueue, lastDirection);
            }

        }

        PLIB_USB_InterruptFlagClear(DRV_USBFS_PERIPHERAL_ID,USB_INT_TOKEN_DONE);
    }
}


