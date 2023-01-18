/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    xmodem.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "xmodem.h"
#include "usb/src/usb_device_cdc_local.h"

/* currently we can only access console data that is provided via usb console 0 */
extern USB_DEVICE_CDC_INSTANCE gUSBDeviceCDCInstance[USB_DEVICE_CDC_INSTANCES_NUMBER];

USB_DEVICE_CDC_EVENT_RESPONSE XMODEMUSBDeviceCDCEventHandler(USB_DEVICE_CDC_INDEX index,
                            USB_DEVICE_CDC_EVENT event, void* pData, uintptr_t userData);

static XMODEM_OBJECT_INSTANCE xmodemDeviceInstance[XMODEM_MAX_OBJ_INSTANCES];

static uint8_t __attribute__((coherent, aligned(16))) xmRxUSBBuffer[XMODEM_READ_BUFFER_SIZE] SYS_CONSOLE_BUFFER_DMA_READY;  /* incoming USB data store */
static uint8_t __attribute__((coherent, aligned(16))) xmTxUSBBuffer[XMODEM_READ_BUFFER_SIZE] SYS_CONSOLE_BUFFER_DMA_READY;  /* buffer for transmitting ack/nak */
static void _DRV_XMODEM_SendChar(SYS_MODULE_INDEX index, uint8_t c);

SYS_MODULE_OBJ DRV_XMODEM_Initialize( const SYS_MODULE_INDEX index, const SYS_MODULE_INIT * const init )
{
    /* Confirm valid arguments */
    if (index >= XMODEM_MAX_OBJ_INSTANCES)
        return SYS_MODULE_OBJ_INVALID;

    (void) init;

    if (xmodemDeviceInstance[index].xmodemStatus == SYS_STATUS_UNINITIALIZED)
    {
        xmodemDeviceInstance[index].defaultEventHandler = NULL;
        xmodemDeviceInstance[index].defaultUserData = (uintptr_t) NULL;
        xmodemDeviceInstance[index].rxLength = 0;
        xmodemDeviceInstance[index].xmHeadIndex = 0;
        xmodemDeviceInstance[index].xmTailIndex = 0;
        xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
        xmodemDeviceInstance[index].xmodemStatus = SYS_STATUS_READY;
        return index;
    }

    return SYS_MODULE_OBJ_INVALID;
}

/* utility function to read from the incoming circular xmodem buffer */
static int XMODEM_Read_Buffer(SYS_MODULE_INDEX index)
{
    uint8_t data;

    /* if the head isn't ahead of the tail, we don't have any characters */
    if (xmodemDeviceInstance[index].xmHeadIndex == xmodemDeviceInstance[index].xmTailIndex)
    {
        return -1;
    }
    else
    {
        data = xmodemDeviceInstance[index].xmBuffer[xmodemDeviceInstance[index].xmTailIndex];
        xmodemDeviceInstance[index].xmTailIndex = (uint32_t) (xmodemDeviceInstance[index].xmTailIndex + 1) % XMODEM_READ_BUFFER_SIZE;
        return data;
    }
}

/* utility function to write to the incoming circular buffer */
static void XMODEM_WriteBuffer(SYS_MODULE_INDEX index, uint8_t data)
{
    uint32_t ringIndex;

    /* write data to the current head */
    ringIndex = xmodemDeviceInstance[index].xmHeadIndex;
    xmodemDeviceInstance[index].xmBuffer[ringIndex] = data;
    /* advance to the next element */
    ringIndex = (ringIndex + 1) % XMODEM_READ_BUFFER_SIZE;
    /* advance if not overwriting the tail */
    if (ringIndex != xmodemDeviceInstance[index].xmTailIndex)
    {
        xmodemDeviceInstance[index].xmHeadIndex = ringIndex;
    }
}

/* XMODEM state machine */
XMODEM_RETURN DRV_XMODEM_Process(SYS_MODULE_OBJ object, uint32_t* count, uint8_t** pData)
{
    int xdata;
    uint8_t c, cmpl;
    uint32_t i, csum;
    XMODEM_RETURN rVal;  
    SYS_MODULE_INDEX index = (SYS_MODULE_INDEX) object;
    uint32_t readIndex;

    if (index >= SYS_CONSOLE_DEVICE_MAX_INSTANCES)
        return XMODEM_RETURN_ERROR;
    
    /* preset return values */
    *count = 0;
    *pData = NULL;
    rVal = XMODEM_RETURN_IDLE;

    /* manage basic USB read/write tasks */
    if (xmodemDeviceInstance[index].isReadComplete)
    {
        /* copy the received data into the ring buffer */
        readIndex = 0;

        /* add the data to the ring buffer */
        while (xmodemDeviceInstance[index].rxLength > 0)
        {
            XMODEM_WriteBuffer(index, xmRxUSBBuffer[readIndex]);
            xmodemDeviceInstance[index].rxLength--;
            readIndex++;
        }

        /* schedule a read */
        xmodemDeviceInstance[index].readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        xmodemDeviceInstance[index].isReadComplete = false;
        USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0, &xmodemDeviceInstance[index].readTransferHandle,
                xmRxUSBBuffer, XMODEM_READ_BUFFER_SIZE);
    }

    /* only proceed if the last transmit completed */
    if (xmodemDeviceInstance[index].isWriteComplete == false)
    {
        return rVal;
    }

    /* xmodem state machine */
    switch (xmodemDeviceInstance[index].xmState)
    {
        case XMODEM_STATE_IDLE:
            break;

        case XMODEM_STATE_INIT:
            /* start a timeout timer */
            xmodemDeviceInstance[index].xmTimer = SYS_TMR_DelayMS(1000);
            xmodemDeviceInstance[index].xmTimeoutCounter = 5;
            xmodemDeviceInstance[index].xmRepeatCounter = 6;
            xmodemDeviceInstance[index].xmSOHchar = XMODEM_NAK;    /* start with XMODEM NAK mode */
            xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_INVITE;
            break;

        case XMODEM_STATE_SEND_INVITE:
            /* send an invite */
            _DRV_XMODEM_SendChar(index, xmodemDeviceInstance[index].xmSOHchar);
            xmodemDeviceInstance[index].xmState = XMODEM_STATE_WAIT_SOH;
            break;

        case XMODEM_STATE_WAIT_SOH:
            /* read the incoming character */
            xdata = XMODEM_Read_Buffer(index);
            if (xdata >= 0)
            {
                xmodemDeviceInstance[index].xmTimeoutCounter = 5;  /* 5 second timeout */

                c = (uint8_t) xdata;
                switch (c)
                {
                    case XMODEM_SOH:
                        /* 128 byte packets */
                        xmodemDeviceInstance[index].xmState = XMODEM_STATE_READ_DATA;
                        xmodemDeviceInstance[index].xmAssembledData[0] = XMODEM_SOH;
                        xmodemDeviceInstance[index].xmAssembledIndex = 1;
                        if (xmodemDeviceInstance[index].xmSOHchar == 'C')
                            xmodemDeviceInstance[index].xmPacketSize = 133; /* SOH + N + N' + data + CRC16 */
                        else
                            xmodemDeviceInstance[index].xmPacketSize = 132; /* SOH + N + N' + data + CHK8 */
                        break;

                    case XMODEM_STX:
                        /* 1024 byte packets */
                        xmodemDeviceInstance[index].xmState = XMODEM_STATE_READ_DATA;
                        xmodemDeviceInstance[index].xmAssembledData[0] = XMODEM_STX;
                        xmodemDeviceInstance[index].xmAssembledIndex = 1;
                        xmodemDeviceInstance[index].xmPacketSize = 1029;
                        break;

                    case XMODEM_EOT:
                        _DRV_XMODEM_SendChar(index, XMODEM_ACK);
                        xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
                        rVal = XMODEM_RETURN_COMPLETE;
                        break;

                    case XMODEM_CAN:
                        /* transfer aborted by host */
                        rVal = XMODEM_RETURN_ABORT;
                        xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
                        break;
                    default:
                        /* unhandled character */
                        break;
                }
            }
            
            /* handle timeouts */
            if (SYS_TMR_DelayStatusGet(xmodemDeviceInstance[index].xmTimer) == true)
            {
                /* 1s timer elapsed */
                xmodemDeviceInstance[index].xmTimeoutCounter--;
                if (xmodemDeviceInstance[index].xmTimeoutCounter == 0)
                {
                    /* timed out whilst waiting for a header message */
                    xmodemDeviceInstance[index].xmRepeatCounter--;
                    if (xmodemDeviceInstance[index].xmRepeatCounter == 0)
                    {
                        /* try reverting to a NAK start */
                        if (xmodemDeviceInstance[index].xmSOHchar == 'C')
                        {
                            /* try sending NAK starts */
                            xmodemDeviceInstance[index].xmSOHchar = XMODEM_NAK;
                            xmodemDeviceInstance[index].xmTimeoutCounter = 5;
                            xmodemDeviceInstance[index].xmRepeatCounter = 6;
                        }
                        else
                        {
                            /* failed even on NAK starts so abort */
                            rVal = XMODEM_RETURN_ABORT;
                            xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
                        }                         
                    }
                    else
                    {
                        /* send another session start */
                        xmodemDeviceInstance[index].xmTimeoutCounter = 5;
                        xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_INVITE;
                    }
                }

                /* restart the timer */
                xmodemDeviceInstance[index].xmTimer = SYS_TMR_DelayMS(1000);
            }          
            break;

        case XMODEM_STATE_READ_DATA:
            /* transfer incoming data to the xmodem buffer for building up into packets */
            xdata = XMODEM_Read_Buffer(index);
            if (xdata >= 0)
            {
                xmodemDeviceInstance[index].xmTimeoutCounter = 5;
                
                c = (uint8_t) xdata;
                /* add it to the buffer */
                xmodemDeviceInstance[index].xmAssembledData[xmodemDeviceInstance[index].xmAssembledIndex] = c;
                xmodemDeviceInstance[index].xmAssembledIndex++;

                /* check for the possible received lengths */
                if (xmodemDeviceInstance[index].xmAssembledIndex == xmodemDeviceInstance[index].xmPacketSize)
                {
                    xmodemDeviceInstance[index].xmState = XMODEM_STATE_CRC_CHECK;
                    break;
                }
            }

            /* check for timeouts */
            if (SYS_TMR_DelayStatusGet(xmodemDeviceInstance[index].xmTimer) == true)
            {
                /* 1s timer elapsed */
                xmodemDeviceInstance[index].xmTimeoutCounter--;
                if (xmodemDeviceInstance[index].xmTimeoutCounter == 0)
                {
                    /* send another session start */
                    xmodemDeviceInstance[index].xmTimeoutCounter = 5;
                    xmodemDeviceInstance[index].xmRepeatCounter = 6;
                    xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_INVITE;
                }

                /* restart the timer */
                xmodemDeviceInstance[index].xmTimer = SYS_TMR_DelayMS(1000);
            }
            break;

        case XMODEM_STATE_CRC_CHECK:
            /* check the data integrity based upon start symbol */
            if (xmodemDeviceInstance[index].xmSOHchar == XMODEM_NAK)
            {
                /* checksum based check*/
                c = 255 - xmodemDeviceInstance[index].xmAssembledData[1];
                cmpl = xmodemDeviceInstance[index].xmAssembledData[2];
                csum = 0;
                for(i = 3; i < 131; i++)
                {
                    csum += xmodemDeviceInstance[index].xmAssembledData[i];
                }

                csum = csum & 0xFF;
                if ((cmpl == c) && (csum == xmodemDeviceInstance[index].xmAssembledData[131]))
                {
                    xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_ACK;
                }
                else
                {
                    xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_NAK;
                }
            }
            else
            {
                /* CRC based check */
                Nop();
                Nop();
                xmodemDeviceInstance[index].xmState = XMODEM_STATE_SEND_ACK;
            }

            break;

        case XMODEM_STATE_SEND_NAK:
            _DRV_XMODEM_SendChar(index, XMODEM_NAK);
            xmodemDeviceInstance[index].xmState = XMODEM_STATE_WAIT_SOH;
            break;

        case XMODEM_STATE_SEND_ACK:
            _DRV_XMODEM_SendChar(index, XMODEM_ACK);
            *count = 128;
            *pData = &xmodemDeviceInstance[index].xmAssembledData[3];
            rVal = XMODEM_RETURN_PROCESS;
            xmodemDeviceInstance[index].xmTimeoutCounter = 5;
            xmodemDeviceInstance[index].xmState = XMODEM_STATE_WAIT_SOH;
            break;

        case XMODEM_STATE_ABORT:
            /* tell the application to abort */
            xmodemDeviceInstance[index].xmTimeoutCounter = 5;
            xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
            break;

        default:
            break;
    }

    return rVal;
}

/*********************************************************************
 * Function:        void APP_XMODEM_Start(SYS_MODULE_OBJ object)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initiate an xmodem transfer
 *
 * Note:            None
 ********************************************************************/
void DRV_XMODEM_Start(SYS_MODULE_OBJ object)
{
    SYS_MODULE_INDEX index = (SYS_MODULE_INDEX) object;

    /* make a copy of the current CDC event handler */
    xmodemDeviceInstance[index].defaultEventHandler = gUSBDeviceCDCInstance[USB_DEVICE_CDC_INDEX_0].appEventCallBack;
    /* make a copy of the current user data */
    xmodemDeviceInstance[index].defaultUserData = gUSBDeviceCDCInstance[USB_DEVICE_CDC_INDEX_0].userData;

    /* Register our xmodem device as the event handler */
    USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, XMODEMUSBDeviceCDCEventHandler,
                                   (uintptr_t) &xmodemDeviceInstance[index]);

    /* pre-init the read/write flags */
    xmodemDeviceInstance[index].isWriteComplete = true;
    xmodemDeviceInstance[index].isReadComplete = true;
    xmodemDeviceInstance[index].rxLength = 0;
    xmodemDeviceInstance[index].xmState = XMODEM_STATE_INIT;
}
/*********************************************************************
 * Function:        void APP_XMODEM_Finish(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        clean up after an xmodem session
 *
 * Note:            None
 ********************************************************************/
void DRV_XMODEM_Finish(SYS_MODULE_OBJ object)
{
    SYS_MODULE_INDEX index = (SYS_MODULE_INDEX) object;

    /* restore the USB message routing */
    USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, xmodemDeviceInstance[index].defaultEventHandler,
                                   xmodemDeviceInstance[index].defaultUserData);
    xmodemDeviceInstance[index].defaultEventHandler = NULL;
    xmodemDeviceInstance[index].defaultUserData = (uintptr_t) NULL;
    xmodemDeviceInstance[index].rxLength = 0;
    xmodemDeviceInstance[index].xmState = XMODEM_STATE_IDLE;
}
/*********************************************************************
 * Function:        SYS_STATUS DRV_XMODEM_Status(SYS_MODULE_OBJ object)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
SYS_STATUS DRV_XMODEM_Status(SYS_MODULE_OBJ object)
{
    SYS_STATUS status;
    SYS_MODULE_INDEX index = (SYS_MODULE_INDEX)object;

    if (index >= XMODEM_MAX_OBJ_INSTANCES)
        return SYS_STATUS_ERROR;

    status = xmodemDeviceInstance[index].xmodemStatus;

    return status;
}
/*********************************************************************
 * Function:        USB_DEVICE_CDC_EVENT_RESPONSE XMODEMUSBDeviceCDCEventHandler(USB_DEVICE_CDC_INDEX index,
 *                                              USB_DEVICE_CDC_EVENT event, void* pData, uintptr_t userData)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        XMODEM customized CDC event handler
 *
 * Note:            None
 ********************************************************************/
USB_DEVICE_CDC_EVENT_RESPONSE XMODEMUSBDeviceCDCEventHandler(USB_DEVICE_CDC_INDEX index,
    USB_DEVICE_CDC_EVENT event, void* pData, uintptr_t userData)
{
    XMODEM_OBJECT_INSTANCE *xmodemDataObject;
    USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *readEventData;

    xmodemDataObject = (XMODEM_OBJECT_INSTANCE*) userData;
    readEventData = (USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE*) pData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /* This means that the host has sent some data*/
            xmodemDataObject->isReadComplete = true;
            xmodemDataObject->rxLength = readEventData->length;
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:
            /* This means that the data write completed. We can schedule the next write. */
            xmodemDataObject->isWriteComplete = true;
            break;

        default:
            /* pass all unhandled events to the default sys_console_usb_cdc handler */
            if (xmodemDataObject->defaultEventHandler != NULL)
            {
                return xmodemDataObject->defaultEventHandler(index, event, pData, xmodemDataObject->defaultUserData);
            }
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}
/*********************************************************************
 * Function:        static void _DRV_XMODEM_SendChar(SYS_MODULE_INDEX index, uint8_t c)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Send a single character over the USB pipe
 *
 * Note:            None
 ********************************************************************/
static void _DRV_XMODEM_SendChar(SYS_MODULE_INDEX index, uint8_t c)
{    
    xmTxUSBBuffer[0] = c;
    xmodemDeviceInstance[index].writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    xmodemDeviceInstance[index].isWriteComplete = false;

    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0, &xmodemDeviceInstance[index].writeTransferHandle,
                    xmTxUSBBuffer, 1, USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);  
}



