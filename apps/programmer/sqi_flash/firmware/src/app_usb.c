/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_scanner.c

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include "app.h"
#include "app_usb.h"


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/
void USB_MSD_Tasks (void );
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************
 * USB HOST Layer Events - Host Event Handler
 *******************************************************/
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
  
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
            usbData.deviceIsConnected = true;
            usbData.msdDeviceIsConnected = true;
            break;
            
        case SYS_FS_EVENT_UNMOUNT:
            usbData.deviceIsConnected = false;
            usbData.msdDeviceIsConnected = false;
            usbData.USBD.State = APP_MSD_STATE_UNMOUNT_DISK;
            
            break;
            
        default:
            break;
    }
}

void APP_USB_Task( void )
{

     switch ( usbData.State )
     {
         case USB_STATE_WFIX:
                   usbData.State = USB_STATE_BUS_ENABLE;                   
             break;
         case  USB_STATE_BUS_ENABLE:
             
             /* Set the event handler and enable the bus */
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t)NULL);
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);
            usbData.State = USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
             
             break;
             
             
         case USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
             
            if(USB_HOST_BusIsEnabled(0))
            {
                usbData.State = USB_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
             
             break;
             
           case USB_STATE_WAIT_FOR_DEVICE_ATTACH:

            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            if(usbData.deviceIsConnected)
            {
              
                usbData.State = USB_STATE_RUN_MSD_CDC_TASKS;
            }

            break;   
             
        
         case USB_STATE_RUN_MSD_CDC_TASKS:

            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            /* Run the Application CDC and MSD Tasks */
             //if(usbData.msdDeviceIsConnected)
             //{
                USB_MSD_Tasks();
             //}
//             if ( usbData.cdcDeviceIsConnected )
//             {
//              USB_CDC_Tasks();
//             }

            break;

          case USB_STATE_DONE:      

             break;

     }
};

//void APP_USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context)
//{
//    /* This function gets called when the CDC device is attached. Update the
//     * application data structure to let the application know that this device
//     * is attached */
//    usbData.deviceIsConnected = true;
//    usbData.cdcDeviceIsConnected = true;
//    usbData.USBC.cdcObj = cdcObj;
//}