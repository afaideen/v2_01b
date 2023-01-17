/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_usb_msd.c

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

void USB_MSD_Initialize (void)
{
    usbData.USBD.State  = APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH;
}


/*************************************************
 * Application MSD Tasks
 *************************************************/

void USB_MSD_Tasks (void )
{
    /* The application task state machine */

    switch(usbData.USBD.State)
    {
         case APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH:

            
            
                /* In this state the application is waiting for the device to be
             * attached */
            if(usbData.msdDeviceIsConnected)
            {
                /* A device is attached. We can open this device */
                usbData.USBD.State = APP_MSD_STATE_CHECK_FILE;
               
            }
            break;
            
            case APP_MSD_STATE_UNMOUNT_DISK:

            /* Update the LED status and unmount the disk */
              SYS_FS_Unmount("/mnt/myDrive");
              usbData.USBD.State =  APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH;
              break;
              
        case APP_MSD_STATE_CHECK_FILE:

            usbData.USBD.fileHandle = SYS_FS_FileOpen(usbData.mntPath, (SYS_FS_FILE_OPEN_READ));
            if(usbData.USBD.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                SYS_FS_FileClose(usbData.USBD.fileHandle);
                usbData.USBD.State = APP_MSD_STATE_IDLE;
            }
            else
            {
                   SYS_FS_FileClose(usbData.USBD.fileHandle);
                   flashData.State = FLH_STATE_LOADRES;       
                   usbData.USBD.State = APP_MSD_STATE_IDLE;
            }
            break;

        case APP_MSD_STATE_CHECK_FILE_RES:
            usbData.USBD.fileHandle = SYS_FS_FileOpen(usbData.mntPath, (SYS_FS_FILE_OPEN_READ));
            if(usbData.USBD.fileHandle != SYS_FS_HANDLE_INVALID)
            {                 
                 SYS_FS_FileClose(usbData.USBD.fileHandle);
                 usbData.USBD.State = APP_MSD_STATE_IDLE;
            };
            
            break;
        case APP_MSD_STATE_NODATA_FILE:
            break;

        case APP_MSD_STATE_LOADRES_FILE:

            /* Close the file */
            //SYS_FS_FileClose(usbData.USBD.fileHandle);

            /* The test was successful. Lets idle. */
            usbData.USBD.State = APP_MSD_STATE_NODATA_FILE;
            break;

        case APP_MSD_STATE_IDLE:

            break;

        case APP_MSD_STATE_ERROR:

            break;

        case APP_MSD_STATE_DO_BOOT:
             //State App boot
            break;

        default:
            break;
    }
}


