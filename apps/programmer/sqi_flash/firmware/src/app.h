/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _APP_H
#define _APP_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "driver/touch/drv_touch.h"
#include "drv_nvm_flash_sqi_sst26.h"
#include "system_definitions.h"
#include "GenericTypeDefs.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define MAX_NUM_OF_BYTES        512

#define MAX_NUM_OF_BYTES_IN_BUF (MAX_NUM_OF_BYTES + 4)

#if defined(GFX_USE_DISPLAY_CONTROLLER_LCC)
#define BOOTLOADER_IMAGE_FILE_NAME  "gfx_resources_ext.hex"
#else
#define BOOTLOADER_IMAGE_FILE_NAME  "image.hex"
#endif
    
// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,

    APP_STATE_FLASH_INIT,
    
    APP_STATE_USB_TIMER_SET,
        /* Application done */
    APP_STATE_DONE,

    /* Application Error state*/
    APP_STATE_ERROR
                
} APP_STATES;

typedef enum
{
    /* Application's state machine's initial state. */
    FLH_STATE_INIT=0,

    /* Application done */
    FLH_STATE_DONE,

    FLH_STATE_SETUP,
    /* TODO: Define states used by the application state machine. */
    FLH_STATE_CHECK_DRVR_STATE,

    FLH_STATE_ERASE,

    FLH_STATE_LOADRES,

    FLH_STATE_READ_FILE,

    FLH_STATE_PROCESS_RECORD,

    FLH_STATE_PROCESS_BUFFER,

    FLH_STATE_LOADRES_COMPLETE,

    FLH_STATE_WAIT_REMOVE,

    FLH_STATE_LOADRES_BOOT,

    FLH_STATE_LOADRES_EFL,

    FLH_STATE_ERASE_EFL,

    FLH_STATE_READ_FILE_EFL,

    /* Application Error state*/
    FLH_STATE_ERROR
    
} FLASH_STATES;



typedef enum
{


    USB_STATE_WFIX,
            
    USB_STATE_BUS_ENABLE,
            
    USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE,
      
    USB_STATE_WAIT_FOR_DEVICE_ATTACH,

    USB_STATE_RUN_MSD_CDC_TASKS,

    USB_STATE_DONE,


} USB_STATES;


typedef enum
{
    APP_MSD_STATE_NOP,

    /* Application waits for device attach */
    APP_MSD_STATE_WAIT_FOR_DEVICE_ATTACH,

    /* Device is connected */
    APP_MSD_STATE_DEVICE_CONNECTED,

    /* Application tries to mount the disk */
    APP_MSD_STATE_MOUNT_DISK,

    /* Application opens the file */
    APP_MSD_STATE_CHECK_FILE,

    /* Applications writes to the file */
    APP_MSD_STATE_WRITE_TO_FILE,

    /* Application closes the file */
    APP_MSD_STATE_CLOSE_FILE,

    /* Application idles */
    APP_MSD_STATE_IDLE,

    /* Application unmounts the disk */
    APP_MSD_STATE_UNMOUNT_DISK,

    APP_MSD_STATE_NODATA_FILE,

    APP_MSD_STATE_CHECK_FILE_RES,

    APP_MSD_STATE_DO_BOOT,

    APP_MSD_STATE_LOADRES_FILE,

    /* Application is in error state */
    APP_MSD_STATE_ERROR

} USB_MSD_STATES;


//typedef enum
//{
//    APP_CDC_STATE_NOP,
//    /* Application waits for CDC Device Attach */
//    APP_CDC_STATE_WAIT_FOR_DEVICE_ATTACH,
//            
//    APP_CDC_STATE_OPEN_DEVICE,
//
//    /* CDC Device is Attached */
//    APP_CDC_STATE_DEVICE_CONNECTED,
//            
//    APP_CDC_STATE_GET_LINE_CODING,
//
//    /* Application waits to get the device line coding */
//    APP_CDC_STATE_WAIT_FOR_GET_LINE_CODING,
//
//    /* Application sets the line coding */
//    APP_CDC_STATE_SEND_SET_LINE_CODING,
//
//    /* Appliction waits till set line coding is done */
//    APP_CDC_STATE_WAIT_FOR_SET_LINE_CODING,
//
//    /* Application sets the contol line state */
//    APP_CDC_STATE_SEND_SET_CONTROL_LINE_STATE,
//
//    /* Application waits for the set control line state to complete */
//    APP_CDC_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE,
//
//    /* Application sends the prompt to the device */
//    APP_CDC_STATE_SEND_PROMPT_TO_DEVICE,
//
//    /* Application waits for prompt send complete */
//    APP_CDC_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE,
//
//    /* Application request to get data from device */
//    APP_CDC_STATE_GET_DATA_FROM_DEVICE,
//
//    /* Application waits for data from device */
//    APP_CDC_STATE_WAIT_FOR_DATA_FROM_DEVICE,
//
//    /* Application has received data from the device */
//    APP_CDC_STATE_DATA_RECEIVED_FROM_DEVICE,
//
//    /* Application is in error state */
//    APP_CDC_STATE_ERROR
//
//} USB_CDC_STATE;
//typedef struct
//{
//    /* CDC application tasks state */
//    USB_CDC_STATE  state;
//    
//    /* CDC Object */
//    USB_HOST_CDC_OBJ cdcObj;
//    
//    /* Handle to the CDC device. */
//    USB_HOST_CDC_HANDLE cdcHostHandle;
//
//    /* Array to hold read data */
//    uint8_t inDataArray[64];
//    
//    /* True if control request is done */
//    bool controlRequestDone;
//    
//    /* Control Request Result */
//    USB_HOST_CDC_RESULT controlRequestResult;
//    
//    /* True when a write transfer has complete */
//    bool writeTransferDone;
//    
//    /* Write Transfer Result */
//    USB_HOST_CDC_RESULT writeTransferResult;
//    
//     /* True when a read transfer has complete */
//    bool readTransferDone;
//    
//    /* Read Transfer Result */
//    USB_HOST_CDC_RESULT readTransferResult;
//    
//      /* True if a device is attached */
//    bool deviceIsAttached;
//    
//
//    /* Transfer Handles */
//    USB_HOST_CDC_TRANSFER_HANDLE getLinecodeTransferHandle;
//    USB_HOST_CDC_TRANSFER_HANDLE setLinecodeTransferHandle;
//    USB_HOST_CDC_TRANSFER_HANDLE transferHandle;
//
//    /* Line coding and other CDC related data */
//    USB_CDC_LINE_CODING cdcHostLineCoding;
//    USB_CDC_LINE_CODING cdcDeviceLineCoding;
//    USB_CDC_CONTROL_LINE_STATE controlLineState;
//
//    bool deviceWasDetached;
//
//} USB_CDC_DATA;

typedef struct
{


    /* SYS_FS File handle for 1st file */
    SYS_FS_HANDLE fileHandle;

    /* SYS_FS File handle for 2nd file */
    SYS_FS_HANDLE fileHandle1;

    /* Application's current state */
    USB_MSD_STATES State;

    /* Application data buffer */
    uint8_t data[1024];

    /* Number of bytes written */
    uint32_t nBytesWritten;

    /* Number for bytes read */
    uint32_t nBytesRead;

} USB_MSD_DATA;

typedef struct
{

    USB_STATES State;
   
    /* Application's current state */
    USB_MSD_DATA USBD;
//    USB_CDC_DATA USBC;
            
    uint8_t  LastError;
    uint32_t TimeFix;
    bool     deviceIsConnected;
    bool     msdDeviceIsConnected;
    bool     cdcDeviceIsConnected;
    char     mntPath[100];
    char     externalResourceFile[100];

} USB_DATA;



typedef struct
{
    /* The application's current state */
    FLASH_STATES  State;
    SYS_FS_HANDLE fileHandle;
    long          fileSize;
    uint8_t      LoadPercent;
    uint8_t       fileBuffer[512] __attribute__((coherent, aligned(4)));
    uint8_t       TestVal;
    bool          Boot;
    uint8_t       LastError;
    char          flashSrc[100];  
} FLH_DATA;


typedef struct
{
    /* The application's current state */
    APP_STATES State;
    uint8_t  LastError;
    DRV_TOUCH_SAMPLE_POINTS samplePoints;
    uint32_t StartTime;
} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );
void APP_FLASH_Task ( void );
void APP_USB_Task ( void );
//void APP_USBHostCDCAttachEventListener
//(
//    USB_HOST_CDC_OBJ cdcObj, 
//    uintptr_t context
//);
//
#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

